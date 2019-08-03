/**************************************************************************//**
 * @file     NucModule.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 13/11/11 11:27a $
 * @brief    NUC123 Series Multiple Module Data Flow Control Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/*!<Includes */
#include "NucInternal.h"
#include "NucBridgeApi.h"

#include "usbd.h"
#include "..\nubridge.h"

/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   port            GPIO port. It could be PA, PB, PC, PD or PF.
 * @param[in]   u32Pin          The pin of specified GPIO port. \n
 *                              It could be 10 ~ 15 for PA GPIO port. \n
 *                              It could be 0 ~ 10 and 12 ~ 15 for PB GPIO port. \n
 *                              It could be 0 ~ 5 and 8 ~ 13 for PC GPIO port. \n
 *                              It could be 0 ~ 5 and 8 ~ 11 for PD GPIO port. \n
 *                              It could be 0 ~ 3 for PF GPIO port.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be :
 *                              - \ref GPIO_INT_RISING
 *                              - \ref GPIO_INT_FALLING
 *                              - \ref GPIO_INT_BOTH_EDGE
 *                              - \ref GPIO_INT_HIGH
 *                              - \ref GPIO_INT_LOW
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    port->IMD |= (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);
    port->IEN |= ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);
}

/*
#define         MODE_I2C_MONITOR        0x00
#define         MODE_SPI0_MASTER        0x01
#define         MODE_SPI_MONITOR        0x02
#define         MODE_I2C0_MASTER        0x03
#define         MODE_SPI2_MASTER        0x04
*/

// Only 4 Group of Setting
uint8_t g_u8arrMapping[4] = {0, 0, 0, 1};

uint8_t g_u8Padding = 0;


volatile uint8_t g_u8DeInit;
volatile uint8_t g_u8InitSetting;

volatile uint8_t g_u8CheckConflict;

volatile uint8_t timer0_tmp;


void Nuc_DataOut(void);
void Nuc_CS_LabelOut(void);
void Nuc_Command(void);
void Nuc_DataIn(void);
void Nuc_InitModule(void);
void Nuc_DeInitModule(void);
void Nuc_CheckConflict(void);
void Nuc_AutoVCOM(void);

/**
  * @brief  Using PB.14 to trigger LED
  * @param  None.
  * @retval None.
  */
void LED_On(void)
{
    NVIC_DisableIRQ(TMR0_IRQn);
    timer0_tmp += 20;
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER0->TCSR |= TIMER_TCSR_CEN_Msk;
}

uint8_t *g_pu8EP6Addr;
IRQn_Type IRQn_Rec;

/**
  * @brief  Initial dynamic memory pointer address
  * @param  None.
  * @retval None.
  */
void Nuc_Bridge_Init(void)
{
    g_pu8EP6Addr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5));
    IRQn_Rec = SPI1_IRQn;   // SPI1_IRQn is non-used
    g_u8ModeStatus = &(g_tdsModeInfo.u8ModeStatus[0]);
    g_u8ParamSet = &(g_tdsModeInfo.u8ParamSet[0]);
    g_u8SPI0RxBuf = g_u8RxBuf;                          // 0x0000
    g_u8SPI2RxBuf = g_u8SPI0RxBuf + RX_SIZE_PART;       // 0x0800
    // I2C0 and SPI2 share the same pins in NuBridge hardware
    g_u8I2C0RxBuf = g_u8SPI2RxBuf;                      // 0x0800
//  g_u8I2C2RxBuf = g_u8I2C0RxBuf + RX_SIZE_PART;       // 0x1000
    g_u8SPI0TxBuf = g_u8RxBuf + 4 * RX_SIZE_PART;       // 0x2000
    g_u8SPI2TxBuf = g_u8SPI0TxBuf + TX_SIZE_PART;       // 0x2400
    g_u8I2C0TxBuf = g_u8SPI2TxBuf;                      // 0x2400
//  g_u8I2C2TxBuf = g_u8I2C0TxBuf + TX_SIZE_PART;       // 0x2800

    if (g_u8VComEnable) {
        g_u8ModeStatus[MODE_VCOM_UART] = MODE_RUN;
    } else {
        g_u8ModeStatus[MODE_VCOM_UART] = MODE_OFF;
    }

    g_u8IsMaster = 0;
    g_u8InitSetting = 0;
    g_u8CheckConflict = 0;
}

/**
  * @brief  PDMA interrupt handler, trigger USB Bulk in start and update queue size
  * @param  None.
  * @retval None.
  */
#define HEADER_PDMA 4
void PDMA_IRQHandler(void)
{
    PDMA1->ISR = PDMA_ISR_BLKD_IF_Msk;
    USBD->ATTR |= 0x400;
    // g_u16RxByte is used by Single Monitor Mode
    g_u16RxByte -= (gu32OutSize - g_u8Padding);
    // Write USB header for Padding
    USBD_SET_PAYLOAD_LEN(EP5, gu32OutSize + HEADER_PDMA);
}

/**
  * @brief  PDMA initial, g_u8PDMARxBuf could be any module queue buffer
  * @param  None.
  * @retval None.
  */
void PDMA_Init(void)
{
    STR_PDMA_T sParam;
    uint32_t ByteCnt = (gu32OutSize + 3) & 0xFFFC;

    if (ByteCnt & 0x40) {
        ByteCnt = 60;
    }

    //PDMA Init
    UNLOCKREG();
    // Enable PDMA Clock
    CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;
    // Trigger PDMA
    sParam.u32ByteCnt = ByteCnt;    // Always copy maximum size
    sParam.u32SrcCtrl    = (0UL << PDMA_CSR_SAD_SEL_Pos);
    sParam.u32SrcAddr    = (uint32_t)&g_u8PDMARxBuf[g_u16RxHead & 0xFFFC];
    sParam.u32DestCtrl   = (0UL << PDMA_CSR_DAD_SEL_Pos);
    sParam.u32DestAddr   = (uint32_t)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5) + HEADER_PDMA);
    sParam.u32TransWidth = (1UL << PDMA_CSR_APB_TWS_Pos);
    sParam.u32Mode       = (0UL << PDMA_CSR_MODE_SEL_Pos);
    //sParam.u32ByteCnt    = 64;        // Update ByteCnt in PDMA_ByteCnt()
    // DrvPDMA_Init(PDMA1, &sParam);
    {
        uint32_t u32Channel = (((uint32_t)PDMA1) & 0x0F00) >> 8;
        PDMA_GCR->GCRCSR |= (1 << (8 + u32Channel));
        PDMA1->CSR |= PDMA_CSR_SW_RST_Msk;
        PDMA1->CSR = sParam.u32SrcCtrl | sParam.u32DestCtrl | sParam.u32TransWidth |
                     sParam.u32Mode | PDMA_CSR_PDMACEN_Msk;
        PDMA1->SAR = sParam.u32SrcAddr;
        PDMA1->DAR = sParam.u32DestAddr;
        PDMA1->BCR = sParam.u32ByteCnt;
    }
    // memory copy
    PDMA1->CSR |= PDMA_CSR_TRIG_EN_Msk;
    //while( (PDMA1->ISR&PDMA_BLKD_IF) == 0){};
    //AHBCLK_DISABLE(CLK_PDMA);
    // DrvPDMA_EnableInt(PDMA1, PDMA_BLKD_INT); //En:Enable PDMA1 transfer done and target abort interrupt
    PDMA1->IER |= PDMA_ISR_BLKD_IF_Msk;
    NVIC_EnableIRQ(PDMA_IRQn);
}

/**
  * @brief  Multiple module data flow control process
  *             Process data  and command from USB host
  *             Update module status
  *             Transfer data back to USB host
  * @param  None.
  * @retval None.
  */
void Nuc_Bridge_Main(void)
{
    uint8_t sts = 0;
    // Parsing Usb Command Here
    Nuc_Command();

    // Some Module is enable, check conflict status
    if (g_u8CheckConflict) {
        Nuc_CheckConflict();
        Nuc_AutoVCOM();
    }

    //  Module is disable by user, or conflict to other module
    if (g_u8DeInit) {
        Nuc_DeInitModule();
        Nuc_AutoVCOM();
    }

    // Module initialized, enable multiple function pin and register corresponding interrupt callback
    Nuc_InitModule();
    Nuc_DataIn();
    // Mode SPI0 Master
    sts = g_u8ModeStatus[MODE_SPI0_MASTER];

    if (sts) {
        switch (sts) {
            case MODE_RESET:
                SPI0_Master_Init();
                sts = MODE_RUN;
                break;

            case MODE_RUN:
                g_u8IsMaster = 1;

                if (g_u8SPI0Ready) {
                    SPI0_Master_MainProcess();
                    g_u8SPI0Ready = 0;
                }

                break;
        }

        // Update New Status
        g_u8ModeStatus[MODE_SPI0_MASTER] = sts;
    }

    // Mode SPI2 Master
    sts = g_u8ModeStatus[MODE_SPI2_MASTER];

    if (sts) {
        switch (sts) {
            case MODE_RESET:
                SPI2_Master_Init();
                sts = MODE_RUN;
                break;

            case MODE_RUN:
                g_u8IsMaster = 1;

                if (g_u8SPI2Ready) {
                    SPI2_Master_MainProcess();
                    g_u8SPI2Ready = 0;
                }

                break;
        }

        // Update New Status
        g_u8ModeStatus[MODE_SPI2_MASTER] = sts;
    }

    // Mode I2C0 Master
    sts = g_u8ModeStatus[MODE_I2C0_MASTER];

    if (sts) {
        switch (sts) {
            case MODE_RESET:
                I2C0_Master_Init();
                sts = MODE_RUN;
                break;

            case MODE_RUN:
                g_u8IsMaster = 1;

                if (g_u8I2C0Ready) {
                    I2C0_Master_MainProcess();
                    g_u8I2C0Ready = 0;
                }

                break;
        }

        // Update New Status
        g_u8ModeStatus[MODE_I2C0_MASTER] = sts;
    }

    ///////////////////////////////////////////////////////////////////////////////////
    //// NucBridge transfer data to PC                                                       ////
    ///////////////////////////////////////////////////////////////////////////////////
    Nuc_CS_LabelOut();
    Nuc_DataOut();
}

/**
  * @brief  Process command from USB host
  * @param  None.
  * @retval None.
  */
void Nuc_Command(void)
{
    if (g_u8IsCmd) {
        USBD_GetSetupPacket(&g_tdsUsbCmd.type);

        switch (g_tdsUsbCmd.type) {
            case WM_SWITCH_MODE:                                                        // Switch

                // if select mode is not avariable, enable it
                if (g_u8ModeStatus[g_tdsUsbCmd.mode] == 0) {
                    g_u8ModeStatus[g_tdsUsbCmd.mode] = MODE_RESET;
                    g_u8CheckConflict = 1;
                }

                break;

            case WM_SET_DISABLE:                                                        // Stop

                // Monitor mode is implement by Interrupt
                if (g_u8ModeStatus[g_tdsUsbCmd.mode]) {
                    g_u8ModeStatus[g_tdsUsbCmd.mode] = MODE_DISABLE;
                    g_u8DeInit = 1;
                }

                break;

            case WM_LED:
                LED_On();
                break;

            // Case Re-Init
            case WM_REINIT_SPI0:
                SPI0_Master_ReInit(g_tdsUsbCmd.mode);
                g_u8ModeStatus[MODE_SPI0_MASTER] = MODE_RUN;
                break;

            case WM_REINIT_SPI2:
                SPI2_Master_ReInit(g_tdsUsbCmd.mode);
                g_u8ModeStatus[MODE_SPI2_MASTER] = MODE_RUN;
                break;

            case WM_REINIT_SPIM:
                SPI0_Monitor_ReInit(g_tdsUsbCmd.mode);
                g_u8ModeStatus[MODE_SPI_MONITOR] = MODE_RUN;
                break;

            case WM_REINIT_I2C0:
                I2C0_Master_ReInit(g_tdsUsbCmd.mode);
                g_u8ModeStatus[MODE_I2C0_MASTER] = MODE_RUN;
                break;

            case WM_RESET:
                Nuc_Reset();
                break;
        } // switch(g_tdsUsbCmd.type)

        g_u8IsCmd = 0;          // Command done
    }
}

/**
  * @brief  Process data from USB host
  * @param  None.
  * @retval None.
  */
void Nuc_DataIn(void)
{
    if (g_u8BulkOutBuf[0]) {
        g_u8IsMaster = 1;

        switch (g_u8BulkOutBuf[0]) {
            case MODE_SPI0_MASTER:
                g_u8SPI0Ready = 1;
                break;

            case MODE_SPI2_MASTER:
                g_u8SPI2Ready = 1;
                break;

            case MODE_I2C0_MASTER:
                g_u8I2C0Ready = 1;
                break;
        }

        g_u8BulkOutBuf[0] = 0;
    }
}

/**
  * @brief  Transfer data back to USB host using PDMA channel
  * @param  None.
  * @retval None.
  */
void Nuc_DataOut(void)
{
    ///////////////////////////////////////////////////////////////////////////////////
    //// NucBridge transfer data to PC                                                            ////
    ///////////////////////////////////////////////////////////////////////////////////
    if (g_u8IsMaster) {
        g_u16RxByte = g_u16SPI0RxByte + g_u16SPI2RxByte + g_u16I2C0RxByte;
    }

    if (g_u16RxByte && (gu32OutSize == 0)) {
        uint8_t u8Module = 0;
        uint16_t BUF_SIZE = 0;
        uint16_t BUF_BOUND = 0;
        USBD->ATTR |= 0x400;
        g_pu8EP6Addr[0] = 0x01;

        if (g_u8IsMaster) { // Multiple Master Module Case
            // Hook Original Data Flow
            // Hook
            // 1. Buffer Bounded
            // 2. Buffer Address
            // 3. Buffer Head Pointer
            // 4. Buffer Content Size
            BUF_SIZE = RX_SIZE_PART;
            BUF_BOUND = RX_BOUND_PART;

            if (g_u16SPI0RxByte) {
                u8Module = MODE_SPI0_MASTER;
                g_u8PDMARxBuf = g_u8SPI0RxBuf;
                g_u16RxHead = g_u16SPI0RxHead;
                g_u16RxByte = g_u16SPI0RxByte;
            } else if (g_u16SPI2RxByte) {
                u8Module = MODE_SPI2_MASTER;
                g_u8PDMARxBuf = g_u8SPI2RxBuf;
                g_u16RxHead = g_u16SPI2RxHead;
                g_u16RxByte = g_u16SPI2RxByte;
            } else if (g_u16I2C0RxByte) {
                u8Module = MODE_I2C0_MASTER;
                g_u8PDMARxBuf = g_u8I2C0RxBuf;
                g_u16RxHead = g_u16I2C0RxHead;
                g_u16RxByte = g_u16I2C0RxByte;
            } else {
                u8Module = 0xFF;
            }
        } else { // Single Monitor Module Case
            BUF_SIZE = RX_SIZE;
            BUF_BOUND = RX_BOUND;
            g_u8PDMARxBuf = &g_u8RxBuf[0];

            if (IRQn_Rec == SPI0_IRQn) {
                u8Module = MODE_SPI_MONITOR;
            } else {
                u8Module = MODE_I2C_MONITOR;
            }
        }

        g_u8Padding = g_u16RxHead % 4; // g_u8Padding = g_u16RxHead & 0x03;
        gu32OutSize = g_u16RxByte;
        // USB 4 Byte Header: Byte 0
        g_pu8EP6Addr[0] = u8Module;
        g_pu8EP6Addr[1] = g_u8Padding;

        if (g_u16RxBufferOverflow == 1) {
            g_pu8EP6Addr[2] = 0xFF;
            g_pu8EP6Addr[3] = 0xFF;
        } else {
            g_pu8EP6Addr[2] = g_u16RxByte & 0xFF;
            g_pu8EP6Addr[3] = g_u16RxByte >> 8;
        }

        // Check Buffer Overrun
        if (gu32OutSize > (BUF_SIZE - g_u16RxHead)) {
            gu32OutSize = (BUF_SIZE - g_u16RxHead);
        }

        // Always using PDMA
        gu32OutSize += g_u8Padding;

        if (gu32OutSize > EP6_MAX_PKT_SIZE - HEADER_PDMA) {         // 4 Bytes Header
            gu32OutSize = EP6_MAX_PKT_SIZE - HEADER_PDMA;
        }

        g_u8PDMAValidByte = gu32OutSize - g_u8Padding;
        USBD->ATTR &= ~0x400;
        PDMA_Init();
        // Update g_u16RxHead
        g_u16RxHead += (gu32OutSize - g_u8Padding);
        g_u16RxHead &= BUF_BOUND;

        if (g_u8IsMaster) {
            // Un Hook
            if (u8Module == MODE_SPI0_MASTER) {
                g_u16SPI0RxHead = g_u16RxHead;
                g_u16SPI0RxByte -= g_u8PDMAValidByte;
            } else if (u8Module == MODE_SPI2_MASTER) {
                g_u16SPI2RxHead = g_u16RxHead;
                g_u16SPI2RxByte -= g_u8PDMAValidByte;
            } else if (u8Module == MODE_I2C0_MASTER) {
                g_u16I2C0RxHead = g_u16RxHead;
                g_u16I2C0RxByte -= g_u8PDMAValidByte;
            } else {
                g_u16I2C0RxHead = g_u16RxHead;
                g_u16I2C0RxByte -=  g_u8PDMAValidByte;
            }
        }
    }
}

/**
  * @brief  Transfer CS_Tag back to USB host whenever SPI_Monitor Function enabled
  * @param  None.
  * @retval None.
  */
void Nuc_CS_LabelOut(void)
{
    if (g_u16CS_LabelByte && (gu32OutSize == 0)) {
        uint8_t i = 0;

        if ((g_u16RxByte >> 4) > g_u16CS_LabelByte) {
            return;
        }

        g_pu8EP6Addr[0] = MODE_CS_TAG;
        gu32OutSize = g_u16CS_LabelByte << 1;

        if (gu32OutSize > (EP6_MAX_PKT_SIZE - HEADER_PDMA)) {         // 4 Bytes Header
            gu32OutSize = (EP6_MAX_PKT_SIZE - HEADER_PDMA);
        }

        g_pu8EP6Addr[1] = 0;    // PADDING

        if (g_u16CS_OverFlow == 1) {
            g_pu8EP6Addr[2] = 0xFF;
            g_pu8EP6Addr[3] = 0xFF;
        } else {
            g_pu8EP6Addr[2] = (g_u16CS_LabelByte << 1) & 0xFF;
            g_pu8EP6Addr[3] = (g_u16CS_LabelByte << 1) >> 8;
        }

        for (i = 0; i < gu32OutSize; i = i + 2) {
            g_pu8EP6Addr[i + HEADER_PDMA] = g_u16CS_LabelBuf[g_u16CS_LabelHead] & 0xFF;
            g_pu8EP6Addr[i + HEADER_PDMA + 1] = g_u16CS_LabelBuf[g_u16CS_LabelHead] >> 8;
            g_u16CS_LabelHead++;
            g_u16CS_LabelHead &= CS_LABEL_BOUND;
        }

        NVIC_DisableIRQ(SPI0_IRQn);
        NVIC_DisableIRQ(GPAB_IRQn);
        g_u16CS_LabelByte -= gu32OutSize >> 1;

        if (g_u16CS_OverFlow == 0) {
            NVIC_EnableIRQ(GPAB_IRQn);
            NVIC_EnableIRQ(SPI0_IRQn);
        }

        USBD_SET_PAYLOAD_LEN(EP5, gu32OutSize + HEADER_PDMA);
    }
}

/**
  * @brief  Maintain Monitor Module function, Monitor module is implemented by interrupt method
  * @param  None.
  * @retval None.
  */
void Nuc_InitModule(void)
{
    if (g_u8ModeStatus[MODE_SPI_MONITOR] == MODE_RESET) {
        SPI_Slave_Init();
        IRQn_Rec = SPI0_IRQn;
        g_u8IsMaster = 0;
        g_u16RxByte = 0;
        g_u16RxHead = 0;
        g_u16RxTail = 0;
        g_u16RxBufferOverflow = 0;
        g_u16CS_LabelTail = 0;
        g_u16CS_LabelHead = 0;
        g_u16CS_LabelByte = 0;
        g_u16CS_OverFlow = 0;
        g_u8ModeStatus[MODE_SPI_MONITOR] = MODE_RUN;
    }

    if (g_u8ModeStatus[MODE_I2C_MONITOR] == MODE_RESET) {
        IRQn_Rec = EINT1_IRQn;
        GPIO_Monitor_Init();
        g_u8IsMaster = 0;
        g_u16RxByte = 0;
        g_u16RxHead = 0;
        g_u16RxTail = 0;
        g_u16RxBufferOverflow = 0;
        g_u8ModeStatus[MODE_I2C_MONITOR] = MODE_RUN;
    }

    if (g_u8ModeStatus[MODE_VCOM_UART] == MODE_RESET) {
        UART1_Init();
        VCOM_LineCoding(1);
        g_u8ModeStatus[MODE_VCOM_UART] = MODE_RUN;
        g_u8VComEnable = 1;
    }
}

/**
  * @brief  Maintain All Module DeInit function
  * @param  None.
  * @retval None.
  */
void Nuc_DeInitModule(void)
{
    uint8_t i = 0;

    for (i = 0; i < 8; i++) {
        if (g_u8ModeStatus[i] == MODE_DISABLE) {
            switch (i) {
                case MODE_I2C_MONITOR:
                    GPIO_DeInit();
                    g_u8IsMaster = 1;
                    g_u16RxByte = 0;
                    g_u16RxHead = 0;
                    g_u16RxTail = 0;
                    g_u16RxBufferOverflow = 0;
                    break;

                case MODE_SPI_MONITOR:
                    SPI_Slave_DeInit();
                    g_u8IsMaster = 1;
                    g_u16RxByte = 0;
                    g_u16RxHead = 0;
                    g_u16RxTail = 0;
                    g_u16RxBufferOverflow = 0;
                    g_u16CS_LabelTail = 0;
                    g_u16CS_LabelHead = 0;
                    g_u16CS_LabelByte = 0;
                    g_u16CS_OverFlow = 0;
                    break;

                case MODE_SPI0_MASTER:
                    SPI0_Master_DeInit();
                    break;

                case MODE_I2C0_MASTER:
                    I2C0_Master_DeInit();
                    break;

                case MODE_SPI2_MASTER:
                    SPI2_Master_DeInit();
                    break;

                case MODE_VCOM_UART:
                    UART1_DeInit();
                    g_u8VComEnable = 0;
                    break;
            }

            g_u8ModeStatus[i] = MODE_OFF;
        }
    }

    g_u8DeInit = 0;
}

/**
  * @brief  Disable All Module Function
  * @param  None.
  * @retval None.
  */
void Nuc_Reset(void)
{
    uint8_t i = 0;

    for (i = 0; i < 8; i++) {
        if (g_u8ModeStatus[i]) {
            g_u8ModeStatus[i] = MODE_DISABLE;
            g_u8DeInit = 1;
        }
    }
}

/**
  * @brief  Check Module Conflict
  * @param  None.
  * @retval None.
  * Rule 1: Monitor Module can not coexist with other module.
  * Rule 2: Modules used the same hardware pin can not be enable at the same time.
  */

// MACRO used by function call Nuc_CheckConflict
#define DIABLE_MODULE(MID)                  \
if(g_u8ModeStatus[MID]) {                   \
    g_u8ModeStatus[MID] = MODE_DISABLE;     \
    g_u8DeInit = 1; }

void Nuc_CheckConflict(void)
{
    uint8_t i = 0;

    for (i = 0; i < 8; i++) {
        if (g_u8ModeStatus[i] == MODE_RESET) {
            switch (i) {
                // Special case for Monitor Module
                case MODE_I2C_MONITOR:
                case MODE_SPI_MONITOR:
                    Nuc_Reset();
                    g_u8ModeStatus[i] = MODE_RESET;
                    g_u8CheckConflict = 0;
                    return;

                case MODE_SPI0_MASTER:
                    DIABLE_MODULE(MODE_I2C_MONITOR);
                    DIABLE_MODULE(MODE_SPI_MONITOR);
                    break;

                case MODE_SPI2_MASTER:
                    DIABLE_MODULE(MODE_I2C_MONITOR);
                    DIABLE_MODULE(MODE_SPI_MONITOR);
                    DIABLE_MODULE(MODE_VCOM_UART);
                    DIABLE_MODULE(MODE_I2C0_MASTER);
                    break;

                case MODE_I2C0_MASTER:
                    DIABLE_MODULE(MODE_I2C_MONITOR);
                    DIABLE_MODULE(MODE_SPI_MONITOR);
                    DIABLE_MODULE(MODE_SPI2_MASTER);
                    break;

                case MODE_VCOM_UART:
                    DIABLE_MODULE(MODE_I2C_MONITOR);
                    DIABLE_MODULE(MODE_SPI_MONITOR);
                    DIABLE_MODULE(MODE_SPI2_MASTER);
                    break;
            }
        }
    }

    g_u8CheckConflict = 0;
}

void Nuc_AutoVCOM(void)
{
    uint8_t status = 0;
    status = g_u8ModeStatus[MODE_VCOM_UART];

    if (status != MODE_OFF) {
        return;
    }

    status = g_u8ModeStatus[MODE_I2C_MONITOR];

    if ((status != MODE_DISABLE) && (status != MODE_OFF)) {
        return;
    }

    status = g_u8ModeStatus[MODE_SPI_MONITOR];

    if ((status != MODE_DISABLE) && (status != MODE_OFF)) {
        return;
    }

    status = g_u8ModeStatus[MODE_SPI2_MASTER];

    if ((status != MODE_DISABLE) && (status != MODE_OFF)) {
        return;
    }

    g_u8ModeStatus[MODE_VCOM_UART] = MODE_RESET;
}


extern uint8_t g_usbd_SetupPacket[8];        /*!< Setup packet buffer */

void USBD_VendorRequest()
{
    if (g_usbd_SetupPacket[0] & 0x80) {
        USBD_PrepareCtrlIn((uint8_t *)&g_tdsModeInfo, 64);
        USBD_PrepareCtrlOut(0, 0);
    } else {
        g_u8IsCmd = 1;
        USBD_SET_DATA1(EP0);
        USBD_SET_PAYLOAD_LEN(EP0, 0);
        // USBD_PrepareCtrlOut(0,0);
    }
}
