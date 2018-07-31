/******************************************************************************
 * @file     usb_device.c
 * @brief    NANO100 series USBD driver Sample file
 * @version  2.0.0
 * @date     22, March, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC123.h"
#include "usbd.h"
#include "nubridge.h"

/*--------------------------------------------------------------------------*/
/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET) {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED()) {
            /* USB Plug In */
            USBD_ENABLE_USB();
        } else {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS) {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST) {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }

        if (u32State & USBD_STATE_SUSPEND) {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME) {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB) {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP) {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);
            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);
            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk Out
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
            // Bulk IN
            EP5_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP6) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
            // Bulk Out
            EP7_Handler();
        }
    }

    /* clear unknown event */
    USBD_CLR_INT_FLAG(u32IntSts);
}


/**
 * @brief       EP2 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP2 event
 */
void EP2_Handler(void)
{
    gu32TxSize = 0;
}


/**
 * @brief       EP3 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP3 event
 */
void EP3_Handler(void)
{
    /* Bulk OUT */
    gu32RxSize = USBD_GET_PAYLOAD_LEN(EP3);
    gpu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));
    /* Set a flag to indicate builk out ready */
    gi8BulkOutReady = 1;
}


/**
 * @brief       EP5 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP5 event
 */
void EP5_Handler(void)
{
    gu32OutSize = 0;
}


/**
 * @brief       EP6 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP6 event
 */
void EP7_Handler(void)
{
    uint8_t *pu8OutBuf;
    /* Bulk OUT */
    gu32InSize = USBD_GET_PAYLOAD_LEN(EP7);
    pu8OutBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7));
    OnDataReceived((uint8_t *)pu8OutBuf, gu32InSize);
    /* Ready to get next BULK out */
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
}




/*--------------------------------------------------------------------------*/
/**
 * @brief       VCOM CDC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for VCOM class
 */
void VCOM_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;
    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);
    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);
    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);
    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);
    /*****************************************************/
    /* EP5 ==> Bulk IN endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_IN | BULK2_IN_EP_NUM);
    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);
    /* EP7 ==> Bulk Out endpoint, address 5 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_OUT | BULK2_OUT_EP_NUM);
    /* Buffer offset for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
}

/**
 * @brief       VCOM class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process VCOM class requests
 */
void VCOM_ClassRequest(void)
{
    uint8_t buf[8];
    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80) { /* request data transfer direction */
        // Device to host
        switch (buf[1]) {
            case GET_LINE_CODE: {
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&gLineCoding, 7);
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            default: {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    } else {
        // Host to device
        switch (buf[1]) {
            case SET_CONTROL_LINE_STATE: {
                {
                    gCtrlSignal = buf[3];
                    gCtrlSignal = (gCtrlSignal << 8) | buf[2];
                }
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE: {
                USBD_PrepareCtrlOut((uint8_t *)&gLineCoding, 7);
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                /* UART setting */
                VCOM_LineCoding(1);
                break;
            }

            default: {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

/**
 * @brief       LineCoding Class Command
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to set UART baudrate
 */
void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32Reg;

    if (port == 1) {
        NVIC_DisableIRQ(UART1_IRQn);
        // Reset software fifo
        comRbytes = 0;
        comRhead = 0;
        comRtail = 0;
        comTbytes = 0;
        comThead = 0;
        comTtail = 0;
        // Reset hardware fifo
        UART1->FCR |= (UART_FCR_RFR_Msk | UART_FCR_TFR_Msk);
        // Set baudrate
        UART1->BAUD = (UART_BAUD_MODE2 | (__HIRC / gLineCoding.u32DTERate) - 2);
			

        // Set parity
        if (gLineCoding.u8ParityType == 0) {
            u32Reg = 0;    // none parity
        } else if (gLineCoding.u8ParityType == 1) {
            u32Reg = 0x08;    // odd parity
        } else if (gLineCoding.u8ParityType == 2) {
            u32Reg = 0x18;    // even parity
        } else {
            u32Reg = 0;
        }

        // bit width
        switch (gLineCoding.u8DataBits) {
            case 5:
                u32Reg |= 0;
                break;

            case 6:
                u32Reg |= 1;
                break;

            case 7:
                u32Reg |= 2;
                break;

            case 8:
                u32Reg |= 3;
                break;

            default:
                break;
        }

        // stop bit
        if (gLineCoding.u8CharFormat > 0) {
            u32Reg |= 0x4;    // 2 or 1.5 bits
        }

        UART1->LCR = u32Reg;
        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART1_IRQn);
    }
}

