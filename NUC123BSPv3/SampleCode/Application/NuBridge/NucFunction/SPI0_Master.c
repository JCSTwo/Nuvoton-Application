/**************************************************************************//**
 * @file     SPI0_Master.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 13/11/11 11:27a $
 * @brief    NUC123 Series SPI0 Master Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/*!<Includes */
#include "NucInternal.h"

//#define           MODE_SPI0_MASTER        0x01

/**
  * @brief  Initial SPI0 to be SPI master with default setting 0xF4.
  * @param  None.
  * @retval None.
  */
void SPI0_Master_Init(void)
{
    // 12 MHz, SS Active Low, Order MSB first, Type 2
    SPI0_Master_ReInit(0xF4);
}

/**
  * @brief  Initial SPI0 to be SPI master.
  *              Set PC5/PC4 as MOSI01/MISO01 of SPI0. PC5=MOSI01,PC4=MISO01
  *              Set PC0 as SS of SPI0.
  *              Set PC1 as CLK of SPI0.
  * @param  setting: the setting of SPI
  *                 bit 0~1 (SPI_TRANS_TYPE)
  *                     -00: Type0
  *                     -01: Type1
  *                     -10: Type2
  *                     -11: Type3
  *                 bit 2
  *                     -0: Least Significant Bit first
  *                     -1: Most Significant Bit first
  *                 bit 3
  *                     -0: SS active Low
  *                     -1: SS active High
  *                 bit  4~7 (clock rate)
  *                     -0000: 1 MHz
  *                     -0001: 2 MHz
  *                     -0010: 3 MHz
  *                     -0011: 4 MHz
  *                     -0100: 4.5 MHz
  *                     -0101: 6 MHz
  *                     -0111: 7.2 MHz
  *                     -1001: 9 MHz
  *                     -1111: 12 MHz
  *                     -others: Reserved
  * @retval None.
  */
void SPI0_Master_ReInit(uint8_t setting)
{
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC4_SPI0_MISO1 | SYS_GPC_MFP_PC5_SPI0_MOSI1);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC4_Msk | SYS_ALT_MFP_PC5_Msk);
    CLK->APBCLK |= CLK_APBCLK_SPI0_EN_Msk;
    SYS->IPRSTC2 |= SYS_IPRSTC2_SPI0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI0_RST_Msk;
    SPI0->CNTRL = (SPI0->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (8 << SPI_CNTRL_TX_BIT_LEN_Pos);
    SPI0->CNTRL &= ~(SPI_CNTRL_TX_NEG_Msk | SPI_CNTRL_RX_NEG_Msk);

    // Bit 0, Bit 1 for SPI_TRANS_TYPE
    switch (setting & 0x03) {
        case 0:
            SPI0->CNTRL |= SPI_MODE_0;
            break;

        case 1:
            SPI0->CNTRL |= SPI_MODE_1;
            break;

        case 2:
            SPI0->CNTRL |= SPI_MODE_3;
            break;

        case 3:
            SPI0->CNTRL |= SPI_MODE_2;
            break;
    }

    CLK->CLKSEL1 |= CLK_CLKSEL1_SPI0_S_HCLK;

    /* Set IP clock divider. SPI clock rate = HCLK / ((divider+1)*2) */
    switch ((setting & 0xF0) >> 4) {
        case 0:
            // DrvSPI_SetClockFreq(SPI0 , 1000000 , 0);
            SPI0->DIVIDER = 35;
            break;

        case 1:
            // DrvSPI_SetClockFreq(SPI0 , 2000000 , 0);
            SPI0->DIVIDER = 17;
            break;

        case 2:
            // DrvSPI_SetClockFreq(SPI0 , 3000000 , 0);
            SPI0->DIVIDER = 11;
            break;

        case 3:
            // DrvSPI_SetClockFreq(SPI0 , 4000000 , 0);
            SPI0->DIVIDER = 8;
            break;

        case 4:
            // DrvSPI_SetClockFreq(SPI0 , 4500000 , 0);
            SPI0->DIVIDER = 7;
            break;

        case 5:
            // DrvSPI_SetClockFreq(SPI0 , 6000000 , 0);
            SPI0->DIVIDER = 5;
            break;

        case 7:
            // DrvSPI_SetClockFreq(SPI0 , 7200000 , 0);
            SPI0->DIVIDER = 4;
            break;

        case 9:
            // DrvSPI_SetClockFreq(SPI0 , 9000000 , 0);
            SPI0->DIVIDER = 3;
            break;

        case 15:
            // DrvSPI_SetClockFreq(SPI0 , 12000000 , 0);
            SPI0->DIVIDER = 2;
            break;
    }

    SPI0->CNTRL &= ~SPI_CNTRL_TWOB_Msk;

    if (setting & 0x04) {
        SPI_SET_MSB_FIRST(SPI0);
    } else {
        SPI_SET_LSB_FIRST(SPI0);
    }

    if (setting & 0x08) {
        SPI0->SSR |= SPI_SSR_SS_LVL_Msk;
    }

    g_u8ParamSet[MODE_SPI0_MASTER] = setting;
}

/**
  * @brief  Disable SPI0 function and clock source. Set PC.0, PC.1, PC.4, and PC.5 as GPIO function
  * @param  None.
  * @retval None.
  */
void SPI0_Master_DeInit(void)
{
    CLK->APBCLK &= ~CLK_APBCLK_SPI0_EN_Msk;
    NVIC_DisableIRQ(SPI0_IRQn);
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC4_Msk | SYS_GPC_MFP_PC5_Msk);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC4_Msk | SYS_ALT_MFP_PC5_Msk);
    // Clear SPI0 Master recorded setting
    g_u8ParamSet[MODE_SPI0_MASTER] = 0x00;
}

/**
  * @brief  SPI0 main process function.
  *               transfer g_u8SPI0TxBuf  data to SPI slave device, update related queue pointer and queue size
  *              store received data in g_u8SPI0RxBuf, update related queue pointer and queue size
  * @param  None.
  * @retval None.
  */
void SPI0_Master_MainProcess(void)
{
    uint8_t     cmdCnt = 0;
    uint8_t     repeat = 0;
    uint8_t     bitLength = 0;
    uint8_t     dataCnt = 0;
    uint16_t    idleTimeL = 0;
    uint16_t    idleTimeH = 0;
    uint16_t    DataIndex = 0;
    uint16_t    DataByte = 0;
    uint32_t    Data = 0;
    cmdCnt = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
    g_u16SPI0TxByte--;

    while (cmdCnt) {
        // Check Valid Byte
        if ('S' == g_u8SPI0TxBuf[g_u16SPI0TxHead++]) {
            repeat = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
            idleTimeL = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
            idleTimeH = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
            idleTimeL += (idleTimeH << 8);
            g_u16SPI0TxByte -= 4;
        } else {
            // Error
            g_u16SPI0TxByte = 0;
            return;
        }

        DataIndex = g_u16SPI0TxHead;
        DataByte =  g_u16SPI0TxByte;

        while (repeat) {
            g_u16SPI0TxHead = DataIndex;
            g_u16SPI0TxByte = DataByte;
            // Start Transation
            SPI0->SSR |= SPI_SS0;
SPI_CHANGE_BIT_LENGTH:
            bitLength = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
            dataCnt = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
            SPI0->CNTRL = (SPI0->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | ((bitLength & 0x1F) << SPI_CNTRL_TX_BIT_LEN_Pos);

            switch ((bitLength + 7) >> 3) {
                case 1:
                    while (dataCnt) {
                        Data = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
                        g_u16SPI0TxByte--;
                        SPI0->TX[1] = Data;
                        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

                        while (SPI_IS_BUSY(SPI0));

                        Data = SPI0->RX[1];
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = Data & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u16SPI0RxByte++;
                        dataCnt--;
                    }

                    break;

                case 2:
                    while (dataCnt) {
                        Data = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
                        Data += (g_u8SPI0TxBuf[g_u16SPI0TxHead++] << 8);
                        g_u16SPI0TxByte -= 2;
                        SPI0->TX[1] = Data;
                        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

                        while (SPI_IS_BUSY(SPI0));

                        Data = SPI0->RX[1];
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = Data & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = (Data >> 8) & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u16SPI0RxByte += 2;
                        dataCnt--;
                    }

                    break;

                case 3:
                    while (dataCnt) {
                        Data = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
                        Data += (g_u8SPI0TxBuf[g_u16SPI0TxHead++] << 8);
                        Data += (g_u8SPI0TxBuf[g_u16SPI0TxHead++] << 16);
                        g_u16SPI0TxByte -= 3;
                        SPI0->TX[1] = Data;
                        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

                        while (SPI_IS_BUSY(SPI0));

                        Data = SPI0->RX[1];
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = Data & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = (Data >> 8) & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = (Data >> 16) & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u16SPI0RxByte += 3;
                        dataCnt--;
                    }

                    break;

                case 4:
                    while (dataCnt) {
                        Data = g_u8SPI0TxBuf[g_u16SPI0TxHead++];
                        Data += (g_u8SPI0TxBuf[g_u16SPI0TxHead++] << 8);
                        Data += (g_u8SPI0TxBuf[g_u16SPI0TxHead++] << 16);
                        Data += (g_u8SPI0TxBuf[g_u16SPI0TxHead++] << 24);
                        g_u16SPI0TxByte -= 4;
                        SPI0->TX[1] = Data;
                        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

                        while (SPI_IS_BUSY(SPI0));

                        Data = SPI0->RX[1];
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = Data & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = (Data >> 8) & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = (Data >> 16) & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u8SPI0RxBuf[g_u16SPI0RxTail++] = (Data >> 24) & 0xFF;
                        g_u16SPI0RxTail &= RX_BOUND_PART;
                        g_u16SPI0RxByte += 4;
                        dataCnt--;
                    }

                    break;

                default:
                    break;
            }

            if ('P' == g_u8SPI0TxBuf[g_u16SPI0TxHead]) {
                SPI0->SSR &= ~SPI_SS0;
                g_u16SPI0TxHead++;
            } else {
                goto SPI_CHANGE_BIT_LENGTH;
            }

            if (idleTimeL) {
                // SYS_SysTickDelay(idleTimeL);
                SysTick->LOAD = idleTimeL * CyclesPerUs;
                SysTick->VAL  = (0x00);
                SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

                // Waiting for down-count to zero
                while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
            }

            repeat--;
        }

        // Command Complete
        cmdCnt--;
    }
}
