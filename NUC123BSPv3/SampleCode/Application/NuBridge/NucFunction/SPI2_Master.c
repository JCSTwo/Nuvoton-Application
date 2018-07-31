/**************************************************************************//**
 * @file     SPI2_Master.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 13/11/11 11:27a $
 * @brief    NUC123 Series SPI2 Master Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/*!<Includes */
#include "NucInternal.h"


/**
  * @brief  Initial SPI2 to be SPI master with default setting 0xF4.
  * @param  None.
  * @retval None.
  */
void SPI2_Master_Init(void)
{
    // 12 MHz, SS Active Low, Order MSB first, Type 2
    SPI2_Master_ReInit(0xF4);
}

/**
  * @brief  Initial SPI2 to be SPI master.
  *              Set PB7~PB4 as SPI2 pins. PB4=SPISS20,PB5=SPICLK2,PB6=MOSI20,PB7=MISO20
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

void SPI2_Master_ReInit(uint8_t setting)
{
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB5_SPI2_CLK | SYS_GPB_MFP_PB6_SPI2_MOSI0 | SYS_GPB_MFP_PB7_SPI2_MISO0);
    SYS->ALT_MFP |= (SYS_ALT_MFP_PB4_SPI2_SS0 | SYS_ALT_MFP_PB5_SPI2_CLK | SYS_ALT_MFP_PB6_SPI2_MOSI0 | SYS_ALT_MFP_PB7_SPI2_MISO0);
    CLK->APBCLK |= CLK_APBCLK_SPI2_EN_Msk;
    SYS->IPRSTC2 |= SYS_IPRSTC2_SPI2_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI2_RST_Msk;
    SPI2->CNTRL = (SPI2->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (8 << SPI_CNTRL_TX_BIT_LEN_Pos);
    SPI2->CNTRL &= ~(SPI_CNTRL_TX_NEG_Msk | SPI_CNTRL_RX_NEG_Msk);

    // Bit 0, Bit 1 for SPI_TRANS_TYPE
    switch (setting & 0x03) {
        case 0:
            SPI2->CNTRL |= SPI_MODE_0;
            break;

        case 1:
            SPI2->CNTRL |= SPI_MODE_1;
            break;

        case 2:
            SPI2->CNTRL |= SPI_MODE_3;
            break;

        case 3:
            SPI2->CNTRL |= SPI_MODE_2;
            break;
    }

    CLK->CLKSEL1 |= CLK_CLKSEL1_SPI2_S_HCLK;

    /* Set IP clock divider. SPI clock rate = HCLK / ((divider+1)*2) */
    switch ((setting & 0xF0) >> 4) {
        case 0:
            // DrvSPI_SetClockFreq(SPI2 , 1000000 , 0);
            SPI2->DIVIDER = 35;
            break;

        case 1:
            // DrvSPI_SetClockFreq(SPI2 , 2000000 , 0);
            SPI2->DIVIDER = 17;
            break;

        case 2:
            // DrvSPI_SetClockFreq(SPI2 , 3000000 , 0);
            SPI2->DIVIDER = 11;
            break;

        case 3:
            // DrvSPI_SetClockFreq(SPI2 , 4000000 , 0);
            SPI2->DIVIDER = 8;
            break;

        case 4:
            // DrvSPI_SetClockFreq(SPI2 , 4500000 , 0);
            SPI2->DIVIDER = 7;
            break;

        case 5:
            // DrvSPI_SetClockFreq(SPI2 , 6000000 , 0);
            SPI2->DIVIDER = 5;
            break;

        case 7:
            // DrvSPI_SetClockFreq(SPI2 , 7200000 , 0);
            SPI2->DIVIDER = 4;
            break;

        case 9:
            // DrvSPI_SetClockFreq(SPI2 , 9000000 , 0);
            SPI2->DIVIDER = 3;
            break;

        case 15:
            // DrvSPI_SetClockFreq(SPI2 , 12000000 , 0);
            SPI2->DIVIDER = 2;
            break;
    }

    if (setting & 0x04) {
        SPI_SET_MSB_FIRST(SPI2);
    } else {
        SPI_SET_LSB_FIRST(SPI2);
    }

    if (setting & 0x08) {
        SPI2->SSR |= SPI_SSR_SS_LVL_Msk;
    }

    g_u8ParamSet[MODE_SPI2_MASTER] = setting;
}

/**
  * @brief  Disable SPI2 function and clock source. Set PB.4, PB.5, PB.6, and PB.7 as GPIO function
  * @param  None.
  * @retval None.
  */
void SPI2_Master_DeInit(void)
{
    CLK->APBCLK &= ~CLK_APBCLK_SPI2_EN_Msk;
    NVIC_DisableIRQ(SPI2_IRQn);
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk | SYS_GPB_MFP_PB6_Msk | SYS_GPB_MFP_PB7_Msk);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB4_Msk | SYS_ALT_MFP_PB5_Msk | SYS_ALT_MFP_PB6_Msk | SYS_ALT_MFP_PB7_Msk);
    // Clear SPI2 Master recorded setting
    g_u8ParamSet[MODE_SPI2_MASTER] = 0x00;
}

/**
  * @brief  SPI2 main process function.
  *               transfer g_u8SPI2TxBuf  data to SPI slave device, update related queue pointer and queue size
  *              store received data in g_u8SPI2RxBuf, update related queue pointer and queue size
  * @param  None.
  * @retval None.
  */
void SPI2_Master_MainProcess(void)
{
    uint8_t     cmdCnt = 0;
    uint8_t     repeat = 0;
    uint8_t     bitLength = 0;
    uint8_t     dataCnt = 0;
    uint8_t     dataRxCnt = 0;
    uint16_t    idleTimeL = 0;
    uint16_t    idleTimeH = 0;
    uint16_t    DataIndex = 0;
    uint16_t    DataByte = 0;
    uint32_t    Data = 0;
    cmdCnt = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
    g_u16SPI2TxByte--;

    while (cmdCnt) {
        // Check Valid Byte
        if ('S' == g_u8SPI2TxBuf[g_u16SPI2TxHead++]) {
            repeat = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
            idleTimeL = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
            idleTimeH = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
            idleTimeL += (idleTimeH << 8);
            g_u16SPI2TxByte -= 4;
        } else {
            // Error
            g_u16SPI2TxByte = 0;
            return;
        }

        DataIndex = g_u16SPI2TxHead;
        DataByte =  g_u16SPI2TxByte;

        while (repeat) {
            g_u16SPI2TxHead = DataIndex;
            g_u16SPI2TxByte = DataByte;
            // Start Transation
            SPI2->SSR |= SPI_SS0;
SPI_CHANGE_BIT_LENGTH:
            bitLength = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
            dataCnt = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
            dataRxCnt = dataCnt;
            SPI2->CNTRL &= ~SPI_CNTRL_FIFO_Msk;
            SPI2->CNTRL = (SPI2->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | ((bitLength & 0x1F) << SPI_CNTRL_TX_BIT_LEN_Pos);
            SPI2->FIFO_CTL |= (SPI_FIFO_CTL_TX_CLR_Msk | SPI_FIFO_CTL_RX_CLR_Msk);
            // SPI_SetFIFOMode(SPI2, 2);
            SPI_SET_SUSPEND_CYCLE(SPI2, 2);
            SPI2->CNTRL |= SPI_CNTRL_FIFO_Msk;

            switch ((bitLength + 7) >> 3) {
                case 1:
                    while (dataCnt || dataRxCnt) {
                        while (dataCnt && (0 == (SPI2->CNTRL & SPI_CNTRL_TX_FULL_Msk))) {
                            Data = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
                            g_u16SPI2TxByte--;
                            SPI_WRITE_TX0(SPI2, Data);
                            dataCnt--;

                            if (0 == (SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk)) {
                                break;
                            }
                        }

                        while (dataRxCnt) {
                            while ((SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk));

                            Data = SPI2->RX[0];
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = Data & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u16SPI2RxByte++;
                            dataRxCnt--;
                        }
                    }

                    break;

                case 2:
                    while (dataCnt || dataRxCnt) {
                        while (dataCnt && (0 == (SPI2->CNTRL & SPI_CNTRL_TX_FULL_Msk))) {
                            Data = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
                            Data += (g_u8SPI2TxBuf[g_u16SPI2TxHead++] << 8);
                            g_u16SPI2TxByte -= 2;
                            SPI_WRITE_TX0(SPI2, Data);
                            dataCnt--;

                            if (0 == (SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk)) {
                                break;
                            }
                        }

                        while (dataRxCnt) {
                            while ((SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk));

                            Data = SPI2->RX[0];
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = Data & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = (Data >> 8) & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u16SPI2RxByte += 2;
                            dataRxCnt--;
                        }
                    }

                    break;

                case 3:
                    while (dataCnt || dataRxCnt) {
                        while (dataCnt && (0 == (SPI2->CNTRL & SPI_CNTRL_TX_FULL_Msk))) {
                            Data = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
                            Data += (g_u8SPI2TxBuf[g_u16SPI2TxHead++] << 8);
                            Data += (g_u8SPI2TxBuf[g_u16SPI2TxHead++] << 16);
                            g_u16SPI2TxByte -= 3;
                            SPI_WRITE_TX0(SPI2, Data);
                            dataCnt--;

                            if (0 == (SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk)) {
                                break;
                            }
                        }

                        while (dataRxCnt) {
                            while ((SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk));

                            Data = SPI2->RX[0];
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = Data & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = (Data >> 8) & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = (Data >> 16) & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u16SPI2RxByte += 3;
                            dataRxCnt--;
                        }
                    }

                    break;

                case 4:
                    while (dataCnt || dataRxCnt) {
                        while (dataCnt && (0 == (SPI2->CNTRL & SPI_CNTRL_TX_FULL_Msk))) {
                            Data = g_u8SPI2TxBuf[g_u16SPI2TxHead++];
                            Data += (g_u8SPI2TxBuf[g_u16SPI2TxHead++] << 8);
                            Data += (g_u8SPI2TxBuf[g_u16SPI2TxHead++] << 16);
                            Data += (g_u8SPI2TxBuf[g_u16SPI2TxHead++] << 24);
                            g_u16SPI2TxByte -= 4;
                            SPI_WRITE_TX0(SPI2, Data);
                            dataCnt--;

                            if (0 == (SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk)) {
                                break;
                            }
                        }

                        while (dataRxCnt) {
                            while ((SPI2->CNTRL & SPI_CNTRL_RX_EMPTY_Msk));

                            Data = SPI2->RX[0];
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = Data & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = (Data >> 8) & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = (Data >> 16) & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u8SPI2RxBuf[g_u16SPI2RxTail++] = (Data >> 24) & 0xFF;
                            g_u16SPI2RxTail &= RX_BOUND_PART;
                            g_u16SPI2RxByte += 4;
                            dataRxCnt--;
                        }
                    }

                default:
                    break;
            }

            if ('P' == g_u8SPI2TxBuf[g_u16SPI2TxHead]) {
                SPI2->SSR &= ~SPI_SS0;
                g_u16SPI2TxHead++;
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




