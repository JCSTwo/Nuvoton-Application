/**************************************************************************//**
 * @file     SPI_Monitor.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 13/11/11 11:27a $
 * @brief    NUC123 Series SPI Slave With Two Bit Mode and FIFO Enable Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/*!<Includes */
#include "NucInternal.h"

//#define           MODE_SPI_MONITOR        0x02

/**
  * @brief  Initial SPI0 to be SPI slave with default setting 0xF4.
  * @param  None.
  * @retval None.
  */
void SPI_Slave_Init(void)
{
    SPI0_Monitor_ReInit(0xF4);
}

/**
  * @brief  SPI0 main interrupt function.
  *             store received data in g_u8RxBuf, update related queue pointer and queue size
  * @param  None.
  * @retval None.
  */
void SPI0_IRQHandler(void)
{
    while (!(SPI0->CNTRL & SPI_CNTRL_RX_EMPTY_Msk)) {
        g_u8RxBuf[g_u16RxTail++] = SPI0->RX[0];
        g_u8RxBuf[g_u16RxTail++] = SPI0->RX[0];
        g_u16RxTail &= RX_BOUND;
        g_u16RxByte += 2;

        if (g_u16RxByte >  RX_BOUND) {
            g_u16RxBufferOverflow = 1;
            NVIC_DisableIRQ(SPI0_IRQn);
            NVIC_DisableIRQ(GPAB_IRQn);
        }
    }

    SPI0->CNTRL |= SPI_CNTRL_IF_Msk;
}

/**
  * @brief  GPAB main interrupt function.
  *             store RxTail in buffer when SS in dis-active
  * @param  None.
  * @retval None.
  */
void GPAB_IRQHandler(void)
{
    PB->ISRC = BIT3;
    g_u16CS_LabelBuf[g_u16CS_LabelTail++] = g_u16RxTail + SPI_GET_RX_FIFO_COUNT(SPI0);
    g_u16CS_LabelTail &= CS_LABEL_BOUND;
    g_u16CS_LabelByte += 1;

    if (g_u16CS_LabelByte >  CS_LABEL_BOUND) {
        g_u16CS_OverFlow = 1;
        NVIC_DisableIRQ(SPI0_IRQn);
        NVIC_DisableIRQ(GPAB_IRQn);
    }
}

/**
  * @brief  Disable SPI0 function and clock source. Set PC.0, PC.1, PC.3, and PC.5 as GPIO function
  * @param  None.
  * @retval None.
  */
void SPI_Slave_DeInit(void)
{
    NVIC_DisableIRQ(GPAB_IRQn);
    NVIC_DisableIRQ(SPI0_IRQn);
    CLK->APBCLK &= ~CLK_APBCLK_SPI0_EN_Msk;
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC3_Msk | SYS_GPC_MFP_PC5_Msk);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC3_Msk | SYS_ALT_MFP_PC5_Msk);
    // Clear SPI0 Monitor recorded setting
    g_u8ParamSet[MODE_SPI_MONITOR] = 0x00;
}

/**
  * @brief  Initial SPI0 to be SPI slave.
  *              Set PC.0 as SPI0 SS.
  *              Set PC.1 as SPI0 CLK.
  *              Set PC.3 as SPI0 MOSI0.
  *              Set PC.5 as SPI0 MOSI1.    // No MISO pin, using SPI two bit mode
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
  *                     -1111: 12 MHz    // Always using 12 MHz
  *                     -others: Reserved
  * @retval None.
  */
void SPI0_Monitor_ReInit(uint8_t setting)
{
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC3_SPI0_MOSI0 | SYS_GPC_MFP_PC5_SPI0_MOSI1);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC3_Msk | SYS_ALT_MFP_PC5_Msk);
    CLK->APBCLK |= CLK_APBCLK_SPI0_EN_Msk;
    SYS->IPRSTC2 |= SYS_IPRSTC2_SPI0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI0_RST_Msk;
    SPI0->CNTRL = (SPI0->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (8 << SPI_CNTRL_TX_BIT_LEN_Pos) | SPI_CNTRL_SLAVE_Msk;
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

    SPI0->DIVIDER = (SPI0->DIVIDER & ~SPI_DIVIDER_DIVIDER_Msk) | 2;
    SPI0->CNTRL |= SPI_CNTRL_TWOB_Msk;

    if (setting & 0x04) {
        SPI_SET_MSB_FIRST(SPI0);
    } else {
        SPI_SET_LSB_FIRST(SPI0);
    }

    SPI0->SSR |= SPI_SS0;

    if (setting & 0x08) {
        SPI0->SSR |= (SPI_SSR_SS_LVL_Msk | SPI_SSR_SS_LTRIG_Msk);
    } else {
        SPI0->SSR |= SPI_SSR_SS_LTRIG_Msk;
    }

    // SPI_SetFIFOMode(SPI0, 2);
    SPI_SET_SUSPEND_CYCLE(SPI0, 2);
    SPI0->CNTRL |= SPI_CNTRL_FIFO_Msk;
    SPI0->FIFO_CTL = (SPI0->FIFO_CTL & ~SPI_FIFO_CTL_RX_THRESHOLD_Msk) | (6 << SPI_FIFO_CTL_RX_THRESHOLD_Pos);
    SPI0->FIFO_CTL |= SPI_FIFO_CTL_RX_CLR_Msk;
    NVIC_EnableIRQ(SPI0_IRQn);
    SPI0->FIFO_CTL |= (SPI_FIFO_CTL_RX_INTEN_Msk | SPI_FIFO_CTL_TIMEOUT_INTEN_Msk);
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
    g_u8ParamSet[MODE_SPI_MONITOR] = setting;
    PB->PMD = (PB->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD3_Pos);

    if (setting & 0x08) {
        GPIO_EnableInt(PB, 3, GPIO_INT_FALLING);
    } else {
        GPIO_EnableInt(PB, 3, GPIO_INT_RISING);
    }

    NVIC_EnableIRQ(GPAB_IRQn);                          //En:Enable NVIC GPAB_IRQn interrupt
}
