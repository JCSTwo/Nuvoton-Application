#include "NuMicro.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif


/**
  * @brief  This function make SPI module be ready to transfer.
  * @param[in]  u32SPIMode Decides the transfer timing. (SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3)
  * @param[in]  u32DataWidth Decides the data width of a SPI transaction.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI peripheral clock.
  * @details By default, the automatic slave selection function is disabled.
  *          The actual clock rate may be different from the target SPI clock rate.
  *          For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the
  *          actual SPI clock rate will be 6MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
  */
uint32_t SPI1_MasterOpen(uint32_t u32SPIMode,
                         uint32_t u32DataWidth,
                         uint32_t u32BusClock,
                         uint32_t u32ActiveLevel,
                         uint32_t u32SuspendInterval,
                         uint32_t u32SendLSBFirst)
{
    uint32_t u32ClkSrc = 0U, u32Div, u32RetValue = 0U;
    // void SPI_Close(SPI_T *spi)
    {
        /* Reset SPI */
        SYS->IPRST1 |= SYS_IPRST1_SPI1RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI1RST_Msk;
    }
    /* Disable I2S mode */
    // SPI1->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;

    if (u32DataWidth >= 32U) {
        u32DataWidth = 0U;
    }

    /* Default setting: disable automatic slave selection function. */
    SPI1->SSCTL = u32ActiveLevel;
    /* Default setting: disable unit transfer interrupt */
    SPI1->CTL = SPI_MASTER | SPI_CTL_SPIEN_Msk
                | (u32DataWidth << SPI_CTL_DWIDTH_Pos)
                | (u32SPIMode)
                | ((u32SuspendInterval & 0xF) << SPI_CTL_SUSPITV_Pos)
                | ((u32SendLSBFirst & 0x1) << SPI_CTL_LSB_Pos);
    /* Check clock source of SPI */
    u32ClkSrc = (FREQ_192MHZ / 2);

    if (u32BusClock >= u32ClkSrc) {
        /* Set DIVIDER = 0 */
        SPI1->CLKDIV = 0U;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    } else if (u32BusClock == 0U) {
        /* Set DIVIDER to the maximum value 0xFF. f_spi = f_spi_clk_src / (DIVIDER + 1) */
        SPI1->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / (0xFFU + 1U));
    } else {
        u32Div = (((u32ClkSrc * 10U) / u32BusClock + 5U) / 10U) - 1U; /* Round to the nearest integer */

        if (u32Div > 0xFFU) {
            u32Div = 0xFFU;
            SPI1->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (0xFFU + 1U));
        } else {
            SPI1->CLKDIV = (SPI1->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);
            /* Return master peripheral clock rate */
            u32RetValue = (u32ClkSrc / (u32Div + 1U));
        }
    }

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    // SPI_SetFIFO(SPI1, 4, 4);
    // SPI_EnableInt(SPI1, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    {
        SPI1->FIFOCTL = (SPI1->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                        (4 << SPI_FIFOCTL_TXTH_Pos) |
                        (4 << SPI_FIFOCTL_RXTH_Pos);
        SPI1->FIFOCTL |= (SPI_FIFOCTL_TXTHIEN_Msk | SPI_FIFOCTL_RXTOIEN_Msk);
    }
    return u32RetValue;
}

void SPI1_Init(uint32_t Pclk0)
{
    SPI1_MasterOpen(SPI_MODE_0, 32, 1000000, SPI_SS_ACTIVE_LOW, 0x08, 0);
}

uint32_t SPI1_Write(uint32_t *buf, uint32_t len)
{
    uint32_t u32Dummy, u32RxDataCount, u32TxDataCount;
    *buf = (*buf | 0x53504900);
    SPI1->SSCTL |= SPI_SSCTL_SS_Msk;
    u32TxDataCount = 0;
    u32RxDataCount = 0;

    /* Wait for transfer done */
    while ((u32RxDataCount < len) || (u32TxDataCount < len)) {
        /* Check TX FULL flag and TX data count */
        if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (u32TxDataCount < len)) {
            /* Write to TX FIFO */
            SPI_WRITE_TX(SPI1, buf[u32TxDataCount++]);
        }

        /* Check RX EMPTY flag */
        if ((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0) && (u32RxDataCount < len)) {
            /* Read RX FIFO */
            u32Dummy += SPI_READ_RX(SPI1); // use "+=" instead of "+" to prevent warning about dummy read
            u32RxDataCount++;
        }
    }

    SPI1->SSCTL &= ~(SPI_SSCTL_SS_Msk);
    return 0;
}

uint32_t SPI1_Read(uint32_t *buf, uint32_t len)
{
    uint32_t u32RxDataCount, u32TxDataCount, u32Dummy;
    SPI1->SSCTL |= SPI_SSCTL_SS_Msk;
    u32TxDataCount = 0;
    u32RxDataCount = 0;
    u32Dummy = 0;

    /* Wait for transfer done */
    while ((u32RxDataCount < (len - 1)) || (u32TxDataCount < len)) {
        /* Check TX FULL flag and TX data count */
        if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (u32TxDataCount < len)) {
            /* Write to TX FIFO */
            SPI_WRITE_TX(SPI1, u32TxDataCount++); // Don't care.
        }

        /* Check RX EMPTY flag */
        if ((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0) && (u32RxDataCount < (len - 1))) {
            if (!u32Dummy) {
                u32Dummy = SPI_READ_RX(SPI1);
                u32Dummy = 1;
            } else {
                /* Read RX FIFO */
                buf[u32RxDataCount++] = SPI_READ_RX(SPI1);
            }
        }
    }

    SPI1->SSCTL &= ~(SPI_SSCTL_SS_Msk);
    return 0;
}

#ifdef __cplusplus
}
#endif
