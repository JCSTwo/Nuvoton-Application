#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define TEST_COUNT 16

uint32_t spi_rcvbuf[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

volatile uint8_t bSpiDataReady = 0;

void SPI_Init(SPI_T *spi)
{
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI as a low level active device. */
    /* Disable I2S mode */
    spi->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
    /* Default setting: slave selection signal is low level active. */
    spi->SSCTL = SPI_SS_ACTIVE_LOW;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    spi->CTL = SPI_SLAVE | ((32 & 0x1F) << SPI_CTL_DWIDTH_Pos) | (SPI_MODE_0) | SPI_CTL_SPIEN_Msk;
    /* Set DIVIDER = 0 */
    spi->CLKDIV = 0U;
    /* Set TX FIFO threshold and enable FIFO mode. */
    spi->FIFOCTL = (spi->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                   (4 << SPI_FIFOCTL_TXTH_Pos) |
                   (4 << SPI_FIFOCTL_RXTH_Pos);
    /* Enable slave selection signal active interrupt flag */
    spi->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
    SPI_WRITE_TX(spi, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */

    if (spi == SPI1) {
        NVIC_EnableIRQ(SPI1_IRQn);
    } else if (spi == SPI2) {
        NVIC_EnableIRQ(SPI2_IRQn);
    } else {
        // Not support yet.
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  SPI IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
__STATIC_INLINE void SPI_IRQHandler(SPI_T *spi)
{
    uint32_t *_response_buff;
    _response_buff = (uint32_t *)response_buff; // in isp_user.c

    if (spi->STATUS & SPI_STATUS_SSACTIF_Msk) {
        spi->STATUS |= SPI_STATUS_SSACTIF_Msk;
        spi->FIFOCTL |= (SPI_FIFOCTL_RXFBCLR_Msk | SPI_FIFOCTL_TXFBCLR_Msk);
        g_u32TxDataCount = 0;
        g_u32RxDataCount = 0;

        // Active
        while (!(spi->STATUS & SPI_STATUS_SSINAIF_Msk)) {
            /* Check TX FULL flag and TX data count */
            if ((SPI_GET_TX_FIFO_FULL_FLAG(spi) == 0) && (g_u32TxDataCount < TEST_COUNT)) {
                SPI_WRITE_TX(spi, _response_buff[g_u32TxDataCount]);    /* Write to TX FIFO */
                g_u32TxDataCount++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            /* Check RX EMPTY flag */
            if (SPI_GET_RX_FIFO_EMPTY_FLAG(spi) == 0) {
                g_u32RxDataCount &= 0x0F;
                spi_rcvbuf[g_u32RxDataCount++] = SPI_READ_RX(spi);    /* Read RX FIFO */
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if (spi->STATUS & SPI_STATUS_SSINAIF_Msk) {
            spi->STATUS |= SPI_STATUS_SSINAIF_Msk;

            if ((g_u32RxDataCount == 16) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900)) {
                bSpiDataReady = 1;
            }

            spi_rcvbuf[0] &= 0x000000FF;
            g_u32TxDataCount = 0;
            g_u32RxDataCount = 0;

            if (SPI_GET_TX_FIFO_FULL_FLAG(spi) == 0) {
                SPI_WRITE_TX(spi, 0xFFFFFFFF);    /* Write to TX FIFO */
            }
        }
    } else {
    }
}

void SPI1_IRQHandler(void)
{
    SPI_IRQHandler(SPI1);
}

void SPI2_IRQHandler(void)
{
    SPI_IRQHandler(SPI2);
}
