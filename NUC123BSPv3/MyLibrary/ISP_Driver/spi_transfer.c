#include <stdio.h>
#include "NUC123.h"
#include "isp_user.h"

void SPI_Init(void)
{
    SPI_T *spi = SPI2;
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. */
    /* Default setting: slave selection signal is low level active. */
    spi->SSR = SPI_SS_ACTIVE_LOW | SPI_SSR_SS_LTRIG_Msk;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    spi->CNTRL = SPI_SLAVE | ((32 & 0x1F) << SPI_CNTRL_TX_BIT_LEN_Pos) | (SPI_MODE_0) | SPI_CNTRL_FIFO_Msk;
    /* Set DIVIDER = 0 */
    spi->DIVIDER = 0UL;
    /* Set TX FIFO threshold and enable FIFO mode. */
    spi->FIFO_CTL = (spi->FIFO_CTL & ~(SPI_FIFO_CTL_TX_THRESHOLD_Msk | SPI_FIFO_CTL_RX_THRESHOLD_Msk)) |
                    (4 << SPI_FIFO_CTL_TX_THRESHOLD_Pos) |
                    (4 << SPI_FIFO_CTL_RX_THRESHOLD_Pos);
}

#define TEST_COUNT 16

uint32_t *g_au32SourceData;
uint32_t g_au32DestinationData[TEST_COUNT];

void SPI_Main_Polling(void)
{
    volatile uint32_t u32TxDataCount, u32RxDataCount;
    SPI_T *spi = SPI2;
    g_au32SourceData = (uint32_t *)response_buff; // in isp_user.c

    while (1) {
        u32TxDataCount = 0;
        u32RxDataCount = 0;
        spi->FIFO_CTL |= (SPI_FIFO_CTL_RX_CLR_Msk | SPI_FIFO_CTL_TX_CLR_Msk);
        SysTick->CTRL = 0UL;

        /* Access TX and RX FIFO */
        while (u32RxDataCount < TEST_COUNT) {
            /* Check TX FULL flag and TX data count */
            if (((spi->STATUS & SPI_STATUS_TX_FULL_Msk) == 0) && (u32TxDataCount < TEST_COUNT)) {
                spi->TX[0] = g_au32SourceData[u32TxDataCount++];    /* Write to TX FIFO */
            }

            /* Check RX EMPTY flag */
            if ((spi->STATUS & SPI_STATUS_RX_EMPTY_Msk) == 0) {
                g_au32DestinationData[u32RxDataCount++] = spi->RX[0];    /* Read RX FIFO */
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                break;
            }
        }

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;

        if ((u32RxDataCount == 16) && ((g_au32DestinationData[0] & 0xFFFFFF00) == 0x53504900)) {
            g_au32DestinationData[0] &= 0x000000FF;
            ParseCmd((unsigned char *)g_au32DestinationData, 64);
        }
    }
}


