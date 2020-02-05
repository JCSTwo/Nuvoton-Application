/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 8 $
 * $Date: 15/07/02 11:17a $
 * @brief
 *           Demonstrate SPI master loop back transfer.
 *           This sample code needs to connect SPI0_MISO0 pin and SPI0_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "..\targetdev.h"

#define printf VCOM_printf

#define TEST_COUNT             64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err;
    volatile uint8_t i = 0;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    VCOM_Initialize();
    /* Init SPI */
    SPI_Init();
    VCOM_getchar();
    printf("\r\n\n");
    printf("\r+--------------------------------------------------------------------+\n");
    printf("\r|                   NUC123 SPI Driver Sample Code                    |\n");
    printf("\r+--------------------------------------------------------------------+\n");
    printf("\r\n");
    printf("\r\nThis sample code demonstrates SPI0 self loop back data transfer.\n");
    printf("\r SPI0 configuration:\n");
    printf("\r     Master mode; data width 32 bits.\n");
    printf("\r I/O connection:\n");
    printf("\r     Pin7 SPI0_MOSI0 <--> Pin8 SPI0_MISO0 \n");
    printf("\r GND (Pin3), DVDD (Pin6)\n");
_main_loop:
    printf("\r\n[%03d] SPI0 Loopback test ", i++);
    u32Err = 0;

    for (u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++) {
        /* set the source data and clear the destination buffer */
        for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
            g_au32SourceData[u32DataCount] = u32DataCount;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if ((u32TestCount & 0x1FF) == 0) {
            printf(".");
        }

        while (1) {
            /* Write to TX register */
            SPI_WRITE_TX0(SPI0, g_au32SourceData[u32DataCount]);
            /* Trigger SPI data transfer */
            SPI_TRIGGER(SPI0);

            /* Check SPI0 busy status */
            while (SPI_IS_BUSY(SPI0));

            /* Read received data */
            g_au32DestinationData[u32DataCount] = SPI_READ_RX0(SPI0);
            u32DataCount++;

            if (u32DataCount > TEST_COUNT) {
                break;
            }
        }

        /*  Check the received data */
        for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++) {
            if (g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount]) {
                u32Err = 1;
            }
        }

        if (u32Err) {
            break;
        }
    }

    if (u32Err) {
        printf(" [FAIL]\r\n\n");
    } else {
        printf(" [PASS]\r\n\n");
    }

    VCOM_getchar();
    goto _main_loop;
    /* Disable SPI0 peripheral clock */
    CLK->APBCLK &= (~CLK_APBCLK_SPI0_EN_Msk);

    while (1);
}

void SYS_Init(void)
{
    SYS_Init_72MHZ_USBD();
    SYS_Init_SPI0();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Enable FIFO mode. */
    SPI0->CNTRL = SPI_CNTRL_FIFO_Msk | SPI_MASTER | SPI_CNTRL_TX_NEG_Msk;
    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI0->SSR = SPI_SSR_AUTOSS_Msk | SPI_SS0;
    /* Set IP clock divider. SPI clock rate = HCLK / ((5+1)*2) = 1 MHz */
    SPI0->DIVIDER = (SPI0->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | 5;
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

