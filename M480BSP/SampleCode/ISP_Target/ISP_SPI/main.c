#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "spi_transfer.h"

__weak uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

__weak uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = __HXT; // TIMER_GetModuleClock(timer);
    uint32_t u32Cmpr = 0UL, u32Prescale = 0UL;

    /* Fastest possible timer working freq is (u32Clk / 2). While cmpr = 2, prescaler = 0. */
    if (u32Freq > (u32Clk / 2UL)) {
        u32Cmpr = 2UL;
    } else {
        u32Cmpr = u32Clk / u32Freq;
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */

        if (u32Prescale > 0UL) {
            u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
        }
    }

    timer->CTL = u32Mode | u32Prescale;
    timer->CMP = u32Cmpr;
    return (u32Clk / (u32Cmpr * (u32Prescale + 1UL)));
}

void TIMER3_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR3CKEN_Msk;
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR3SEL_Msk)) | CLK_CLKSEL1_TMR3SEL_HXT;
    // Set timer frequency to 3HZ
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 3);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER3);
    // Start Timer 3
    TIMER_Start(TIMER3);
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_SPI1(); // PIN 5, 6, 7, 8 = SPI1_SS, SPI1_CLK, SPI1_MOSI, SPI1_MISO
    SYS_Init_SPI2(); // For ISP Target Only, PIN 1, 2, 15 (-5), 17 (-3) = SPI2_CLK, SPI2_MISO, SPI2_SS, SPI2_MOSI
}

int main(void)
{
    uint32_t cmd_buff[16];
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);
    SPI_Init(SPI1);
    SPI_Init(SPI2);
#if (EN_ISP_TIMEOUT)
    TIMER3_Init();

    while (1) {
        if (bSpiDataReady == 1) {
            goto _ISP;
        }

        if (TIMER3->INTSTS & TIMER_INTSTS_TIF_Msk) {
            goto _APROM;
        }
    }

#endif
_ISP:

    while (1) {
        if (bSpiDataReady == 1) {
            memcpy(cmd_buff, spi_rcvbuf, 64);
            bSpiDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

_APROM:
    RUN_APROM();
}
