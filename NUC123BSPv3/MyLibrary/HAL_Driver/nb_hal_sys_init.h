#ifndef __HAL_SYS_INIT_H__
#define __HAL_SYS_INIT_H__

#include "NUC123.h"

__STATIC_INLINE void SYS_Init_72MHZ(void)
{
    /* Enable XT1_OUT (PF.0) and XT1_IN (PF.1) */
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184 MHz clock */
    /* Enable external XTAL 12 MHz clock */
    CLK->PWRCON |= (CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    while (!(CLK->CLKSTATUS & CLK_PWRCON_XTL12M_EN_Msk));

    /* Set core clock as CLK_PLLCON_144MHz_HXT from PLL */
    CLK->PLLCON = CLK_PLLCON_144MHz_HXT;

    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(2);
    CLK->CLKSEL0 &= (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 144000000;
    SystemCoreClock = 72000000;
    CyclesPerUs     = 72;
    /* Enable module clock */
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;   // (1ul << 2)
}

__STATIC_INLINE void SYS_Init_USBD(void)
{
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_USB_N_Msk)) | CLK_CLKDIV_USB(3);
    CLK->APBCLK |= CLK_APBCLK_USBD_EN_Msk;
}

// PIN 5, 4 = I2C0_SCL, I2C0_SDA
__STATIC_INLINE void SYS_Init_I2C0(void)
{
    /* Enable I2C controller */
    CLK->APBCLK |= CLK_APBCLK_I2C0_EN_Msk;
    /* Set GPF multi-function pins for I2C0 SDA and SCL */
    SYS->GPF_MFP &= ~(SYS_GPF_MFP_PF2_Msk | SYS_GPF_MFP_PF3_Msk);
    SYS->GPF_MFP |= (SYS_GPF_MFP_PF2_I2C0_SDA | SYS_GPF_MFP_PF3_I2C0_SCL);
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PF2_Msk | SYS_ALT_MFP1_PF3_Msk);
    SYS->ALT_MFP1 |= (SYS_ALT_MFP1_PF2_I2C0_SDA | SYS_ALT_MFP1_PF3_I2C0_SCL);
}

// PIN 1, 2 = I2C1_SCL, I2C1_SDA
__STATIC_INLINE void SYS_Init_I2C1(void)
{
    /* Enable I2C controller */
    CLK->APBCLK |= CLK_APBCLK_I2C1_EN_Msk;
    /* Set GPA multi-function pins for I2C1 SDA and SCL */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA10_Msk | SYS_GPA_MFP_PA11_Msk);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA10_Msk | SYS_ALT_MFP_PA11_Msk);
    SYS->GPA_MFP |= (SYS_GPA_MFP_PA10_I2C1_SDA | SYS_GPA_MFP_PA11_I2C1_SCL);
    SYS->ALT_MFP |= (SYS_ALT_MFP_PA10_I2C1_SDA | SYS_ALT_MFP_PA11_I2C1_SCL);
}

// PIN 7, 8 = UART0_RXD, UART0_TXD
__STATIC_INLINE void SYS_Init_UART0(void)
{
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC4_UART0_RXD | SYS_GPC_MFP_PC5_UART0_TXD);
    SYS->ALT_MFP |= (SYS_ALT_MFP_PC4_UART0_RXD | SYS_ALT_MFP_PC5_UART0_TXD);
}

// PIN 1, 2 = UART1_RXD, UART1_TXD
__STATIC_INLINE void SYS_Init_UART1(void)
{
    CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);
}

#endif  /* __HAL_SYS_INIT_H__ */
