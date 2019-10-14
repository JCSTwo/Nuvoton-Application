#ifndef __HAL_SYS_INIT_H__
#define __HAL_SYS_INIT_H__

#include "NUC123.h"

#define GPIO_SETMODE(port, pin, u32Mode) port->PMD = (port->PMD & ~(0x3ul << (pin << 1))) | (u32Mode << (pin << 1)); // Pin Indexs

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
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 144000000;
    SystemCoreClock = 72000000;
    CyclesPerUs     = 72;
    /* Enable module clock */
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;   // (1ul << 2)
}

__STATIC_INLINE void SYS_Init_72MHZ_USBD(void)
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

    CLK->CLKDIV = (CLK->CLKDIV & ~(CLK_CLKDIV_HCLK_N_Msk | CLK_CLKDIV_USB_N_Msk))
                  | (CLK_CLKDIV_HCLK(2) | CLK_CLKDIV_USB(3));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;
    /* Update System Core Clock */
    PllClock        = 144000000;
    SystemCoreClock = 72000000;
    CyclesPerUs     = 72;
    /* Enable module clock */
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
    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HIRC;
}

// PIN 1, 2, 4, 5 = PB4_SPI2_SS0, PB5_SPI2_CLK, PB6_SPI2_MOSI0, PB7_SPI2_MISO0
__STATIC_INLINE void SYS_Init_SPI2(void)
{
    /* Select HCLK as the clock source of SPI2 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_SPI2_S_Msk)) | CLK_CLKSEL1_SPI2_S_HCLK;
    /* Enable SPI2 peripheral clock */
    CLK->APBCLK |= CLK_APBCLK_SPI2_EN_Msk;
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB5_SPI2_CLK | SYS_GPB_MFP_PB6_SPI2_MOSI0 | SYS_GPB_MFP_PB7_SPI2_MISO0);
    SYS->ALT_MFP |= (SYS_ALT_MFP_PB4_SPI2_SS0 | SYS_ALT_MFP_PB5_SPI2_CLK | SYS_ALT_MFP_PB6_SPI2_MOSI0 | SYS_ALT_MFP_PB7_SPI2_MISO0);
    CLK->APBCLK |= CLK_APBCLK_SPI2_EN_Msk;
    // Pin 1
    GPIO_SETMODE(PA, 11, GPIO_PMD_INPUT);
    GPIO_SETMODE(PA, 15, GPIO_PMD_INPUT);
    // Pin 2
    GPIO_SETMODE(PA, 10, GPIO_PMD_INPUT);
    GPIO_SETMODE(PA, 14, GPIO_PMD_INPUT);
    // Pin 4
    GPIO_SETMODE(PA, 12, GPIO_PMD_INPUT);
    GPIO_SETMODE(PF, 2, GPIO_PMD_INPUT);
    // Pin 5
    GPIO_SETMODE(PB, 15, GPIO_PMD_INPUT);
    GPIO_SETMODE(PF, 3, GPIO_PMD_INPUT);
}

// PIN 7 ~ 10 = PC3_SPI0_MOSI0, PC2_SPI0_MISO0, PC1_SPI0_CLK, PC0_SPI0_SS0
__STATIC_INLINE void SYS_Init_SPI0(void)
{
    /* Select HCLK as the clock source of SPI0 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_SPI0_S_Msk)) | CLK_CLKSEL1_SPI0_S_HCLK;
    /* Enable SPI0 peripheral clock */
    CLK->APBCLK |= CLK_APBCLK_SPI0_EN_Msk;
    /* Setup SPI0 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC2_Msk | SYS_ALT_MFP_PC3_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PC0_SPI0_SS0 | SYS_ALT_MFP_PC1_SPI0_CLK | SYS_ALT_MFP_PC2_SPI0_MISO0 | SYS_ALT_MFP_PC3_SPI0_MOSI0;
    // Pin 7
    GPIO_SETMODE(PD, 0, GPIO_PMD_INPUT);
    GPIO_SETMODE(PC, 4, GPIO_PMD_INPUT);
    GPIO_SETMODE(PD, 4, GPIO_PMD_INPUT);
    // Pin 8
    GPIO_SETMODE(PD, 1, GPIO_PMD_INPUT);
    GPIO_SETMODE(PA, 13, GPIO_PMD_INPUT);
    GPIO_SETMODE(PC, 5, GPIO_PMD_INPUT);
    GPIO_SETMODE(PD, 5, GPIO_PMD_INPUT);
    // Pin 9
    GPIO_SETMODE(PD, 2, GPIO_PMD_INPUT);
    GPIO_SETMODE(PC, 12, GPIO_PMD_INPUT);
    GPIO_SETMODE(PB, 2, GPIO_PMD_INPUT);
    // Pin 10
    GPIO_SETMODE(PD, 3, GPIO_PMD_INPUT);
    GPIO_SETMODE(PC, 13, GPIO_PMD_INPUT);
    GPIO_SETMODE(PB, 3, GPIO_PMD_INPUT);
}

__STATIC_INLINE void LED_Red_Only(void)
{
    PB13 = 0;
    PB14 = 1;
}

__STATIC_INLINE void LED_Green_Only(void)
{
    PB13 = 1;
    PB14 = 0;
}

__STATIC_INLINE void LED_All_On(void)
{
    PB13 = 0;
    PB14 = 0;
}

__STATIC_INLINE void LED_All_Off(void)
{
    PB13 = 1;
    PB14 = 1;
}

__STATIC_INLINE void LED_ISP_Ready(void)
{
    LED_Red_Only();
}


#endif  /* __HAL_SYS_INIT_H__ */
