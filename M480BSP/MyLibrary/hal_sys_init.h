#ifndef __HAL_SYS_INIT_H__
#define __HAL_SYS_INIT_H__


// Nu-Link2 (M48SKIDAE)
// LDROM: 4K, APROM:512K, SPROM:4K, SRAM:160K

// Nuvoton MCU Peripheral Access Layer Header File
#include "M480.h"

__STATIC_INLINE void SYS_Init_192MHZ(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    // Waiting for clock switching ok
    while (CLK->STATUS & CLK_STATUS_CLKSFAIL_Msk);

    CLK->PLLCTL = CLK_PLLCTL_PD_Msk; // Disable PLL
    CLK->PLLCTL = 0x8842E;           // Enable PLL & set frequency 192MHz

    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    while ((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) != CLK_STATUS_PLLSTB_Msk);

    CLK->CLKDIV0 = CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk);   /* PLL/1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;
    PllClock        = FREQ_192MHZ;
    SystemCoreClock = FREQ_192MHZ;
    CyclesPerUs     = (SystemCoreClock + 500000UL) / 1000000UL; // For SYS_SysTickDelay()
}


#define nRTSPin						(PE12)
#define RECEIVE_MODE			(0)
#define REVEIVE_MODE			(0)
#define TRANSMIT_MODE			(1)

// Making Names Less Ambiguous.
#define Transceiver_Direction_485    (PE12)
#define DIR_RECEIVE                  (0)
#define DIR_TRANSMIT                 (1)

#define GPIO_SETMODE(port, pin, u32Mode) port->MODE = (port->MODE & ~(0x3ul << (pin << 1))) | (u32Mode << (pin << 1)); // Pin Indexs
#define GPIO_SETPUSEL(port, pin, u32Mode) port->PUSEL = (port->PUSEL & ~(0x3ul << (pin << 1))) | (u32Mode << (pin << 1)); // Pin Indexs


// This is the default DEBUG PORT setting for M487 EVB (the same as BSP Sample)
__STATIC_INLINE void SYS_Init_UART0(void)
{
    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
}

// PIN 1, 2 = UART2_TXD, UART2_RXD
__STATIC_INLINE void SYS_Init_UART2(void)
{
    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART2CKEN_Msk;
    /* Select UART module clock source */
    CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART2SEL_Msk)) | CLK_CLKSEL3_UART2SEL_HIRC;
    /* Set GPE multi-function pins for UART2 RXD and TXD */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE8MFP_Msk | SYS_GPE_MFPH_PE9MFP_Msk))
                    | (SYS_GPE_MFPH_PE8MFP_UART2_TXD | SYS_GPE_MFPH_PE9MFP_UART2_RXD);
    // Pin 2, Enable pull up of UART2_RXD pin to solve floating issue
    GPIO_SETPUSEL(PE, 9, GPIO_PUSEL_PULL_UP);
}

// PIN 3, 4 = USCI0_CLK, USCI0_DAT0
__STATIC_INLINE void SYS_Init_UI2C0(void)
{
    /* Enable IP clock */
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;
    /* Set UI2C0 multi-function pins */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE2MFP_Msk | SYS_GPE_MFPL_PE3MFP_Msk)) |
                    (SYS_GPE_MFPL_PE2MFP_USCI0_CLK | SYS_GPE_MFPL_PE3MFP_USCI0_DAT0);
    /* USCI_I2C clock pin enable schmitt trigger */
    PE->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
}

// PIN 5, 6, 7, 8 = SPI1_SS, SPI1_CLK, SPI1_MOSI, SPI1_MISO
__STATIC_INLINE void SYS_Init_SPI1(void)
{
    /* Select PCLK0 as the clock source of SPI1 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
    /* Enable SPI1 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;
    /* Setup SPI1 multi-function pins */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE0MFP_Msk | SYS_GPE_MFPL_PE1MFP_Msk))
                    | (SYS_GPE_MFPL_PE0MFP_SPI1_MOSI | SYS_GPE_MFPL_PE1MFP_SPI1_MISO);
    SYS->GPH_MFPH = (SYS->GPH_MFPH & ~(SYS_GPH_MFPH_PH8MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk))
                    | (SYS_GPH_MFPH_PH8MFP_SPI1_CLK | SYS_GPH_MFPH_PH9MFP_SPI1_SS);
    /* Enable SPI1 clock pin (PH8) schmitt trigger */
    PH->SMTEN |= GPIO_SMTEN_SMTEN8_Msk;
    // SPI2 Pins are connected with SPI1
    GPIO_SETMODE(PG, 2, GPIO_MODE_INPUT);
    GPIO_SETMODE(PG, 3, GPIO_MODE_INPUT);
    GPIO_SETMODE(PG, 4, GPIO_MODE_INPUT);
    GPIO_SETMODE(PF, 11, GPIO_MODE_INPUT);
}

// For ISP Target Only, PIN 1, 2, 15 (-5), 17 (-3) = SPI2_CLK, SPI2_MISO, SPI2_SS, SPI2_MOSI
__STATIC_INLINE void SYS_Init_SPI2(void)
{
    /* Select PCLK1 as the clock source of SPI2 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK1;
    /* Enable SPI2 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI2CKEN_Msk;
    /* Setup SPI2 multi-function pins */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE8MFP_Msk | SYS_GPE_MFPH_PE9MFP_Msk | SYS_GPE_MFPH_PE10MFP_Msk | SYS_GPE_MFPH_PE11MFP_Msk))
                    | (SYS_GPE_MFPH_PE8MFP_SPI2_CLK | SYS_GPE_MFPH_PE9MFP_SPI2_MISO | SYS_GPE_MFPH_PE10MFP_SPI2_MOSI | SYS_GPE_MFPH_PE11MFP_SPI2_SS);
    /* Enable SPI2 clock pin (PE8) schmitt trigger */
    PE->SMTEN |= GPIO_SMTEN_SMTEN8_Msk;
}

// SPI Monitor = SPI1 Slave + SPI2 Slave
__STATIC_INLINE void SYS_Init_SPI12(void)
{
    // Pin 5, Enable pull up of SS pin to solve floating issue
    GPIO_SETPUSEL(PG, 2, GPIO_PUSEL_PULL_UP);
    GPIO_SETPUSEL(PH, 9, GPIO_PUSEL_PULL_UP);
    // Pin 6
    GPIO_SETPUSEL(PG, 3, GPIO_PUSEL_PULL_UP);
    GPIO_SETPUSEL(PH, 8, GPIO_PUSEL_PULL_UP);
    // Pin 7
    GPIO_SETPUSEL(PE, 0, GPIO_PUSEL_PULL_UP);
    // Pin 8
    GPIO_SETPUSEL(PE, 1, GPIO_PUSEL_PULL_UP);
    GPIO_SETPUSEL(PF, 11, GPIO_PUSEL_PULL_UP);
    /* Select PCLK0 as the clock source of SPI1 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
    /* Enable SPI1 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;
    /* Setup SPI1 multi-function pins */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE0MFP_Msk))
                    | SYS_GPE_MFPL_PE0MFP_SPI1_MOSI;
    SYS->GPH_MFPH = (SYS->GPH_MFPH & ~(SYS_GPH_MFPH_PH8MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk))
                    | (SYS_GPH_MFPH_PH8MFP_SPI1_CLK | SYS_GPH_MFPH_PH9MFP_SPI1_SS);
    /* Enable SPI1 clock pin (PH8) schmitt trigger */
    PH->SMTEN |= GPIO_SMTEN_SMTEN8_Msk;
    /* Select PCLK1 as the clock source of SPI2 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK1;
    /* Enable SPI2 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI2CKEN_Msk;
    /* Setup SPI2 multi-function pins */
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF11MFP_Msk))
                    | SYS_GPF_MFPH_PF11MFP_SPI2_MOSI;
    SYS->GPG_MFPL = (SYS->GPG_MFPL & ~(SYS_GPG_MFPL_PG2MFP_Msk | SYS_GPG_MFPL_PG3MFP_Msk))
                    | (SYS_GPG_MFPL_PG2MFP_SPI2_SS | SYS_GPG_MFPL_PG3MFP_SPI2_CLK);
    /* Enable SPI2 clock pin (PG3) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN3_Msk;
    GPIO_SETMODE(PE, 1, GPIO_MODE_INPUT);
    GPIO_SETMODE(PG, 4, GPIO_MODE_INPUT);
}

// RS485 = UART1 (RS232) + PE12
__STATIC_INLINE void SYS_Init_RS485(void)
{
    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;
    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART1SEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART1SEL_HIRC;
    // SYS_GPC_MFPH_PC8MFP_UART1_RXD
    // SYS_GPE_MFPH_PE12MFP_UART1_nRTS
    // SYS_GPE_MFPH_PE13MFP_UART1_TXD
    PE->MODE = (PE->MODE & ~(0x3ul << (12 << 1))) | (GPIO_MODE_OUTPUT << (12 << 1));
    Transceiver_Direction_485 = DIR_RECEIVE;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC8MFP_Msk)) | SYS_GPC_MFPH_PC8MFP_UART1_RXD;
    SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE13MFP_Msk)) | SYS_GPE_MFPH_PE13MFP_UART1_TXD;
}

__STATIC_INLINE void SYS_Init_CAN1(void)
{
    /* Enable CAN1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN1CKEN_Msk;
    /* Set PC multi-function pins for CAN1 RXD(PC.9) and TXD(PC.10) */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC9MFP_Msk | SYS_GPC_MFPH_PC10MFP_Msk)) |
                    (SYS_GPC_MFPH_PC9MFP_CAN1_RXD | SYS_GPC_MFPH_PC10MFP_CAN1_TXD);
    /* Set CAN transceiver to high speed mode */
    GPIO_SETMODE(PC, 11, GPIO_MODE_OUTPUT);
    PC11 = 0;
}

// For ISP Target Only, PIN 7, 8 = I2C1_SDA, I2C1_SCL
__STATIC_INLINE void SYS_Init_I2C1(void)
{
    /* Enable I2C1 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C1CKEN_Msk;
    /* Set I2C1 multi-function pins */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE0MFP_Msk | SYS_GPE_MFPL_PE1MFP_Msk)) |
                    (SYS_GPE_MFPL_PE0MFP_I2C1_SDA | SYS_GPE_MFPL_PE1MFP_I2C1_SCL);
    /* I2C clock pin enable schmitt trigger */
    PE->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
}

// I2C is connected to Audio Codec on M487 EVB (so, there are pull resistors on PD0 & PD1)
// For NuMaker-PFM-M487, PIN 94, 95 = I2C2_SCL, I2C2_SDA
__STATIC_INLINE void SYS_Init_I2C2(void)
{
    /* Enable I2C module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C2CKEN_Msk;
    /* Set I2C2 multi-function pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_I2C2_SDA | SYS_GPD_MFPL_PD1MFP_I2C2_SCL);
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
}

__STATIC_INLINE void SYS_Init_HSUSBD(void)
{
    uint32_t volatile i;
    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */
    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;

    for (i = 0; i < 0x1000; i++);  // delay > 10 us

    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;
    /* Enable IP clock */
    CLK->AHBCLK |= CLK_AHBCLK_HSUSBDCKEN_Msk;   /* USBD20 */
}

__STATIC_INLINE void SYS_Init_USBD(void)
{
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_USBDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_USB(4);
    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;
    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    /* Set PA.12 ~ PA.14 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);
}

#define LED0 PF4 // RED
#define LED1 PC6 // YELLOW
#define LED2 PB9 // RED
#define LED3 PB8 // GREEN

__STATIC_INLINE void SYS_Init_LED(uint32_t v0, uint32_t v1, uint32_t v2, uint32_t v3)
{
    GPIO_SETMODE(PB, 8, GPIO_MODE_QUASI);
    GPIO_SETMODE(PB, 9, GPIO_MODE_QUASI);
    GPIO_SETMODE(PC, 6, GPIO_MODE_QUASI);
    GPIO_SETMODE(PF, 4, GPIO_MODE_QUASI);
    LED0 = v0;
    LED1 = v1;
    LED2 = v2;
    LED3 = v3;
}

__STATIC_INLINE void LED_Set(uint32_t v0, uint32_t v1, uint32_t v2, uint32_t v3)
{
    LED0 = v0;
    LED1 = v1;
    LED2 = v2;
    LED3 = v3;
}

#endif  /* __HAL_SYS_INIT_H__ */
