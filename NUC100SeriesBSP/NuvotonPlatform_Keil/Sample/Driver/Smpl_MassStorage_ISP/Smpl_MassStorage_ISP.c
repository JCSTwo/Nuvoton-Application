/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "MassStorage_ISP.h"
volatile uint32_t GPIOA_TEMP, GPIOB_TEMP, GPIOC_TEMP, GPIOD_TEMP;
volatile uint32_t GPIOA_TEMP1, GPIOB_TEMP1, GPIOC_TEMP1, GPIOD_TEMP1;
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32INTSTS;
    GPIOA_TEMP = inp32(GPIOA_BASE + 0x10); //0xfc00
    GPIOB_TEMP = inp32(GPIOB_BASE + 0x10); //0xf7ff
    GPIOC_TEMP = inp32(GPIOC_BASE + 0x10); //0x3f3f
    GPIOD_TEMP = inp32(GPIOD_BASE + 0x10); //0xf3f
    UNLOCKREG();
    //test mode
    outp32(0X50004040, 0xfffff7ff); //PB5 OUT MODE
    outp32(0X50004048, 0x00000ffdf); //PB5 IS LOW

    if ((inp32(GPIOC_BASE + 0x10) & BIT4) == 0) {
        outp32(0X50004048, 0x00000ffff); //pb5 is high

        if ((inp32(GPIOC_BASE + 0x10) & BIT4) != 0) {
            goto MSDISP;
        }
    }

JMP_AP:
    outp32(0X50004040, 0xffffffff); //GPIO for QUASI MODE
    outp32(0X50004000, 0xffffffff); //
    outp32(0X50004080, 0xffffffff); //
    outp32(0X500040c0, 0xffffffff); //
    FMC->ISPCON.BS = 0;
    outp32(&SYS->IPRSTC1, 0x02);

    while (1);

MSDISP:
    outpw(0x50000044, 0x0000000f); //for nuc1230 external rc pin enable
    /* Enable 12M Crystal */
    SYSCLK->PWRCON.XTL12M_EN = 1;
    RoughDelay(0x2000);
    /* Enable PLL */
    outp32(&SYSCLK->PLLCON, 0xC22E);
    RoughDelay(0x2000);
    /* Switch HCLK source to PLL */
    SYSCLK->CLKSEL0.HCLK_S = 2;
    /* Initialize USB Device function */
    /* Enable PHY to send bus reset event */
    _DRVUSB_ENABLE_USB();
    outp32(&USBD->DRVSE0, 0x01);
    RoughDelay(1000);
    outp32(&USBD->DRVSE0, 0x00);
    RoughDelay(1000);
    /* Disable PHY */
    _DRVUSB_DISABLE_USB();
    /* Enable USB device clock */
    outp32(&SYSCLK->APBCLK, BIT27);
    /* Reset IP */
    outp32(&SYS->IPRSTC2, BIT27);
    outp32(&SYS->IPRSTC2, 0x0);
    _DRVUSB_ENABLE_USB();
    outp32(&USBD->DRVSE0, 0x01);
    RoughDelay(1000);
    outp32(&USBD->DRVSE0, 0x00);
    g_u8UsbState = USB_STATE_DETACHED;
    _DRVUSB_TRIG_EP(1, 0x08);
    UsbFdt();
    /* Initialize mass storage device */
    udcFlashInit();

    /* Start USB Mass Storage */

    /* Handler the USB ISR by polling */
    while (1) {
        u32INTSTS = _DRVUSB_GET_EVENT_FLAG();

        if (u32INTSTS & INTSTS_FLDET) {
            /* Handle the USB attached/detached event */
            UsbFdt();
        } else if (u32INTSTS & INTSTS_BUS) {
            /* Handle the USB bus event: Reset, Suspend, and Resume */
            UsbBus();
        } else if (u32INTSTS & INTSTS_USB) {
            /* Handle the USB Protocol/Clase event */
            UsbUsb(u32INTSTS);
        }
    }
}



