/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "MassStorage_ISP.h"

#define DetectPin       PA13

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32INTSTS;
    UNLOCKREG();

    if (DetectPin) {
        goto JMP_AP;
    }

//MSDISP:
    FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
    outpw(0x50000044, 0x0000000f); //for nuc1230 external rc pin enable
    /* Enable 12M Crystal */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;
    RoughDelay(0x2000);
    /* Enable PLL */
    outp32(&CLK->PLLCON, 0xC22E);
    RoughDelay(0x2000);
    /* Switch HCLK source to PLL */
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;
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
    CLK->APBCLK |= CLK_APBCLK_USBD_EN_Msk;
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
    while (DetectPin == 0) {
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

JMP_AP:
    outp32(0X50004040, 0xffffffff); //GPIO for QUASI MODE
    outp32(0X50004000, 0xffffffff); //
    outp32(0X50004080, 0xffffffff); //
    outp32(0X500040c0, 0xffffffff); //
    outpw(&SYS->RSTSRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, 0x05FA0004);

    /* Trap the CPU */
    while (1);
}



