#include <stdio.h>
#include "targetdev.h"
#include "usbd_transfer.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init_72MHZ_USBD();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;   // (1ul << 2)
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init system and multi-funcition I/O */
    SYS_Init();
    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_LDUEN_Msk | FMC_ISPCON_APUEN_Msk);

    if (DetectPin) {
        goto _APROM;
    }

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);
    /* Endpoint configuration */
    HID_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
    LED_ISP_Ready();

    while (DetectPin == 0) {
        if (bHidDataReady == TRUE) {
            LED_Red_Only();
            ParseCmd((uint8_t *)usb_rcvbuf, 64);
            EP7_Handler();
            bHidDataReady = FALSE;
        }

        if (bVcomDataReady == TRUE) {
            LED_Green_Only();
            ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
            EP2_Handler();
            bVcomDataReady = FALSE;
        }
    }

_APROM:
    outpw(&SYS->RSTSRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}

