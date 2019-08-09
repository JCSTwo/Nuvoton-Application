#include <stdio.h>
#include "targetdev.h"
#include "usbd_transfer.h"
#include "uart_transfer.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init_72MHZ_USBD();
    SYS_Init_UART1();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32UsbdReady = 0;
    /* Init system and multi-funcition I/O */
    SYS_Init();
    UART_Init(UART1);
    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_LDUEN_Msk | FMC_ISPCON_APUEN_Msk);

    if (DetectPin == 0) {
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);
        /* Endpoint configuration */
        HID_Init();
        /* Start USB device */
        USBD_Start();
        NVIC_EnableIRQ(USBD_IRQn);
        u32UsbdReady = 1;
    } else {
        SysTick->LOAD = 300000 * CyclesPerUs;
        SysTick->VAL   = (0x00);
        SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

        while (1) {
            if ((bufhead >= 4) || bUartDataReady) {
                uint32_t lcmd;
                lcmd = inpw(uart_rcvbuf);

                if (lcmd == CMD_CONNECT) {
                    break;
                } else {
                    bUartDataReady = 0;
                    bufhead = 0;
                }
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                goto _APROM;
            }
        }
    }

    LED_ISP_Ready();

    while (1) {
        if ((DetectPin == 0) && (bUsbDataReady == TRUE)) {
            LED_Red_Only();
            ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
            EP2_Handler();
            bUsbDataReady = FALSE;
        }

        if (bUartDataReady) {
            LED_Green_Only();
            ParseCmd((uint8_t *)uart_rcvbuf, 64);
            PutString((UART_T *)bUartDataReady);
            bUartDataReady = 0;
        }

        if (u32UsbdReady && (DetectPin != 0)) {
            break;
        }
    }

_APROM:
    outpw(&SYS->RSTSRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
