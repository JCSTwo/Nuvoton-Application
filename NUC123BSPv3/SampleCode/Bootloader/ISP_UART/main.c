#include <stdio.h>
#include <string.h>
#include "NUC123.h"
#include "uart_transfer.h"
#include "targetdev.h"
#include "nb_hal_sys_init.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    SYS_Init_72MHZ();
    SYS_Init_UART0();
    SYS_Init_UART1();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32BootAP;
    u32BootAP = (FMC->ISPCON & FMC_ISPCON_BS_Msk) ? 0 : 1;
    PB13 = u32BootAP;
    PB14 = !u32BootAP;

    // DetectPin is conflict with UART0_TXD pin, so check DetectPin before UART configuration
    if (DetectPin != 0) {
        outpw(&SYS->RSTSRC, 3);//clear bit
        outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

        /* Trap the CPU */
        while (1);
    }

    /* Init system and multi-funcition I/O */
    SYS_Init();
    UART_Init(UART0);
    UART_Init(UART1);
    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_LDUEN_Msk | FMC_ISPCON_APUEN_Msk);

    while (1) {
        if (bUartDataReady) {
            ParseCmd((uint8_t *)uart_rcvbuf, 64);
            PutString((UART_T *)bUartDataReady);
            bUartDataReady = 0;
        }
    }
}

