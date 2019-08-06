#include <stdio.h>
#include <string.h>
#include "NUC123.h"
#include "uart_transfer.h"
#include "i2c_transfer.h"
#include "targetdev.h"
#include "nb_hal_sys_init.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    SYS_Init_72MHZ();
    SYS_Init_UART1();
    SYS_Init_I2C0();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32BootAP;
    uint32_t cmd_buff[16];
    u32BootAP = (FMC->ISPCON & FMC_ISPCON_BS_Msk) ? 0 : 1;
    PB13 = u32BootAP;
    PB14 = !u32BootAP;
    /* Init system and multi-funcition I/O */
    SYS_Init();
    UART_Init(UART1);
    I2C_Init(I2C0, 100000);
    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_LDUEN_Msk | FMC_ISPCON_APUEN_Msk);

    while (DetectPin == 0) {
        if (bUartDataReady) {
            ParseCmd((uint8_t *)uart_rcvbuf, 64);
            PutString((UART_T *)bUartDataReady);
            bUartDataReady = 0;
        }

        if (bI2cDataReady == 1) {
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

    outpw(&SYS->RSTSRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}

