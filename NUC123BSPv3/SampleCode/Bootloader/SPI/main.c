#include <stdio.h>
#include <string.h>
#include "NUC123.h"
#include "targetdev.h"
#include "spi_transfer.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    SYS_UnlockReg();
    SYS_Init_72MHZ();
    SYS_Init_SPI2();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init system and multi-funcition I/O */
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_APUEN_Msk);
    LED_Red_Only();
    SPI_Init();
    SPI_Main_Polling();

//    SYS->RSTSRC = (SYS_RSTSRC_RSTS_POR_Msk | SYS_RSTSRC_RSTS_RESET_Msk);
//    FMC->ISPCON &= ~(FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_BS_Msk);
//    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}

