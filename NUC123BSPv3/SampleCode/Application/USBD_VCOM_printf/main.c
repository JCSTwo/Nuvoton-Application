/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB virtual com port device.
 *           It supports one virtual comport.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "cdc_serial.h"

#define printf VCOM_printf

void SYS_Init(void)
{
    SYS_Init_72MHZ_USBD();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint8_t i = 0;
    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init();
    VCOM_Initialize();

    while (1) {
        VCOM_getchar();
        printf("[%03d] Hello World ~ \r\n", i++);
    }
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

