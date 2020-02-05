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


// https://www.cnblogs.com/jiangzhaowei/p/8982172.html
#include <stdarg.h>
#include <string.h>

#define STRBUFSIZE           256
char str[STRBUFSIZE];

void VCOM_printf(const char *pFmt, ...)
{
    uint8_t *pstr;
    uint8_t u8Len, u8Offset;
    va_list args;
    va_start(args, pFmt);
    u8Len = (uint8_t)vsnprintf(str, STRBUFSIZE, pFmt, args);
    va_end(args);
    pstr = (uint8_t *)str;

    while (u8Len > 0) {
        while (gu32TxSize) {};

        u8Offset = u8Len;

        if (u8Offset > EP2_MAX_PKT_SIZE) {
            u8Offset = EP2_MAX_PKT_SIZE;
        }

        gu32TxSize = u8Offset;
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pstr, gu32TxSize);
        USBD_SET_PAYLOAD_LEN(EP2, gu32TxSize);
        u8Len -= u8Offset;
        pstr += u8Offset;
    }
}

void VCOM_Initialize(void)
{
    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);
    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
}

int VCOM_getchar(void)
{
    while (gi8BulkOutReady == 0);

    gi8BulkOutReady = 0;
    /* Ready to get next BULK out */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    return *gpu8RxBuf;
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

