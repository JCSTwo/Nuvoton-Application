/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __TARGET_H__
#define __TARGET_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "..\hal_sys_init.h"
#include "isp_user.h"

#define I2C_ADDR     0x60
//#define DetectPin    PB12
#define DetectPin    0

#define USE_SPI1           (1) // 0: SPI2, 1: SPI1
#define EN_ISP_TIMEOUT     (0) // 0: Disalbe, 1: Enable 300ms timeout when startup


#ifdef __cplusplus
}
#endif

#endif //__TARGET_H__
