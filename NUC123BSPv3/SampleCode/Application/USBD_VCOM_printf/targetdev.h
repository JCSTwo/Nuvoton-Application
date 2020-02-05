#ifndef __TARGET_H__
#define __TARGET_H__

#ifdef __cplusplus
extern "C"
{
#endif

// Nuvoton MCU Peripheral Access Layer Header File
#include "..\..\..\MyLibrary\HAL_Driver\hal_sys_init.h"

#include <stdarg.h>
// VCOM_printf.c
void VCOM_printf(const char *pFmt, ...);
void VCOM_Initialize(void);
int VCOM_getchar(void);

#include "cdc_serial.h"

#ifdef __cplusplus
}
#endif

#endif //__TARGET_H__
