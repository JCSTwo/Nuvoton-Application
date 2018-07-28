/******************************************************************************
 * @file     uart_transfer.h
 * @brief    General UART ISP slave header file
 * @version  1.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>
#include "NUC123.h"

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE      64
#define PAGE_SIZE					0x200

/*-------------------------------------------------------------*/
extern uint8_t I2C_rcvbuf[];
extern uint8_t I2C_sendbuf[];
extern uint8_t I2C_aprom_buf[];
extern uint8_t volatile bI2CDataReady;
extern uint8_t volatile bufhead;

/*-------------------------------------------------------------*/
void I2C_Init(void);
void I2C_SlaveRcvSendData(void);

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
