/******************************************************************************
 * @file     NucData.c
 * @brief    Memory Allocation Module
 * @version  1.0.0
 * @date     11, Nov, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NucInternal.h"

// VCOM control Variable
volatile uint8_t    g_u8VComEnable;

volatile uint8_t    g_u8SPI0Ready;
volatile uint8_t    g_u8SPI2Ready;
volatile uint8_t    g_u8I2C0Ready;

uint8_t                 g_u8BulkOutBuf[64];


// Global Memory Allocation
uint8_t                 g_u8RxBuf[RX_SIZE];
volatile uint16_t       g_u16RxTail;
volatile uint16_t       g_u16RxHead;
volatile uint16_t       g_u16RxByte;
volatile uint16_t       g_u16RxBufferOverflow;

uint8_t                 g_u8TxBuf[ADCTX_SIZE];//TX_SIZE
volatile uint16_t       g_u16TxTail;
volatile uint16_t       g_u16TxHead;
volatile uint16_t       g_u16TxByte;

uint8_t                *g_u8PDMARxBuf;//[PART_RX_SIZE];
volatile uint8_t        g_u8PDMAValidByte;


#define PART_RX_SIZE    (0x800)
#define PART_TX_SIZE    (0x400)

// SPI0 Master
uint8_t                *g_u8SPI0RxBuf;//[PART_RX_SIZE];
volatile uint16_t       g_u16SPI0RxTail;
volatile uint16_t       g_u16SPI0RxHead;
volatile uint16_t       g_u16SPI0RxByte;

uint8_t                *g_u8SPI0TxBuf;//[PART_TX_SIZE];
volatile uint16_t       g_u16SPI0TxTail;
volatile uint16_t       g_u16SPI0TxHead;
volatile uint16_t       g_u16SPI0TxByte;

// SPI2 Master
uint8_t                *g_u8SPI2RxBuf;//[PART_RX_SIZE];
volatile uint16_t       g_u16SPI2RxTail;
volatile uint16_t       g_u16SPI2RxHead;
volatile uint16_t       g_u16SPI2RxByte;

uint8_t                *g_u8SPI2TxBuf;//[PART_TX_SIZE];
volatile uint16_t       g_u16SPI2TxTail;
volatile uint16_t       g_u16SPI2TxHead;
volatile uint16_t       g_u16SPI2TxByte;


uint8_t                *g_u8I2C0RxBuf;//[PART_RX_SIZE];
volatile uint16_t       g_u16I2C0RxTail;
volatile uint16_t       g_u16I2C0RxHead;
volatile uint16_t       g_u16I2C0RxByte;

uint8_t                *g_u8I2C0TxBuf;//[PART_TX_SIZE];
volatile uint16_t       g_u16I2C0TxTail;
volatile uint16_t       g_u16I2C0TxHead;
volatile uint16_t       g_u16I2C0TxByte;

//CS_LABEL
uint16_t                 g_u16CS_LabelBuf[CS_LABEL_SIZE];
volatile uint16_t       g_u16CS_LabelTail;
volatile uint16_t       g_u16CS_LabelHead;
volatile uint16_t       g_u16CS_LabelByte;
volatile uint16_t       g_u16CS_OverFlow;

volatile uint8_t    g_u8IsCmd = 0;
volatile uint8_t    g_u8IsMaster = 0;

_USB_CMD g_tdsUsbCmd;


S_MODE_REC_T g_tdsModeInfo;

uint8_t g_u8CurrentMode;
uint8_t *g_u8ModeStatus;
uint8_t *g_u8ParamSet;


