
#ifndef __NUC_INTERNAL__
#define __NUC_INTERNAL__

#include <string.h>
#include "NUC123.h"
#include "nubridge.h"
#include "NucBridgeApi.h"

// Flow Control Variable
extern volatile uint8_t g_u8IsCmd;

extern volatile uint8_t g_u8VComEnable;

extern volatile uint8_t timer0_tmp;

typedef struct {
    uint8_t type;
    uint8_t mode;
    uint16_t data1;
    uint16_t data2;
    uint16_t data3;
} _USB_CMD;

extern _USB_CMD g_tdsUsbCmd;

typedef struct s_mode_rec {
    uint8_t u8ParamSet[8];
    uint8_t u8ModeStatus[8];
} S_MODE_REC_T;

extern volatile uint8_t     g_u8SPI0Ready;
extern volatile uint8_t     g_u8SPI2Ready;
extern volatile uint8_t     g_u8I2C0Ready;

extern S_MODE_REC_T g_tdsModeInfo;

extern uint8_t g_u8CurrentModeSts;
extern uint8_t *g_u8ModeStatus;
extern uint8_t *g_u8ParamSet;

// Global Memory Allocation
#define RX_SIZE         0x4000
#define RX_BOUND        0x3FFF

#define TX_SIZE         0x400
#define ADCTX_SIZE      0x20

#define RX_SIZE_PART    0x800
#define RX_BOUND_PART   0x7FF

#define TX_SIZE_PART    0x400

#define CS_LABEL_SIZE   0x200
#define CS_LABEL_BOUND  0x1FF

extern uint8_t             *g_u8PDMARxBuf;
extern volatile uint8_t     g_u8PDMAValidByte;
extern volatile uint8_t     g_u8IsMaster;

extern uint8_t                      g_u8BulkOutBuf[64];

extern uint8_t                      g_u8RxBuf[RX_SIZE];
extern volatile uint16_t            g_u16RxTail;
extern volatile uint16_t            g_u16RxHead;
extern volatile uint16_t            g_u16RxByte;
extern volatile uint16_t            g_u16RxBufferOverflow;

extern uint8_t                      g_u8TxBuf[ADCTX_SIZE];//TX_SIZE
extern volatile uint16_t            g_u16TxTail;
extern volatile uint16_t            g_u16TxHead;
extern volatile uint16_t            g_u16TxByte;

// SPI0 Master
extern uint8_t                     *g_u8SPI0RxBuf;
extern volatile uint16_t            g_u16SPI0RxTail;
extern volatile uint16_t            g_u16SPI0RxHead;
extern volatile uint16_t            g_u16SPI0RxByte;

extern uint8_t                     *g_u8SPI0TxBuf;
extern volatile uint16_t            g_u16SPI0TxTail;
extern volatile uint16_t            g_u16SPI0TxHead;
extern volatile uint16_t            g_u16SPI0TxByte;

// SPI2 Master
extern uint8_t                     *g_u8SPI2RxBuf;
extern volatile uint16_t            g_u16SPI2RxTail;
extern volatile uint16_t            g_u16SPI2RxHead;
extern volatile uint16_t            g_u16SPI2RxByte;

extern uint8_t                     *g_u8SPI2TxBuf;
extern volatile uint16_t            g_u16SPI2TxTail;
extern volatile uint16_t            g_u16SPI2TxHead;
extern volatile uint16_t            g_u16SPI2TxByte;


extern uint8_t                     *g_u8I2C0RxBuf;
extern volatile uint16_t            g_u16I2C0RxTail;
extern volatile uint16_t            g_u16I2C0RxHead;
extern volatile uint16_t            g_u16I2C0RxByte;

extern uint8_t                     *g_u8I2C0TxBuf;
extern volatile uint16_t            g_u16I2C0TxTail;
extern volatile uint16_t            g_u16I2C0TxHead;
extern volatile uint16_t            g_u16I2C0TxByte;

extern uint16_t                     g_u16CS_LabelBuf[];
extern volatile uint16_t            g_u16CS_LabelTail;
extern volatile uint16_t            g_u16CS_LabelHead;
extern volatile uint16_t            g_u16CS_LabelByte;
extern volatile uint16_t            g_u16CS_OverFlow;

extern uint8_t g_FindSlave;
extern uint8_t I2C_IndentifiedSlaveAddrBuf[];

// main.c
void UART0_DeInit(void);
void UART1_DeInit(void);



extern IRQn_Type IRQn_Rec;

void Nuc_Bridge_Init(void);
void Nuc_Bridge_Main(void);

#define GPIO_MONITOR1       1
#define GPIO_MONITOR2       2
#define SOFTWARE_I2C_MONITOR        GPIO_MONITOR2
void GPIO_Monitor_Init(void);
void GPIO_DeInit(void);

// SPI Master
void SPI0_Master_Init(void);
void SPI0_Master_DeInit(void);
void SPI0_DeInit_Param(SPI_T *port);
void SPI0_Master_MainProcess(void);

void SPI2_Master_Init(void);
void SPI2_Master_DeInit(void);
void SPI2_DeInit_Param(SPI_T *port);
void SPI2_Master_MainProcess(void);

void SPI_Slave_Init(void);
void SPI0_IRQHandler(void);
void SPI_Slave_DeInit(void);
void SPI_Slave_MainProcess(void);

void SPI0_Master_ReInit(uint8_t setting);
void SPI2_Master_ReInit(uint8_t setting);
void SPI0_Monitor_ReInit(uint8_t setting);
void I2C0_Master_ReInit(uint8_t setting);

//I2C Master Function Declaration
void I2C0_Master_Init(void);
void I2C0_Master_DeInit(void);
void I2C0_Master_MainProcess(void);

//I2C Monitor Function Declaration
void I2C_Monitor_DeInit(void);
void I2C_Monitor_Init(void);
void I2C_Monitor_MainProcess(void);

// VCOM
void UART1_Init(void);

void Nuc_Reset(void);

typedef struct DRVPDMA_STRUCT {                     /* PDMA structure */
    uint32_t u32SrcCtrl;    /* Source Control */
    uint32_t u32SrcAddr;    /* Source address */
    uint32_t u32DestCtrl;   /* Destination Control */
    uint32_t u32DestAddr;   /* Destination address */
    uint32_t u32TransWidth; /* Transfer Width */
    uint32_t u32Mode;       /* Operation Mode */
    uint32_t u32ByteCnt;    /* Byte Count */
} STR_PDMA_T;

#endif  /* __NUC_INTERNAL__ */

