/***************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to implement a USB virtual com port device.
 * @version  2.0.0
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "..\vcom_serial\vcom_serial.h"
#include "hal_sys_init.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           0x10000 /* RX buffer size */
#define TXBUFSIZE           0x10000 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART1 */
#ifdef __ICCARM__
#pragma data_alignment=4
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint8_t comTbuf[TXBUFSIZE];
#else
volatile uint8_t comRbuf[RXBUFSIZE] __attribute__((aligned(4)));
volatile uint8_t comTbuf[TXBUFSIZE]__attribute__((aligned(4)));
#endif

// index trick.
// The maximum value of uint16_t is 0xFFFF, which is always less then the buffer size 0x10000.
// In such case, user doesn't need to check whether the index exceed the buffer boundary or not.
volatile uint32_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint32_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
    SYS_Init_RS485();
}

void UART1_IRQHandler(void)
{
    uint32_t u32IntStatus;
    u32IntStatus = UART1->INTSTS;

    // if ((u32IntStatus & UART_INTSTS_RDAINT_Msk) || (u32IntStatus & UART_INTSTS_RXTOINT_Msk)) {
    if (u32IntStatus & (UART_INTSTS_RDAIF_Msk | UART_INTSTS_RXTOIF_Msk)) {
        /* Receiver FIFO threshold level is reached or Rx time out */
        while ((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) {	//RX fifo not empty
            /* Enqueue the character */
            comRbuf[comRtail++] = UART1->DAT;
            comRbytes++;
        }
    }
}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check if any data to send to USB & USB is ready to send them out */
    if (comRbytes && (gu32TxSize == 0)) {
        i32Len = comRbytes;

        if (i32Len > (HSUSBD->EP[EPA].EPMPS & HSUSBD_EPMPS_EPMPS_Msk)) {
            i32Len = (HSUSBD->EP[EPA].EPMPS & HSUSBD_EPMPS_EPMPS_Msk);
        }

        for (i = 0; i < i32Len; i++) {
            HSUSBD->EP[EPA].EPDAT_BYTE = comRbuf[comRhead++];
        }

        NVIC_DisableIRQ(UART1_IRQn);
        comRbytes -= i32Len;
        NVIC_EnableIRQ(UART1_IRQn);
        gu32TxSize = i32Len;
        HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
        HSUSBD->EP[EPA].EPTXCNT = i32Len;
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk);
    }

    /* Process the software Tx FIFO */
    if (comTbytes) {
        i32Len = comTbytes;
        NVIC_DisableIRQ(UART1_IRQn);
        Transceiver_Direction_485 = DIR_TRANSMIT;

        for (i = 0; i < i32Len; i++) {
            while ((UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

            UART_WRITE(UART1, comTbuf[comThead++]);
        }

        NVIC_DisableIRQ(USBD20_IRQn);
        comTbytes -= i32Len;
        NVIC_EnableIRQ(USBD20_IRQn);

        while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

        Transceiver_Direction_485 = DIR_RECEIVE;
        NVIC_EnableIRQ(UART1_IRQn);
    }
}

void VCOM_BulkOut(void)
{
    __IO uint32_t i, IrqSt, u32RxSize;
    IrqSt = HSUSBD->EP[EPB].EPINTSTS & HSUSBD->EP[EPB].EPINTEN;
    u32RxSize = HSUSBD->EP[EPB].EPDATCNT & 0xffff;

    for (i = 0; i < u32RxSize; i++) {
        comTbuf[comTtail++] = HSUSBD->EP[EPB].EPDAT_BYTE;
    }

    comTbytes += u32RxSize;
    /* Set a flag to indicate bulk out ready */
    gi8BulkOutReady = 1;
    HSUSBD_CLR_EP_INT_FLAG(EPB, IrqSt);
}

void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32Reg;
    uint32_t u32Baud_Div;

    if (port == 0) {
        NVIC_DisableIRQ(UART1_IRQn);
        // Reset software fifo
        comRbytes = 0;
        comRhead = 0;
        comRtail = 0;
        comTbytes = 0;
        comThead = 0;
        comTtail = 0;
        // Reset hardware fifo
        UART1->FIFO = 0x3;
        // Set baudrate
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(__HXT, gLineCoding.u32DTERate);

        if (u32Baud_Div > 0xFFFF) {
            UART1->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HXT, gLineCoding.u32DTERate));
        } else {
            UART1->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
        }

        // Set parity
        if (gLineCoding.u8ParityType == 0) {
            u32Reg = 0;    // none parity
        } else if (gLineCoding.u8ParityType == 1) {
            u32Reg = 0x08;    // odd parity
        } else if (gLineCoding.u8ParityType == 2) {
            u32Reg = 0x18;    // even parity
        } else {
            u32Reg = 0;
        }

        // bit width
        switch (gLineCoding.u8DataBits) {
            case 5:
                u32Reg |= 0;
                break;

            case 6:
                u32Reg |= 1;
                break;

            case 7:
                u32Reg |= 2;
                break;

            case 8:
                u32Reg |= 3;
                break;

            default:
                break;
        }

        // stop bit
        if (gLineCoding.u8CharFormat > 0) {
            u32Reg |= 0x4;    // 2 or 1.5 bits
        }

        UART1->LINE = u32Reg;
        // Re-enable UART interrupt
        UART1->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
        UART1->TOUT = (UART1->TOUT & ~UART_TOUT_TOIC_Msk) | (10);
        NVIC_EnableIRQ(UART1_IRQn);
        UART1->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
    }
}

int32_t main(void)
{
    SYS_Init();
    HSUSBD_Open(&gsHSInfo, VCOM_ClassRequest, NULL);
    /* Endpoint configuration */
    VCOM_Init();
    NVIC_EnableIRQ(USBD20_IRQn);

    /* Start transaction */
    while (1) {
        if (HSUSBD_IS_ATTACHED()) {
            HSUSBD_Start();
            break;
        }
    }

    while (1) {
        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

