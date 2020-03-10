/***************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to implement a USB virtual com port device.
 * @version  2.0.0
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"
#include "hal_sys_init.h"

// Error: L6218E: Undefined symbol CLK_GetPLLClockFreq (referred from system_m480.o).
uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

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
/* UART2 */
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
    SYS_Init_UART2();
    SYS_Init_LED(1, 1, 1, 1);
}

volatile uint32_t comRerror = 0;

void UART2_IRQHandler(void)
{
    uint8_t bInChar;
    int32_t size;
    uint32_t u32IntStatus;
    u32IntStatus = UART2->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAINT_Msk) || (u32IntStatus & UART_INTSTS_RXTOINT_Msk)) {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((!UART_GET_RX_EMPTY(UART2))) {
            /* Get the character from UART Buffer */
            bInChar = UART_READ(UART2);    /* Rx trigger level is 1 byte*/

            /* Check if buffer full */
            if (comRbytes < RXBUFSIZE) {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;
                comRbytes++;
            } else {
                /* FIFO over run */
                comRerror = 404;
            }
        }
    }

    if (u32IntStatus & UART_INTSTS_THREINT_Msk) {
        if (comTbytes) {
            /* Fill the Tx FIFO */
            size = comTbytes;

            if (size >= TX_FIFO_SIZE) {
                size = TX_FIFO_SIZE;
            }

            while (size) {
                bInChar = comTbuf[comThead++];
                UART_WRITE(UART2, bInChar);
                comTbytes--;
                size--;
            }
        } else {
            /* No more data, just stop Tx (Stop work) */
            UART2->INTEN &= ~UART_INTEN_THREIEN_Msk;
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

        NVIC_DisableIRQ(UART2_IRQn);
        comRbytes -= i32Len;
        NVIC_EnableIRQ(UART2_IRQn);
        gu32TxSize = i32Len;
        HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
        HSUSBD->EP[EPA].EPTXCNT = i32Len;
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk);
    }

    /* Process the software Tx FIFO */
    if (comTbytes) {
        /* Check if Tx is working */
        if ((UART2->INTEN & UART_INTEN_THREIEN_Msk) == 0) {
            /* Send one bytes out */
            UART_WRITE(UART2, comTbuf[comThead++]);
            NVIC_DisableIRQ(UART2_IRQn);
            comTbytes--;
            NVIC_EnableIRQ(UART2_IRQn);
            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART2->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}




int32_t main(void)
{
    SYS_Init();
    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
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

