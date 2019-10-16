/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "uart_transfer.h"

__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};

uint32_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* INTSTS to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
__STATIC_INLINE void UART_IRQHandler(UART_T *uart)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = uart->INTSTS;

    if (u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while (((uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) {    //RX fifo not empty
            uart_rcvbuf[bufhead++] = uart->DAT;
        }
    }

    if (bufhead == MAX_PKT_SIZE) {
        bUartDataReady = (uint32_t)uart;
        bufhead = 0;
    } else if (u32IntSrc & 0x10) {
        bufhead = 0;
    }
}

void UART0_IRQHandler()
{
    UART_IRQHandler(UART0);
}

void UART1_IRQHandler()
{
    UART_IRQHandler(UART1);
}

void UART2_IRQHandler()
{
    UART_IRQHandler(UART2);
}

extern __align(4) uint8_t response_buff[64];
void PutString(UART_T *uart)
{
    uint32_t i;

    for (i = 0; i < MAX_PKT_SIZE; i++) {
        while ((uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        uart->DAT = response_buff[i];
    }
}

void UART_Init(UART_T *uart)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function */
    uart->FUNCSEL = UART_FUNCSEL_UART;
    /* Set UART line configuration */
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Set UART Rx and RTS trigger level */
    uart->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    /* Set UART baud rate */
    uart->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparator */
    uart->TOUT = (uart->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);
    /* 0x0811 */
    uart->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);

    if (uart == (UART_T *)UART0) {
        NVIC_SetPriority(UART0_IRQn, 2);
        NVIC_EnableIRQ(UART0_IRQn);
    } else if (uart == (UART_T *)UART1) {
        NVIC_SetPriority(UART1_IRQn, 2);
        NVIC_EnableIRQ(UART1_IRQn);
    } else if (uart == (UART_T *)UART2) {
        NVIC_SetPriority(UART2_IRQn, 2);
        NVIC_EnableIRQ(UART2_IRQn);
    } else {
        // No supoirt yet.
    }
}

