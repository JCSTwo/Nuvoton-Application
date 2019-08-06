/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC123.h"
#include "uart_transfer.h"

__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};

// bUartDataReady is a pointer to uart port which is ready. 0: Not ready, UART0: UART0 is ready, UART1: UART1 is ready.
uint32_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */

/*---------------------------------------------------------------------------------------------------------*/
/*  UART IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
__STATIC_INLINE void UART_IRQHandler(UART_T *uart)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = uart->ISR;

    if (u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while (((uart->FSR & UART_FSR_RX_EMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) { //RX fifo not empty
            uart_rcvbuf[bufhead++] = uart->RBR;
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

extern __align(4) uint8_t response_buff[64];
void PutString(UART_T *uart)
{
    uint32_t i;

    for (i = 0; i < MAX_PKT_SIZE; i++) {
        while ((uart->FSR & UART_FSR_TX_FULL_Msk));

        uart->THR = response_buff[i];
    }
}

void UART_Init(UART_T *uart)
{
    IRQn_Type IRQn;

    if (uart == UART0) {
        IRQn = UART0_IRQn;
    } else {
        IRQn = UART1_IRQn;
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
//  uart->FUN_SEL = UART_FUNC_SEL_UART;
    uart->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    uart->FCR = UART_FCR_RFITL_14BYTES | UART_FCR_RTS_TRI_LEV_14BYTES;
    uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));
//  uart->TOR = (uart->TOR & ~UART_TOR_TOIC_Msk)| (0x40);
    uart->TOR = 0x40;
    NVIC_SetPriority(IRQn, 2);
    NVIC_EnableIRQ(IRQn);
    uart->IER = (UART_IER_TIME_OUT_EN_Msk | UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk);
}
