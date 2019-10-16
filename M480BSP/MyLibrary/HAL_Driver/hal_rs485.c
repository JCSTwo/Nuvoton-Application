#include <stdio.h>
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_api.h"

#define MAX_PKT_SIZE			(64)

void RS485_Init(void)
{
    SYS_Init_RS485();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function */
    UART1->FUNCSEL = UART_FUNCSEL_UART;
    /* Set UART line configuration */
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Set UART Rx and RTS trigger level */
    UART1->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    /* Set UART baud rate */
    UART1->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparator */
    UART1->TOUT = (UART1->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);
    NVIC_SetPriority(UART1_IRQn, 2);
    NVIC_EnableIRQ(UART1_IRQn);
    /* 0x0811 */
    UART1->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

__align(4) uint8_t  uart_rcvbuf[64] = {0};

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */
void UART1_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = UART1->INTSTS;

    if (u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while (((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) {	//RX fifo not empty
            uart_rcvbuf[bufhead++] = UART1->DAT;
        }
    }

    if (bufhead == MAX_PKT_SIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
        _EP_HID_IN_Handler(EP_HID_IN, uart_rcvbuf, 64);
    } else if (u32IntSrc & 0x10) {
        bufhead = 0;
    }
}

void RS485_WriteMultiBytes(uint8_t *data)
{
    uint32_t i;
    NVIC_DisableIRQ(UART1_IRQn);
    bUartDataReady = 0;
    nRTSPin = TRANSMIT_MODE;

    for (i = 0; i < MAX_PKT_SIZE; i++) {
        while ((UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        UART1->DAT = data[i];
    }

    while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

    nRTSPin = REVEIVE_MODE;
    NVIC_EnableIRQ(UART1_IRQn);
}

#ifdef __cplusplus
}
#endif
