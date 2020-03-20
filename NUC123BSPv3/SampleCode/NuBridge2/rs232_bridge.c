#include "NuBridge2.h"

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */

void UART_LineCoding()
{
    uint32_t u32Reg;
    // Reset hardware fifo
    UART->FCR = 0x6;
    // Set baudrate
    UART->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, gLineCoding.u32DTERate));

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

    /* bit width */
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

    /* stop bit */
    if (gLineCoding.u8CharFormat > 0) {
        u32Reg |= 0x4;    // 2 or 1.5 bits
    }

    UART->LCR = u32Reg;
    /* Enable UART Interrupt */
    UART->IER = UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk;
    NVIC_EnableIRQ(UART_IRQn);
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;
    __NOP();
    __NOP();
    /* Configure UART and set UART Baudrate */
    UART->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Enable UART Interrupt */
    UART->IER = UART_IER_RTO_IEN_Msk | UART_IER_RDA_IEN_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;
    u32IntStatus = UART->ISR;

    if ((u32IntStatus & 0x1 /* RDAIF */) || (u32IntStatus & 0x10 /* TOUT_IF */)) {
        /* Receiver FIFO threashold level is reached or RX time out */

        /* Get all the input characters */
        while ((UART->FSR & UART_FSR_RX_EMPTY_Msk) == 0) {
            /* Get the character from UART Buffer */
            bInChar = UART->DATA;

            /* Check if buffer full */
            if (comRbytes < RXBUFSIZE) {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;

                if (comRtail >= RXBUFSIZE) {
                    comRtail = 0;
                }

                comRbytes++;
            } else {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & 0x2 /* THRE_IF */) {
        if (comTbytes) {
            /* Fill the TX FIFO */
            size = comTbytes;

            if (size >= TX_FIFO_SIZE) {
                size = TX_FIFO_SIZE;
            }

            while (size) {
                bInChar = comTbuf[comThead++];
                UART->DATA = bInChar;

                if (comThead >= TXBUFSIZE) {
                    comThead = 0;
                }

                comTbytes--;
                size--;
            }
        } else {
            /* No more data, just stop TX (Stop work) */
            UART->IER &= (~UART_IER_THRE_IEN_Msk);
        }
    }
}

void UART_TransferData(void)
{
    int32_t i, i32Len;

    /* Check wether USB is ready for next packet or not*/
    if (gu32TxSize == 0) {
        /* Check wether we have new COM Rx data to send to USB or not */
        if (comRbytes) {
            i32Len = comRbytes;

            if (i32Len > EP2_MAX_PKT_SIZE) {
                i32Len = EP2_MAX_PKT_SIZE;
            }

            for (i = 0; i < i32Len; i++) {
                gRxBuf[i] = comRbuf[comRhead++];

                if (comRhead >= RXBUFSIZE) {
                    comRhead = 0;
                }
            }

            __set_PRIMASK(1);
            comRbytes -= i32Len;
            __set_PRIMASK(0);
            gu32TxSize = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        } else {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);

            if (i32Len == EP2_MAX_PKT_SIZE) {
                USBD_SET_PAYLOAD_LEN(EP2, 0);
            }
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes)) {
        for (i = 0; i < gu32RxSize; i++) {
            comTbuf[comTtail++] = gpu8RxBuf[i];

            if (comTtail >= TXBUFSIZE) {
                comTtail = 0;
            }
        }

        __set_PRIMASK(1);
        comTbytes += gu32RxSize;
        __set_PRIMASK(0);
        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */
        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software TX FIFO */
    if (comTbytes) {
        /* Check if TX is working */
        if ((UART->IER & UART_IER_THRE_IEN_Msk) == 0) {
            /* Send one bytes out */
            UART->DATA = comTbuf[comThead++];

            if (comThead >= TXBUFSIZE) {
                comThead = 0;
            }

            __set_PRIMASK(1);
            comTbytes--;
            __set_PRIMASK(0);
            /* Enable TX Empty Interrupt. (Trigger first one) */
            UART->IER |= UART_IER_THRE_IEN_Msk;
        }
    }
}

__weak void SYS_Init(void)
{
    SYS_UnlockReg();
    SYS_Init_72MHZ_USBD();
    SYS_Init_UART();
    s_BridgeInitFn = &UART_LineCoding;
    s_BridgeMainFn = &UART_TransferData;
}

__weak int32_t main(void)
{
    SYS_Init();
    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);
    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    while (1) {
        VCOM_TransferData();
    }
}
