#include "nubridge2.h"

void RS232_LineCoding(void)
{
    uint32_t u32Reg;
    uint32_t u32Baud_Div;
    NVIC_DisableIRQ(UART2_IRQn);
    // Reset software fifo
    comRbytes = 0;
    comRhead = 0;
    comRtail = 0;
    comTbytes = 0;
    comThead = 0;
    comTtail = 0;
    // Reset hardware fifo
    UART2->FIFO = 0x3;
    // Set baudrate
    u32Baud_Div = UART_BAUD_MODE2_DIVIDER(__HXT, gLineCoding.u32DTERate);

    if (u32Baud_Div > 0xFFFF) {
        UART2->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HXT, gLineCoding.u32DTERate));
    } else {
        UART2->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
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

    UART2->LINE = u32Reg;
    UART_ENABLE_INT(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    // Re-enable UART interrupt
    NVIC_EnableIRQ(UART2_IRQn);
}

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
#if 0

            /* Check if buffer full */
            if (comRbytes < RXBUFSIZE) {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;
                comRbytes++;
            } else {
                /* FIFO over run */
            }

#else
            /* Enqueue the character */
            comRbuf[comRtail++] = bInChar;
            comRbytes++;
#endif
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

void RS232_TransferData(void)
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

void RS485_LineCoding(void)
{
    uint32_t u32Reg;
    uint32_t u32Baud_Div;
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

void RS485_TransferData(void)
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

