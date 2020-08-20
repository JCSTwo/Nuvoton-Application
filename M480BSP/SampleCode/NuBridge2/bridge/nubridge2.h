#ifndef __NU_BRIDGE2_H__
#define __NU_BRIDGE2_H__

#include <stdio.h>
#include <stdint.h>
#include "hal_sys_init.h"
#include "..\vcom_serial\vcom_serial.h"

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           0x10000 /* RX buffer size */
#define TXBUFSIZE           0x10000 /* TX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */

// nubridge2.c
extern uint8_t *comTbuf;
extern uint8_t *comRbuf;
extern uint16_t *monRbuf;

extern volatile uint32_t comTbytes;
extern volatile uint32_t comRbytes;

// nubridge2.c
void VCOM_BulkOut(void);

// uart_bridge.c
void RS232_LineCoding(void);
void RS232_TransferData(void);
void RS485_LineCoding(void);
void RS485_TransferData(void);
void RS232Monitor_LineCoding(void);
void RS232Monitor_TransferData(void);

#endif  /* __NU_BRIDGE2_H__ */
