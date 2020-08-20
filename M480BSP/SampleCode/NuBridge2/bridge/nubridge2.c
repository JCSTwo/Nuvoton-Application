#include "nubridge2.h"

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

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

typedef union {
    struct {
        __IO uint8_t  comTbuf[TXBUFSIZE];
        __IO uint8_t  comRbuf[RXBUFSIZE];
    };

    __IO union {
        __IO uint16_t _Short;
        __IO uint8_t  _Byte;
        struct {
            __IO uint8_t  _Byte_L;
            __IO uint8_t  _Byte_H;
        };
    } monRbuf[RXBUFSIZE];
} STACK_Buf;

#if defined ( __CC_ARM   )
#pragma no_anon_unions
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

volatile uint32_t monRshorts = 0;
volatile uint16_t monRhead = 0;
volatile uint16_t monRtail = 0;

uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

/*--------------------------------------------------------------------------*/

STACK_Buf gBuf;

uint8_t *comTbuf = (uint8_t *) &gBuf.comTbuf;
uint8_t *comRbuf = (uint8_t *) &gBuf.comRbuf;
uint16_t *monRbuf = (uint16_t *) &gBuf.monRbuf;

void VCOM_BulkOut(void)
{
    __IO uint32_t i, IrqSt;
    IrqSt = HSUSBD->EP[EPB].EPINTSTS & HSUSBD->EP[EPB].EPINTEN;
    gu32RxSize = HSUSBD->EP[EPB].EPDATCNT & 0xffff;

    for (i = 0; i < gu32RxSize; i++) {
        comTbuf[comTtail++] = HSUSBD->EP[EPB].EPDAT_BYTE;
    }

    comTbytes += gu32RxSize;
    /* Set a flag to indicate bulk out ready */
    gi8BulkOutReady = 1;
    HSUSBD_CLR_EP_INT_FLAG(EPB, IrqSt);
}
