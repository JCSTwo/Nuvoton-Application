#include <stdio.h>
#include "NuMicro.h"
#include "hal_sys_init.h"
#include "..\vcom_serial\vcom_serial.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */


volatile int8_t gi8CanTxOK = 1;

#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */
#define TXIDXMASK           (TXBUFSIZE - 1)

STR_CANMSG_T comRbuf[RXBUFSIZE] __attribute__((aligned(4)));
STR_CANMSG_T comTbuf[TXBUFSIZE]__attribute__((aligned(4)));
uint8_t gRxBuf[64] __attribute__((aligned(4))) = {0};
uint8_t gUsbRxBuf[64] __attribute__((aligned(4))) = {0};

volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

//uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
    SYS_Init_UART2();
    SYS_Init_CAN1();
}

void VCOM_ClassRequest(void);
void VCOM_Init(void);
void HSUSBD_VCOM_Init(void)
{
    HSUSBD_Open(&gsHSInfo, VCOM_ClassRequest, NULL);
    /* Endpoint configuration */
    VCOM_Init();
    /* Enable USBD interrupt */
    NVIC_EnableIRQ(USBD20_IRQn);

    /* Start transaction */
    while (1) {
        if (HSUSBD_IS_ATTACHED()) {
            HSUSBD_Start();
            break;
        }
    }
}

void VCOM_LineCoding(uint8_t port)
{
}

void CAN_STOP(void)
{
    /* Disable CAN1 Clock and Reset it */
    SYS->IPRST1 |= SYS_IPRST1_CAN1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_CAN1RST_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_CAN1CKEN_Msk;
}

void CAN_ShowMsg(STR_CANMSG_T *Msg)
{
    uint8_t u8Len, i;
    char *data  = (char *)Msg;

    while ((HSUSBD->EP[EPA].EPINTSTS & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk) == 0);

    u8Len = sizeof(STR_CANMSG_T);
    gu32TxSize = u8Len;

    for (i = 0; i < u8Len; i++) {
        HSUSBD->EP[EPA].EPDAT_BYTE = data[i];
    }

    HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
    HSUSBD->EP[EPA].EPTXCNT = u8Len;
    HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_INTKIEN_Msk);
}

typedef void (*CAN_FUNC)(void);
volatile static CAN_FUNC s_CAN1HandlerFn = NULL;

void CAN1_IRQHandler_Basic(void)
{
    if (CAN1->IIDR == 0x00008000) {     /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if (CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            /* Check if buffer full */
            if (comRbytes < RXBUFSIZE) {
                if (CAN_BasicReceiveMsg(CAN1, &comRbuf[comRtail])) {
                    comRtail++;

                    /* Enqueue the character */
                    if (comRtail >= RXBUFSIZE) {
                        comRtail = 0;
                    }

                    comRbytes++;
                }
            } else {
                /* FIFO over run */
            }
        }
    }
}

void CAN1_IRQHandler_Normal(void)
{
    uint32_t u8IIDRstatus;
    u8IIDRstatus = CAN1->IIDR;

    if (u8IIDRstatus == 0x00008000) {     /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if (CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if (CAN1->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
            gi8CanTxOK = 1;
        }
    } else if (u8IIDRstatus != 0) {
        // CAN_MsgInterrupt(CAN1, u8IIDRstatus);
        u8IIDRstatus -= 1;

        if (u8IIDRstatus < 4) { // config MSG(0) ~ MSG(3) by CAN_SetRxMsg
            /* Check if buffer full */
            if (comRbytes < RXBUFSIZE) {
                CAN_Receive(CAN1, u8IIDRstatus, &comRbuf[comRtail]);
                comRtail++;

                /* Enqueue the character */
                if (comRtail >= RXBUFSIZE) {
                    comRtail = 0;
                }

                comRbytes++;
            } else {
                /* FIFO over run */
            }
        }

        CAN_CLR_INT_PENDING_BIT(CAN1, u8IIDRstatus); /* Clear Interrupt Pending */
    }
}

void CAN_Bridge_Init(uint32_t u32BaudRate, uint32_t u32Mode)
{
    CAN_STOP();
    CLK->APBCLK0 |= CLK_APBCLK0_CAN1CKEN_Msk;
    CAN_Open(CAN1, u32BaudRate, u32Mode);
    CAN_EnableInt(CAN1, (CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk));
    NVIC_SetPriority(CAN1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    if (u32Mode == CAN_BASIC_MODE) {
        // Set Silent Mode
        CAN1->TEST |= CAN_TEST_SILENT_Msk;
        s_CAN1HandlerFn = CAN1_IRQHandler_Basic;
    } else if (u32Mode == CAN_NORMAL_MODE) {
        // CAN_SetRxMsg(CAN1, MSG(0), CAN_STD_ID, 0x784);
        s_CAN1HandlerFn = CAN1_IRQHandler_Normal;
    } else {
        while (1);
    }

    NVIC_EnableIRQ(CAN1_IRQn);
}

void CAN_Bridge_SetRxMsg(uint32_t u32IDType, uint32_t *u32ID)
{
    if (u32IDType == CAN_STD_ID) {
        if (u32ID[0] != 0xFFFFFFFF) {
            CAN_SetRxMsg(CAN1, MSG(0), CAN_STD_ID, u32ID[0]);
        }

        if (u32ID[1] != 0xFFFFFFFF) {
            CAN_SetRxMsg(CAN1, MSG(1), CAN_STD_ID, u32ID[1]);
        }
    } else if (u32IDType == CAN_EXT_ID) {
        if (u32ID[0] != 0xFFFFFFFF) {
            CAN_SetRxMsg(CAN1, MSG(2), CAN_EXT_ID, u32ID[0]);
        }

        if (u32ID[1] != 0xFFFFFFFF) {
            CAN_SetRxMsg(CAN1, MSG(3), CAN_EXT_ID, u32ID[1]);
        }
    }
}

void CAN_TransferData(void)
{
    if (comRbytes && (gu32TxSize == 0)) {
        CAN_ShowMsg(&comRbuf[comRhead]);
        comRhead++;

        if (comRhead >= RXBUFSIZE) {
            comRhead = 0;
        }

        NVIC_DisableIRQ(CAN1_IRQn);
        comRbytes --;
        NVIC_EnableIRQ(CAN1_IRQn);
    }

    if (comTbytes && gi8CanTxOK) {
        if (CAN_Transmit(CAN1, MSG(5), &comTbuf[comThead]) == FALSE) { // Configure Msg RAM and send the Msg in the RAM
            return;
        }

        gi8CanTxOK = 0;
        NVIC_DisableIRQ(USBD20_IRQn);
        comTbytes--;
        NVIC_EnableIRQ(USBD20_IRQn);
        comThead++;
        comThead &= TXIDXMASK;
    }
}

volatile int8_t gi8CanCmdReady = 0;

void CAN_ProcessCommand(void)
{
    if (gi8CanCmdReady) {
        uint32_t u32BaudRate, u32Mode;
        u32BaudRate = inpw(gUsbRxBuf + 4);
        u32Mode = inpw(gUsbRxBuf + 8);
        CAN_Bridge_Init(u32BaudRate, u32Mode);

        if (u32Mode == CAN_NORMAL_MODE) {
            CAN_Bridge_SetRxMsg(CAN_STD_ID, (uint32_t *)(gUsbRxBuf + 12));
            CAN_Bridge_SetRxMsg(CAN_EXT_ID, (uint32_t *)(gUsbRxBuf + 20));
        }

        gi8CanTxOK = 1;
        gi8CanCmdReady = 0;
    }
}

void VCOM_BulkOut(void)
{
    __IO uint32_t i, IrqSt, u32RxSize;
    uint32_t u32CanHeader;
    uint8_t u8Dummy;
    uint8_t *CanTxBuf = (uint8_t *)&u32CanHeader;
    // uint8_t *CanTxBuf = (uint8_t *)(&comTbuf[comTtail]);
    IrqSt = HSUSBD->EP[EPB].EPINTSTS & HSUSBD->EP[EPB].EPINTEN;
    u32RxSize = HSUSBD->EP[EPB].EPDATCNT & 0xffff;

    if ((u32RxSize < 4) || (u32RxSize > 64)) {
        goto QUIT;
    }

    CanTxBuf[0] = HSUSBD->EP[EPB].EPDAT_BYTE;
    CanTxBuf[1] = HSUSBD->EP[EPB].EPDAT_BYTE;
    CanTxBuf[2] = HSUSBD->EP[EPB].EPDAT_BYTE;
    CanTxBuf[3] = HSUSBD->EP[EPB].EPDAT_BYTE;
    u32RxSize -= 4;

    if (u32CanHeader == 0x444e4143) { // Data
        CanTxBuf = (uint8_t *)(&comTbuf[comTtail]);
        comTtail++;
        comTbytes++;
        comTtail &= TXIDXMASK;
    } else if (u32CanHeader == 0x434e4143) { // Command
        CanTxBuf = gUsbRxBuf;
        gi8CanCmdReady = 1;
    } else {
        goto QUIT;
    }

    for (i = 0; i < u32RxSize; i++) {
        CanTxBuf[i] = HSUSBD->EP[EPB].EPDAT_BYTE;
    }

    u32RxSize = 0;
QUIT:

    while (u32RxSize) {
        u8Dummy = HSUSBD->EP[EPB].EPDAT_BYTE;
        u32RxSize--;
    }

    /* Set a flag to indicate bulk out ready */
    gi8BulkOutReady = 1;
    HSUSBD_CLR_EP_INT_FLAG(EPB, IrqSt);
}


void CAN1_IRQHandler(void)
{
    if (s_CAN1HandlerFn != NULL) {
        s_CAN1HandlerFn();
    }
}

int main()
{
    SYS_Init();
    HSUSBD_VCOM_Init();

    while (gi8BulkOutReady == 0);

    gi8BulkOutReady = 0;

    if ((comTbytes == 0) && (gi8CanCmdReady == 0)) {
        CAN_Bridge_Init(500000, CAN_BASIC_MODE);
    }

    while (1) {
        CAN_ProcessCommand();
        CAN_TransferData();
    };
}
