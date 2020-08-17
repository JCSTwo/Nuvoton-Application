#include <stdio.h>
#include "NuMicro.h"
#include "..\vcom_serial\vcom_serial.h"
#include "hal_sys_init.h"

volatile int8_t gi8BulkOutReady; // vcom_serial.c

// main.c
/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
// Buffer for I2C_MONITOR_MODE
#define MONBUFSIZE 0x10000
#define MONBUFMASK (MONBUFSIZE - 1)

volatile uint16_t monRbuf[MONBUFSIZE];
volatile uint16_t monRshorts = 0;
volatile uint16_t monRhead = 0;
volatile uint16_t monRtail = 0;

// Buffer for I2C_NORMAL_MODE
#define TXBUFSIZE           0x2000 /* TX buffer size */
#define TXBUFMASK           (TXBUFSIZE - 1)
volatile uint8_t  comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

#define RXBUFSIZE           0x2000 /* RX buffer size */
#define RXBUFMASK           (RXBUFSIZE - 1)
volatile uint8_t  comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

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




// UI2C_Open(UI2C0, u32ClkSpeed)
static __INLINE void UI2C0_Open(uint32_t u32BusClock)
{
    uint32_t u32ClkDiv;
    uint32_t u32Pclk = FREQ_192MHZ / 2; // PCLK0 & PCKL1 are constant in this application
    u32ClkDiv = (uint32_t)((((((u32Pclk / 2U) * 10U) / (u32BusClock)) + 5U) / 10U) - 1U); /* Compute proper divider for USCI_I2C clock */
    /* Enable USCI_I2C protocol */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4U << UI2C_CTL_FUNMODE_Pos;
    /* Data format configuration */
    /* 8 bit data length */
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8U << UI2C_LINECTL_DWIDTH_Pos;
    /* MSB data format */
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;
    /* Set USCI_I2C bus clock */
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32ClkDiv << UI2C_BRGEN_CLKDIV_Pos);
    // !!!! Don't enable I2C protocol. It will affect the I2C bus.
    // UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C0->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
}

// Monitor (Slave)
void UI2C0_Monitor_Init(void)
{
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    /* Set USCI_I2C0 Slave Addresses */
    // UI2C_SetSlaveAddr(UI2C0, 0, 0xFF, UI2C_GCMODE_DISABLE);
    UI2C0->DEVADDR0  = 0xFF;
    UI2C0->PROTCTL  = (UI2C0->PROTCTL & ~UI2C_PROTCTL_GCFUNC_Msk) | UI2C_GCMODE_DISABLE;
    /* Enable Monitor Mode */
#if SCLOUT_ENABLE
    UI2C0->PROTCTL |= (UI2C_PROTCTL_MONEN_Msk | UI2C_PROTCTL_SCLOUTEN_Msk);
#else
    UI2C0->PROTCTL |= UI2C_PROTCTL_MONEN_Msk;
#endif
    // UI2C_SetSlaveAddrMask(UI2C0, 0, 0xFF);
    UI2C0->ADDRMSK0  = 0xFF;
    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk |
                                 UI2C_PROTIEN_NACKIEN_Msk |
                                 UI2C_PROTIEN_STORIEN_Msk |
                                 UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI0_IRQn);
    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_AA);
}

#define SCLOUT_ENABLE 0
#define I2C_MONITOR_MODE    0ul
#define I2C_NORMAL_MODE     1ul

void I2C_Bridge_Init(uint32_t u32Mode, uint32_t u32BusClock)
{
    NVIC_DisableIRQ(USCI0_IRQn);
    // UI2C_Close(UI2C0);
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C0_Open(u32BusClock);

    if (u32Mode == I2C_MONITOR_MODE) {
        UI2C0_Monitor_Init();
    } else {
        UI2C0->PROTCTL  = (UI2C0->PROTCTL & ~UI2C_PROTCTL_GCFUNC_Msk) | UI2C_GCMODE_DISABLE;
        /* Enable UI2C0 protocol interrupt */
        UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    }
}

void USCI0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = (UI2C0->PROTSTS);

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk) {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);
        monRbuf[monRtail++] = 0x5300; // 'S';
        monRtail &= MONBUFMASK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    } else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk) {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);
        monRbuf[monRtail++] = (0x4100 | UI2C_GET_DATA(UI2C0)); // 'A';
        monRtail &= MONBUFMASK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    } else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk) {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);
        monRbuf[monRtail++] = (0x4E00 | UI2C_GET_DATA(UI2C0)); // 'N';
        monRtail &= MONBUFMASK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    } else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk) {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);
        monRbuf[monRtail++] = 0x5000; // 'P';
        monRtail &= MONBUFMASK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    } else {
        return;
    }

    monRshorts++;
}


static STR_VCOM_LINE_CODING lLineCoding = {0, 0, 0, 0};

// vcom_serial.c
void VCOM_LineCoding(uint8_t port)
{
    __IO uint32_t u32BusClock, u32Mode;

    /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
    if ((gCtrlSignal & BIT0) == 0) {
        return;
    }

    u32BusClock = gLineCoding.u32DTERate;
    u32Mode = gLineCoding.u8CharFormat;

    // when host change gCtrlSignal, it will also set gLineCoding again.
    if ((lLineCoding.u32DTERate != u32BusClock) || (lLineCoding.u8CharFormat != u32Mode)) {
        lLineCoding.u32DTERate = u32BusClock;
        lLineCoding.u8CharFormat = u32Mode;
        // Reset software fifo
        comRbytes = 0;
        comRhead = 0;
        comRtail = 0;
        comTbytes = 0;
        comThead = 0;
        comTtail = 0;
        // Monitor
        monRshorts = 0;
        monRhead = 0;
        monRtail = 0;
        // Set baudrate
        I2C_Bridge_Init(u32Mode, u32BusClock);
    }
}

static uint8_t u8SlaveAddr;
static uint16_t u16rwLen;

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check if any data to send to USB & USB is ready to send them out */
    if (monRshorts && (gu32TxSize == 0)) {
        uint16_t u16Data;
        i32Len = monRshorts * 2;

        if (i32Len > (HSUSBD->EP[EPA].EPMPS & HSUSBD_EPMPS_EPMPS_Msk)) {
            i32Len = (HSUSBD->EP[EPA].EPMPS & HSUSBD_EPMPS_EPMPS_Msk);
        }

        for (i = 0; i < i32Len; i += 2) {
            u16Data = monRbuf[monRhead++];
            HSUSBD->EP[EPA].EPDAT_BYTE = (u16Data >> 8);
            HSUSBD->EP[EPA].EPDAT_BYTE = (u16Data & 0x00FF);
            monRhead &= MONBUFMASK;
        }

        NVIC_DisableIRQ(USCI0_IRQn);
        monRshorts -= (i32Len >> 1);
        NVIC_EnableIRQ(USCI0_IRQn);
        gu32TxSize = i32Len;
        HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
        HSUSBD->EP[EPA].EPTXCNT = i32Len;
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk);
    } else if (comTbytes && ((gCtrlSignal & 0x03) == 0x01)) {
        uint8_t *pu8Data;
        u8SlaveAddr = comTbuf[0];
        UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;

        if (u8SlaveAddr & BIT7) { // i2c read
            u16rwLen = inps(comTbuf + 2);
            pu8Data = (uint8_t *)comRbuf;
            u8SlaveAddr &= 0x7F; // Slave address(7-bit)

            if (1 == u16rwLen) {
                pu8Data[0] = UI2C_ReadByte(UI2C0, u8SlaveAddr);
                comRbytes = 1;
            } else {
                comRbytes = UI2C_ReadMultiBytes(UI2C0, u8SlaveAddr, pu8Data, u16rwLen);
            }
        } else { // i2c write
            u16rwLen = (comTbytes - 2); // [BYTE0 BYTE1: I2C Address], [BYTE2 ~ ...: Payload]
            pu8Data = (uint8_t *)(comTbuf + 2);

            if (1 == u16rwLen) {
                if (0 == UI2C_WriteByte(UI2C0, u8SlaveAddr, pu8Data[0])) {
                    comRbytes = 1;
                } else {
                    comRbytes = 0;
                }
            } else {
                comRbytes = UI2C_WriteMultiBytes(UI2C0, u8SlaveAddr, pu8Data, u16rwLen);
            }

            if (comRbytes == u16rwLen) {
                comRbuf[0] = 'O';
                comRbuf[1] = 'K';
            } else {
                comRbuf[0] = 'N';
                comRbuf[1] = 'G';
            }

            comRbytes = 2;
        }

        UI2C0->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
        NVIC_DisableIRQ(USBD20_IRQn);
        comTbytes = 0;
        comTtail = 0; //reset index
        NVIC_EnableIRQ(USBD20_IRQn);

        while (comRbytes && (comTbytes == 0)) {
            while ((HSUSBD->EP[EPA].EPINTSTS & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk) == 0);

            i32Len = comRbytes;

            if (i32Len > (HSUSBD->EP[EPA].EPMPS & HSUSBD_EPMPS_EPMPS_Msk)) {
                i32Len = (HSUSBD->EP[EPA].EPMPS & HSUSBD_EPMPS_EPMPS_Msk);
            }

            for (i = 0; i < i32Len; i++) {
                HSUSBD->EP[EPA].EPDAT_BYTE = comRbuf[i];
            }

            gu32TxSize = i32Len;
            HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
            HSUSBD->EP[EPA].EPTXCNT = i32Len;
            HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk);
            comRbytes -= i32Len;
        }
    }
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
    SYS_Init_UI2C0();
    SYS_Init_LED(1, 1, 1, 1);
}

int32_t main(void)
{
    SYS_Init();
    /* Enable Interrupt and install the call back function */
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

