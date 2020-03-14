#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

// .\obj\NL2_RS232_Bridge.axf: Error: L6218E: Undefined symbol CLK_GetPCLK0Freq (referred from usci_i2c.o).
// .\obj\NL2_RS232_Bridge.axf: Error: L6218E: Undefined symbol CLK_GetPCLK1Freq (referred from usci_i2c.o).

__weak uint32_t CLK_GetPCLK0Freq()
{
    return FREQ_192MHZ / 2; // PCLK0 & PCKL1 are constant in this application
}

__weak uint32_t CLK_GetPCLK1Freq()
{
    return FREQ_192MHZ / 2; // PCLK0 & PCKL1 are constant in this application
}

#define printf VCOM_printf

volatile int8_t gi8BulkOutReady; // vcom_serial.c

// main.c
/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */


STR_VCOM_LINE_CODING ngLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
uint16_t ngCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

volatile uint8_t ngIndex = 0;

void MonitorCdcState()
{
    /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
    if ((gCtrlSignal & BIT0) == 0) {
        return;
    }

    // Only u32DTERate & u8CharFormat are used by I2C.
    if (gLineCoding.u32DTERate != ngLineCoding.u32DTERate) {
        printf("\r\n[%02X]u32DTERate %d -> %d", ngIndex++, ngLineCoding.u32DTERate, gLineCoding.u32DTERate);
        ngLineCoding.u32DTERate = gLineCoding.u32DTERate;
    }

    if (gLineCoding.u8CharFormat != ngLineCoding.u8CharFormat) {
        printf("\r\n[%02X]u8CharFormat %d -> %d", ngIndex++, ngLineCoding.u8CharFormat, gLineCoding.u8CharFormat);
        ngLineCoding.u8CharFormat = gLineCoding.u8CharFormat;
    }

    //

    if (gCtrlSignal != ngCtrlSignal) {
        printf("\r\n[%02X]gCtrlSignal %d -> %d", ngIndex++, ngCtrlSignal, gCtrlSignal);
        ngCtrlSignal = gCtrlSignal;
    }
}

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

// UART2_IRQHandler - for I2C Monitor mode only
// main.c

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

// vcom_serial.c
void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32BusClock;
    uint32_t u32Mode;

    /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
    if ((gCtrlSignal & BIT0) == 0) {
        return;
    }

    if (port == 0) {
        u32BusClock = gLineCoding.u32DTERate;
        u32Mode = gLineCoding.u8CharFormat;
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

#if 1
void VCOM_TransferData(void)
{
    MonitorCdcState();

    if (monRshorts) {
        uint8_t ch;
        uint16_t _monRshorts = monRshorts, i, data;

        for (i = 0; i < _monRshorts; i++) {
            data = monRbuf[monRhead++];

            if (data == 0x5300) {
                // printf("\nS ");
                printf("\r\n(I2C) S ");
            } else 	if (data == 0x5000) {
                printf("P");
            } else {
                ch = (data >> 8);
                printf("%02X ", (data & 0xFF));

                // printf("%c ", ch);
                if (ch == 'N') {
                    printf("%c ", ch);
                }
            }

            monRhead &= MONBUFMASK;
        }

        NVIC_DisableIRQ(USCI0_IRQn);

        if (monRshorts) {
            monRshorts -= _monRshorts;
            NVIC_EnableIRQ(USCI0_IRQn);
        }
    } else if (comTbytes && ((gCtrlSignal & 0x03) == 0x03)) {
        uint8_t *data;
        uint8_t u8SlaveAddr = comTbuf[0];
        UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;

        if (u8SlaveAddr & BIT7) {
            uint16_t u16rLen = inps(comTbuf + 2);
            data = (uint8_t *)comRbuf;
            u8SlaveAddr &= 0x7F; // Slave address(7-bit)
            comRbytes = UI2C_ReadMultiBytes(UI2C0, u8SlaveAddr, data, u16rLen);
        } else {
            uint16_t u16wLen = (comTbytes - 2); // [BYTE0 BYTE1: I2C Address], [BYTE2 ~ ...: Payload]
            data = (uint8_t *)(comTbuf + 2);
            comRbytes = UI2C_WriteMultiBytes(UI2C0, u8SlaveAddr, data, u16wLen);
        }

        UI2C0->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
        comTbytes = 0;

        if (comRbytes) {
            uint8_t ch;
            uint16_t i = 0;
            printf("\r\n");

            for (i = 0; i < comRbytes; i++) {
                ch = data[i];
                printf("%02X ", ch);
            }

            comRbytes = 0;
        }
    }
}
#else // Test code here
void VCOM_TransferData(void)
{
    NVIC_DisableIRQ(USCI0_IRQn);
    // UI2C_Close(UI2C0);
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C0_Open(400000);
    UI2C0->PROTCTL  = (UI2C0->PROTCTL & ~UI2C_PROTCTL_GCFUNC_Msk) | UI2C_GCMODE_DISABLE;
    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));

    while (1);
}
#endif
