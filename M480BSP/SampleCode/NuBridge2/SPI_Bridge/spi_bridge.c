#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"
#include "hal_sys_init.h"

// Error: L6218E: Undefined symbol CLK_GetPCLK0Freq (referred from spi.o).
// Error: L6218E: Undefined symbol CLK_GetPCLK1Freq (referred from spi.o).
// Error: L6218E: Undefined symbol CLK_GetHCLKFreq (referred from spi.o).

__weak uint32_t CLK_GetPCLK0Freq()
{
    return FREQ_192MHZ / 2; // PCLK0 & PCKL1 are constant in this application
}

__weak uint32_t CLK_GetPCLK1Freq()
{
    return FREQ_192MHZ / 2; // PCLK0 & PCKL1 are constant in this application
}

__weak  uint32_t CLK_GetHCLKFreq(void)
{
    return FREQ_192MHZ; // HCLK is constant in this application
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


#define SPI_MONITOR_MODE             (0x0U)
#define SPI_MASTER_MODE              (0x1U)
#define SPI_SLAVE_MODE               (0x2U)
#define SPI_MSB_FIRST                (0x0U)
#define SPI_LSB_FIRST                (SPI_CTL_LSB_Msk)
#define SPI_DATABIT_8                (8U << SPI_CTL_DWIDTH_Pos)
#define SPI_FIFO_TX_LEVEL_BYTE_8     (7U << SPI_FIFOCTL_TXTH_Pos)
#define SPI_FIFO_RX_LEVEL_BYTE_8     (7U << SPI_FIFOCTL_RXTH_Pos)
//
typedef void (*SPI_FUNC)(void);

volatile static SPI_FUNC s_SPIHandlerFn = NULL;
void SPI1_Monitor(void);

static STR_VCOM_LINE_CODING lLineCoding = {0, 0, 0, 0};

// vcom_serial.c
void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32BusClock, u32ActiveLevel, u32MsbLsb, u32SPI1Mode, u32SPI2Mode;

    /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
    if ((gCtrlSignal & BIT0) == 0) {
        return;
    }

    // when host change gCtrlSignal, it will also set gLineCoding again.
    if ((lLineCoding.u32DTERate == gLineCoding.u32DTERate)
            && (lLineCoding.u8CharFormat == gLineCoding.u8CharFormat)
            && (lLineCoding.u8ParityType == gLineCoding.u8ParityType)
            && (lLineCoding.u8DataBits == gLineCoding.u8DataBits)) {
        return;
    }

    lLineCoding.u32DTERate   = gLineCoding.u32DTERate;
    lLineCoding.u8CharFormat = gLineCoding.u8CharFormat;
    lLineCoding.u8ParityType = gLineCoding.u8ParityType;
    lLineCoding.u8DataBits   = gLineCoding.u8DataBits;
    //
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
    //
    u32BusClock = gLineCoding.u32DTERate;

    switch (gLineCoding.u8ParityType & 0x03) {
        case 0:
            u32SPI1Mode = SPI_MODE_0;
            u32SPI2Mode = SPI_MODE_1;
            break;

        case 1:
            u32SPI1Mode = SPI_MODE_1;
            u32SPI2Mode = SPI_MODE_0;
            break;

        case 2:
            u32SPI1Mode = SPI_MODE_2;
            u32SPI2Mode = SPI_MODE_3;
            break;

        case 3:
            u32SPI1Mode = SPI_MODE_3;
            u32SPI2Mode = SPI_MODE_2;
            break;
    }

    if (gLineCoding.u8DataBits & BIT0) { // BIT0: 0(MSB First), 1(LSB First)
        u32MsbLsb = SPI_LSB_FIRST;
    } else {
        u32MsbLsb = SPI_MSB_FIRST;
    }

    if (gLineCoding.u8DataBits & BIT1) { // BIT1: 0(Low), 1(High)
        u32ActiveLevel = SPI_SS_ACTIVE_HIGH;
    } else {
        u32ActiveLevel = SPI_SS_ACTIVE_LOW;
    }

    if (gLineCoding.u8CharFormat == SPI_MONITOR_MODE) {
        s_SPIHandlerFn = SPI1_Monitor;
        SYS_Init_SPI12(); // SPI Monitor = SPI1 Slave + SPI2 Slave
        /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
        /* Configure SPI as a low level active device. */
        /* Disable I2S mode */
        SPI1->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
        SPI2->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
        /* setting from user */
        SPI1->SSCTL = u32ActiveLevel;
        SPI2->SSCTL = u32ActiveLevel;
        /* setting from user */
        SPI1->CTL = SPI_SLAVE | SPI_DATABIT_8 | (u32SPI1Mode) | SPI_CTL_SPIEN_Msk | (u32MsbLsb);
        SPI2->CTL = SPI_SLAVE | SPI_DATABIT_8 | (u32SPI2Mode) | SPI_CTL_SPIEN_Msk | (u32MsbLsb);
        /* Set DIVIDER = 0 */
        SPI1->CLKDIV = 0U;
        SPI2->CLKDIV = 0U;
        /* Set TX FIFO threshold and enable FIFO mode. */
        SPI1->FIFOCTL = (SPI1->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk))
                        | SPI_FIFO_TX_LEVEL_BYTE_8
                        | SPI_FIFO_RX_LEVEL_BYTE_8;
        SPI2->FIFOCTL = (SPI2->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk))
                        | SPI_FIFO_TX_LEVEL_BYTE_8
                        | SPI_FIFO_RX_LEVEL_BYTE_8;
        /* Enable slave selection signal active interrupt flag */
        SPI1->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
        // SPI2->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
        // SPI_WRITE_TX(spi, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */
        NVIC_EnableIRQ(SPI1_IRQn);
    } else {
        s_SPIHandlerFn = NULL;
        SYS_Init_SPI1();
        /* Default setting: disable automatic slave selection function. */
        SPI1->SSCTL = u32ActiveLevel;
        /* Default setting: disable unit transfer interrupt */
        SPI1->CTL = SPI_MASTER | SPI_CTL_SPIEN_Msk
                    | SPI_DATABIT_8
                    | (u32SPI1Mode)
                    | (3 << SPI_CTL_SUSPITV_Pos)
                    | (u32MsbLsb);
        {
            uint32_t u32ClkSrc = 0U, u32Div;
            /* Check clock source of SPI */
            u32ClkSrc = (FREQ_192MHZ / 2);

            if (u32BusClock >= u32ClkSrc) {
                /* Set DIVIDER = 0 */
                SPI1->CLKDIV = 0U;
            } else if (u32BusClock == 0U) {
                /* Set DIVIDER to the maximum value 0xFF. f_spi = f_spi_clk_src / (DIVIDER + 1) */
                SPI1->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
            } else {
                u32Div = (((u32ClkSrc * 10U) / u32BusClock + 5U) / 10U) - 1U; /* Round to the nearest integer */

                if (u32Div > 0xFFU) {
                    u32Div = 0xFFU;
                    SPI1->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
                } else {
                    SPI1->CLKDIV = (SPI1->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);
                }
            }
        }
        /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
        // SPI_SetFIFO(SPI1, 4, 4);
        // SPI_EnableInt(SPI1, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
        {
            SPI1->FIFOCTL = (SPI1->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk))
                            | SPI_FIFO_TX_LEVEL_BYTE_8
                            | SPI_FIFO_RX_LEVEL_BYTE_8;
            SPI1->FIFOCTL |= (SPI_FIFOCTL_TXTHIEN_Msk | SPI_FIFOCTL_RXTOIEN_Msk);
        }
        NVIC_DisableIRQ(SPI1_IRQn);
    }
}


void SPI1_IRQHandler(void)
{
    if (s_SPIHandlerFn != NULL) {
        s_SPIHandlerFn();
    }
}

void SPI1_Monitor(void)
{
    if (SPI1->STATUS & SPI_STATUS_SSACTIF_Msk) {
        SPI1->STATUS |= SPI_STATUS_SSACTIF_Msk;
        SPI1->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
        SPI2->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
        SysTick->LOAD = 1000 * CyclesPerUs;
        SysTick->VAL   = (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        // Active
        while (!(SPI1->STATUS & SPI_STATUS_SSINAIF_Msk)) {
            /* Check RX EMPTY flag */
            if ((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
                    && (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI2) == 0)) {
                monRbuf[monRtail++] = (SPI_READ_RX(SPI1) | (SPI_READ_RX(SPI2) << 8));
                monRtail &= MONBUFMASK;
                monRshorts++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if (SPI1->STATUS & SPI_STATUS_SSINAIF_Msk) {
            SPI1->STATUS |= SPI_STATUS_SSINAIF_Msk;
        }

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;
    } else {
    }
}

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

        __set_PRIMASK(1);
        monRshorts -= (i32Len >> 1);
        __set_PRIMASK(0);
        gu32TxSize = i32Len;
        HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
        HSUSBD->EP[EPA].EPTXCNT = i32Len;
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk);
    } else if (comTbytes && ((gCtrlSignal & 0x03) == 0x01)) {
        uint16_t i = 0, j = 0;
        SPI1->SSCTL |= SPI_SSCTL_SS_Msk;

        /* Wait for transfer done */
        while ((i < comTbytes) || (j < comTbytes)) {
            /* Check TX FULL flag and TX data count */
            if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (i < comTbytes)) {
                /* Write to TX FIFO */
                SPI_WRITE_TX(SPI1, comTbuf[i++]);
            }

            /* Check RX EMPTY flag */
            if ((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0) && (j < comTbytes)) {
                /* Read RX FIFO */
                comRbuf[j++] = SPI_READ_RX(SPI1);
                comRbytes++;
            }
        }

        SPI1->SSCTL &= ~(SPI_SSCTL_SS_Msk);
        //
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

