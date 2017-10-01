/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB dual virtual COM port device.
 * @note
 ******************************************************************************/
#include <stdio.h>
#include "NUC123.h"
#include "cdc_serial.h"

static __INLINE void MemCopy(volatile uint8_t *dest, volatile uint8_t *src, int32_t size)
{
    while (size--) {
        *dest++ = *src++;
    }
}

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING gLineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */

uint16_t gCtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t gCtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           64 /* RX buffer size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf0[RXBUFSIZE + 64];
volatile uint16_t comRbytes0 = 0;
volatile uint16_t comRhead0 = 0;
volatile uint16_t comRtail0 = 0;

uint8_t gRxBuf0[64] = {0};
volatile uint8_t *gpu8RxBuf0 = 0;
volatile uint32_t gu32RxSize0 = 0;
volatile uint32_t gu32TxSize0 = 0;

volatile int8_t gi8BulkOutReady0 = 0;

/* UART1 */
volatile uint8_t comRbuf1[RXBUFSIZE + 64];
volatile uint16_t comRbytes1 = 0;
volatile uint16_t comRhead1 = 0;
volatile uint16_t comRtail1 = 0;

uint8_t gRxBuf1[64] = {0};
volatile uint8_t *gpu8RxBuf1 = 0;
volatile uint32_t gu32RxSize1 = 0;
volatile uint32_t gu32TxSize1 = 0;

volatile int8_t gi8BulkOutReady1 = 0;



void SYS_Init(void)
{
    /* Enable XT1_OUT (PF.0) and XT1_IN (PF.1) */
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK->PWRCON |= (CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_XTL12M_EN_Msk);

    while (!(CLK->CLKSTATUS & CLK_PWRCON_XTL12M_EN_Msk));

    CLK->PLLCON = CLK_PLLCON_144MHz_HXT;

    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    /* Set core clock */
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(2);
    CLK->CLKDIV &= ~CLK_CLKDIV_USB_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_USB(3);
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;
    /* Enable peripheral clock */
    CLK->APBCLK |= (CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_UART1_EN_Msk | CLK_APBCLK_USBD_EN_Msk);
    PllClock        = 144000000;
    SystemCoreClock = 144000000 / 2;
    CyclesPerUs     = SystemCoreClock / 1000000;
}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not */
    if (gu32TxSize0 == 0) {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (comRbytes0) {
            i32Len = comRbytes0;

            if (i32Len > EP2_MAX_PKT_SIZE) {
                i32Len = EP2_MAX_PKT_SIZE;
            }

            for (i = 0; i < i32Len; i++) {
                gRxBuf0[i] = comRbuf0[comRhead0++];

                if (comRhead0 >= RXBUFSIZE) {
                    comRhead0 = 0;
                }
            }

            __set_PRIMASK(1);
            comRbytes0 -= i32Len;
            __set_PRIMASK(0);
            gu32TxSize0 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf0, i32Len);
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

    if (gu32TxSize1 == 0) {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (comRbytes1) {
            i32Len = comRbytes1;

            if (i32Len > EP7_MAX_PKT_SIZE) {
                i32Len = EP7_MAX_PKT_SIZE;
            }

            for (i = 0; i < i32Len; i++) {
                gRxBuf1[i] = comRbuf1[comRhead1++];

                if (comRhead1 >= RXBUFSIZE) {
                    comRhead1 = 0;
                }
            }

            __set_PRIMASK(1);
            comRbytes1 -= i32Len;
            __set_PRIMASK(0);
            gu32TxSize1 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)gRxBuf1, i32Len);
            USBD_SET_PAYLOAD_LEN(EP7, i32Len);
        } else {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP7);

            if (i32Len == EP7_MAX_PKT_SIZE) {
                USBD_SET_PAYLOAD_LEN(EP7, 0);
            }
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (gi8BulkOutReady0 && (gu32RxSize0 <= RXBUFSIZE - comRbytes1)) {
        for (i = 0; i < gu32RxSize0; i++) {
            comRbuf1[comRtail1++] = gpu8RxBuf0[i];
        }

        if (comRtail1 & RXBUFSIZE) {
            comRtail1 &= (RXBUFSIZE - 1);
            MemCopy(comRbuf1, comRbuf1 + RXBUFSIZE, comRtail1 + 1);
        }

        __set_PRIMASK(1);
        comRbytes1 += gu32RxSize0;
        __set_PRIMASK(0);
        gu32RxSize0 = 0;
        gi8BulkOutReady0 = 0; /* Clear bulk out ready flag */
        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    if (gi8BulkOutReady1 && (gu32RxSize1 <= RXBUFSIZE - comRbytes0)) {
        for (i = 0; i < gu32RxSize1; i++) {
            comRbuf0[comRtail0++] = gpu8RxBuf1[i];
        }

        if (comRtail0 & RXBUFSIZE) {
            comRtail0 &= (RXBUFSIZE - 1);
            MemCopy(comRbuf0, comRbuf0 + RXBUFSIZE, comRtail0 + 1);
        }

        __set_PRIMASK(1);
        comRbytes0 += gu32RxSize1;
        __set_PRIMASK(0);
        gu32RxSize1 = 0;
        gi8BulkOutReady1 = 0; /* Clear bulk out ready flag */
        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }
}

#define DetectPin       PA13

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init();
    /* Initial USB engine */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);
    /* Endpoint configuration */
    VCOM_Init();
    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
    PB13 = 0; // Red LED ON

    while (PA13) {
        VCOM_TransferData();
    }

    outp32(0X50004040, 0xffffffff); //GPIO for QUASI MODE
    outp32(0X50004000, 0xffffffff); //
    outp32(0X50004080, 0xffffffff); //
    outp32(0X500040c0, 0xffffffff); //
    outpw(&SYS->RSTSRC, 3);//clear bit
    FMC_SET_LDROM_BOOT();
    outpw(&SCB->AIRCR, 0x05FA0004);

    /* Trap the CPU */
    while (1);
}
