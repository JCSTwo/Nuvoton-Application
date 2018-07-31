/******************************************************************************
 * @file     mouse.c
 * @brief    NANO100 series USBD driver source file
 * @version  2.0.0
 * @date     22, March, 2013
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "..\NucFunction\NucInternal.h"
#include "sys.h"
#include "usbd.h"
#include "nubridge.h"


/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint8_t comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint8_t gRxBuf[64] = {0};
uint8_t *gpu8RxBuf = 0;
uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;
/********************************************************************/
uint32_t gu32InBuf[EP5_MAX_PKT_SIZE / 4] = {0};
uint32_t gu32InSize  = 0;
uint32_t gu32OutSize = 0;

/********************************************************************/
void SYS_Init(void)
{
    SYS_UnlockReg();
    /* Enable XT1_OUT(PF0) and XT1_IN(PF1) */
    SYS->GPF_MFP |= SYS_GPF_MFP_PF0_XT1_OUT | SYS_GPF_MFP_PF1_XT1_IN;
    /* Enable Internal RC 22.1184 MHz clock */
    /* Enable external XTAL 12MHz clock */
    /* Enable Internal RC 10 KHz clock for WDT */
    CLK->PWRCON |= (CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk);

    /* Waiting for external XTAL clock ready */
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as CLK_PLLCON_144MHz_HXT from PLL */
    CLK->PLLCON = CLK_PLLCON_144MHz_HXT;

    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    CLK->CLKDIV &= ~(CLK_CLKDIV_HCLK_N_Msk | CLK_CLKDIV_USB_N_Msk);
    CLK->CLKDIV |= (CLK_CLKDIV_HCLK(2) | CLK_CLKDIV_USB(3));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 144000000;                // PLL
    SystemCoreClock = 144000000 / 2;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;                   // For SYS_SysTickDelay()
    /* Enable USB device clock */
    CLK->APBCLK |= CLK_APBCLK_USBD_EN_Msk;
    LOCKREG(); //en: Re-lock the locked registers
}

void UART1_Init(void)
{
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB4_Msk | SYS_ALT_MFP_PB5_Msk);
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;
    CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_UART_N_Msk)) | CLK_CLKDIV_UART(1);
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART1->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Enable RDA\RLS Interrupt */
    UART1->IER |= (UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk);
    NVIC_EnableIRQ(UART1_IRQn);
}

void UART1_DeInit(void)
{
    NVIC_DisableIRQ(UART1_IRQn);
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB4_Msk | SYS_ALT_MFP_PB5_Msk);
}

void Timer0_Init(void)
{
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0_S_Msk)) | CLK_CLKSEL1_TMR0_S_HCLK;
    ////Timer0 configure. Toggle PIN64:PB8_TM0 every 10ms ///////////////////////////////////////////
    // DrvTIMER_Init(TIMER0, 255/*prescale*/, 10000/*TCMPR*/, TIM_MODE_PERIODIC);
    CLK->APBCLK |= CLK_APBCLK_TMR0_EN_Msk;
    TIMER0->TISR   |=  TIMER_TISR_TIF_Msk | TIMER_TISR_TWF_Msk ;
    TIMER0->TEXISR |=  TIMER_TEXISR_TEXIF_Msk  ;
    TIMER0->TCMPR   =  10000 ;
    TIMER0->TCSR    =  TIMER_PERIODIC_MODE + TIMER_TCSR_TDR_EN_Msk + TIMER_TCSR_CACT_Msk + 255 ;
    // DrvTIMER_EnableInt(TIMER0, TIM_INT_EN);
    TIMER0->TISR = TIMER0->TISR;  // clear pending interrupt
    TIMER0->TCSR |= TIMER_TCSR_IE_Msk;
    NVIC_EnableIRQ(TMR0_IRQn);
}

void TMR0_IRQHandler(void)
{
    PB14 = (timer0_tmp & 0x01);
    timer0_tmp--;

    if (timer0_tmp == 0) {
        NVIC_DisableIRQ(TMR0_IRQn);
    }

    TIMER0->TISR = TIMER_TISR_TIF_Msk;  //Clear IF flag
}




/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;
    u32IntStatus = UART1->ISR;

    if ((u32IntStatus & 0x1 /* RDAIF */) || (u32IntStatus & 0x10 /* TOUT_IF */)) {
        /* Receiver FIFO threashold level is reached or Rx time out */

        /* Get all the input characters */
        while ((UART1->FSR & UART_FSR_RX_EMPTY_Msk) == 0) {
            /* Get the character from UART Buffer */
            bInChar = UART1->DATA;

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
            /* Fill the Tx FIFO */
            size = comTbytes;

            if (size >= TX_FIFO_SIZE) {
                size = TX_FIFO_SIZE;
            }

            while (size) {
                bInChar = comTbuf[comThead++];
                UART1->DATA = bInChar;

                if (comThead >= TXBUFSIZE) {
                    comThead = 0;
                }

                comTbytes--;
                size--;
            }
        } else {
            /* No more data, just stop Tx (Stop work) */
            UART1->IER &= (~UART_IER_THRE_IEN_Msk);
        }
    }
}

/* Trasnfer data between USBD and UART */
void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check if any data to send to USB & USB is ready to send them out */
    if (comRbytes && (gu32TxSize == 0)) {
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

        NVIC_DisableIRQ(UART1_IRQn);
        comRbytes -= i32Len;
        NVIC_EnableIRQ(UART1_IRQn);
        gu32TxSize = i32Len;
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf, i32Len);
        USBD_SET_PAYLOAD_LEN(EP2, i32Len);
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes)) {
        for (i = 0; i < gu32RxSize; i++) {
            comTbuf[comTtail++] = gpu8RxBuf[i];

            if (comTtail >= TXBUFSIZE) {
                comTtail = 0;
            }
        }

        NVIC_DisableIRQ(UART1_IRQn);
        comTbytes += gu32RxSize;
        NVIC_EnableIRQ(UART1_IRQn);
        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */
        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if (comTbytes) {
        /* Check if Tx is working */
        if ((UART1->IER & UART_IER_THRE_IEN_Msk) == 0) {
            /* Send one bytes out */
            UART1->DATA = comTbuf[comThead++];

            if (comThead >= TXBUFSIZE) {
                comThead = 0;
            }

            NVIC_DisableIRQ(UART1_IRQn);
            comTbytes--;
            NVIC_EnableIRQ(UART1_IRQn);
            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->IER |= UART_IER_THRE_IEN_Msk;
        }
    }
}

void PrepareSendData(uint8_t *pData, uint32_t u32Length)
{
    gu32InSize = u32Length;
    USBD_MemCopy((uint8_t *)gu32InBuf, pData, u32Length);
}

void OnDataReceived(uint8_t *pData, uint32_t u32Length)
{
    uint8_t DataCnt = u32Length - 1;

    switch (pData[0]) {
        case 0:
            memcpy(g_u8BulkOutBuf, pData + 1, DataCnt);
            break;

        // SPI Master
        case MODE_SPI0_MASTER:
            if (g_u16SPI0TxByte == 0) {
                g_u16SPI0TxTail = 0;
                g_u16SPI0TxHead = 0;
            }

            memcpy(&(g_u8SPI0TxBuf[g_u16SPI0TxTail]), pData + 1, DataCnt);
            g_u16SPI0TxTail += DataCnt;
            g_u16SPI0TxByte += DataCnt;
            break;

        case MODE_SPI2_MASTER:
            if (g_u16SPI2TxByte == 0) {
                g_u16SPI2TxTail = 0;
                g_u16SPI2TxHead = 0;
            }

            memcpy(&(g_u8SPI2TxBuf[g_u16SPI2TxTail]), pData + 1, DataCnt);
            g_u16SPI2TxTail += DataCnt;
            g_u16SPI2TxByte += DataCnt;
            break;

        // I2C Master
        case MODE_I2C0_MASTER:
            if (g_u16I2C0TxByte == 0) {
                g_u16I2C0TxTail = 0;
                g_u16I2C0TxHead = 0;
            }

            memcpy(&(g_u8I2C0TxBuf[g_u16I2C0TxTail]), pData + 1, DataCnt);
            g_u16I2C0TxTail += DataCnt;
            g_u16I2C0TxByte += DataCnt;
            break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    SYS_Init();
    UNLOCKREG();
    SYS->BODCR |= SYS_BODCR_BOD_EN_Msk;
    SYS->BODCR = (SYS->BODCR & ~SYS_BODCR_BOD_VL_Msk) | (SYS_BODCR_BOD_VL_2_7V);
    LOCKREG();
    // Solved LD Rom GPIO Pin status
    SYS->IPRSTC2 |=  SYS_IPRSTC2_GPIO_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_GPIO_RST_Msk;
    // VCom Default Setting, 0: Disable, 1: Enable
    g_u8VComEnable = 1;

    if (g_u8VComEnable) {
        UART1_Init();
    }

    //PB14 = output, LED
    PB->PMD = (PB->PMD & (~GPIO_PMD_PMD14_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD14_Pos);
    Timer0_Init();
    /* Initial USBD, set the descriptors and class request to driver */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);
    USBD_SetVendorRequest(USBD_VendorRequest);
    /* Endpoint configuration for CDC class (VCOM) */
    VCOM_Init();
    /* Enable USBD Interrupt */
    NVIC_EnableIRQ(USBD_IRQn);
    /* Start to transfer USB packet */
    USBD_Start();
    Nuc_Bridge_Init();

    while (1) {
        Nuc_Bridge_Main();

        if (g_u8VComEnable) {
            VCOM_TransferData();
        }
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

