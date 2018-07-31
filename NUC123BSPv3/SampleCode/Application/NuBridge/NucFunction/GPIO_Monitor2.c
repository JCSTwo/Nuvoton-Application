/**************************************************************************//**
 * @file     GPIO_Monitor.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/04/29 11:27a $
 * @brief    NUC123 Series General Purpose I/O Monitor I2C Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NucInternal.h"
#include "timer.h"

#if(SOFTWARE_I2C_MONITOR == GPIO_MONITOR2)

#define GPIOB15_DOUT        ((__IO int32_t *)0x5000427C)
#define GPIOF2_DOUT         ((__IO int32_t *)0x50004348)

#define CLK_PIN             (*GPIOB15_DOUT)
#define DAT_PIN             (*GPIOF2_DOUT)

#define POLL_CLK    while(CLK_PIN&WDTFlag)//while(CLK_PIN)


volatile uint8_t Data;
volatile uint8_t Ack;
volatile uint8_t Addr;
volatile uint8_t RW;

volatile int WDTFlag = 1;
/**
  * @brief  IRQ Handler for watch dog timer
  * @param  None.
  * @retval None.
  */
void WDT_IRQHandler(void)
{
    UNLOCKREG();
    WDT->WTCR |= WDT_WTCR_WTIF_Msk;
    WDTFlag = 0;
}

/**
  * @brief  Set up the Watch dog timer
  * @param  None.
  * @retval None.
  */
void WDT_Setup(void)
{
    UNLOCKREG();
    WDT->WTCR &= ~WDT_WTCR_WTE_Msk;
    CLK->APBCLK |= CLK_APBCLK_WDT_EN_Msk;
    WDT->WTCR &= ~WDT_WTCR_WTIS_Msk;
    WDT->WTCR |= 0x5ul << WDT_WTCR_WTIS_Pos; //Time out interval : 1.6s
    WDT->WTCR |= WDT_WTCR_WTIE_Msk;
    NVIC_EnableIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0);
}


void GPIO_Monitor_Init(void)
{
    PB->PMD = (PB->PMD & (~GPIO_PMD_PMD15_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD15_Pos);
    PF->PMD = (PF->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD2_Pos);
    GPIO_EnableInt(PF, 2, GPIO_INT_FALLING);            //set falling edge type of interrupt to capture I2C start bit
    WDT_Setup();
    NVIC_EnableIRQ(GPCDF_IRQn);                          //En:Enable NVIC GPCDF_IRQn interrupt
    NVIC_SetPriority(GPCDF_IRQn, 1);
    NVIC_SetPriority(PDMA_IRQn, 2);
    NVIC_SetPriority(USBD_IRQn, 2);
}

// Push one char to Rx Buffer, and update correspoinding pointer
#define PUSH_CHAR(Ch)       {   \
                                g_u8RxBuf[g_u16RxTail] = Ch;    \
                                g_u16RxTail++;  \
                                g_u16RxTail &= RX_BOUND;    \
                            }

/**
  * @brief  GPCDF main interrupt function.
  *             store received data in g_u8RxBuf, update related queue pointer and queue size
  *             received pattern is 2-byte pattern explained as follows
  *                 0x00 0xXX   : Data byte 0xXX    (Low Byte = 0 means data)
  *                 'S'  0x00   : I2C Start Bit     (non-zero Low Byte is used for specific pattern)
  *                 'P'  0x00   : I2C Stop Bit
  *                 'R'  0x00   : I2C Read Bit
  *                 'W'  0x00   : I2C Write Bit
  *                 0x01 0xXX   : I2C 7-Bit Slave Addr.
  *                 0x02 0x01   : I2C NACK Bit
  *                 0x02 0x00   : I2C ACK Bit
  *                 0xFF 0xXX   : Watch dog timer timeout
  * @param  None.
  * @retval None.
  */
void GPCDF_IRQHandler(void)
{
    // Clear Data Pin IRQ
    if (CLK_PIN) {  // Valid Start
        WDT->WTCR |= WDT_WTCR_WTE_Msk;
        WDTFlag = 1;
GET_ADDRESS:
        POLL_CLK;

        while (0 == CLK_PIN);

        Addr = DAT_PIN; // MSB bit, Bit 7
        POLL_CLK {
            if (Addr != DAT_PIN)
            {
                goto ERROR_RET;
            }
        }

        while (0 == CLK_PIN);

        Addr = (Addr << 6 | DAT_PIN << 5);
        POLL_CLK;
        // Start Bit
        PUSH_CHAR('S');
        PUSH_CHAR(0);
        g_u16RxByte += 2;

        while (0 == CLK_PIN);

        Addr |= (DAT_PIN << 4);
        POLL_CLK;

        while (0 == CLK_PIN);

        Addr |= (DAT_PIN << 3);
        POLL_CLK;

        while (0 == CLK_PIN);

        Addr |= (DAT_PIN << 2);
        POLL_CLK;

        while (0 == CLK_PIN);

        Addr |= (DAT_PIN << 1);
        POLL_CLK;

        while (0 == CLK_PIN);

        Addr |= (DAT_PIN << 0);
        POLL_CLK;
        PUSH_CHAR(1);
        PUSH_CHAR(Addr);
        g_u16RxByte += 2;

        while (0 == CLK_PIN);

        RW = DAT_PIN;   // R/W bit
        POLL_CLK;

        if (RW) {
            PUSH_CHAR('R');
        } else {
            PUSH_CHAR('W');
        }

        PUSH_CHAR(0);
        g_u16RxByte += 2;

        while (0 == CLK_PIN);

        Ack = DAT_PIN;      // Ack bit
        PUSH_CHAR(2);
        PUSH_CHAR(Ack);
        g_u16RxByte += 2;
        POLL_CLK;
WHILE1:

        while (0 == CLK_PIN);

        Data = DAT_PIN; // MSB bit, Bit 7
        POLL_CLK {
            if (Data != DAT_PIN)
            {
                if (Data) { // H -> L, Repeat Start
                    goto GET_ADDRESS;
                } else {
                    goto STOP_P;
                }
            }
        }

        while (0 == CLK_PIN);

        Data = (Data << 7 | DAT_PIN << 6);
        POLL_CLK;

        while (0 == CLK_PIN);

        Data |= (DAT_PIN << 5);
        POLL_CLK;

        while (0 == CLK_PIN);

        Data |= (DAT_PIN << 4);
        POLL_CLK;

        while (0 == CLK_PIN);

        Data |= (DAT_PIN << 3);
        POLL_CLK;

        while (0 == CLK_PIN);

        Data |= (DAT_PIN << 2);
        POLL_CLK;

        while (0 == CLK_PIN);

        Data |= (DAT_PIN << 1);
        POLL_CLK;

        while (0 == CLK_PIN);

        Data |= DAT_PIN;
        POLL_CLK;
        PUSH_CHAR(0);
        PUSH_CHAR(Data);
        g_u16RxByte += 2;

        while (0 == CLK_PIN);

        Ack = DAT_PIN;      // Ack bit
        PUSH_CHAR(2);
        PUSH_CHAR(Ack);
        g_u16RxByte += 2;
        POLL_CLK;

        if (WDTFlag) {
            goto WHILE1;
        } else {
            goto ERROR_RET;
        }

STOP_P:
        WDT->WTCR &= ~WDT_WTCR_WTE_Msk;
        WDT->WTCR |= WDT_WTCR_WTR_Msk;
        // Stot Bit
        PUSH_CHAR('P');
        PUSH_CHAR(0);
        g_u16RxByte += 2;
    }

ERROR_RET:

    if (WDTFlag == 0) {
        PUSH_CHAR(0xFF);
        PUSH_CHAR(0);
        g_u16RxByte += 2;
    }

    PF->ISRC = BIT2;
}

/**
  * @brief  Disable GPIO interrupt function
  * @param  None.
  * @retval None.
  */
void GPIO_DeInit(void)
{
    NVIC_DisableIRQ(GPCDF_IRQn);
    NVIC_SetPriority(PDMA_IRQn, 0);
    NVIC_SetPriority(USBD_IRQn, 0);
    // DrvWDT_DeInit();
    UNLOCKREG();
    WDT->WTCR &= ~WDT_WTCR_WTE_Msk;  // WDT_DISABLE_COUNTING();
    WDT->WTCR &= ~WDT_WTCR_WTIE_Msk; // WDT_DISABLE_INT();
    NVIC_DisableIRQ(WDT_IRQn);
    CLK->APBCLK &= ~CLK_APBCLK_WDT_EN_Msk;
    SYS_LockReg();
}

#endif  // #if(SOFTWARE_I2C_MONITOR == GPIO_MONITOR2)
