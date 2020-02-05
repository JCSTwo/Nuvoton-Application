/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 16/06/17 4:53p $
 * @brief
 *           Demonstrate initial I2C Master & I2C Slave, and show a Master how to acces Slave
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "..\targetdev.h"

#define printf VCOM_printf

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8SlvDataLen;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C0->I2CSTATUS;

    if (I2C0->I2CTOC & I2C_I2CTOC_TIF_Msk) {
        /* Clear I2C0 Timeout Flag */
        I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    } else {
        if (s_I2C0HandlerFn != NULL) {
            s_I2C0HandlerFn(u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C1->I2CSTATUS;

    if (I2C1->I2CTOC & I2C_I2CTOC_TIF_Msk) {
        /* Clear I2C1 Timeout Flag */
        I2C1->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    } else {
        if (s_I2C1HandlerFn != NULL) {
            s_I2C1HandlerFn(u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Rx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C0->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C0->I2CDAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8MstDataLen != 2) {
            I2C0->I2CDAT = g_au8MstTxData[g_u8MstDataLen++];
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
        }
    } else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C0->I2CDAT = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    } else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    } else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
        g_u8MstRxData = I2C0->I2CDAT;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
        g_u8MstEndFlag = 1;
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\r\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Tx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted */
        I2C0->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C0->I2CDAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8MstDataLen != 3) {
            I2C0->I2CDAT = g_au8MstTxData[g_u8MstDataLen++];
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            g_u8MstEndFlag = 1;
        }
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\r\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 TRx Callback Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60) {                    /* Own SLA+W has been receive; ACK has been return */
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C1);

        if (g_u8SlvDataLen < 2) {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8data;
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        } else {
            g_au8SlvData[slave_buff_addr++] = u8data;

            if (slave_buff_addr == 256) {
                slave_buff_addr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else if (u32Status == 0xA8) {             /* Own SLA+R has been receive; ACK has been return */
        I2C1->I2CDAT = g_au8SlvData[slave_buff_addr];
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else if (u32Status == 0xB8) {             /* Data byte in I2CDAT has been transmitted ACK has been received */
        I2C_SET_DATA(I2C1, g_au8SlvData[slave_buff_addr++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\r\n", u32Status);
    }
}

void SYS_Init(void)
{
    SYS_Init_72MHZ_USBD();
    SYS_Init_I2C0();
    SYS_Init_I2C1();
}

void I2C0_Init(void)
{
    uint32_t u32BusClock;
    /* Reset I2C0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
    /* Enable I2C0 Controller */
    I2C0->I2CON |= I2C_I2CON_ENS1_Msk;
    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->I2CLK = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\r\n", (SystemCoreClock / (((I2C0->I2CLK) + 1) << 2)));
    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->I2CADDR0 = (I2C0->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x15 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->I2CADDR1 = (I2C0->I2CADDR1 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x35 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->I2CADDR2 = (I2C0->I2CADDR2 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x55 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->I2CADDR3 = (I2C0->I2CADDR3 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x75 << I2C_I2CADDR_I2CADDR_Pos);
    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    uint32_t u32BusClock;
    /* Reset I2C1 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C1_RST_Msk;
    /* Enable I2C1 Controller */
    I2C1->I2CON |= I2C_I2CON_ENS1_Msk;
    /* I2C1 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C1->I2CLK = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\r\n", (SystemCoreClock / (((I2C1->I2CLK) + 1) << 2)));
    /* Set I2C1 4 Slave Addresses */
    /* Slave Address : 0x16 */
    I2C1->I2CADDR0 = (I2C1->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x16 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x36 */
    I2C1->I2CADDR1 = (I2C1->I2CADDR1 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x36 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x56 */
    I2C1->I2CADDR2 = (I2C1->I2CADDR2 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x56 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x76 */
    I2C1->I2CADDR3 = (I2C1->I2CADDR3 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x76 << I2C_I2CADDR_I2CADDR_Pos);
    /* Set I2C1 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x04 */
    I2C1->I2CADM0 = (I2C1->I2CADM0 & ~I2C_I2CADM_I2CADM_Msk) | (0x04 << I2C_I2CADM_I2CADM_Pos);
    /* Slave Address Mask Bits: 0x02 */
    I2C1->I2CADM1 = (I2C1->I2CADM1 & ~I2C_I2CADM_I2CADM_Msk) | (0x02 << I2C_I2CADM_I2CADM_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C1->I2CADM2 = (I2C1->I2CADM2 & ~I2C_I2CADM_I2CADM_Msk) | (0x04 << I2C_I2CADM_I2CADM_Pos);
    /* Slave Address Mask Bits: 0x02 */
    I2C1->I2CADM3 = (I2C1->I2CADM3 & ~I2C_I2CADM_I2CADM_Msk) | (0x02 << I2C_I2CADM_I2CADM_Pos);
    /* Enable I2C1 interrupt and set corresponding NVIC bit */
    I2C1->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->I2CON &= ~I2C_I2CON_EI_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);
    /* Disable I2C0 and close I2C0 clock */
    I2C0->I2CON &= ~I2C_I2CON_ENS1_Msk;
    CLK->APBCLK &= ~CLK_APBCLK_I2C0_EN_Msk;
}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C1->I2CON &= ~I2C_I2CON_EI_Msk;
    NVIC_DisableIRQ(I2C1_IRQn);
    /* Disable I2C1 and close I2C1 clock */
    I2C1->I2CON &= ~I2C_I2CON_ENS1_Msk;
    CLK->APBCLK &= ~CLK_APBCLK_I2C1_EN_Msk;
}

int32_t Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;
    g_u8DeviceAddr = slvaddr;

    for (i = 0; i < 0x100; i++) {
        g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
        g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);
        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;
        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);

        /* Wait I2C Tx Finish */
        while (g_u8MstEndFlag == 0);

        g_u8MstEndFlag = 0;
        /* I2C function to read data from slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
        g_u8MstDataLen = 0;
        g_u8DeviceAddr = slvaddr;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);

        /* Wait I2C Rx Finish */
        while (g_u8MstEndFlag == 0);

        /* Compare data */
        if (g_u8MstRxData != g_au8MstTxData[2]) {
            printf("I2C Byte Write/Read Failed, Data 0x%x\r\n", g_u8MstRxData);
            return -1;
        }
    }

    printf("Master Access Slave (0x%X) Test OK\r\n", slvaddr);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;
    volatile uint8_t j = 0;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    VCOM_Initialize();
    VCOM_getchar();
    printf("\n");
    printf("\r +-------------------------------------------------------+\n");
    printf("\r | I2C Driver Sample Code(Slave) for access Slave        |\n");
    printf("\r |                                                       |\n");
    printf("\r | I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
    printf("\r +-------------------------------------------------------+\n");
    printf("\r Configure I2C0 as a Master, I2C1 as a Mlave.\n");
    printf("\r The I/O connection I2C0 and I2C1:\n");
    printf("\r I2C0_SDA (Pin4), I2C0_SCL (Pin5)\n");
    printf("\r I2C1_SDA (Pin2), I2C1_SCL (Pin1)\n");
    printf("\r GND      (Pin3), DVDD     (Pin6)\n");
    /* Init I2C0, I2C1 */
    I2C0_Init();
    I2C1_Init();
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI_AA);

    for (i = 0; i < 0x100; i++) {
        g_au8SlvData[i] = 0;
    }

_main_loop:
    /* I2C function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_SlaveTRx;
    printf("\r\n");
    printf("[%03d] I2C Slave Mode is Running.\r\n", j++);
    /* Access Slave with no address mask */
    printf("\r\n");
    printf(" == No Mask Address ==\r\n");
    Read_Write_SLAVE(0x16);
    Read_Write_SLAVE(0x36);
    Read_Write_SLAVE(0x56);
    Read_Write_SLAVE(0x76);
    printf("SLAVE Address test OK.\r\n");
    /* Access Slave with address mask */
    printf("\r\n");
    printf(" == Mask Address ==\r\n");
    Read_Write_SLAVE(0x16 & ~0x04);
    Read_Write_SLAVE(0x36 & ~0x02);
    Read_Write_SLAVE(0x56 & ~0x04);
    Read_Write_SLAVE(0x76 & ~0x02);
    printf("SLAVE Address Mask test OK.\r\n");
    s_I2C0HandlerFn = NULL;
    VCOM_getchar();
    goto _main_loop;
    /* Close I2C0, I2C1 */
    I2C0_Close();
    I2C1_Close();

    while (1);
}
