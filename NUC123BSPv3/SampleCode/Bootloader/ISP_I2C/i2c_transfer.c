#include <stdio.h>
#include <string.h>
#include "i2c_transfer.h"
#include "targetdev.h"

__align(4) uint8_t I2C_rcvbuf[MAX_PKT_SIZE] = {0};
__align(4) uint8_t I2C_sendbuf[MAX_PKT_SIZE];
__align(4) uint8_t I2C_aprom_buf[PAGE_SIZE];

uint8_t volatile bI2CDataReady = 0;
uint8_t volatile bufhead = 0;

volatile uint8_t I2C_Read = FALSE;
uint8_t Send_Cnt = 0;
uint8_t Rcv_Cnt = 0;
volatile uint8_t EndFlag = 0;

uint8_t I2C_Slave_Addr = 0x36;

void I2C_SlaveTRx(uint32_t u32Status);
/*---------------------------------------------------------------------------------------------------------*/
/* INTSTS to handle I2C Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0)) {
        /* Clear I2C0 Timeout Flag */
        I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    } else {
        I2C_SlaveTRx(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    /* SLA + R */
    if ((u32Status == 0xA8) || (u32Status == 0xB0)) {	/* Own SLA+R has been received and ACK/NACK has been return */
        I2C_Read = FALSE;
        Send_Cnt = 0;
        I2C0->I2CDAT = I2C_sendbuf[Send_Cnt++];
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    } else if (u32Status == 0xB8) {										/* Data byte has been transmitted and ACK has been received */
        if (Send_Cnt < MAX_PKT_SIZE) {
            I2C0->I2CDAT = I2C_sendbuf[Send_Cnt++];
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
            EndFlag = 1;
        }
    } else if (u32Status == 0xC0) {    /* Data byte or last data in I2CDAT has been transmitted Not ACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
        EndFlag = 1;
    }
    /* SLA + W */
    else if ((u32Status == 0x60) || (u32Status == 0x68)) {	/* Own SLA+W has been received and ACK has been return */
        I2C_Read = TRUE;
        Rcv_Cnt = 0;
        bI2CDataReady = TRUE;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    } else if (u32Status == 0x80) {													/* Data byte has been received and ACK has been received */
        if (Rcv_Cnt < (MAX_PKT_SIZE - 1)) {
            I2C_rcvbuf[Rcv_Cnt++] = I2C0->I2CDAT;
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
        } else {
            I2C_rcvbuf[Rcv_Cnt++] = I2C0->I2CDAT;
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
            EndFlag = 1;
        }
    } else if (u32Status == 0xA0) {					/* A STOP or repeated START has been received while still addressed as SLA/REC */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    } else {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);

        if (I2C_Read) {
            bI2CDataReady = FALSE;
        }

        EndFlag = 1;
    }
}

void I2C_Init()
{
    /* Enable I2C controller */
    CLK->APBCLK |= CLK_APBCLK_I2C0_EN_Msk;
    /* Set GPF multi-function pins for I2C0 SDA and SCL */
    SYS->GPF_MFP |= (SYS_GPF_MFP_PF2_I2C0_SDA | SYS_GPF_MFP_PF3_I2C0_SCL);
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PF2_Msk | SYS_ALT_MFP1_PF3_Msk);
    SYS->ALT_MFP1 |= (SYS_ALT_MFP1_PF2_I2C0_SDA | SYS_ALT_MFP1_PF3_I2C0_SCL);
    /* Reset I2C */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
    /* Enable I2C Controller */
    I2C0->I2CON |= I2C_I2CON_ENS1_Msk;
    /* I2C bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    I2C0->I2CLK = (uint32_t)(((SystemCoreClock * 10) / (100000 * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    /* Set I2C0 Slave Addresses */
    /* Slave Address : 0x36 */
    I2C0->I2CADDR0 = (I2C0->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (I2C_Slave_Addr << I2C_I2CADDR_I2CADDR_Pos);
    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C_SlaveRcvSendData()
{
    EndFlag = 0;
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);

    while ((DetectPin == 0) && (EndFlag == 0));
}
