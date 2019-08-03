#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t i2c_rcvbuf[64];
volatile uint8_t bI2cDataReady;

volatile uint8_t g_u8DeviceAddr = 0x60;
volatile uint8_t g_u8SlvDataLen;

__STATIC_INLINE void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status);

void I2C_Init(I2C_T *i2c, uint32_t u32BusClock)
{
    IRQn_Type IRQn;

    /* Reset I2C Controller */
    if ((uint32_t)i2c == I2C0_BASE) {
        SYS->IPRSTC2 |= SYS_IPRSTC2_I2C0_RST_Msk;
        SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
        IRQn = I2C0_IRQn;
    } else if ((uint32_t)i2c == I2C1_BASE) {
        SYS->IPRSTC2 |= SYS_IPRSTC2_I2C1_RST_Msk;
        SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C1_RST_Msk;
        IRQn = I2C1_IRQn;
    }

    /* Enable I2C Controller */
    i2c->I2CON |= I2C_I2CON_ENS1_Msk;
    /* I2C bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    i2c->I2CLK = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    /* Set I2C Slave Addresses */
    i2c->I2CADDR0 = (i2c->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (g_u8DeviceAddr << I2C_I2CADDR_I2CADDR_Pos);
    /* Enable I2C interrupt and set corresponding NVIC bit */
    i2c->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(IRQn);
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(i2c, I2C_I2CON_SI_AA);
}


#define STATUS              I2CSTATUS
#define TOCTL               I2CTOC
#define I2C_TOCTL_TOIF_Msk  I2C_I2CTOC_TIF_Msk
#define I2C_CTL_SI_AA       I2C_I2CON_SI_AA
#define I2C_CTL_SI          I2C_I2CON_SI

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
__STATIC_INLINE void I2C_IRQHandler(I2C_T *i2c)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(i2c);

    if (I2C_GET_TIMEOUT_FLAG(i2c)) {
        /* Clear I2C Timeout Flag */
        i2c->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    } else {
        I2C_SlaveTRx(i2c, u32Status);
    }
}

void I2C0_IRQHandler()
{
    I2C_IRQHandler(I2C0);
}

void I2C1_IRQHandler()
{
    I2C_IRQHandler(I2C1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60) {                    /* Own SLA+W has been receive; ACK has been return */
        bI2cDataReady = 0;
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    } else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        i2c_rcvbuf[g_u8SlvDataLen] = I2C_GET_DATA(i2c);
        g_u8SlvDataLen++;
        g_u8SlvDataLen &= 0x3F;
        bI2cDataReady = (g_u8SlvDataLen == 0);

        if (g_u8SlvDataLen == 0x3F) {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        } else {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    } else if (u32Status == 0xA8) {             /* Own SLA+R has been receive; ACK has been return */
        g_u8SlvDataLen = 0;
        u8data = response_buff[g_u8SlvDataLen];
        I2C_SET_DATA(i2c, u8data);
        g_u8SlvDataLen++;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    } else if (u32Status == 0xB8) {
        u8data = response_buff[g_u8SlvDataLen];
        I2C_SET_DATA(i2c, u8data);
        g_u8SlvDataLen++;
        g_u8SlvDataLen &= 0x3F;

        if (g_u8SlvDataLen == 0x00) {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        } else {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    } else if (u32Status == 0xC8) {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    } else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    } else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        i2c_rcvbuf[g_u8SlvDataLen] = I2C_GET_DATA(i2c);
        g_u8SlvDataLen++;
        bI2cDataReady = (g_u8SlvDataLen == 64);
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    } else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    } else {
        /* TO DO */
        // printf("Status 0x%x is NOT processed\n", u32Status);
    }
}


