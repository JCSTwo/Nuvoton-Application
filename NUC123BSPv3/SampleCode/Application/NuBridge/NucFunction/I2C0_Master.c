/**************************************************************************//**
 * @file     I2C0_Master.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 13/11/11 11:27a $
 * @brief    NUC123 Series I2C0 Master Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include "NucInternal.h"

uint8_t g_FindSlave = 0;

#define CHECK_I2C_TIMEOUT(a)    I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;               \
                                while ((I2C0->I2CON & I2C_I2CON_SI_Msk) == a) {   \
                                    if(I2C0->I2CTOC & I2C_I2CTOC_TIF_Msk) {       \
                                        I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;       \
                                        timeOutFlag = 1;                    \
                                        goto I2C_ERROR_RET;                 \
                                    }                                       \
                                };                                          \

/**
  * @brief  Initial I2C0 to be I2C master with the bus clock frequency set as 200kHz.
  * @param  None.
  * @retval None.
  */
void I2C0_Master_Init(void)
{
    /* set I2CLK = 200KHz */
    I2C0_Master_ReInit(0x02);
}

/**
  * @brief  Initial I2C0 to be I2C master.
  * @param  setting: the setting of I2C bus clock frequency
                     -1: 100 kHz
                     -2: 200 kHz
                     -3: 300 kHz
  * @retval None.
  */
void I2C0_Master_ReInit(uint8_t setting)
{
    uint32_t divider, u32BusClock;
    SYS->GPF_MFP |= (SYS_GPF_MFP_PF2_I2C0_SDA | SYS_GPF_MFP_PF3_I2C0_SCL);
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PF2_Msk | SYS_ALT_MFP1_PF3_Msk);
    SYS->ALT_MFP1 |= (SYS_ALT_MFP1_PF2_I2C0_SDA | SYS_ALT_MFP1_PF3_I2C0_SCL);

    switch (setting & 0x03) {
        case 1:
            u32BusClock = 100000;
            break;

        case 2:
            u32BusClock = 200000;
            break;

        case 3:
            u32BusClock = 400000;
            break;
    }

    divider = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1);	/* Compute proper divider for I2C clock */
    {
        SYS->IPRSTC2 |= SYS_IPRSTC2_I2C0_RST_Msk;
        SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;
        CLK->APBCLK |= CLK_APBCLK_I2C0_EN_Msk;
    }
    I2C0->I2CLK |= divider;
    I2C0->I2CTOC |= (I2C_I2CTOC_ENTI_Msk | I2C_I2CTOC_DIV4_Msk);
    I2C0->I2CON |= (I2C_I2CON_EI_Msk | I2C_I2CON_ENS1_Msk);
    g_u8ParamSet[MODE_I2C0_MASTER] = setting;
}


/**
  * @brief  Disable I2C function and clock source. Set PF.2 and PF.3 as GPIO function
  * @param  None.
  * @retval None.
  */
void I2C0_Master_DeInit(void)
{
    // DrvI2C_DeInit(I2C0);
    I2C0->I2CTOC &= ~I2C_I2CTOC_ENTI_Msk;
    I2C0->I2CON &= ~(I2C_I2CON_EI_Msk | I2C_I2CON_ENS1_Msk);
    NVIC_DisableIRQ(I2C0_IRQn);
    outpw(&SYS->GPF_MFP, inpw(&SYS->GPF_MFP) & ~(0x3 << 2));
    outpw(&SYS->ALT_MFP1, (inpw(&SYS->ALT_MFP1) & ~(0xF << 24)) | (0xa << 24));
    g_u8ParamSet[MODE_I2C0_MASTER] = 0x00;
}

static uint8_t ErrCmd = 0;

/**
  * @brief  Reset the variable ErrCmd to be 0.
  * @param  None.
  * @retval None.
  */
void ClearErrCmdCnt(void)
{
    ErrCmd = 0;
}


/**
  * @brief  Parse the command script of g_u8I2C0TxBuf[] and trigger the relative communication.
  * @param  None.
  * @retval command remains not executed.
  *
  * For example:
  * Byte 0: Total Command Count
  * Byte 1: Command, 'R' ==> Read, 'W' ==> Write
  * Byte 2: I2C Slave Address(7 bit addressing)
  * Byte 3: Data Count
  * Byte 4: Data (for Write Command Only)
  *
  * 03                           // Byte 0: Total Command Count = 3
  * 'R' 4C 04                    // Byte 1 ~ 3: Read 4 Bytes from 0x4C
  * 'W' 55 05 AA BB CC DD EE     // Byte 4 ~ 11: Write 5 Bytes data (AA BB CC DD EE) to 0x55
  * 'R' 12 06                    // Byte 12 ~ 14: Read 6 Bytes from 0x12
  */
uint8_t I2C0_Run_Script(int *CmdCnt)
{
    uint8_t /*cmd_cnt,*/ cmd, addr, data_cnt, timeOutFlag = 0, errStatus = 0;
    // Return data 5-Bytes header (0: Error Status, 1: Error Cmd Index, 2: Error Byte Index, 3,4: Recevied Data Count)
    uint16_t RetHeader;// = g_u16I2C0RxTail;
    uint16_t Retcount = 0;
    uint8_t ErrByte = 0;
    RetHeader = g_u16I2C0RxTail;
    // Return data start address
    g_u16I2C0RxTail += 5;
    g_u16I2C0RxTail &= RX_BOUND_PART;
    g_u16I2C0RxByte += 5;

    while (*CmdCnt) {
        cmd = g_u8I2C0TxBuf[g_u16I2C0TxHead++];
        addr = g_u8I2C0TxBuf[g_u16I2C0TxHead++];
        data_cnt = g_u8I2C0TxBuf[g_u16I2C0TxHead++];

        if (cmd == 'R') {
            timeOutFlag = 0;
            // Header Infomation Byte
            ErrByte = 0;
            //send i2c start and check if i2c timeout happened
            I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STA_Msk | I2C_I2CON_SI_Msk;
            CHECK_I2C_TIMEOUT(0);

            if (I2C0->I2CSTATUS != 0x08) {
                errStatus = I2C0->I2CSTATUS;
                goto I2C_ERROR_RET;
            }

            //send read command and check if i2c timeout happened
            I2C0->I2CDAT = (addr << 1) | 0x01;
            I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_SI_Msk | I2C_I2CON_AA_Msk;
            CHECK_I2C_TIMEOUT(0);

            if (I2C0->I2CSTATUS != 0x40) {
                //SLA+R has been transmitted and NOT ACK has been received.
                //send i2c stop and check if i2c timeout happened
                errStatus = I2C0->I2CSTATUS;
                I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
                CHECK_I2C_TIMEOUT(1);
                goto I2C_ERROR_RET;
            }

            while (data_cnt) {
                // read #data_cnt bytes data from addr
                I2C0->I2CDAT = 0xFF;

                if (data_cnt > 1) {
                    I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_SI_Msk | I2C_I2CON_AA_Msk;
                } else {
                    I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_SI_Msk;
                }

                CHECK_I2C_TIMEOUT(0);

                if ((I2C0->I2CSTATUS != 0x50) && ((I2C0->I2CSTATUS != 0x58))) {
                    errStatus = I2C0->I2CSTATUS;
                    I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
                    CHECK_I2C_TIMEOUT(1);
                    goto I2C_ERROR_RET;
                }

                g_u8I2C0RxBuf[g_u16I2C0RxTail++] = I2C0->I2CDAT;
                g_u16I2C0RxTail &= RX_BOUND_PART;
                g_u16I2C0RxByte ++;
                data_cnt --;
                // Data Header
                ErrByte++;
                Retcount++;
            }

            //send i2c stop and check if i2c timeout happened
            I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
            CHECK_I2C_TIMEOUT(1);
        } else if (cmd == 'W') {
            timeOutFlag = 0;
            // Header Infomation Byte
            ErrByte = 0;
            //send i2c start and check if i2c timeout happened
            I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STA_Msk | I2C_I2CON_SI_Msk;
            CHECK_I2C_TIMEOUT(0);

            if (I2C0->I2CSTATUS != 0x08) {
                errStatus = I2C0->I2CSTATUS;
                goto I2C_ERROR_RET;
            }

            //send write command and check if i2c timeout happened
            I2C0->I2CDAT = addr << 1;
            I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_SI_Msk;
            CHECK_I2C_TIMEOUT(0);

            if (I2C0->I2CSTATUS != 0x18) {
                //SLA+W has been transmitted and NOT ACK has been received.
                //send i2c stop and check if i2c timeout happened
                errStatus = I2C0->I2CSTATUS;
                I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
                CHECK_I2C_TIMEOUT(1);
                goto I2C_ERROR_RET;
            }

            while (data_cnt) {
                // write #data_cnt bytes data to addr
                I2C0->I2CDAT = g_u8I2C0TxBuf[g_u16I2C0TxHead];
                I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_SI_Msk | I2C_I2CON_AA_Msk;
                CHECK_I2C_TIMEOUT(0);

                // return cmd_cnt if write failed
                if (I2C0->I2CSTATUS != 0x28) {
                    //Data has been transmitted and NOT ACK has been received.
                    //send i2c stop and check if i2c timeout happened
                    errStatus = I2C0->I2CSTATUS;
                    I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
                    CHECK_I2C_TIMEOUT(1);
                    goto I2C_ERROR_RET;
                }

                g_u16I2C0TxHead++;
                ErrByte ++;
                data_cnt --;
            }

            //send i2c stop and check if i2c timeout happened
            I2C0->I2CON = (I2C0->I2CON & ~0x3C) | I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
            CHECK_I2C_TIMEOUT(1);
        } else {
            return *CmdCnt;
        }

        // parsing next command
        ErrCmd++;
        *CmdCnt = (*CmdCnt) - 1;
    }

//I2C_SUCCESS_RET:
    g_u8I2C0RxBuf[RetHeader] = 0x01;    // No Error
    RetHeader += 3;
    RetHeader &= RX_BOUND_PART;
    g_u8I2C0RxBuf[RetHeader++] = (Retcount & 0x00FF);   // Received Data Count
    RetHeader &= RX_BOUND_PART;
    g_u8I2C0RxBuf[RetHeader++] = ((Retcount & 0xFF00) >> 8);    // Received Data Count
    RetHeader &= RX_BOUND_PART;
    return 0;
I2C_ERROR_RET:

    // Return data 5-Bytes header (0: Error Status, 1: Error Cmd Index, 2: Error Byte Index, 3,4: Recevied Data Count)
    if (timeOutFlag) {
        g_u8I2C0RxBuf[RetHeader++] = 0xFF;
    } else {
        g_u8I2C0RxBuf[RetHeader++] = errStatus;    // Error Status
    }

    RetHeader &= RX_BOUND_PART;
    g_u8I2C0RxBuf[RetHeader++] = ErrCmd;    // Error Command Index
    RetHeader &= RX_BOUND_PART;
    g_u8I2C0RxBuf[RetHeader++] = ErrByte;   // Error Data Byte Index
    RetHeader &= RX_BOUND_PART;
    g_u8I2C0RxBuf[RetHeader++] = (Retcount & 0x00FF);   // Received Data Count
    RetHeader &= RX_BOUND_PART;
    g_u8I2C0RxBuf[RetHeader++] = ((Retcount & 0xFF00) >> 8);    // Received Data Count
    RetHeader &= RX_BOUND_PART;

    if (cmd == 'W') {
        while (data_cnt) {
            g_u16I2C0TxHead++;
            data_cnt--;
        }
    }

    ErrCmd++;
    *CmdCnt = (*CmdCnt) - 1;
    return (*CmdCnt);
}



/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_Master_MainProcess(void)
{
    static int CmdCnt;

    if (g_u16I2C0TxByte) {
        ClearErrCmdCnt();
        I2C0_Master_ReInit(g_u8ParamSet[MODE_I2C0_MASTER]);
        CmdCnt = g_u8I2C0TxBuf[g_u16I2C0TxHead++];

        while (I2C0_Run_Script(&CmdCnt)) {};

        g_u16I2C0TxByte = 0;

        g_u16I2C0TxHead = 0;
    }

    if (g_FindSlave) {
//        I2C_Find_Slave();
        g_FindSlave = 0;
    }
}





