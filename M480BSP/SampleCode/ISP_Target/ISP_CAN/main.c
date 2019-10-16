#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "..\hal_sys_init.h"

#define EN_ISP_TIMEOUT (0)

#define V6M_AIRCR_VECTKEY_DATA            0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ             0x00000004UL
#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08
#define CAN_RETRY_COUNTS                  0x1fffffff

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

/* Declare a CAN message structure */
typedef struct {
    uint32_t  Address;
    uint32_t  Data;
} STR_CANMSG_ISP;

STR_CANMSG_T rrMsg;
volatile uint8_t u8CAN_PackageFlag = 0, u8CAN_AckFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if (u32IIDR == 1) {
        CAN_Receive(tCAN, 0, &rrMsg);
        u8CAN_PackageFlag = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN1 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;
    u8IIDRstatus = CAN1->IIDR;

    if (u8IIDRstatus == 0x00008000) {     /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if (CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if (CAN1->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
            u8CAN_AckFlag = 0;
        }
    } else if (u8IIDRstatus != 0) {
        CAN_MsgInterrupt(CAN1, u8IIDRstatus);
        CAN_CLR_INT_PENDING_BIT(CAN1, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
}

uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_CAN1();
}

/*----------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                    */
/*----------------------------------------------------------------------------*/
void CAN_Package_ACK(CAN_T *tCAN)
{
    STR_CANMSG_T tMsg;
    u8CAN_AckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    tMsg.FrameType = CAN_DATA_FRAME;
    tMsg.IdType    = CAN_STD_ID;
    tMsg.Id        = Device0_ISP_ID;
    tMsg.DLC       = CAN_ISP_DtatLength;
    memcpy(&tMsg.Data, &rrMsg.Data, 8);

    if (CAN_Transmit(tCAN, MSG(5), &tMsg) == FALSE) { // Configure Msg RAM and send the Msg in the RAM
        return;
    }

    while (u8CAN_AckFlag);
}

void CAN_Init(void)
{
    CAN_Open(CAN1, CAN_BAUD_RATE, CAN_NORMAL_MODE);
    CAN_EnableInt(CAN1, (CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk));
    NVIC_SetPriority(CAN1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN1_IRQn);
    /* Set CAN reveive message */
    CAN_SetRxMsg(CAN1, MSG(0), CAN_STD_ID, Master_ISP_ID);
}

extern int FMC_Erase_User(unsigned int u32Addr);
extern int FMC_Read_User(unsigned int u32Addr, unsigned int *data);
extern int FMC_Write_User(unsigned int u32Addr, unsigned int u32Data);

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t  Address, Data;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_CFGUEN_Msk);
    /* Init CAN port */
    CAN_Init();
#if (EN_ISP_TIMEOUT)
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1) {
        if (u8CAN_PackageFlag == 1) {
            break;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

#endif

    /* stat update program */
    while (1) {
        if (u8CAN_PackageFlag) {
            u8CAN_PackageFlag = 0;
            Address = inpw(&rrMsg.Data);
            Data = inpw(&rrMsg.Data[4]);

            if (Address == CMD_GET_DEVICEID) {
                outpw(&rrMsg.Data[4], SYS->PDID);
            } else if (Address == CMD_READ_CONFIG) {
                Address = inpw(&rrMsg.Data[4]);
                FMC_Read_User(Address, &Data);
                outpw(&rrMsg.Data[4], Data);
            } else if (Address == CMD_RUN_APROM) {
                break;
            } else {
                if ((Address % FMC_FLASH_PAGE_SIZE) == 0) {
                    FMC_Erase_User(Address);
                }

                FMC_Write_User(Address, Data);         //program ROM
                FMC_Read_User(Address, &Data);
                memcpy(&rrMsg.Data[4], &Data, 4); //update data
            }

            CAN_Package_ACK(CAN1);            //send CAN ISP Package (ACK)
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
