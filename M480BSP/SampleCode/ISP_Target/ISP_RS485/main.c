#include <stdio.h>
#include "targetdev.h"
#include "uart_transfer.h"


__weak uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SYS_Init_RS485();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 */
    UART_Init(UART1);
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
#if (EN_ISP_TIMEOUT)
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1) {
        if ((bufhead >= 4) || bUartDataReady) {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT) {
                goto _ISP;
            } else {
                bUartDataReady = 0;
                bufhead = 0;
            }
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

#endif
_ISP:

    while (1) {
        if (bUartDataReady) {
            bUartDataReady = 0;
            ParseCmd(uart_rcvbuf, 64);
            NVIC_DisableIRQ(UART1_IRQn);
            nRTSPin = TRANSMIT_MODE;
            PutString(UART1);

            while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

            nRTSPin = REVEIVE_MODE;
            NVIC_EnableIRQ(UART1_IRQn);
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
