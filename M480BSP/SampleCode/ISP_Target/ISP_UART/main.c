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
    SYS_Init_UART0(); // For NuMaker-PFM-M487
    SYS_Init_UART2(); // For Nu-Link2 PIN 1, 2 = UART2_TXD, UART2_RXD
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 */
    UART_Init(UART0);
    UART_Init(UART2);
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
#if (EN_ISP_TIMEOUT)
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

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
            ParseCmd(uart_rcvbuf, 64);
            PutString((UART_T *)bUartDataReady);
            bUartDataReady = 0;
        }
    }

_APROM:
    RUN_APROM();
}
