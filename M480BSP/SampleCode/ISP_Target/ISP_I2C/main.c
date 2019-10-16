#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

__weak uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_I2C1(); // For ISP Target Only, PIN 7, 8 = I2C1_SDA, I2C1_SCL
    SYS_Init_I2C2(); // For NuMaker-PFM-M487, PIN 94, 95 = I2C2_SCL, I2C2_SDA
}

int main(void)
{
    uint32_t cmd_buff[16];
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);
    I2C_Init(I2C1, 100000);
    I2C_Init(I2C2, 100000);
#if (EN_ISP_TIMEOUT)
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1) {
        if (bI2cDataReady == 1) {
            goto _ISP;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

#endif
_ISP:

    while (1) {
        if (bI2cDataReady == 1) {
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

_APROM:
    RUN_APROM();
}
