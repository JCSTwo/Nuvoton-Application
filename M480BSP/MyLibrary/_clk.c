// Nuvoton MCU Peripheral Access Layer Header File
#include "M480.h"

// User can use the CLOCK setting in Library\StdDriver\src\clk.c
// or use the const CLOCK setting below

__weak uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

__weak  uint32_t CLK_GetHCLKFreq(void)
{
    return FREQ_192MHZ;
}

__weak uint32_t CLK_GetPCLK0Freq()
{
    return FREQ_192MHZ / 2;
}

__weak uint32_t CLK_GetPCLK1Freq()
{
    return FREQ_192MHZ / 2;
}

