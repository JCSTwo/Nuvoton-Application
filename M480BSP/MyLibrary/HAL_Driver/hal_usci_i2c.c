#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Master
void UI2C0_Init(uint32_t Pclk0, uint32_t u32BusClock)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32BusClock);
    /* Get USCI_I2C0 Bus Clock */
    // printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));
    /* Set USCI_I2C0 Slave Addresses */
    // UI2C0->DEVADDR0  = 0x60;
    UI2C0->PROTCTL  = (UI2C0->PROTCTL & ~UI2C_PROTCTL_GCFUNC_Msk) | UI2C_GCMODE_DISABLE;
    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    // NVIC_EnableIRQ(USCI0_IRQn);
}

/*
// please ref. to usci_i2c.c for theses io api
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *data, uint32_t u32wLen);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);
*/

#ifdef __cplusplus
}
#endif
