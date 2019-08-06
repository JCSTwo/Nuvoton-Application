#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bI2cDataReady;
extern uint8_t i2c_rcvbuf[];

/*-------------------------------------------------------------*/
void I2C_Init(I2C_T *i2c, uint32_t u32BusClock);

#endif  /* __I2C_TRANS_H__ */
