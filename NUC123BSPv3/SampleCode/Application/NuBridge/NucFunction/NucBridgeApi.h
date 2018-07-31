
#ifndef __NUC_BRIDGE_API__
#define __NUC_BRIDGE_API__

#define         WM_LED                  0x40
#define         WM_SWITCH_MODE          0x42
#define         WM_SET_RUN              0x43
//#define           WM_SET_PAUSE            0x43
#define         WM_SET_DISABLE          0x44

#define         WM_REINIT_SPI0          0x46
#define         WM_REINIT_SPIM          0x47
#define         WM_REINIT_SPI2          0x48
#define         WM_RESET                0x49
#define         WM_REINIT_I2C0          0x4A
/* SUB Command for WM_SET_MODE */
//
#define         MODE_OFF                0x00
#define         MODE_RESET              0x01
#define         MODE_INIT               0x02
#define         MODE_PAUSE              0x03
#define         MODE_RUN                0x04
#define         MODE_DISABLE            0x05


#define         MODE_I2C_MONITOR        0x00
#define         MODE_SPI0_MASTER        0x01
#define         MODE_SPI_MONITOR        0x02
#define         MODE_I2C0_MASTER        0x03
#define         MODE_SPI2_MASTER        0x04
#define         MODE_VCOM_UART          0x05
#define         MODE_CS_TAG             0xFF





#endif  /* __NUC_BRIDGE_API__ */

