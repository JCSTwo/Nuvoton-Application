#include <stdio.h>
#include "NuMicro.h"
#include "isp_bridge.h"
#include <string.h>
#include "hal_api.h"

#include "ISP_Driver\isp_parser.h"

#ifdef __cplusplus
extern "C"
{
#endif


static volatile uint8_t g_lib_CmdFromTool = 0ul;
static volatile uint8_t g_lib_CmdToTarget = 0ul;
static volatile uint8_t g_lib_IspModule = 0ul; // 3: SPI, 4: I2C, 5: RS485, 6: CAN

static uint8_t g_lib_CmdBuf[64] __attribute__((aligned(4)));
static uint8_t g_lib_AckBuf[64] __attribute__((aligned(4)));

void ISP_Bridge_UsbDataIn()
{
    _EP_HID_IN_Handler(EP_HID_IN, g_lib_AckBuf, 64);
}

void ISP_Bridge_UsbDataOut()
{
    _EP_HID_OUT_Handler(EP_HID_OUT, g_lib_CmdBuf);
    g_lib_CmdFromTool = TRUE;
}

void ISP_Bridge_Init(void)
{
    uint32_t Pclk0 = FREQ_192MHZ / 2;
    // System Init (SYS, CLK, GPIO and multi-function setting)
    SYS_Init_SPI1();
    SYS_Init_UI2C0();
    SYS_Init_RS485();
    SYS_Init_CAN1();
    // Peripheral Init
    SPI1_Init(Pclk0);
    UI2C0_Init(Pclk0, 100000);
    // !!!! Don't enable I2C protocol. It will affect the I2C bus.
    UI2C0->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
    RS485_Init();
    CAN_Init();
    // Init Ready (turn on all LED)
    SYS_Init_LED(0, 0, 0, 0);
}

void ISP_Bridge_Main(void)
{
    static uint32_t cks;
    static uint8_t cmd;

    // forward isp command to target device (I2C, SPI, RS485 or CAN)
    if (g_lib_CmdFromTool == TRUE) {
        g_lib_CmdFromTool = FALSE;
        cmd = g_lib_CmdBuf[0];
        g_lib_IspModule = g_lib_CmdBuf[1];
        g_lib_CmdBuf[1] = 0;
        // Checksum is the only way to verify correctness of ACK according to spec.
        cks = ISP_Checksum(g_lib_CmdBuf, 64);

        switch (g_lib_IspModule) {
            case 3:
                LED_Set(1, 0, 1, 1);
                // add specific pattern "0x53504900" to word0 of g_lib_CmdBuf
                SPI1_Write((uint32_t *)g_lib_CmdBuf, 16);
                g_lib_CmdToTarget = 1;
                break;

            case 4:
                LED_Set(0, 1, 1, 1);
                UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;

                if (64 == UI2C_WriteMultiBytes(UI2C0, 0x60, g_lib_CmdBuf, 64)) {
                    g_lib_CmdToTarget = 1;
                } else {
                }

                UI2C0->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                break;

            case 5:
                LED_Set(1, 1, 0, 1);
                RS485_WriteMultiBytes(g_lib_CmdBuf);
                g_lib_CmdToTarget = 0;
                return;

            case 6:
                LED_Set(1, 1, 1, 0);
                CAN_Package_Tx(CAN1, g_lib_CmdBuf + 2);
                g_lib_CmdToTarget = 0;
                return;

            default:
                LED_Set(1, 1, 1, 1);
                g_lib_CmdToTarget = 0;
                return;
        }

        // There's no ACK for these commands according to spec. (RESET command)
        // #define CMD_RUN_APROM         0x000000AB
        // #define CMD_RUN_LDROM         0x000000AC
        // #define CMD_RESET             0x000000AD
        if (g_lib_CmdToTarget && ((cmd == 0xAB) || (cmd == 0xAC) || (cmd == 0xAD))) {
            g_lib_CmdToTarget = 0;
        }

        return;
    }

    // checking response for CAN interface, this flag is set in CAN1_IRQHandler
    if (u8CAN_PackageFlag && (6 == g_lib_IspModule)) {
        u8CAN_PackageFlag = 0;
        _EP_HID_IN_Handler(EP_HID_IN, &rrMsg.Data[0], 64);
    }

    // polling response for SPI & I2C interface
    if (g_lib_CmdToTarget && (g_lib_CmdFromTool == FALSE)) {
        uint32_t delay = ISP_ResponseDelay(cmd);
        SysTick->LOAD = delay * _CyclesPerUs;
        SysTick->VAL  = 0x0UL;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        while (g_lib_CmdFromTool == FALSE) {
            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                uint32_t u32rxLen = 64;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;

                switch (g_lib_IspModule) {
                    case 3:
                        SPI1_Read((uint32_t *)g_lib_AckBuf, 16);
                        break;

                    case 4:
                    default:
                        UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
                        u32rxLen = UI2C_ReadMultiBytes(UI2C0, 0x60, g_lib_AckBuf, 64);
                        UI2C0->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                        break;
                }

                if ((u32rxLen == 64) && cks == inpw(g_lib_AckBuf)) {
                    ISP_Bridge_UsbDataIn();
                    g_lib_CmdToTarget = 0;
                    return;
                } else if (cmd == 0xAE) {
                    g_lib_CmdToTarget = 0;
                } else { // try again
                    SysTick->LOAD = delay * _CyclesPerUs;
                    SysTick->VAL  = 0x0UL;
                    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock
                }
            }
        }
    }
}

#ifdef __cplusplus
}
#endif
