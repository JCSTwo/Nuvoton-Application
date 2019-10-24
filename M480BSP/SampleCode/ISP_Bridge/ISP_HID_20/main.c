#include <stdio.h>
#include "NuMicro.h"
#include "hid_transfer.h"
#include "isp_bridge.h"
#include "hal_sys_init.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
}

int32_t main(void)
{
    SYS_Init();
    HSUSBD_Open(NULL, NULL, NULL);
    /* Endpoint configuration */
    HID_Init();
    /* Enable USBD interrupt */
    NVIC_EnableIRQ(USBD20_IRQn);
    /* Start transaction */
    while (1) {
        if (HSUSBD_IS_ATTACHED()) {
            HSUSBD_Start();
            break;
        }
    }

    ISP_Bridge_Init();

    while (1) {
        ISP_Bridge_Main();
    }
}
