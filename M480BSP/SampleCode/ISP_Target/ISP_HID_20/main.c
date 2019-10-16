#include <stdio.h>
#include "targetdev.h"
#include "hid_transfer.h"

__weak uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
}

extern uint8_t bUsbDataReady;
extern __align(4) uint8_t usb_rcvbuf[];

void USBD20_IRQHandler(void);
int32_t main(void)
{
    SYS_Init();
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;    // (1ul << 0)

    if (DetectPin != 0) {
        goto _APROM;
    }

    /* Open HSUSB controller */
    HSUSBD_Open(NULL, NULL, NULL);
    /* Endpoint configuration */
    HID_Init();
    /* Enable USBD interrupt */
    // NVIC_EnableIRQ(USBD20_IRQn);
    /* Start transaction */
    HSUSBD_Start();

    while (DetectPin == 0) {
        // polling USBD interrupt flag
        USBD20_IRQHandler();

        if (bUsbDataReady == TRUE) {
            ParseCmd((uint8_t *)usb_rcvbuf, 64);
            EPA_Handler();
            bUsbDataReady = FALSE;
        }
    }

_APROM:
    RUN_APROM();
}
