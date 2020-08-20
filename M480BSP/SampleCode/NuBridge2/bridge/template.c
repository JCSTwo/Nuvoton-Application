#include "nubridge2.h"

void VCOM_LineCoding(uint8_t port)
{
    RS232Monitor_LineCoding();
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
    SYS_Init_UART13(); // Pin 13, 15 (-7, -5)
    GPIO_SETMODE(PC, 12, GPIO_MODE_QUASI); // SwitchPin
    SYS_Init_LED(1, 1, 1, 1); // RED, YELLOW, RED, GREEN
}

void VCOM_TransferData(void)
{
    RS232Monitor_TransferData();
}

int32_t main(void)
{
    SYS_Init();
    HSUSBD_Open(&gsHSInfo, VCOM_ClassRequest, NULL);
    /* Endpoint configuration */
    VCOM_Init();
    NVIC_EnableIRQ(USBD20_IRQn);

    /* Start transaction */
    while (1) {
        if (HSUSBD_IS_ATTACHED()) {
            HSUSBD_Start();
            break;
        }
    }

    while (1) {
        VCOM_TransferData();
    }
}
