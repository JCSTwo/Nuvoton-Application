#include "nubridge2.h"

#define SwitchPin    PC12

void VCOM_LineCoding(uint8_t port)
{
    if (SwitchPin != 0) {
        LED0 = 0;
        LED1 = 1;
        RS232_LineCoding();
    } else {
        LED0 = 1;
        LED1 = 0;
        RS485_LineCoding();
    }
}

void SYS_Init(void)
{
    SYS_Init_192MHZ();
    SYS_Init_HSUSBD();
    SYS_Init_UART2();
    SYS_Init_RS485();
    GPIO_SETMODE(PC, 12, GPIO_MODE_QUASI); // SwitchPin
    SYS_Init_LED(1, 1, 1, 1);
    LED0 = 0; // RED
    LED1 = 0; // YELLOW
    LED2 = 0; // RED
    LED3 = 0; // GREEN
    LED0 = 1;
    LED1 = 1;
    LED2 = 1;
    LED3 = 1;
}

void VCOM_TransferData(void)
{
    LED2 = 0;
    LED3 = 1;

    while (SwitchPin != 0) {
        RS232_TransferData();
    }

    LED2 = 1;
    LED3 = 0;

    while (SwitchPin == 0) {
        RS485_TransferData();
    }
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
