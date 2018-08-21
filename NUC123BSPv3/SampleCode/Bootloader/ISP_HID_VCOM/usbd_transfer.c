/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NUC123.h"
#include "usbd_transfer.h"

__align(4) uint8_t usb_rcvbuf[64];
uint8_t bVcomDataReady;
uint8_t bHidDataReady;

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET) {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED()) {
            /* USB Plug In */
            USBD_ENABLE_USB();
        } else {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_WAKEUP) {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS) {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST) {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }

        if (u32State & USBD_STATE_SUSPEND) {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME) {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

//------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB) {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP) {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);
            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);
            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);

            // Bulk IN
            // EP2_Handler();
            if (USBD_GET_PAYLOAD_LEN(EP2) == EP2_MAX_PKT_SIZE) {
                USBD_SET_PAYLOAD_LEN(EP2, 0);
            }
        }

        if (u32IntSts & USBD_INTSTS_EP3) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk OUT
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
            // Interrupt IN
            //EP5_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP6) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
            // Interrupt OUT
            EP6_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP7) {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
            // Interrupt IN
            // EP7_Handler();
        }
    }
}

extern __align(4) uint8_t response_buff[64];
void EP2_Handler(void)  /* Interrupt IN handler */
{
    uint8_t *ptr;
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
    /* Prepare the data for next HID IN transfer */
    USBD_MemCopy(ptr, response_buff, EP2_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
}

void EP3_Handler(void)  /* Interrupt OUT handler */
{
    uint8_t *ptr;
    /* Interrupt OUT */
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));
    USBD_MemCopy(usb_rcvbuf, ptr, EP3_MAX_PKT_SIZE);
    bVcomDataReady = TRUE;
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}

void EP6_Handler(void)  /* Interrupt OUT handler */
{
    uint8_t *ptr;
    /* Interrupt OUT */
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP6));
    USBD_MemCopy(usb_rcvbuf, ptr, EP6_MAX_PKT_SIZE);
    bHidDataReady = TRUE;
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
}

void EP7_Handler(void)  /* Interrupt IN handler */
{
    uint8_t *ptr;
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7));
    /* Prepare the data for next HID IN transfer */
    USBD_MemCopy(ptr, response_buff, EP7_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;
    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);
    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);
    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);
    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);
    /*****************************************************/
    /* EP7 ==> Interrupt IN endpoint, address 6 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_1);
    /* Buffer range for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);
    /* EP6 ==> Interrupt OUT endpoint, address 5 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_OUT | INT_OUT_EP_NUM_1);
    /* Buffer range for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
}

void HID_ClassRequest(void)
{
    uint8_t buf[8];
    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80) { /* request data transfer direction */
        // Device to host
        switch (buf[1]) {
            case GET_LINE_CODE: {
                if (buf[4] == 0) { /* VCOM-1 */
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&gLineCoding, 7);
                }

                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_REPORT:
            case GET_IDLE:
            case GET_PROTOCOL:
            default: {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    } else {
        // Host to device
        switch (buf[1]) {
            case SET_CONTROL_LINE_STATE: {
                if (buf[4] == 0) { /* VCOM-1 */
                    gCtrlSignal = buf[3];
                    gCtrlSignal = (gCtrlSignal << 8) | buf[2];
                    //printf("RTS=%d  DTR=%d\n", (gCtrlSignal0 >> 1) & 1, gCtrlSignal0 & 1);
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE: {
                //g_usbd_UsbConfig = 0100;
                if (buf[4] == 0) { /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&gLineCoding, 7);
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_REPORT: {
                if (buf[3] == 3) {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }

                break;
            }

            case SET_IDLE: {
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_PROTOCOL:
            default: {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}
