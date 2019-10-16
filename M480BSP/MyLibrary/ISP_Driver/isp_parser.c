#include "isp_parser.h"

uint8_t  volatile g_ISP_CMD = 0;
uint16_t volatile g_ISP_CKS = 0;
uint8_t  volatile g_ISP_ACK = 0;
uint32_t volatile g_ISP_DLY = 0;

void ParseISP(const unsigned char *buf)
{
    int i;
    g_ISP_CMD = buf[0];

    for (g_ISP_CKS = 0, i = 0 ; i < 64; i++) {
        g_ISP_CKS += buf[i];
    }

    if ((g_ISP_CMD == CMD_RUN_APROM)
            || (g_ISP_CMD == CMD_RUN_LDROM)
            || (g_ISP_CMD == CMD_RESET)) {
        g_ISP_ACK = 0;
    } else {
        g_ISP_ACK = 1;
    }

    if ((g_ISP_CMD == CMD_UPDATE_APROM)
            || (g_ISP_CMD == CMD_ERASE_ALL)
            || (g_ISP_CMD == CMD_UPDATE_DATAFLASH)) {
        g_ISP_DLY = 300000;
    } else if (g_ISP_CMD == CMD_CONNECT) {
        g_ISP_DLY = 20000;
    } else {
        g_ISP_DLY = 50000;
    }
}
