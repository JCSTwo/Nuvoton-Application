#ifndef ISP_PARSER_H
#define ISP_PARSER_H
#include <stdint.h>

#define CMD_UPDATE_APROM            0x000000A0
#define CMD_UPDATE_CONFIG           0x000000A1
#define CMD_ERASE_ALL               0x000000A3
#define CMD_SYNC_PACKNO             0x000000A4
#define CMD_GET_FWVER               0x000000A6
#define CMD_RUN_APROM               0x000000AB
#define CMD_RUN_LDROM               0x000000AC
#define CMD_RESET                   0x000000AD
#define CMD_CONNECT                 0x000000AE
#define CMD_GET_DEVICEID            0x000000B1
#define CMD_UPDATE_DATAFLASH        0x000000C3
#define CMD_RESEND_PACKET           0x000000FF

extern uint8_t  volatile g_ISP_CMD;
extern uint16_t volatile g_ISP_CKS;
extern uint8_t  volatile g_ISP_ACK;
extern uint32_t volatile g_ISP_DLY;

extern void ParseISP(const unsigned char *buf);

#endif  // #ifndef ISP_PARSER_H
