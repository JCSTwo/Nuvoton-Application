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

// ISP Application Note v1-5: 1.1 Receive/Send data
__STATIC_INLINE uint16_t ISP_Checksum(uint8_t *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

// ISP Application Note v1-5: 4.3.2 Commands response delay
__STATIC_INLINE uint32_t ISP_ResponseDelay(uint8_t cmd)
{
    if ((cmd == 0xA0) || (cmd == 0xA3) || (cmd == 0xC3)) {
        // #define CMD_UPDATE_APROM      0x000000A0
        // #define CMD_ERASE_ALL         0x000000A3
        // #define CMD_UPDATE_DATAFLASH  0x000000C3
        return 300000;
    } else if (cmd == 0xAE) {
        // #define CMD_CONNECT           0x000000AE
        return 20000;
    } else {
        return 50000;
    }
}

#endif  // #ifndef ISP_PARSER_H
