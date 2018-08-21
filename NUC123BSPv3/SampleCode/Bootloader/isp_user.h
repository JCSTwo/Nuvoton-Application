#ifndef ISP_USER_H
#define ISP_USER_H

// Special Version for Nu-Bridge Only
// 1. No Software protection & Security Lock protection
// 2. To Entry ISP mode, user needs to tie PA13 to GND pin
// 3. DO NOT support CMD_ERASE_ALL
// 4. Update internal flash using CMD_UPDATE_APROM or CMD_UPDATE_DATAFLASH
//    - The flash size & address are specified by PC Tool (Only Support APROM & NVM)
//    - Remove GetDataFlashInfo to reduce code size

#define FW_VERSION                  0x9A

#include "fmc_user.h"

#define CMD_UPDATE_APROM            0x000000A0
#define CMD_UPDATE_CONFIG           0x000000A1
#define CMD_SYNC_PACKNO             0x000000A4
#define CMD_GET_FWVER               0x000000A6
#define CMD_RUN_APROM               0x000000AB
#define CMD_RUN_LDROM               0x000000AC
#define CMD_RESET                   0x000000AD
#define CMD_CONNECT                 0x000000AE
#define CMD_GET_DEVICEID            0x000000B1
#define CMD_UPDATE_DATAFLASH        0x000000C3
#define CMD_RESEND_PACKET           0x000000FF
#define V6M_AIRCR_VECTKEY_DATA      0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ       0x00000004UL

extern int ParseCmd(unsigned char *buffer, uint8_t len);

extern __align(4) uint8_t usb_rcvbuf[];
extern __align(4) uint8_t usb_sendbuf[];
extern __align(4) uint8_t response_buff[64];
#endif  // #ifndef ISP_USER_H
