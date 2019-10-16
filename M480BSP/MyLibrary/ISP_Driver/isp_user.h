#ifndef ISP_USER_H
#define ISP_USER_H

#define FW_VERSION                          0xC2
// Do NOT support CMD_ERASE_ALL command.
// Do NOT support CMD_RUN_LDROM command.
// Do NOT support CMD_RESET command.
// Do NOT support CMD_DISCONNECT command.
// Do NOT support CMD_WRITE_CHECKSUM command.
// Do NOT support CMD_GET_FLASHMODE command.

#include "fmc_user.h"

#define CMD_UPDATE_APROM            0x000000A0
#define CMD_UPDATE_CONFIG           0x000000A1
#define CMD_READ_CONFIG             0x000000A2
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

#define V6M_AIRCR_VECTKEY_DATA      0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ       0x00000004UL

extern int ParseCmd(unsigned char *buffer, uint8_t len);

extern __align(4) uint8_t response_buff[64];

__STATIC_INLINE void RUN_APROM(void)
{
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}

#endif  // #ifndef ISP_USER_H
