#include <stdio.h>
#include "string.h"
#include "isp_user.h"

__align(4) uint8_t response_buff[64];

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

int ParseCmd(unsigned char *buffer, uint8_t len)
{
    static uint32_t StartAddress, TotalLen, g_packno = 1;
    uint8_t *response;
    uint16_t lcksum;
    uint32_t lcmd, srclen, i;
    unsigned char *pSrc;
    static uint32_t gcmd;
    response = response_buff;
    pSrc = buffer;
    srclen = len;
    lcmd = inpw(pSrc);
    outpw(response + 4, 0);
    pSrc += 8;
    srclen -= 8;
    ReadData(Config0, Config0 + 16, (uint32_t *)(response + 8)); //read config

    if (lcmd == CMD_SYNC_PACKNO) {
        g_packno = inpw(pSrc);
    }

    if ((lcmd) && (lcmd != CMD_RESEND_PACKET)) {
        gcmd = lcmd;
    }

    if (lcmd == CMD_GET_FWVER) {
        response[8] = FW_VERSION;//version 2.3
    } else if (lcmd == CMD_GET_DEVICEID) {
        outpw(response + 8, SYS->PDID);
        goto out;
    } else if (lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET) {
        outpw(&SYS->RSTSTS, 3);//clear bit

        /* Set BS */
        if (lcmd == CMD_RUN_APROM) {
            i = (FMC->ISPCTL & 0xFFFFFFFC);
        } else if (lcmd == CMD_RUN_LDROM) {
            i = (FMC->ISPCTL & 0xFFFFFFFC);
            i |= 0x00000002;
        } else {
            i = (FMC->ISPCTL & 0xFFFFFFFE);//ISP disable
        }

        outpw(&FMC->ISPCTL, i);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

        /* Trap the CPU */
        while (1);
    } else if (lcmd == CMD_CONNECT) {
        g_packno = 1;
        goto out;
    } else if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH)) {
        StartAddress = inpw(pSrc);
        TotalLen = inpw(pSrc + 4);
        pSrc += 8;
        srclen -= 8;
    } else if (lcmd == CMD_UPDATE_CONFIG) {
        UpdateConfig((uint32_t *)(pSrc), (uint32_t *)(response + 8));
        goto out;
    }

    if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH)) {
        if (TotalLen < srclen) {
            srclen = TotalLen;//prevent last package from over writing
        }

        TotalLen -= srclen;
        StartAddress += srclen;
    }

out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response + 4, g_packno);
    g_packno++;
    return 0;
}

