#include <stdio.h>
#include "string.h"
#include "isp_user.h"

__align(4) uint8_t response_buff[64];
__align(4) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0 , i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

#if 0
static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
    int i;
    register uint16_t lcksum = 0;

    for (i = 0; i < len; i += FMC_FLASH_PAGE_SIZE) {
        ReadData(start + i, start + i + FMC_FLASH_PAGE_SIZE, (uint32_t *)aprom_buf);

        if (len - i >= FMC_FLASH_PAGE_SIZE) {
            lcksum += Checksum(aprom_buf, FMC_FLASH_PAGE_SIZE);
        } else {
            lcksum += Checksum(aprom_buf, len - i);
        }
    }

    return lcksum;
}
#endif

int ParseCmd(unsigned char *buffer, uint8_t len)
{
    static uint32_t StartAddress, TotalLen, LastDataLen, g_packno = 1;
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
    ReadData(Config0, Config0 + 8, (uint32_t *)(response + 8)); //read config

    if (lcmd == CMD_SYNC_PACKNO) {
        g_packno = inpw(pSrc);
    }

    if ((lcmd) && (lcmd != CMD_RESEND_PACKET)) {
        gcmd = lcmd;
    }

    if (lcmd == CMD_GET_FWVER) {
        response[8] = FW_VERSION;
    } else if (lcmd == CMD_GET_DEVICEID) {
        outpw(response + 8, SYS->PDID);
        goto out;
    } else if (lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET) {
        outpw(&SYS->RSTSRC, 3);//clear bit

        /* Set BS */
        if (lcmd == CMD_RUN_APROM) {
            i = (FMC->ISPCON & 0xFFFFFFFC);
        } else if (lcmd == CMD_RUN_LDROM) {
            i = (FMC->ISPCON & 0xFFFFFFFC);
            i |= 0x00000002;
        } else {
            i = (FMC->ISPCON & 0xFFFFFFFE);//ISP disable
        }

        outpw(&FMC->ISPCON, i);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

        /* Trap the CPU */
        while (1);
    } else if (lcmd == CMD_CONNECT) {
        g_packno = 1;
        goto out;
    } else if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL)) {
        EraseAP(FMC_APROM_BASE, (g_apromSize < g_dataFlashAddr) ? g_apromSize : g_dataFlashAddr); // erase APROM // g_dataFlashAddr, g_apromSize

        if (lcmd == CMD_ERASE_ALL) { //erase data flash
            EraseAP(g_dataFlashAddr, g_dataFlashAddr + g_dataFlashSize);
            UpdateConfig((uint32_t *)(response + 8), NULL);
        }
    }

    if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH)) {
        if (lcmd == CMD_UPDATE_DATAFLASH) {
            StartAddress = g_dataFlashAddr;

            if (g_dataFlashSize) { //g_dataFlashAddr
                EraseAP(g_dataFlashAddr, g_dataFlashAddr + g_dataFlashSize);
            } else {
                goto out;
            }
        } else {
            StartAddress = FMC_APROM_BASE;
        }

        //StartAddress = inpw(pSrc);
        TotalLen = inpw(pSrc + 4);
        pSrc += 8;
        srclen -= 8;
    } else if (lcmd == CMD_UPDATE_CONFIG) {
        UpdateConfig((uint32_t *)(pSrc), (uint32_t *)(response + 8));
        GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
        goto out;
    } else if (lcmd == CMD_RESEND_PACKET) {
        StartAddress -= LastDataLen;
        TotalLen += LastDataLen;

        if ((StartAddress & 0xFFE00) >= Config0) {
            goto out;
        }

        ReadData(StartAddress & 0xFFE00, StartAddress, (uint32_t *)aprom_buf);
        FMC_Erase_User(StartAddress & 0xFFE00);
        WriteData(StartAddress & 0xFFE00, StartAddress, (uint32_t *)aprom_buf);

        if ((StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - LastDataLen)) {
            FMC_Erase_User((StartAddress & 0xFFE00) + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH)) {
        if (TotalLen < srclen) {
            srclen = TotalLen;//prevent last package from over writing
        }

        TotalLen -= srclen;
        WriteData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        memset(pSrc, 0, srclen);
        ReadData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        StartAddress += srclen;
        LastDataLen =  srclen;
    }

out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response + 4, g_packno);
    g_packno++;
    return 0;
}

