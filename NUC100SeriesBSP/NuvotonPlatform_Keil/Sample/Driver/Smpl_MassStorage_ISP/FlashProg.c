/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include "MassStorage_ISP.h"
#include "NUC123.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define FLASH_PAGE_SIZE         512

extern uint32_t g_u32FlashSize;
extern uint8_t g_u8SecurityLockBit;

uint8_t u8FormatData[62] = {
    0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53,
    0x35, 0x2E, 0x30, 0x00, 0x02, 0x01, 0x06, 0x00,
    0x02, 0x00, 0x02, 0xA8, 0x00, 0xF8, 0x01, 0x00,
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0xB9,
    0xC1, 0xAA, 0x42, 0x4E, 0x4F, 0x20, 0x4E, 0x41,
    0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, 0x41,
    0x54, 0x31, 0x32, 0x20, 0x20, 0x20
};


uint8_t u8RootDirData[96] = {
    0x42, 0x20, 0x00, 0x49, 0x00, 0x6E, 0x00, 0x66,
    0x00, 0x6F, 0x00, 0x0F, 0x00, 0x72, 0x72, 0x00,
    0x6D, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00,
    0x6F, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x00, 0x00,
    0x01, 0x53, 0x00, 0x79, 0x00, 0x73, 0x00, 0x74,
    0x00, 0x65, 0x00, 0x0F, 0x00, 0x72, 0x6D, 0x00,
    0x20, 0x00, 0x56, 0x00, 0x6F, 0x00, 0x6C, 0x00,
    0x75, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x65, 0x00,
    0x53, 0x59, 0x53, 0x54, 0x45, 0x4D, 0x7E, 0x31,
    0x20, 0x20, 0x20, 0x16, 0x00, 0x99, 0x0D, 0x5C,
    0x6D, 0x43, 0x6D, 0x43, 0x00, 0x00, 0x0E, 0x5C,
    0x6D, 0x43, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
};


void FMC_ReadPage(uint32_t u32startAddr, uint32_t *u32buff)
{
    uint32_t i;

    for (i = 0; i < FLASH_PAGE_SIZE / 4; i++) {
        u32buff[i] = 0;
    }

    if (u32startAddr == 0x00000000) {
        my_memcpy((uint8_t *)u32buff, u8FormatData, 62);
        u32buff[FLASH_PAGE_SIZE / 4 - 1] = 0xAA550000;
    } else {
        if ((u32startAddr == (FAT_SECTORS * 512)) || (u32startAddr == ((FAT_SECTORS + 1) * 512))) {
            u32buff[0] = 0x00FFFFF8;
        } else if (u32startAddr == (8 * 512)) { /* root dir */
            my_memcpy((uint8_t *)u32buff, u8RootDirData, 96);
        }
    }
}

void myFMC_Write(uint32_t u32addr, uint32_t u32data)
{
    outp32(FMC_ISPCMD, 0x21);
    outp32(FMC_ISPADR, u32addr);
    outp32(FMC_ISPDAT, u32data);
    outp32(FMC_ISPTRG, 0x01);
    __ISB();
}

void myFMC_Erase(uint32_t u32addr)
{
    outp32(FMC_ISPCMD, 0x22);
    outp32(FMC_ISPADR, u32addr);
    outp32(FMC_ISPTRG, 0x01);
    __ISB();
}

void FMC_ProgramPage(uint32_t u32startAddr, uint32_t *u32buff)
{
    uint32_t i;

    for (i = 0; i < FLASH_PAGE_SIZE / 4; i++) {
        myFMC_Write(u32startAddr + i * 4, u32buff[i]);
    }
}


void DataFlashWrite(uint32_t addr, uint32_t buffer)
{
    /* This is low level write function of USB Mass Storage */
    if ((addr >= DATA_SECTOR_ADDRESS) && (addr < (DATA_SECTOR_ADDRESS + g_u32FlashSize))) {
        addr -= DATA_SECTOR_ADDRESS;

        if ((FMC->ISPCON & FMC_ISPCON_BS_Msk) == 0) {
            addr += FMC_LDROM_BASE;
        }

        myFMC_Erase(addr);
        FMC_ProgramPage(addr, (uint32_t *) buffer);
    }
}

