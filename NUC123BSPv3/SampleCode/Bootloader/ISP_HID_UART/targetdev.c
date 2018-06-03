
#include "targetdev.h"
#include "isp_user.h"

#define CONFIG0_DFEN                0x01
#define CONFIG0_DFVSEN              0x04

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    g_apromSize = 64 * 1024; // only support Nu-Bridge (NUC123SD4AN0, 64K)
    *size = 0;
    /* Note: DFVSEN = 1, DATA Flash Size is 4K bytes
             DFVSEN = 0, DATA Flash Size is based on CONFIG1 */
    FMC_Read_User(Config0, &uData);

    if (uData & CONFIG0_DFVSEN) {
        *addr = 0x1F000;
        *size = 4096;//4K
    } else if (uData & CONFIG0_DFEN) {
        g_apromSize += 4096;
        *addr = g_apromSize;
        *size = 0;
    } else {
        g_apromSize += 4096;
        FMC_Read_User(Config1, &uData);

        if (uData > g_apromSize || (uData & 0x1FF)) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
        g_apromSize -= *size;
    }
}
