#ifndef FMC_USER_H
#define FMC_USER_H

#define Config0         FMC_CONFIG_BASE
#define Config1         (FMC_CONFIG_BASE+4)

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


int FMC_Write_User(unsigned int u32Addr, unsigned int u32Data);
int FMC_Read_User(unsigned int u32Addr, unsigned int *data);
int FMC_Erase_User(unsigned int u32Addr);
void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);
void WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);
void EraseAP(unsigned int addr_start, unsigned int addr_end);
void UpdateConfig(unsigned int *data, unsigned int *res);

#endif

