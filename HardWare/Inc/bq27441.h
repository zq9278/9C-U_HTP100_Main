#ifndef __BQ27441_H
#define __BQ27441_H
#include "stm32g0xx_hal.h"
#define BQ27441Address	0xaa
#include <stdbool.h>
#define BQ27441_I2C_ADDRESS    (0x55 << 1)
#define BQ27441_COMMAND_FLAGS         0x06

// Extended Data
#define BQ27441_EXTENDED_CONTROL      0x61
#define BQ27441_EXTENDED_DATACLASS    0x3E
#define BQ27441_EXTENDED_DATABLOCK    0x3F
#define BQ27441_EXTENDED_CHECKSUM     0x60
#define BQ27441_EXTENDED_BLOCKDATA    0x40
#define BQ27441_CHECKSUM_DATA         0x60

#define BQ27441_ID_STATE              0x52
#define BQ27441_ID_RACOMP             0x59
typedef struct {
    uint16_t Temperature;
    uint16_t Voltage;
    uint16_t Flags;
    uint16_t NomAvailableCap;
    uint16_t FullAvailableCap;
    uint16_t RemainingCap;
    uint16_t FullChargeCap;
    int16_t  AvgCurrent;
    int16_t  StandbyCurrent;
    int16_t  MaxLoadCurrent;
    int16_t AvgPower;
    uint16_t SOC;
    uint16_t InternalTemp;
    uint8_t percent;  // ÊÙÃü°Ù·Ö±È£¨0x20£©
    uint8_t status;   // ×´Ì¬Âë£¨0x21£©
} BQ27441_typedef;

void I2C_Semaphore_Init(void);
void battery_status_update_bq27441(void);
void BATCheckDIS(void);
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ);
HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size);
bool BQ27441_Unseal(void);
bool BQ27441_EnterConfigMode(void);
bool BQ27441_ExitConfigMode(void);
uint8_t BQ27441_ReadExtended(uint8_t classID, uint8_t offset);
void BQ27441_DEMO(void);
void BQ27441_VerifyConfig(void);
void BQ27441_PrintRaTable(void);
bool BQ27441_EnableIT(void);
#endif









