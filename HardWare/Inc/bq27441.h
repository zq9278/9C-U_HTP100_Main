#ifndef __BQ27441_H
#define __BQ27441_H
#include "stm32g0xx_hal.h"

#define BQ27441Address	0xaa
#define I2C_TIMEOUT_MS 100  // ³¬Ê±Ê±¼ä

typedef struct
{
  uint16_t Voltage;
	uint16_t Temperature;
	int16_t AvgCurrent;
	uint16_t SOC;
	uint16_t FullChargeCapacity;
}BQ27441_typedef;

void I2C_Semaphore_Init(void);
uint8_t BQ27441_Init(void);
void BQ27441_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void BQ27441_MultiRead(BQ27441_typedef *BQ_State);
void BQ27441_WriteByte(uint8_t WriteAddr,uint8_t WriteData);
void BQ27441_WriteWord(uint8_t WriteAddr,uint16_t WriteData);
void battery_status_update_bq27441(void);
void BATCheckDIS(void);
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ_State);
HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size);
HAL_StatusTypeDef BQ27441_Write_IT(uint8_t regAddr, uint8_t *pData, uint16_t size);
void main_app(void);









/***********************************************/
#define  COLUMB_FLAG_VOLT   (1 << 0)
#define  COLUMB_FLAG_CURR   (1 << 1)
#define  COLUMB_FLAG_SOC    (1 << 2)
#define  COLUMB_FLAG_FCC    (1 << 3)
/***********************************************/
#define SLAVE_I2C_GENERIC_RETRY_MAX  5

//#define  BQ2744_ADDRESS    0x55
#define  BQ2744_ADDRESS    0xAA

/***********************************************/

#define bq27421CMD_CNTL_LSB           0x00
#define bq27421CMD_CNTL_MSB           0x01
#define bq27421CMD_TEMP_LSB           0x02
#define bq27421CMD_TEMP_MSB           0x03
#define bq27421CMD_VOLT_LSB           0x04
#define bq27421CMD_VOLT_MSB           0x05
#define bq27421CMD_FLAG_LSB           0x06
#define bq27421CMD_FLAG_MSB           0x07
#define bq27421CMD_NAC_LSB            0x08
#define bq27421CMD_NAC_MSB            0x09
#define bq27421CMD_FAC_LSB            0x0a
#define bq27421CMD_FAC_MSB            0x0b
#define bq27421CMD_RM_LSB             0x0c
#define bq27421CMD_RM_MSB             0x0d
#define bq27421CMD_FCC_LSB            0x0e
#define bq27421CMD_FCC_MSB            0x0f
#define bq27421CMD_AI_LSB             0x10
#define bq27421CMD_AI_MSB             0x11
#define bq27421CMD_SI_LSB             0x12
#define bq27421CMD_SI_MSB             0x13
#define bq27421CMD_MLI_LSB            0x14
#define bq27421CMD_MLI_MSB            0x15
#define bq27421CMD_AP_LSB             0x18
#define bq27421CMD_AP_MSB             0x19
#define bq27421CMD_SOC_LSB            0x1c
#define bq27421CMD_SOC_MSB            0x1d
#define bq27421CMD_ITEMP_LSB          0x1e
#define bq27421CMD_ITEMP_MSB          0x1f
#define bq27421CMD_SOH_LSB            0x20
#define bq27421CMD_SOH_MSB            0x21
/**************************************************************************************
**************************************************************************************/
typedef struct __BQ27XX_Drive{
    void     (*init)(void);
    void     (*uninit)(void);
    uint32_t (*itpor)(void);
    void     (*fullreset)(void);
    uint32_t (*updategauge)(uint16_t *volt, uint16_t *avgCur, uint16_t *soc, uint16_t *fcc);
    uint32_t (*config)(void);
    uint32_t (*confirmconfig)(void);
    void     (*showtaandmax)(void);
}BQ27XX_INTERFACE_T;

enum{
    Rod_Charging=1,
    BQ_Charge_Done,
    BQ_Charging,
};
extern const BQ27XX_INTERFACE_T g_pfBQ27xxFunction[];
uint32_t bq_Config(void);
uint8_t BQ25895_STATE();
uint8_t Batter_Gauge_Test();
uint8_t getBoxBattery(void);
void bq27441_init(void);
static uint32_t bq_cmdWrite(uint8_t subaddr, uint8_t cmd);
static uint32_t bq_read(uint8_t *pBuf,  uint8_t addr, uint8_t len);
static uint32_t bq_MemoryWrite(uint8_t subaddr, uint16_t data );
static uint8_t GetCheckSum(uint8_t  addr);
static void  bq_Read_Ta_and_Qmax(void);
static void bq_fullReset(void);
static void bq_CONFIG_subclass82(void);
static void bq_CONFIG_subclass81(void);
static void bq_CONFIG_subclass89(void);
static uint32_t bq_ConfigRead(void);
static uint32_t bq_Rdarg(uint16_t *volt, int16_t *avgCur, uint16_t *soc, uint16_t *fcc);
uint16_t bq_ITPOR(void);

























#include <stdbool.h>
#define BQ27441_I2C_ADDRESS    (0x55 << 1)
#define BQ27441_DEVICE_ID      0x0421

// Control Subcommands
#define BQ27441_CONTROL_DEVICE_TYPE   0x0001
#define BQ27441_CONTROL_SEALED        0x0020
#define BQ27441_CONTROL_UNSEAL        0x8000
#define BQ27441_CONTROL_SET_CFGUPDATE 0x0013
#define BQ27441_CONTROL_SOFT_RESET    0x0042

// Commands
#define BQ27441_COMMAND_VOLTAGE       0x04
#define BQ27441_COMMAND_AVG_CURRENT   0x10
#define BQ27441_COMMAND_SOC           0x2C
#define BQ27441_COMMAND_FLAGS         0x06

// Extended Data
#define BQ27441_EXTENDED_CONTROL      0x61
#define BQ27441_EXTENDED_DATACLASS    0x3E
#define BQ27441_EXTENDED_DATABLOCK    0x3F
#define BQ27441_EXTENDED_CHECKSUM     0x60
#define BQ27441_EXTENDED_BLOCKDATA    0x40

// Class IDs
#define BQ27441_ID_STATE              82
#define BQ27441_ID_RACOMP             89


bool BQ27441_Init_STR(I2C_HandleTypeDef *hi2c);
bool BQ27441_Unseal(void);
bool BQ27441_EnterConfigMode(void);
bool BQ27441_ExitConfigMode(void);

bool BQ27441_WriteExtended(uint8_t classID, uint8_t offset, const uint8_t *data, uint8_t len);
uint8_t BQ27441_ReadExtended(uint8_t classID, uint8_t offset);

uint16_t BQ27441_ReadVoltage(void);
uint16_t BQ27441_ReadSOC(void);
int16_t  BQ27441_ReadAverageCurrent(void);
void BQ27441_DEMO(void);
void BQ27441_VerifyConfig(void);
void BQ27441_PrintRaTable(void);

#endif









