#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"

#define FLASH_USER_DATA_ADDR   0x08020000

#define RX_ADR_WIDTH           5

#define OFFSET_CALIB_FLAG      0
#define OFFSET_CALIB_DATA      4  // 3 * 4 = 12 bytes
#define OFFSET_PAIR_FLAG       16
#define OFFSET_PID_FLAG        20
#define OFFSET_PID_DATA        24 // 8 objects * 3 params * 4 bytes = 96 bytes
#define OFFSET_RX_ADDRESS      120 // 5 bytes for RX_ADDRESS

#define CALIBRATION_FLAG       0x5555AAAA
#define PID_SAVED_FLAG         0x7777CCCC

void Flash_Write_Data(uint32_t address, uint32_t *data, uint16_t length);
void Flash_Read_Data(uint32_t address, uint32_t *data, uint16_t length);
void Flash_Erase_Sector5(void);

void User_Data_Save_All(void);
void User_Data_Load_All(void);
void User_Data_Reset_PID(void);
void Flash_Save_RxAddress(void);
void Flash_Load_RxAddress(void);
int8_t User_Data_Check_Calibrated(void);
int8_t User_Data_Check_Paired(void);

#endif
