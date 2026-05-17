#include "flash.h"
#include "all_data.h"
#include "mpu6050.h"

extern int16_t MpuOffset[6];
extern _st_ALL_flag ALL_flag;
extern PidObject pidRateX, pidRateY, pidRateZ, pidRoll, pidPitch, pidYaw, pidHeightRate, pidHeightHigh;
extern uint8_t RX_ADDRESS[RX_ADR_WIDTH];

static PidObject *pPids[] = {&pidRateX, &pidRateY, &pidRateZ, &pidRoll, &pidPitch, &pidYaw, &pidHeightRate, &pidHeightHigh};

// 擦除第5扇区（128KB，0x08020000 - 0x0803FFFF）
void Flash_Erase_Sector5(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_5;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return;
    }
    HAL_FLASH_Lock();
}

// 向指定地址写入32位数据（写入前必须确保扇区已擦除）
void Flash_Write_Data(uint32_t address, uint32_t *data, uint16_t length)
{
    HAL_FLASH_Unlock();
    for (uint16_t i = 0; i < length; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i * 4), data[i]) != HAL_OK)
        {
            break;
        }
    }
    HAL_FLASH_Lock();
}

// 从指定地址读取32位数据
void Flash_Read_Data(uint32_t address, uint32_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        data[i] = *(__IO uint32_t *)(address + (i * 4));
    }
}

// 保存所有用户数据到Flash
void User_Data_Save_All(void)
{
    uint32_t buffer[128] = {0};

    // 1. Prepare flags and data
    buffer[OFFSET_CALIB_FLAG / 4] = CALIBRATION_FLAG;
    buffer[OFFSET_CALIB_DATA / 4 + 0] = ((uint32_t)MpuOffset[1] << 16) | (uint16_t)MpuOffset[0];
    buffer[OFFSET_CALIB_DATA / 4 + 1] = ((uint32_t)MpuOffset[3] << 16) | (uint16_t)MpuOffset[2];
    buffer[OFFSET_CALIB_DATA / 4 + 2] = ((uint32_t)MpuOffset[5] << 16) | (uint16_t)MpuOffset[4];
    
    // 保存完整的 ALL_flag 结构体（4 个 uint8_t 成员，共 4 字节）
    buffer[OFFSET_PAIR_FLAG / 4] = *(uint32_t *)&ALL_flag;
    
    buffer[OFFSET_PID_FLAG / 4] = PID_SAVED_FLAG;

    for (int i = 0; i < 8; i++)
    {
        uint32_t *pPidData = (uint32_t *)&buffer[OFFSET_PID_DATA / 4 + i * 3];
        pPidData[0] = *(uint32_t *)&pPids[i]->kp;
        pPidData[1] = *(uint32_t *)&pPids[i]->ki;
        pPidData[2] = *(uint32_t *)&pPids[i]->kd;
    }

    // 保存 RX_ADDRESS (5 字节)
    buffer[OFFSET_RX_ADDRESS / 4] = ((uint32_t)RX_ADDRESS[1] << 16) | ((uint32_t)RX_ADDRESS[0] << 0) | ((uint32_t)RX_ADDRESS[2] << 24);
    buffer[OFFSET_RX_ADDRESS / 4 + 1] = RX_ADDRESS[3] | ((uint32_t)RX_ADDRESS[4] << 8);

    // 2. Erase and write
    Flash_Erase_Sector5();
    Flash_Write_Data(FLASH_USER_DATA_ADDR, buffer, 128);
}

void Flash_Save_RxAddress(void)
{
    uint32_t buffer[128];
    Flash_Read_Data(FLASH_USER_DATA_ADDR, buffer, 128);

    buffer[OFFSET_RX_ADDRESS / 4] = ((uint32_t)RX_ADDRESS[1] << 16) | ((uint32_t)RX_ADDRESS[0] << 0) | ((uint32_t)RX_ADDRESS[2] << 24);
    buffer[OFFSET_RX_ADDRESS / 4 + 1] = RX_ADDRESS[3] | ((uint32_t)RX_ADDRESS[4] << 8);

    Flash_Erase_Sector5();
    Flash_Write_Data(FLASH_USER_DATA_ADDR, buffer, 128);
}

void Flash_Load_RxAddress(void)
{
    uint32_t rxBuffer[2];
    Flash_Read_Data(FLASH_USER_DATA_ADDR + OFFSET_RX_ADDRESS, rxBuffer, 2);

    RX_ADDRESS[0] = (uint8_t)(rxBuffer[0] & 0xFF);
    RX_ADDRESS[1] = (uint8_t)((rxBuffer[0] >> 16) & 0xFF);
    RX_ADDRESS[2] = (uint8_t)((rxBuffer[0] >> 24) & 0xFF);
    RX_ADDRESS[3] = (uint8_t)(rxBuffer[1] & 0xFF);
    RX_ADDRESS[4] = (uint8_t)((rxBuffer[1] >> 8) & 0xFF);
}

// 从Flash加载所有用户数据
void User_Data_Load_All(void)
{
    uint32_t buffer[128];
    Flash_Read_Data(FLASH_USER_DATA_ADDR, buffer, 128);

    // 初始化状态，避免未写入Flash时误判为已配对
    ALL_flag.unlock = 0;
    ALL_flag.sleep_timeout_s = 0;
    ALL_flag.paired = 0;
    ALL_flag.mpu_calibrated = 0;

    // 1. Calibration
    if (buffer[OFFSET_CALIB_FLAG / 4] == CALIBRATION_FLAG)
    {
        MpuOffset[0] = (int16_t)(buffer[OFFSET_CALIB_DATA / 4 + 0] & 0xFFFF);
        MpuOffset[1] = (int16_t)(buffer[OFFSET_CALIB_DATA / 4 + 0] >> 16);
        MpuOffset[2] = (int16_t)(buffer[OFFSET_CALIB_DATA / 4 + 1] & 0xFFFF);
        MpuOffset[3] = (int16_t)(buffer[OFFSET_CALIB_DATA / 4 + 1] >> 16);
        MpuOffset[4] = (int16_t)(buffer[OFFSET_CALIB_DATA / 4 + 2] & 0xFFFF);
        MpuOffset[5] = (int16_t)(buffer[OFFSET_CALIB_DATA / 4 + 2] >> 16);
    }

    // 2. Pairing
    // 只有当Flash中Pair数据不是空白(0xFFFFFFFF)时，才加载ALL_flag
    if (buffer[OFFSET_PAIR_FLAG / 4] != 0xFFFFFFFFU)
    {
        *(uint32_t *)&ALL_flag = buffer[OFFSET_PAIR_FLAG / 4];
    }

    // 3. PID
    if (buffer[OFFSET_PID_FLAG / 4] == PID_SAVED_FLAG)
    {
        for (int i = 0; i < 8; i++)
        {
            uint32_t *pPidData = &buffer[OFFSET_PID_DATA / 4 + i * 3];
            pPids[i]->kp = *(float *)&pPidData[0];
            pPids[i]->ki = *(float *)&pPidData[1];
            pPids[i]->kd = *(float *)&pPidData[2];
        }
    }

    // 4. RX_ADDRESS
    Flash_Load_RxAddress();
}

// 重置PID：从Flash重新读取
void User_Data_Reset_PID(void)
{
    uint32_t buffer[128];
    Flash_Read_Data(FLASH_USER_DATA_ADDR, buffer, 128);
    if (buffer[OFFSET_PID_FLAG / 4] == PID_SAVED_FLAG)
    {
        for (int i = 0; i < 8; i++)
        {
            uint32_t *pPidData = &buffer[OFFSET_PID_DATA / 4 + i * 3];
            pPids[i]->kp = *(float *)&pPidData[0];
            pPids[i]->ki = *(float *)&pPidData[1];
            pPids[i]->kd = *(float *)&pPidData[2];
        }
    }
}

// 检查是否校准过
int8_t User_Data_Check_Calibrated(void)
{
    uint32_t flag;
    Flash_Read_Data(FLASH_USER_DATA_ADDR + OFFSET_CALIB_FLAG, &flag, 1);
    return (flag == CALIBRATION_FLAG);
}

// 检查是否配对过
int8_t User_Data_Check_Paired(void)
{
    uint32_t flag;
    Flash_Read_Data(FLASH_USER_DATA_ADDR + OFFSET_PAIR_FLAG, &flag, 1);
    if (flag == 0xFFFFFFFFU)
    {
        return 0;
    }
    _st_ALL_flag temp_flag;
    *(uint32_t *)&temp_flag = flag;
    return temp_flag.paired;
}

// 检查 MPU6050 是否已校准
int8_t User_Data_Check_MpuCalibrated(void)
{
    uint32_t flag;
    Flash_Read_Data(FLASH_USER_DATA_ADDR + OFFSET_PAIR_FLAG, &flag, 1);
    if (flag == 0xFFFFFFFFU)
    {
        return 0;
    }
    // 从 Flash 读取的数据中解析 ALL_flag 结构体
    _st_ALL_flag temp_flag;
    *(uint32_t *)&temp_flag = flag;
    // 直接检查结构体中的 mpu_calibrated 成员
    return temp_flag.mpu_calibrated;
}
