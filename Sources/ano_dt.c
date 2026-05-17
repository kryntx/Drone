#include "ano_dt.h"
#include <stdlib.h>
#include <string.h>
#include "all_data.h"
#include "uart.h"
#include "main.h"
#include "mymath.h"
#include "remote.h"
#include "nrf24l01.h"
#include "led.h"
#include "stdio.h"
#include "flash.h"
#include "stm32f4xx_hal_tim.h"
#include "mpu6050.h"

extern TIM_HandleTypeDef htim3;

static uint8_t RatePID[19]; // 前18个字节为三组PID，最后1个字节为检验SUM,下同
static uint8_t AnglePID[19];
static uint8_t HighPID[19];
static uint8_t PID4[19];
static uint8_t PID5[19];
static uint8_t PID6[19];
extern uint8_t nrf2401_txbuf[];
extern uint8_t txbuf_pos;
extern uint8_t nrf2401_tx_flag;
extern uint16_t voltage;
extern uint8_t RX_ADDRESS[RX_ADR_WIDTH];
extern uint8_t TX_ADDRESS[TX_ADR_WIDTH];

uint8_t mpu_cal_flag = 0; // MPU校准标志位，1表示需要校准，0表示校准完成

extern UART_HandleTypeDef huart1;
extern uint32_t baro_height;

static struct
{
  uint8_t PID1 : 1;          // 接受到上位机PID组1
  uint8_t PID2 : 1;          // 接受到上位机PID组2
  uint8_t PID3 : 1;          // 接受到上位机PID组3
  uint8_t PID4 : 1;          // 接受到上位机PID组4
  uint8_t PID5 : 1;          // 接受到上位机PID组5
  uint8_t PID6 : 1;          // 接受到上位机PID组6
  uint8_t CMD2_READ_PID : 1; // 接受到上位机读取PID的请求
} ANTO_Recived_flag;

extern uint8_t nrf_addr_init;
/***********************************************************************
 *
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void ANO_Recive(uint8_t* pt) // 接收到上位机的数据
{
  switch (pt[2])
  {
    case ANTO_CMD2: // 0x02: 上位机发来的CMD2 包含请求读取PID等
    {
      switch (*(uint8_t*)&pt[4]) // 判断上位机发来CMD的内容
      {
        case 0x01: // 请求读取PID参数
          if (ANTO_Recived_flag.CMD2_READ_PID == 0)
          {
            ANTO_Recived_flag.CMD2_READ_PID = 1;
          }
          break;
        case 0xA1: // 恢复默认PID参数
          pid_param_Init();
          break;
        default:
          break;
      }
    }
    break;
    case ANTO_RCDATA:                                  // 0x03   //遥控器发的油门和姿态角的控制命令
      Remote.thr = ((uint16_t)pt[4] << 8) | pt[5];     // 通道1  油门
      Remote.yaw = ((uint16_t)pt[6] << 8) | pt[7];     // 通道2  航向角
      Remote.roll = ((uint16_t)pt[8] << 8) | pt[9];    // 通道3  横滚角
      Remote.pitch = ((uint16_t)pt[10] << 8) | pt[11]; // 通道4  俯仰角
      Remote.AUX1 = ((uint16_t)pt[12] << 8) | pt[13];  // 通道5  左上角按键都属于通道5
      Remote.AUX2 = ((uint16_t)pt[14] << 8) | pt[15];  // 通道6  右上角按键都属于通道6
      Remote.AUX3 = ((uint16_t)pt[16] << 8) | pt[17];  // 通道7  左下边按键都属于通道7
      Remote.AUX4 = ((uint16_t)pt[18] << 8) | pt[19];  // 通道8  右下边按键都属于通道8
      LIMIT(Remote.thr, 1000, 2000);
      LIMIT(Remote.yaw, 1000, 2000);
      LIMIT(Remote.roll, 1000, 2000);
      LIMIT(Remote.pitch, 1000, 2000);
      LIMIT(Remote.AUX1, 1000, 2000);
      LIMIT(Remote.AUX2, 1000, 2000);
      LIMIT(Remote.AUX3, 1000, 2000);
      LIMIT(Remote.AUX4, 1000, 2000);

      const float roll_pitch_ratio = 0.04f; // 0.02f
      const float yaw_ratio = 0.0015f;

      pidPitch.desired = (1500 - Remote.pitch) * roll_pitch_ratio; // 将遥杆值作为飞行角度的期望值
      pidRoll.desired = (1500 - Remote.roll) * roll_pitch_ratio;
      pidYaw.desired = (1500 - Remote.yaw) * yaw_ratio; // 直接将遥杆值作为偏航角的期望值，偏航环不使用PID控制了

      if (Remote.yaw > 1820)
      {
        pidYaw.desired += 0.75f;
      }
      else if (Remote.yaw < 1180)
      {
        pidYaw.desired -= 0.75f;
      }
      // printf("thr=%d, yaw=%d, roll=%d, pitch=%d\r\nAUX1=%d, AUX2=%d, AUX3=%d, AUX4=%d\r\n", Remote.thr, Remote.yaw, Remote.roll, Remote.pitch, Remote.AUX1, Remote.AUX2, Remote.AUX3, Remote.AUX4);
      remote_unlock();
      break;
    case ANTO_RATE_PID:            // 0x10  这组的PID是给速度环用的
      ANTO_Recived_flag.PID1 = 1;  // 接收到上位机发来的PID数据
      memcpy(RatePID, &pt[4], 19); // 先把接收到的数据提出来，防止被下一组PID数据覆盖
      break;
    case ANTO_ANGLE_PID: // 0x11      这组的PID是给角度环用的
      memcpy(AnglePID, &pt[4], 19);
      ANTO_Recived_flag.PID2 = 1;
      break;
    case ANTO_HEIGHT_PID: // 0x12      这组的PID是给高度环用的
      memcpy(HighPID, &pt[4], 19);
      ANTO_Recived_flag.PID3 = 1;
      break;
    case ANTO_PID4:
      memcpy(PID4, &pt[4], 19);
      ANTO_Recived_flag.PID4 = 1;
      break;
    case ANTO_PID5:
      memcpy(PID5, &pt[4], 19);
      ANTO_Recived_flag.PID5 = 1;
      break;
    case ANTO_PID6:
      memcpy(PID6, &pt[4], 19);
      ANTO_Recived_flag.PID6 = 1;
      break;
    case ANTO_CMD1: // 0x01: 上位机发来的CMD1 包含各种校准
    {
      switch (*(uint8_t*)&pt[4]) // 判断上位机发来CMD的内容
      {
        case 0x01: // 校准陀螺仪
          mpu_cal_flag = 1;
          break;
        case 0x02: // 校准加速度计
          mpu_cal_flag = 1;
          break;
        default:
          break;
      }
    }
    break;
    case ANTO_PAIRING:
      memcpy(RX_ADDRESS, &pt[4], 5);
      // RX_ADDRESS[0] = pt[4];
      // RX_ADDRESS[1] = pt[5];
      // RX_ADDRESS[2] = pt[6];
      // RX_ADDRESS[3] = pt[7];
      // RX_ADDRESS[4] = pt[8];

      // TX_ADDRESS[0] = pt[4];
      // TX_ADDRESS[1] = pt[5];
      // TX_ADDRESS[2] = pt[6];
      // TX_ADDRESS[3] = pt[7];
      // TX_ADDRESS[4] = pt[8];
      ALL_flag.paired = 1;
      nrf_addr_init = 1;
      // RX_Mode();
      User_Data_Save_All();
      
      break;
    case 0x20:
      RX_ADDRESS[0] = TX_ADDRESS[0];
      RX_ADDRESS[1] = TX_ADDRESS[1];
      RX_ADDRESS[2] = TX_ADDRESS[2];
      RX_ADDRESS[3] = TX_ADDRESS[3];
      RX_ADDRESS[4] = TX_ADDRESS[4];
      nrf_addr_init = 1;
      // RX_Mode();
      ALL_flag.paired = 0;
    break;
		case 0x21:
			ALL_flag.unlock = 0;
			LED.status = AllFlashLight;
			break;
    case 0x18:
      if (pt[4] == 0x88)
      {
        ALL_flag.unlock = 1;
        LED.status = AlwaysOn;
				break;
      }
      ALL_flag.unlock = 0;
			LED.status = AllFlashLight;
      break;
    case 0x17:
      // printf("%d", pt[4]);
      ALL_flag.sleep_timeout_s = (uint8_t)pt[4];
      break;
    case 0x19:
      ANTO_Send(ANTO_STATUS);
      break;
    case 0x09: // 接收定高数据
      if (pt[4] == 0x30)
      {
        pidHeightHigh.desired = ((uint32_t)pt[5] << 24) | ((uint32_t)pt[6] << 16) | ((uint32_t)pt[7] << 8) | pt[8];
        pidHeightHigh.desired /= 100.0f;
      }
      break;
    default:
      break;
  }
  return;
}

/***********************************************************************
 *
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void ANTO_Send(const enum ANTO_SEND FUNCTION) // 发送数据到上位机
{
  uint8_t i;
  uint8_t len = 2;
  int16_t Anto[12];
  int8_t* pt = (int8_t*)(Anto);
  PidObject* pidX = 0;
  PidObject* pidY = 0;
  PidObject* pidZ = 0;

  switch (FUNCTION)
  {
    case ANTO_STATUS: // 0x01  send angle// AAAF01
      Anto[2] = (int16_t)(-Angle.roll * 100);
      Anto[3] = (int16_t)(Angle.pitch * 100);
      Anto[4] = (int16_t)(-Angle.yaw * 100);
      Anto[5] = baro_height >> 16;
      Anto[6] = baro_height;
      Anto[7] = (0x01 << 8) | (ALL_flag.unlock);
      len = 12;
      break;
    case ANTO_MPU_MAGIC: // 0x02  发送MPU6050和磁力计的数据
      memcpy(&Anto[2], (int8_t*)&MPU6050, sizeof(_st_Mpu));
      memcpy(&Anto[8], (int8_t*)&AK8975, sizeof(_st_Mag));
      len = 18;
      break;
    case ANTO_RATE_PID: // 0x10  PID1
      pidX = &pidRateX;
      pidY = &pidRateY;
      pidZ = &pidRateZ;
      goto send_pid;
    case ANTO_ANGLE_PID: // 0x11  PID2
      pidX = &pidRoll;
      pidY = &pidPitch;
      pidZ = &pidYaw;
      goto send_pid;
    case ANTO_HEIGHT_PID: // 0x12  PID3
      pidX = &pidHeightRate;
      pidY = &pidHeightHigh;
      goto send_pid;
    case ANTO_PID4: // PID4
    case ANTO_PID5: // PID5
    case ANTO_PID6: // PID6
    send_pid:
      if (pidX != NULL)
      {
        Anto[2] = (int16_t)(pidX->kp * 1000);
        Anto[3] = (int16_t)(pidX->ki * 1000);
        Anto[4] = (int16_t)(pidX->kd * 1000);
      }
      if (pidY != NULL)
      {
        Anto[5] = (int16_t)(pidY->kp * 1000);
        Anto[6] = (int16_t)(pidY->ki * 1000);
        Anto[7] = (int16_t)(pidY->kd * 1000);
      }
      if (pidZ != NULL)
      {
        Anto[8] = (int16_t)(pidZ->kp * 1000);
        Anto[9] = (int16_t)(pidZ->ki * 1000);
        Anto[10] = (int16_t)(pidZ->kd * 1000);
      }
      len = 18;
      break;
    case ANTO_CHECK:
      if (ANTO_Recived_flag.PID1)
      {
        pt[5] = 0x10;        //  后面会交换pt[4]和pt[5]的值
        pt[4] = RatePID[18]; // 最后一个字节为校验SUM
      }
      else if (ANTO_Recived_flag.PID2)
      {
        pt[5] = 0x11;         //
        pt[4] = AnglePID[18]; // 最后一个字节为校验SUM
      }
      else if (ANTO_Recived_flag.PID3)
      {
        pt[5] = 0x12;        //
        pt[4] = HighPID[18]; // 最后一个字节为校验SUM
      }
      else if (ANTO_Recived_flag.PID4)
        pt[5] = 0x13; //
      else if (ANTO_Recived_flag.PID5)
        pt[5] = 0x14; //
      else if (ANTO_Recived_flag.PID6)
        pt[5] = 0x15; //
      len = 2;
      break;
    case ANTO_RCDATA: // 0x03  send RC data
      Anto[2] = Remote.thr;
      Anto[3] = Remote.yaw;
      Anto[4] = Remote.roll;
      Anto[5] = Remote.pitch;
      Anto[6] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
      Anto[7] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
      Anto[8] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
      Anto[9] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4);
      len = 20;
      break;
    case ANTO_POWER: // 0x05 adv
      Anto[2] = voltage;
      Anto[3] = 0;
      len = 4;
      break;
    case ANTO_MOTOR:

      break;
    case ANTO_SENSER2: // 0x07

      break;
    default:
      break;
  }

  Anto[0] = 0xAAAA;
  Anto[1] = (FUNCTION << 8) | len;
  pt[len + 4] = (int8_t)(0xAA + 0xAA);
  for (i = 2; i < len + 4; i += 2) // a swap with b;
  {
    pt[i] ^= pt[i + 1];
    pt[i + 1] ^= pt[i];
    pt[i] ^= pt[i + 1];
    pt[len + 4] += pt[i] + pt[i + 1];
  }

  // send_char_array(&huart1, (uint8_t*)pt, len + 5);

  if ((FUNCTION != ANTO_STATUS) || (nrf2401_tx_flag == 1))
  {
    txbuf_pos = len + 5;
    for (i = 0; i < txbuf_pos; i++)
      nrf2401_txbuf[i] = pt[i];
    nrf2401_tx_flag = 0;
  }
}
/***********************************************************************
 * polling  work.
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void ANTO_polling(void) // 轮询扫描上位机端口
{
  volatile static uint8_t status = 1;
  volatile static uint8_t status1 = 1;
  volatile static uint8_t status2 = 1;
  switch (status)
  {
    case 1:
    {
      // 一旦接收到上位机的数据，则暂停发送状态数据到上位机，转而去判断上位机要求飞控做什么。
      if (*(uint8_t*)&ANTO_Recived_flag != 0)
      {
        status = 2;
      }
      if (nrf2401_tx_flag == 1)
      {
        switch (status1)
        {
          case 1:
            ANTO_Send(ANTO_STATUS);
            status1 = 2;
            break;
          case 2:
            // ANTO_Send(ANTO_MPU_MAGIC);
            status1 = 3;
            break;
          case 3:
            // ANTO_Send(ANTO_RCDATA);
            status1 = 4;
            break;
          case 4:
            ANTO_Send(ANTO_POWER);
            status1 = 1;
            break;
          default:
            status1 = 1;
            break;
        }
      }
    }

    break;
    case 2:
      if (*(uint8_t*)&ANTO_Recived_flag == 0) // 上位机的发过来的数据都被处理了，则返回专心的发送数据到上位机
      {
        status = 1;
      }
      // 判断上位机是否请求发发送PID数据到上位机
      if ((ANTO_Recived_flag.CMD2_READ_PID) && (nrf2401_tx_flag == 1))
      {
        switch (status2)
        {
          case 1:
            ANTO_Send(ANTO_RATE_PID);
            status2 = 2;
            break;
          case 2:
            ANTO_Send(ANTO_ANGLE_PID);
            status2 = 3;
            break;
          case 3:
            ANTO_Send(ANTO_HEIGHT_PID);
            ANTO_Recived_flag.CMD2_READ_PID = 0;
            status2 = 1;
            break;
          default:
            break;
        }
      }

      if (*(uint8_t*)&ANTO_Recived_flag & 0x3f) // 接收到上位机发来的PID数据 0011 1111
      {
        PidObject* pidX = 0;
        PidObject* pidY = 0;
        PidObject* pidZ = 0;
        uint8_t* P;
        ANTO_Send(ANTO_CHECK);
        if (ANTO_Recived_flag.PID1)
        {
          pidX = &pidRateX;
          pidY = &pidRateY;
          pidZ = &pidRateZ;
          P = RatePID;
          ANTO_Recived_flag.PID1 = 0;
        }
        else if (ANTO_Recived_flag.PID2)
        {
          pidX = &pidRoll;
          pidY = &pidPitch;
          pidZ = &pidYaw;
          P = AnglePID;
          ANTO_Recived_flag.PID2 = 0;
        }
        else if (ANTO_Recived_flag.PID3)
        {
          pidX = &pidHeightRate;
          pidY = &pidHeightHigh;
          P = HighPID;
          ANTO_Recived_flag.PID3 = 0;
        }
        else
        {
          ANTO_Recived_flag.PID4 = 0;
          ANTO_Recived_flag.PID5 = 0;
          ANTO_Recived_flag.PID6 = 0;
          break;
        }
        {
          union
          {
            uint16_t _16;
            uint8_t _u8[2];
          } data;

          if (pidX != NULL)
          {
            data._u8[1] = P[0];
            data._u8[0] = P[1];
            pidX->kp = data._16 / 1000.0f;
            data._u8[1] = P[2];
            data._u8[0] = P[3];
            pidX->ki = data._16 / 1000.0f;
            data._u8[1] = P[4];
            data._u8[0] = P[5];
            pidX->kd = data._16 / 1000.0f;
          }
          if (pidY != NULL)
          {
            data._u8[1] = P[6];
            data._u8[0] = P[7];
            pidY->kp = data._16 / 1000.0f;
            data._u8[1] = P[8];
            data._u8[0] = P[9];
            pidY->ki = data._16 / 1000.0f;
            data._u8[1] = P[10];
            data._u8[0] = P[11];
            pidY->kd = data._16 / 1000.0f;
          }
          if (pidZ != NULL)
          {
            data._u8[1] = P[12];
            data._u8[0] = P[13];
            pidZ->kp = data._16 / 1000.0f;
            data._u8[1] = P[14];
            data._u8[0] = P[15];
            pidZ->ki = data._16 / 1000.0f;
            data._u8[1] = P[16];
            data._u8[0] = P[17];
            pidZ->kd = data._16 / 1000.0f;
          }
        }
      }
      break;
    default:
      break;
  }
}
