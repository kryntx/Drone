#include "all_data.h"
#include "control.h"
#include <math.h>
#include "led.h"
#include "remote.h"
#include "ano_dt.h"
#include "nrf24l01.h"
#include "uart.h"

// #define SUCCESS 0
// #undef FAILED
// #define FAILED  1
/*****************************************************************************************
 *  通道数据处理
 * @param[in]
 * @param[out]
 * @return
 ******************************************************************************************/
uint16_t nrf_cnt = 0;
uint8_t RC_rxData[32];
extern UART_HandleTypeDef huart1;

void remote_unlock(void);

void RC_Analy(void)
{
  uint8_t rxlen;
  rxlen = NRF24L01_RxPacket(RC_rxData);
  if (rxlen > 0)
  {
    uint8_t i;
    uint8_t CheckSum = 0;
    nrf_cnt = 0;
    for (i = 0; i < rxlen - 1; i++)
    {
      CheckSum += RC_rxData[i];
    }
    if (RC_rxData[rxlen - 1] == CheckSum)
    {
      if (RC_rxData[0] == 0xAA && RC_rxData[1] == 0xAF)
      {
        send_char_array(&huart1, RC_rxData, rxlen);
        ANO_Recive(RC_rxData);
      }
    }
  }
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] if
 * @return
 ******************************************************************************************/
void remote_unlock(void)
{
  static uint8_t status = WAITING_1;
  static uint16_t cnt = 0;
  static uint16_t lock_cnt = 0;

  if (Remote.thr < 1200 && Remote.yaw < 1200) // 油门遥杆左下角锁定
  {
    if (lock_cnt++ > 100) // 需持续处于左下角一段时间（约0.6s-1s，取决于调用频率）
    {
      status = EXIT_255;
      lock_cnt = 0;
    }
  }
  else
  {
    lock_cnt = 0;
  }

  switch (status)
  {
  case WAITING_1:          // 等待解锁
    if (Remote.thr < 1150) // 解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
    {
      status = WAITING_2;
    }
    break;
  case WAITING_2:
    if (Remote.thr > 1800)
    {
      status = WAITING_3;
    }
    break;
  case WAITING_3:
    if (Remote.thr < 1150)
    {
      ALL_flag.unlock = 1; // 解锁前准备
      status = PROCESS_31;
      LED.status = AlwaysOn;
    }
    break;
  case PROCESS_31: // 进入解锁状态
    if (Remote.thr < 1020)
    {
      if (cnt++ > 1500) // 油门遥杆处于最低30S自动上锁
      {
        cnt = 0;
        status = EXIT_255;
      }
    }
    else if (!ALL_flag.unlock) // Other conditions lock
    {
      status = EXIT_255;
    }
    else
      cnt = 0;
    break;
  case EXIT_255:                // 进入锁定
    LED.status = AllFlashLight; // exit
    cnt = 0;
    LED.FlashTime = 100; // 100*3ms
    ALL_flag.unlock = 0;
    status = WAITING_1;
    break;
  default:
    status = EXIT_255;
    break;
  }
}
