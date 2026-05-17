#include "led.h"

sLED LED = {300, AllFlashLight};

void PilotLED()
{
  static uint32_t LastTime = 0;
  uint32_t SysTick_count = HAL_GetTick();

  if ((SysTick_count - LastTime) < LED.FlashTime)
  {
    return;
  }
  else
  {
    LastTime = SysTick_count;
  }

  switch (LED.status)
  {
  case AlwaysOff: // 常暗
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    LED4_OFF();
    break;
  case AllFlashLight: // 全部同时闪烁
    LED1_Toggle();
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    break;
  case AlwaysOn: // 常亮
    LED1_ON();
    LED2_ON();
    LED3_ON();
    LED4_ON();
    break;
  case AlternateFlash: // 1、2和3、4交替闪烁
    LED1_Toggle();
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    break;
  case WARNING: // 1、2快速闪烁 3、4常亮
    LED1_Toggle();
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    LED3_ON();
    LED4_ON();
    LED.FlashTime = 100;
    break;
  case DANGEROURS: // 1、2常亮 3、4快速闪烁
    LED1_ON();
    LED2_ON();
    LED3_Toggle();
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    LED.FlashTime = 70;
    break;
  case PAIRING: // 1、2交替闪烁 3、4常亮
    LED1_Toggle();
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) ? GPIO_PIN_RESET : GPIO_PIN_SET));
    LED3_ON();
    LED4_ON();
    LED.FlashTime = 200;
    break;
  default:
    LED.status = AlwaysOff;
    break;
  }
}
