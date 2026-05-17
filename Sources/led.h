#ifndef __LED_H
#define __LED_H

#include "main.h"

// 机身前灯   LED1右前灯，LED2左前灯
#define LED1_OFF() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_ON() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_Toggle() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define LED2_OFF() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_ON() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_Toggle() HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)

// 机身后灯  LED3左后灯，LED4右后灯
#define LED3_OFF() HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_ON() HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_Toggle() HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

#define LED4_OFF() HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED4_ON() HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)
#define LED4_Toggle() HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin)

typedef struct
{
  uint16_t FlashTime;
  enum
  {
    AlwaysOn,
    AlwaysOff,
    AllFlashLight,
    AlternateFlash,
    WARNING,
    DANGEROURS,
    PAIRING,
    GET_OFFSET
  } status;
} sLED;

extern sLED LED;
extern void PilotLED(void);

#endif
