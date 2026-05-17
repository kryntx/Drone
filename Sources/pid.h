#ifndef __PID_H
#define __PID_H

#include "all_data.h"
#include <stdbool.h>

extern void CascadePID(PidObject *pidRate, PidObject *pidAngE, const float dt); // 串级PID
extern void pidRest(PidObject **pid, const uint8_t len);                        // pid数据复位
extern void pidUpdate(PidObject *pid, const float dt);                          // PID

#endif
