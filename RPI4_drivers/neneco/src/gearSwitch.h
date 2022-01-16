
#ifndef _GEAR_SWITCH_H_
#define _GEAR_SWITCH_H_

#include "FreeRTOS.h"
#include "semphr.h"

typedef enum gearSwitch{GEAR_D=0, GEAR_P, GEAR_R}gearSwitch_t;
xSemaphoreHandle gearSem;
gearSwitch_t currentGearSwitchValue;

#endif //_GEAR_SWITCH_H_