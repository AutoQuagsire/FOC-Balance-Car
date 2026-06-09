#ifndef   APP_FOC_ITTIMER_H
#define   APP_FOC_ITTIMER_H

#include <stdint.h>
#include "debug_link.h"
#include "usb_debug.h"
#include "BLDCMotor.h"


uint8_t App_FOCControlIT_ConfigFromTimer(TIM_HandleTypeDef *htim);
uint8_t App_FOCControlIT_Enable(TIM_HandleTypeDef *htim);
void App_FOCControlIT_Disable(void);




#endif