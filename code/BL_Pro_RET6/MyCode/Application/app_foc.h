#ifndef APP_FOC_H
#define APP_FOC_H


#define FOC_ENABLE_CURRENT_LOOP 1
#define FOC_ENABLE_POSITION_LOOP 0
#define FOC_ENABLE_VELOCITY_LOOP 0



#include <stdint.h>
#include "app_foc_control.h"

#define APP_FOC_STATUS_FLAG_SPEED_FAULT_L        (1U << 0)
#define APP_FOC_STATUS_FLAG_SPEED_FAULT_R        (1U << 1)
#define APP_FOC_STATUS_FLAG_STACK_READY          (1U << 8)
#define APP_FOC_STATUS_FLAG_CONTROL_IT_ENABLED   (1U << 9)
#define APP_FOC_STATUS_FLAG_BUS_VALID            (1U << 10)
#define APP_FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE  (1U << 11)
#define APP_FOC_STATUS_FLAG_SPEED_LOOP_ENABLED   (1U << 12)
#define APP_FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED (1U << 13)
#define APP_FOC_STATUS_FLAG_POWER_STAGE_OFF      (1U << 14)
#define APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON  (1U << 15)

uint8_t App_FOCStack_Init(void);
uint8_t App_StartupCalibrate(void);
void App_Loop(void);
void App_LoopForIT(void);
void DebuginWhile(void);
void App_ResetSpeedPIDs(void);
void App_FOC_SetIqTarget(float left_iq, float right_iq);
float App_FOC_GetAverageWheelSpeedRadps(void);
uint8_t App_FOC_SetPowerStageEnabled(uint8_t enable);
uint8_t App_FOC_IsPowerStageEnabled(void);
uint8_t App_FOC_SetDriverGateEnabled(uint8_t enable);

/* DebugLink 用的状态变量 */
extern float uq_cmd1;
extern float uq_cmd2;
extern float g_speed_fault1;
extern float g_speed_fault2;

#endif
