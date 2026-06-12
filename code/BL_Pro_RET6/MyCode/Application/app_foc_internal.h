#ifndef APP_FOC_INTERNAL_H
#define APP_FOC_INTERNAL_H

#include <stdint.h>

#include "BLDCMotor.h"
#include "BusVoltage.h"
#include "PID.h"
#include "app_foc_control.h"
#include "current_sense.h"
#include "driver.h"
#include "sensor.h"
#include "app_foc_debug.h"

#define APP_CURRENT_I_LIMIT             (5.0f)
#define APP_CURRENT_I_ERR_MIN           (0.05f)
#define APP_CURRENT_OUT_LIMIT           (10.963f)
#define APP_CURRENT_DEBUG_PRINT_PERIOD_MS (100U)

#ifndef APP_SPEED_LOOP_ENABLE
#define APP_SPEED_LOOP_ENABLE           (1U)
#endif

#ifndef APP_CURRENT_LOOP_ENABLE
#define APP_CURRENT_LOOP_ENABLE         (1U)
#endif

extern volatile uint8_t g_foc_stack_ready;
extern volatile uint8_t g_foc_system_enabled;
extern uint8_t g_bus_voltage_valid;
extern float g_bus_voltage_filtered;
extern volatile uint8_t g_foc_control_it_enabled;
extern volatile uint32_t g_foc_loop_count;
extern volatile uint32_t g_foc_last_loop_tick_ms;

extern volatile BusVoltageDebug_t g_bus_voltage_debug;

extern Motor_t g_motor1;
extern Motor_t g_motor2;
extern Driver_t *g_driver1;
extern Driver_t *g_driver2;
extern Sensor_t g_sensor1;
extern Sensor_t g_sensor2;
extern CurrentSense_t g_current_sense1;
extern CurrentSense_t g_current_sense2;

extern volatile CurrentLoopDebugSnapshot_t g_current_loop_debug1;
extern volatile CurrentLoopDebugSnapshot_t g_current_loop_debug2;

extern volatile uint8_t g_current_pid_mode;
extern volatile float g_current_i_sep_ratio_ff;
extern volatile float g_current_i_sep_ratio_pure;

extern uint8_t g_current_i_unload_limit_ticks1;
extern uint8_t g_current_i_unload_limit_ticks2;
extern float g_speed_fault1;
extern float g_speed_fault2;

void App_PIDResetRuntime(PID_t *pid);
void App_FastRingPushDual(void);
uint8_t App_FOC_BusInit(void);

#endif
