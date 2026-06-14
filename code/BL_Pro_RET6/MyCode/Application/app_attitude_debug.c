#include "app_attitude_debug.h"

#include <math.h>

#include "app_attitude_internal.h"
#include "stm32g4xx_hal.h"

static uint8_t App_Attitude_IsFiniteInRange(float value, float min_value, float max_value)
{
    if (!isfinite(value)) {
        return 0U;
    }
    if ((value < min_value) || (value > max_value)) {
        return 0U;
    }
    return 1U;
}

void App_Attitude_GetTelemetry(App_AttitudeTelemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    __disable_irq();
    *telemetry = g_attitude_telemetry;
    __enable_irq();
}

uint8_t App_Attitude_SetSpeedKp(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.0f, 1000.0f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.Speed_Control.pid.Kp = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetSpeedKp(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.Speed_Control.pid.Kp;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetSpeedKi(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.0f, 1000.0f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.Speed_Control.pid.Ki = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetSpeedKi(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.Speed_Control.pid.Ki;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetSpeedPitchLimitRad(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.001f, 3.2f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.Speed_Control.pid.output_limit = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetSpeedPitchLimitRad(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.Speed_Control.pid.output_limit;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetSpeedUnwindGain(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.1f, 20.0f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.Speed_Control.pid.unwind_gain = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetSpeedUnwindGain(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.Speed_Control.pid.unwind_gain;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetAttitudeKp(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.0f, 1000.0f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.attitude_control.kp = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetAttitudeKp(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.attitude_control.kp;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetAttitudeKd(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.0f, 1000.0f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.attitude_control.kd = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetAttitudeKd(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.attitude_control.kd;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetAttitudeIqLimit(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.001f, 1000.0f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.attitude_control.output_limit = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetAttitudeIqLimit(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.attitude_control.output_limit;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_SetAttitudeShutdownRad(float value)
{
    if (App_Attitude_IsFiniteInRange(value, 0.001f, 3.2f) == 0U) {
        return 0U;
    }
    __disable_irq();
    g_attitude_control.attitude_control.shutdown_limit = value;
    __enable_irq();
    return 1U;
}

uint8_t App_Attitude_GetAttitudeShutdownRad(float *value)
{
    if (value == NULL) {
        return 0U;
    }
    __disable_irq();
    *value = g_attitude_control.attitude_control.shutdown_limit;
    __enable_irq();
    return 1U;
}
