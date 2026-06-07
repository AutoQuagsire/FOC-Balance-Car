#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdint.h>
#include "attitude_estimator.h"

#ifndef APP_ATTITUDE_USB_DEBUG_ENABLE
#define APP_ATTITUDE_USB_DEBUG_ENABLE 0
#endif

typedef struct
{
    float pitch_target_rad;
    float speed_p_term_rad;
    float speed_i_term_rad;
    float pitch_meas_rad;
    float pitch_rate_meas_radps;
    float speed_target_radps;
    float speed_meas_radps;
    float speed_raw_radps;
    float attitude_p_term_a;
    float attitude_d_term_a;
    float iq_cmd_a;
    float iq_cmd_clamped_a;
    float speed_output_limit_rad;
    float attitude_output_limit_a;
} App_AttitudeTelemetry_t;

uint8_t App_Attitude_Init(void);
void App_Attitude_Loop(void);
uint8_t App_Attitude_SetControlEnabled(uint8_t enable);
void App_Attitude_OnDrdyExtiISR(void);
void App_Attitude_OnSpi2DmaCpltISR(void);
void App_Attitude_OnSpi2DmaErrorISR(void);
float App_Attitude_GetPitch(void);
float App_Attitude_GetPitchRate(void);
void App_Attitude_GetTelemetry(App_AttitudeTelemetry_t *telemetry);
uint8_t App_Attitude_IsReady(void);
uint8_t App_Attitude_IsControlEnabled(void);
uint8_t App_Attitude_SetSpeedKp(float value);
uint8_t App_Attitude_GetSpeedKp(float *value);
uint8_t App_Attitude_SetSpeedKi(float value);
uint8_t App_Attitude_GetSpeedKi(float *value);
uint8_t App_Attitude_SetSpeedPitchLimitRad(float value);
uint8_t App_Attitude_GetSpeedPitchLimitRad(float *value);
uint8_t App_Attitude_SetSpeedUnwindGain(float value);
uint8_t App_Attitude_GetSpeedUnwindGain(float *value);
uint8_t App_Attitude_SetAttitudeKp(float value);
uint8_t App_Attitude_GetAttitudeKp(float *value);
uint8_t App_Attitude_SetAttitudeKd(float value);
uint8_t App_Attitude_GetAttitudeKd(float *value);
uint8_t App_Attitude_SetAttitudeIqLimit(float value);
uint8_t App_Attitude_GetAttitudeIqLimit(float *value);
uint8_t App_Attitude_SetAttitudeShutdownRad(float value);
uint8_t App_Attitude_GetAttitudeShutdownRad(float *value);

#endif
