#ifndef APP_ATTITUDE_DEBUG_H
#define APP_ATTITUDE_DEBUG_H

#include <stdint.h>

#include "app_attitude.h"

void App_Attitude_GetTelemetry(App_AttitudeTelemetry_t *telemetry);

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
