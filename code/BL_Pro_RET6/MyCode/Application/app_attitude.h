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
uint8_t App_Attitude_IsControlEnabled(void);

#endif
