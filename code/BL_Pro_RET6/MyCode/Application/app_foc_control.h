#ifndef APP_FOC_CONTROL_H
#define APP_FOC_CONTROL_H

#include "BLDCMotor.h"
#include "Filter.h"
#include "PID.h"

typedef struct {
    PID_t speed_pid;
    LowPassFilter_t speed_lpf;

    float speed_meas_radps;
    float speed_target_radps;
} VelocityCube_t;

typedef struct {
    PID_t pid_ff;
    PID_t pid_pi;
    LowPassFilter_t current_lpf;

    float iq_target;
    float iq_ref;
    float iq_meas;
} CurrentCube_t;

typedef struct {
    Motor_t *motor;
    VelocityCube_t velocity;
    CurrentCube_t current;
} App_FOCMotorControl_t;

extern App_FOCMotorControl_t g_foc_left_control;
extern App_FOCMotorControl_t g_foc_right_control;

#endif
