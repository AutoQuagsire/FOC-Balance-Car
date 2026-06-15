#ifndef APP_ATTITUDE_CONTROL_H
#define APP_ATTITUDE_CONTROL_H

#include "BLDCMotor.h"
#include "PID.h"

/*
 * 速度环参数与运行对象集合。
 *
 * Unit convention:
 * - pid:                     速度环 PID 参数与运行时状态
 * - target_radps:            rad/s
 * - meas_radps:              rad/s
 */
typedef struct
{
    PID_t pid;
    float target_radps;
    float meas_radps;
} SpeedCube_t;

/*
 * 姿态环参数集合。
 *
 * Unit convention:
 * - kp / kd:        按姿态环实际控制量定义
 * - output_limit:   A，姿态环输出的电流指令限幅
 * - shutdown_limit: rad
 * - target_pitch_rad: rad
 * - meas_pitch_rad:   rad
 */
typedef struct
{
    float kp;
    float kd;
    float output_limit;
    float shutdown_limit;
    float target_pitch_rad;
    float meas_pitch_rad;
} AttitudeCube_t;

typedef struct
{
    AttitudeCube_t attitude_control;
    SpeedCube_t Speed_Control;
} App_AttitudeControl_t;

#endif
