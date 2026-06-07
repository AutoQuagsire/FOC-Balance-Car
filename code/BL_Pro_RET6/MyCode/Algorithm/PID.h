#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "BLDCMotor.h"

typedef struct {
    float Kp, Ki, Kd;      // 参数
    float error_integral;  // 内部：误差累计值 (外部不要修改)
    float last_error;      // 内部：上次误差 (外部不要修改)
    float last_last_error; // 内部：上上次误差，增量式PID用 (外部不要修改)
    float output;          // 输出
	float integral_limit;  // 积分限幅幅度
    float output_limit;  // 输出限幅幅度
    float I_SEP_RATIO;  //积分带比例
    float I_ERR_MIN;//控制量量纲大小，避免积分带过窄
    float unwind_gain; // per-loop integral unload gain
} PID_t;

#define PID_CURRENT_FREEZE_INTEGRAL   (0x01U)
#define PID_CURRENT_LIMIT_I_UNLOAD    (0x02U)
// PID控制函数
void PID_Calculate(PID_t *pid, float target, float measure, uint8_t freeze_external);
void PID_CalculateDt(PID_t *pid, float target, float measure, float dt, uint8_t freeze_external);
float PID_SlewLimit(float prev_value, float target, float step_up_max, float step_down_max);
void PID_CalculateTest(PID_t *pid, float target, float measure);
void PID_CalculateIncrementalTest(PID_t *pid, float target, float measure);
void PID_CalCurrent(PID_t *pid, float target, float measure, uint8_t freeze_external);
void PID_Reset(PID_t *pid);
void PID_ParameterInit(PID_t *pid,float kp,float ki,float kd,float integral_limit);
void PID_ParameterInitEx(PID_t *pid,float kp,float ki,float kd,float integral_limit,
                 float output_limit,float i_err_min,float i_sep_ratio);

#endif
