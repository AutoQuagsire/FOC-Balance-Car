#include "PID.h"
#include "foc_common.h"
#include "BLDCMotor.h"
#include <math.h>

#ifndef Uq_max
#define Uq_max 6.0f
#endif


/* 清空 PID 运行状态。
 * 注意：这里只清积分、上次误差和输出，不改变 Kp/Ki/Kd 等参数。
 */
void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error_integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->output = 0.0f;
}


/* 通用 PID 计算函数。
 *
 * 特点：
 * 1. 带积分分离：误差进入 i_band 后才正常积分；
 * 2. 带反向卸载：积分项方向和误差方向相反时，允许积分回退；
 * 3. 带输出饱和冻结：输出已经顶限且误差继续推向饱和时，冻结积分；
 * 4. freeze_external 可由外部强制冻结积分。
 *
 * 当前更适合速度环/位置环等通用 PID；
 * 电流环建议使用下面的 PID_CalCurrent()。
 */
__attribute__((optimize("O2,fast-math")))
void PID_Calculate(PID_t *pid, float target, float measure, uint8_t freeze_external)
{
    float error = target - measure;
    float unwind_gain;
    float i_band = pid->I_SEP_RATIO * fabsf(target);
    float derivative;
    float p_term;
    float d_term;
    float i_term_pre;
    float u_raw_pre;
    uint8_t freeze_by_sat;
    uint8_t freeze_integral;
    uint8_t allow_integrate;
    uint8_t allow_unwind;
    float i_term;

    if (pid == NULL) {
        return;
    }

    unwind_gain = (pid->unwind_gain > 0.0f) ? pid->unwind_gain : 1.0f;

    /* 积分分离阈值下限，避免 target 接近 0 时积分区间过小 */
    if (i_band < pid->I_ERR_MIN) {
        i_band = pid->I_ERR_MIN;
    }

    derivative = error - pid->last_error;
    p_term = pid->Kp * error;
    d_term = pid->Kd * derivative;

    /* 用限幅前输出预判是否会继续冲向饱和 */
    i_term_pre = pid->Ki * pid->error_integral;
    u_raw_pre = p_term + i_term_pre + d_term;

    /* 饱和方向和误差方向一致时，冻结积分，避免 windup */
    freeze_by_sat =
        ((u_raw_pre > pid->output_limit) && (error > 0.0f)) ||
        ((u_raw_pre < -pid->output_limit) && (error < 0.0f));

    freeze_integral = (freeze_external || freeze_by_sat);
    allow_integrate = (fabsf(error) <= i_band);

    /* 积分项和误差方向相反，说明积分正在被卸载，允许回退 */
    allow_unwind = (pid->error_integral * error < 0.0f);

    if (!freeze_integral && allow_integrate) {
        pid->error_integral += error;
    } else if (allow_unwind) {
        pid->error_integral += unwind_gain * error;
    }

    /* 积分输出限幅，并反推 integral 状态，保证状态和输出一致 */
    i_term = pid->Ki * pid->error_integral;
    if (i_term > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_term < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    i_term = pid->Ki * pid->error_integral;
    pid->output = p_term + i_term + d_term;

    /* 最终输出限幅 */
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = error;
}

/*
 * PID_CalculateDt():
 * Same control logic as PID_Calculate(), but integral/derivative are
 * normalized by dt to decouple tuning from loop rate.
 */
__attribute__((optimize("O2,fast-math")))
void PID_CalculateDt(PID_t *pid, float target, float measure, float dt, uint8_t freeze_external)
{
    float error = target - measure;
    float unwind_gain;
    float i_band = 0.0f;
    float dt_safe;
    float derivative;
    float p_term;
    float d_term;
    float i_term_pre;
    float u_raw_pre;
    uint8_t freeze_by_sat;
    uint8_t freeze_integral;
    uint8_t allow_integrate;
    uint8_t allow_unwind;
    float i_term;

    if (pid == NULL) {
        return;
    }

    unwind_gain = (pid->unwind_gain > 0.0f) ? pid->unwind_gain : 1.0f;
    dt_safe = (dt > 1e-6f) ? dt : 1e-6f;
    i_band = pid->I_SEP_RATIO * fabsf(target);

    if (i_band < pid->I_ERR_MIN) {
        i_band = pid->I_ERR_MIN;
    }

    derivative = (error - pid->last_error) / dt_safe;
    p_term = pid->Kp * error;
    d_term = pid->Kd * derivative;

    i_term_pre = pid->Ki * pid->error_integral;
    u_raw_pre = p_term + i_term_pre + d_term;

    freeze_by_sat =
        ((u_raw_pre > pid->output_limit) && (error > 0.0f)) ||
        ((u_raw_pre < -pid->output_limit) && (error < 0.0f));

    freeze_integral = (freeze_external || freeze_by_sat);
    allow_integrate = (fabsf(error) <= i_band);
    allow_unwind = (pid->error_integral * error < 0.0f);

    if (!freeze_integral && allow_integrate) {
        pid->error_integral += error * dt_safe;
    } else if (allow_unwind) {
        pid->error_integral += unwind_gain * error * dt_safe;
    }

    i_term = pid->Ki * pid->error_integral;
    if (i_term > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_term < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    i_term = pid->Ki * pid->error_integral;
    pid->output = p_term + i_term + d_term;

    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = error;
}

/*
 * Generic asymmetric slew limiter.
 *
 * prev_value: previous value
 * target: desired value
 * step_up_max: max positive step per call
 * step_down_max: max negative-step magnitude per call
 *
 * A non-positive step limit means that direction is unlimited.
 */
float PID_SlewLimit(float prev_value, float target, float step_up_max, float step_down_max)
{
    float delta = target - prev_value;
    float up = fabsf(step_up_max);
    float down = fabsf(step_down_max);

    if (delta > 0.0f) {
        if ((up > 0.0f) && (delta > up)) {
            delta = up;
        }
    } else if (delta < 0.0f) {
        if ((down > 0.0f) && ((-delta) > down)) {
            delta = -down;
        }
    }

    return prev_value + delta;
}


/* 限制电流环积分卸载速度。
 *
 * 只限制“积分项正在变小”的情况：
 * - 正积分被负误差拉低；
 * - 负积分被正误差拉高。
 *
 * 目的：
 * 在目标电流阶跃变化时，避免残余稳态积分被瞬间卸掉，
 * 导致 Uq_final 过低/过高，引发反向下冲或力矩突变。
 */
static float PID_LimitCurrentIntegralUnload(PID_t *pid, float delta_integral, float i_term_pre)
{
    float i_delta;

    if ((pid == NULL) ||
        (CURRENT_LOOP_I_UNLOAD_STEP_MAX <= 0.0f) ||
        (fabsf(pid->Ki) <= 1e-6f) ||
        (delta_integral == 0.0f) ||
        (i_term_pre == 0.0f)) {
        return delta_integral;
    }

    /* 把 integral 增量换算成 I 输出增量，限幅在输出量级上做 */
    i_delta = pid->Ki * delta_integral;

    /* i_term_pre 和 i_delta 同号，说明不是卸载，而是在继续堆积分 */
    if ((i_term_pre * i_delta) >= 0.0f) {
        return delta_integral;
    }

    /* 只限制 I 输出单周期卸载幅度 */
    if (i_delta > CURRENT_LOOP_I_UNLOAD_STEP_MAX) {
        i_delta = CURRENT_LOOP_I_UNLOAD_STEP_MAX;
    } else if (i_delta < -CURRENT_LOOP_I_UNLOAD_STEP_MAX) {
        i_delta = -CURRENT_LOOP_I_UNLOAD_STEP_MAX;
    }

    return i_delta / pid->Ki;
}


/* 电流环专用 PI 计算函数。
 *
 * 相比通用 PID：
 * 1. 不使用 D 项，避免电流采样噪声被放大；
 * 2. freeze_external 使用 bit 标志：
 *    - PID_CURRENT_FREEZE_INTEGRAL：外部请求冻结积分；
 *    - PID_CURRENT_LIMIT_I_UNLOAD：启用积分卸载限速；
 * 3. 保留积分分离、饱和冻结、反向卸载和积分限幅。
 *
 * 当前用于 CurrentLoop_FFPI_V1。
 */
__attribute__((optimize("O2,fast-math")))
void PID_CalCurrent(PID_t *pid, float target, float measure, uint8_t freeze_external)
{
    float error;
    float unwind_gain;
    float i_band;
    float p_term;
    float i_term_pre;
    float u_raw_pre;
    uint8_t freeze_requested;
    uint8_t limit_unload;
    uint8_t freeze_by_sat;
    uint8_t freeze_integral;
    uint8_t allow_integrate;
    uint8_t allow_unwind;
    float i_term;
    float i_delta = 0.0f;

    if (pid == NULL) {
        return;
    }

    unwind_gain = (pid->unwind_gain > 0.0f) ? pid->unwind_gain : 1.0f;
    error = target - measure;

    /* 积分分离区间随目标电流幅值变化，并设置最小下限 */
    i_band = pid->I_SEP_RATIO * fabsf(target);
    if (i_band < pid->I_ERR_MIN) {
        i_band = pid->I_ERR_MIN;
    }

    p_term = pid->Kp * error;
    i_term_pre = pid->Ki * pid->error_integral;
    u_raw_pre = p_term + i_term_pre;

    /* 外部控制标志 */
    freeze_requested = (uint8_t)((freeze_external & PID_CURRENT_FREEZE_INTEGRAL) != 0U);
    limit_unload = (uint8_t)((freeze_external & PID_CURRENT_LIMIT_I_UNLOAD) != 0U);

    /* 输出将继续冲向饱和时冻结积分，防止 windup */
    freeze_by_sat =
        ((u_raw_pre > pid->output_limit) && (error > 0.0f)) ||
        ((u_raw_pre < -pid->output_limit) && (error < 0.0f));

    freeze_integral = (uint8_t)(freeze_requested || freeze_by_sat);
    allow_integrate = (fabsf(error) <= i_band);

    /* 误差与积分方向相反时，允许积分卸载 */
    allow_unwind = (pid->error_integral * error < 0.0f);

    if (!freeze_integral && allow_integrate) {
        i_delta = error;
    } else if (allow_unwind) {
        i_delta = unwind_gain * error;
    }

    /* 目标阶跃阶段可限制积分卸载速度，降低反向下冲 */
    if ((i_delta != 0.0f) && limit_unload) {
        i_delta = PID_LimitCurrentIntegralUnload(pid, i_delta, i_term_pre);
    }

    if (i_delta != 0.0f) {
        pid->error_integral += i_delta;
    }

    /* 积分输出限幅，并回写 integral 状态 */
    i_term = pid->Ki * pid->error_integral;
    if (i_term > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_term < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    i_term = pid->Ki * pid->error_integral;
    pid->output = p_term + i_term;

    /* PI 总输出限幅 */
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = error;
}


/* 简化测试版 PID。
 *
 * 用途：
 * 仅用于对比实验或调试，不包含积分分离、饱和冻结、
 * 反向卸载等工程保护逻辑。
 */
void PID_CalculateTest(PID_t *pid, float target, float measure)
{
    float error = target - measure;
    float i_output;
    float derivative;

    pid->error_integral += error;

    /* 只保留积分限幅 */
    i_output = pid->Ki * pid->error_integral;
    if (i_output > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_output < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    derivative = error - pid->last_error;
    pid->last_error = error;

    pid->output = (pid->Kp * error) +
                  (pid->Ki * pid->error_integral) +
                  (pid->Kd * derivative);

        /* PI 总输出限幅 */
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }
    
}


/* 增量式 PID 测试版。
 *
 * 与 PID_CalculateTest（位置式）对照用，不接入程序。
 *
 * 位置式：u[k] = Kp*e[k] + Ki*Σe + Kd*(e[k]-e[k-1])
 * 增量式：Δu = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2*e[k-1]+e[k-2])
 *         u[k] = u[k-1] + Δu
 *
 * 增量式特点：
 * - 输出增量只与最近三次误差有关，无需累加历史全部误差；
 * - 无积分饱和问题（自然截断）；
 * - 手动切换无扰动更容易（只需设 Δu=0）。
 */
void PID_CalculateIncrementalTest(PID_t *pid, float target, float measure)
{
    float error = target - measure;
    float delta_u;

    delta_u = pid->Kp * (error - pid->last_error)
            + pid->Ki * error
            + pid->Kd * (error - 2.0f * pid->last_error + pid->last_last_error);

    pid->output += delta_u;

    /* 总输出限幅（增量式本身无积分饱和，但仍需限幅保护执行器） */
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_last_error = pid->last_error;
    pid->last_error = error;
}


/* 完整参数初始化。
 *
 * 初始化内容：
 * - PID 三参数；
 * - 积分输出限幅；
 * - 总输出限幅；
 * - 积分分离最小误差；
 * - 积分分离比例。
 */
void PID_ParameterInitEx(PID_t *pid, float kp, float ki, float kd, float integral_limit,
                         float output_limit, float i_err_min, float i_sep_ratio)
{
    if (pid == NULL) {
        return;
    }

    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->I_ERR_MIN = i_err_min;
    pid->I_SEP_RATIO = i_sep_ratio;
    pid->unwind_gain = 1.0f;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    PID_Reset(pid);
}


/* 兼容旧接口的参数初始化。
 *
 * 如果 pid 中已经预先设置了 output_limit / I_ERR_MIN / I_SEP_RATIO，
 * 则沿用原值；否则使用默认值。
 */
void PID_ParameterInit(PID_t *pid, float kp, float ki, float kd, float integral_limit)
{
    float output_limit = (pid->output_limit > 0.0f) ? pid->output_limit : Uq_max;
    float i_err_min = (pid->I_ERR_MIN > 0.0f) ? pid->I_ERR_MIN : 0.05f;
    float i_sep_ratio = (pid->I_SEP_RATIO > 0.0f) ? pid->I_SEP_RATIO : 0.5f;

    PID_ParameterInitEx(pid, kp, ki, kd, integral_limit,
                        output_limit, i_err_min, i_sep_ratio);
}
