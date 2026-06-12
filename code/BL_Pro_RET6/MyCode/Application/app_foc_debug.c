#include "app_foc_debug.h"

#include <math.h>

#include "app_attitude.h"
#include "app_foc.h"
#include "app_foc_internal.h"
#include "stm32g4xx_hal.h"
#include "usb_debug.h"

static uint32_t s_last_current_debug_print_ms = 0U;
static FastRingSample_t g_fastring_buf[APP_FASTRING_SIZE];
static FastRingSample_t g_fastring_snapshot_buf[APP_FASTRING_SIZE];
static volatile uint16_t g_fastring_head = 0U;
static volatile uint16_t g_fastring_count = 0U;
static volatile uint32_t g_fastring_write_seq = 0U;
static volatile uint16_t g_fastring_snapshot_count = 0U;
static volatile uint32_t g_fastring_snapshot_write_seq = 0U;

static void App_CurrentLoopDebugClear(volatile CurrentLoopDebugSnapshot_t *debug)
{
    if (debug == NULL) {
        return;
    }

    debug->target_iq = 0.0f;
    debug->iq_ref = 0.0f;
    debug->filtered_iq = 0.0f;
    debug->raw_iq = 0.0f;
    debug->error = 0.0f;
    debug->pi_out = 0.0f;
    debug->ff_term = 0.0f;
    debug->uq_final = 0.0f;
    debug->ff_coef = 0.0f;
    debug->integral_limit = 0.0f;
    debug->pid_integral = 0.0f;
}

void App_PrintCurrentLoopDebugIfDue(void)
{
    CurrentLoopDebugSnapshot_t left_debug;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - s_last_current_debug_print_ms) < APP_CURRENT_DEBUG_PRINT_PERIOD_MS) {
        return;
    }
    s_last_current_debug_print_ms = now_ms;

    __disable_irq();
    left_debug = g_current_loop_debug1;
    __enable_irq();

    USB_Debug_Printf("[CL][L] %.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                     left_debug.target_iq,
                     left_debug.iq_ref,
                     left_debug.filtered_iq,
                     left_debug.raw_iq,
                     left_debug.error,
                     left_debug.pi_out,
                     left_debug.ff_term,
                     left_debug.uq_final,
                     left_debug.ff_coef,
                     left_debug.integral_limit,
                     left_debug.pid_integral);
    USB_Debug_Printf("[BUS] %u,%.4f,%.4f\r\n",
                     (unsigned)g_bus_voltage_debug.raw_adc,
                     g_bus_voltage_debug.adc_pin_voltage,
                     g_bus_voltage_debug.bus_voltage);
}

void App_FOC_GetTelemetry(App_FOCTelemetry_t *telemetry)
{
    App_FOCTelemetry_t snapshot;
    uint8_t bus_valid;
    uint8_t stack_ready;
    uint8_t control_it_enabled;
    uint32_t loop_count;
    uint32_t last_loop_tick_ms;
    float speed_fault_left;
    float speed_fault_right;
    uint16_t flags = 0U;
    uint32_t now_ms;

    if (telemetry == NULL) {
        return;
    }

    snapshot.wheel_vel_left_radps = 0.0f;
    snapshot.wheel_vel_right_radps = 0.0f;
    snapshot.filtered_iq_left_a = 0.0f;
    snapshot.filtered_iq_right_a = 0.0f;
    snapshot.uq_left_v = 0.0f;
    snapshot.uq_right_v = 0.0f;
    snapshot.bus_voltage_v = 0.0f;
    snapshot.status_flags = 0U;

    __disable_irq();
    snapshot.wheel_vel_left_radps = g_foc_left_control.velocity.speed_meas_radps;
    snapshot.wheel_vel_right_radps = g_foc_right_control.velocity.speed_meas_radps;
    snapshot.filtered_iq_left_a = g_current_loop_debug1.filtered_iq;
    snapshot.filtered_iq_right_a = g_current_loop_debug2.filtered_iq;
    snapshot.uq_left_v = g_current_loop_debug1.uq_final;
    snapshot.uq_right_v = g_current_loop_debug2.uq_final;
    snapshot.bus_voltage_v = g_bus_voltage_filtered;
    bus_valid = g_bus_voltage_valid;
    stack_ready = g_foc_stack_ready;
    control_it_enabled = g_foc_control_it_enabled;
    loop_count = g_foc_loop_count;
    last_loop_tick_ms = g_foc_last_loop_tick_ms;
    speed_fault_left = g_speed_fault1;
    speed_fault_right = g_speed_fault2;
    __enable_irq();

    now_ms = HAL_GetTick();

    if (speed_fault_left > 0.5f) {
        flags |= APP_FOC_STATUS_FLAG_SPEED_FAULT_L;
    }
    if (speed_fault_right > 0.5f) {
        flags |= APP_FOC_STATUS_FLAG_SPEED_FAULT_R;
    }
    if (stack_ready != 0U) {
        flags |= APP_FOC_STATUS_FLAG_STACK_READY;
    }
    if (control_it_enabled != 0U) {
        flags |= APP_FOC_STATUS_FLAG_CONTROL_IT_ENABLED;
    }
    if (bus_valid != 0U) {
        flags |= APP_FOC_STATUS_FLAG_BUS_VALID;
    }
    if ((loop_count > 0U) && ((now_ms - last_loop_tick_ms) <= 100U)) {
        flags |= APP_FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE;
    }
#if APP_SPEED_LOOP_ENABLE
    flags |= APP_FOC_STATUS_FLAG_SPEED_LOOP_ENABLED;
#endif
#if APP_CURRENT_LOOP_ENABLE
    flags |= APP_FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED;
#endif

    if (g_foc_system_enabled == 0U) {
        flags |= APP_FOC_STATUS_FLAG_POWER_STAGE_OFF;
    }
    if (App_Attitude_IsControlEnabled() != 0U) {
        flags |= APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON;
    }

    snapshot.status_flags = flags;
    *telemetry = snapshot;
}

static void App_CurrentPIDApplyOne(PID_t *pid, float kp, float ki, float kd, float integral_limit)
{
    float output_limit;
    float i_err_min;
    float i_sep_ratio;

    if (pid == NULL) {
        return;
    }

    output_limit = (pid->output_limit > 0.0f) ? pid->output_limit : APP_CURRENT_OUT_LIMIT;
    i_err_min = (pid->I_ERR_MIN > 0.0f) ? pid->I_ERR_MIN : APP_CURRENT_I_ERR_MIN;
    i_sep_ratio = (pid->I_SEP_RATIO > 0.0f) ? pid->I_SEP_RATIO : g_current_i_sep_ratio_ff;

    PID_ParameterInitEx(pid, kp, ki, kd, integral_limit,
                        output_limit, i_err_min, i_sep_ratio);

    App_PIDResetRuntime(pid);
}

static PID_t *App_CurrentPIDGetOneByMode(uint8_t mode)
{
    if (mode == 0U) {
        return &g_foc_left_control.current.pid_ff;
    }

    return &g_foc_left_control.current.pid_pi;
}

static void App_CurrentPIDSelectPairByMode(uint8_t mode, PID_t **pid1, PID_t **pid2)
{
    if ((pid1 == NULL) || (pid2 == NULL)) {
        return;
    }

    if (mode == 0U) {
        *pid1 = &g_foc_left_control.current.pid_ff;
        *pid2 = &g_foc_right_control.current.pid_ff;
        return;
    }

    *pid1 = &g_foc_left_control.current.pid_pi;
    *pid2 = &g_foc_right_control.current.pid_pi;
}

static void App_CurrentPIDReinitOneWithExisting(PID_t *pid,
                                                float output_limit,
                                                float i_err_min,
                                                float i_sep_ratio)
{
    if (pid == NULL) {
        return;
    }

    PID_ParameterInitEx(pid,
                        pid->Kp,
                        pid->Ki,
                        pid->Kd,
                        pid->integral_limit,
                        output_limit,
                        i_err_min,
                        i_sep_ratio);
    App_PIDResetRuntime(pid);
}

void App_CurrentPID_SetSame(float kp, float ki, float kd, float integral_limit)
{
    PID_t *pid1;
    PID_t *pid2;

    if (integral_limit <= 0.0f) {
        integral_limit = APP_CURRENT_I_LIMIT;
    }

    App_CurrentPIDSelectPairByMode(g_current_pid_mode, &pid1, &pid2);

    __disable_irq();
    App_CurrentPIDApplyOne(pid1, kp, ki, kd, integral_limit);
    App_CurrentPIDApplyOne(pid2, kp, ki, kd, integral_limit);
    __enable_irq();
}

void App_CurrentPID_GetSame(float *kp, float *ki, float *kd, float *integral_limit)
{
    float local_kp;
    float local_ki;
    float local_kd;
    float local_ilim;
    PID_t *pid;

    pid = App_CurrentPIDGetOneByMode(g_current_pid_mode);

    __disable_irq();
    local_kp = pid->Kp;
    local_ki = pid->Ki;
    local_kd = pid->Kd;
    local_ilim = pid->integral_limit;
    __enable_irq();

    if (local_ilim <= 0.0f) {
        local_ilim = APP_CURRENT_I_LIMIT;
    }

    if (kp != NULL) {
        *kp = local_kp;
    }
    if (ki != NULL) {
        *ki = local_ki;
    }
    if (kd != NULL) {
        *kd = local_kd;
    }
    if (integral_limit != NULL) {
        *integral_limit = local_ilim;
    }
}

uint8_t App_CurrentPID_SetMode(uint8_t mode)
{
    if (mode > 1U) {
        return 0U;
    }

    __disable_irq();
    g_current_pid_mode = mode;
    __enable_irq();
    App_ResetCurrentPIDs(&g_foc_left_control);
    App_ResetCurrentPIDs(&g_foc_right_control);
    return 1U;
}

uint8_t App_CurrentPID_GetMode(void)
{
    return g_current_pid_mode;
}

uint8_t App_CurrentPID_SetOutputLimit(float output_limit)
{
    PID_t *pid1;
    PID_t *pid2;

    if ((!isfinite(output_limit)) || (output_limit <= 0.0f)) {
        return 0U;
    }

    __disable_irq();
    App_CurrentPIDSelectPairByMode(g_current_pid_mode, &pid1, &pid2);
    App_CurrentPIDReinitOneWithExisting(pid1, output_limit, pid1->I_ERR_MIN, pid1->I_SEP_RATIO);
    App_CurrentPIDReinitOneWithExisting(pid2, output_limit, pid2->I_ERR_MIN, pid2->I_SEP_RATIO);
    __enable_irq();
    return 1U;
}

uint8_t App_CurrentPID_GetOutputLimit(float *output_limit)
{
    PID_t *pid;
    float value;

    if (output_limit == NULL) {
        return 0U;
    }

    pid = App_CurrentPIDGetOneByMode(g_current_pid_mode);

    __disable_irq();
    value = pid->output_limit;
    __enable_irq();
    *output_limit = value;
    return 1U;
}

uint8_t App_CurrentPID_SetIErrMin(float i_err_min)
{
    PID_t *pid1;
    PID_t *pid2;

    if ((!isfinite(i_err_min)) || (i_err_min <= 0.0f)) {
        return 0U;
    }

    __disable_irq();
    App_CurrentPIDSelectPairByMode(g_current_pid_mode, &pid1, &pid2);
    App_CurrentPIDReinitOneWithExisting(pid1, pid1->output_limit, i_err_min, pid1->I_SEP_RATIO);
    App_CurrentPIDReinitOneWithExisting(pid2, pid2->output_limit, i_err_min, pid2->I_SEP_RATIO);
    __enable_irq();
    return 1U;
}

uint8_t App_CurrentPID_GetIErrMin(float *i_err_min)
{
    PID_t *pid;
    float value;

    if (i_err_min == NULL) {
        return 0U;
    }

    pid = App_CurrentPIDGetOneByMode(g_current_pid_mode);

    __disable_irq();
    value = pid->I_ERR_MIN;
    __enable_irq();
    *i_err_min = value;
    return 1U;
}

uint8_t App_CurrentPID_SetISepRatio(float i_sep_ratio)
{
    PID_t *pid1;
    PID_t *pid2;

    if ((!isfinite(i_sep_ratio)) || (i_sep_ratio <= 0.0f) || (i_sep_ratio > 1.0f)) {
        return 0U;
    }

    __disable_irq();
    if (g_current_pid_mode == 0U) {
        g_current_i_sep_ratio_ff = i_sep_ratio;
    } else {
        g_current_i_sep_ratio_pure = i_sep_ratio;
    }
    App_CurrentPIDSelectPairByMode(g_current_pid_mode, &pid1, &pid2);
    App_CurrentPIDReinitOneWithExisting(pid1, pid1->output_limit, pid1->I_ERR_MIN, i_sep_ratio);
    App_CurrentPIDReinitOneWithExisting(pid2, pid2->output_limit, pid2->I_ERR_MIN, i_sep_ratio);
    __enable_irq();
    return 1U;
}

uint8_t App_CurrentPID_GetISepRatio(float *i_sep_ratio)
{
    if (i_sep_ratio == NULL) {
        return 0U;
    }

    __disable_irq();
    if (g_current_pid_mode == 0U) {
        *i_sep_ratio = g_current_i_sep_ratio_ff;
    } else {
        *i_sep_ratio = g_current_i_sep_ratio_pure;
    }
    __enable_irq();
    return 1U;
}

void App_ResetCurrentPIDs(App_FOCMotorControl_t *control)
{
    volatile CurrentLoopDebugSnapshot_t *debug = NULL;
    uint8_t *i_unload_limit_ticks = NULL;

    if (control == NULL) {
        return;
    }

    if (control == &g_foc_left_control) {
        debug = &g_current_loop_debug1;
        i_unload_limit_ticks = &g_current_i_unload_limit_ticks1;
    } else if (control == &g_foc_right_control) {
        debug = &g_current_loop_debug2;
        i_unload_limit_ticks = &g_current_i_unload_limit_ticks2;
    } else {
        return;
    }

    __disable_irq();

    App_PIDResetRuntime(&control->current.pid_ff);
    App_PIDResetRuntime(&control->current.pid_pi);

    App_CurrentLoopDebugClear(debug);

    *i_unload_limit_ticks = 0U;

    control->current.iq_ref = control->current.iq_target;

    __enable_irq();
}

static uint16_t app_fastring_status_flags_snapshot(void)
{
    uint16_t status = 0U;

    if (g_speed_fault1 > 0.5f) {
        status |= APP_FOC_STATUS_FLAG_SPEED_FAULT_L;
    }
    if (g_speed_fault2 > 0.5f) {
        status |= APP_FOC_STATUS_FLAG_SPEED_FAULT_R;
    }
    if (g_foc_stack_ready != 0U) {
        status |= APP_FOC_STATUS_FLAG_STACK_READY;
    }
    if (g_foc_control_it_enabled != 0U) {
        status |= APP_FOC_STATUS_FLAG_CONTROL_IT_ENABLED;
    }
    if (g_bus_voltage_valid != 0U) {
        status |= APP_FOC_STATUS_FLAG_BUS_VALID;
    }

    status |= APP_FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE;
    status |= APP_FOC_STATUS_FLAG_SPEED_LOOP_ENABLED;
    status |= APP_FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED;

    if (g_foc_system_enabled == 0U) {
        status |= APP_FOC_STATUS_FLAG_POWER_STAGE_OFF;
    }
    if (App_Attitude_IsControlEnabled() != 0U) {
        status |= APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON;
    }

    return status;
}

static int16_t app_fastlog_scale_to_i16(float value, float scale)
{
    float scaled;

    if (scale == 0.0f) {
        return 0;
    }

    scaled = value * scale;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t)scaled;
}

void App_ResetFastRing(void)
{
    __disable_irq();
    g_fastring_head = 0U;
    g_fastring_count = 0U;
    g_fastring_write_seq = 0U;
    g_fastring_snapshot_count = 0U;
    g_fastring_snapshot_write_seq = 0U;
    __enable_irq();
}

void App_GetFastRingStatus(uint16_t *count,
                           uint16_t *capacity,
                           uint16_t *head,
                           uint32_t *write_seq)
{
    __disable_irq();
    if (count != NULL) {
        *count = g_fastring_count;
    }
    if (capacity != NULL) {
        *capacity = APP_FASTRING_SIZE;
    }
    if (head != NULL) {
        *head = g_fastring_head;
    }
    if (write_seq != NULL) {
        *write_seq = g_fastring_write_seq;
    }
    __enable_irq();
}

uint16_t App_CopyFastRingLatest(uint16_t start_idx,
                                uint8_t max_samples,
                                FastRingSample_t *out)
{
    uint16_t count;
    uint16_t head;
    uint16_t oldest;
    uint16_t copied;
    uint16_t i;

    if ((out == NULL) || (max_samples == 0U)) {
        return 0U;
    }

    __disable_irq();
    count = g_fastring_count;
    head = g_fastring_head;
    __enable_irq();

    if (start_idx >= count) {
        return 0U;
    }

    copied = (uint16_t)(count - start_idx);
    if (copied > (uint16_t)max_samples) {
        copied = (uint16_t)max_samples;
    }

    oldest = (uint16_t)((head + APP_FASTRING_SIZE - count) % APP_FASTRING_SIZE);

    __disable_irq();
    for (i = 0U; i < copied; i++) {
        uint16_t ring_idx = (uint16_t)((oldest + start_idx + i) % APP_FASTRING_SIZE);
        out[i] = g_fastring_buf[ring_idx];
    }
    __enable_irq();

    return copied;
}

void App_SnapshotFastRing(uint16_t *count,
                          uint16_t *capacity,
                          uint32_t *write_seq)
{
    uint16_t live_count;
    uint16_t live_head;
    uint16_t oldest;
    uint16_t i;

    __disable_irq();
    live_count = g_fastring_count;
    live_head = g_fastring_head;
    oldest = (uint16_t)((live_head + APP_FASTRING_SIZE - live_count) % APP_FASTRING_SIZE);

    for (i = 0U; i < live_count; i++) {
        uint16_t ring_idx = (uint16_t)((oldest + i) % APP_FASTRING_SIZE);
        g_fastring_snapshot_buf[i] = g_fastring_buf[ring_idx];
    }
    g_fastring_snapshot_count = live_count;
    g_fastring_snapshot_write_seq = g_fastring_write_seq;

    if (count != NULL) {
        *count = g_fastring_snapshot_count;
    }
    if (capacity != NULL) {
        *capacity = APP_FASTRING_SIZE;
    }
    if (write_seq != NULL) {
        *write_seq = g_fastring_snapshot_write_seq;
    }
    __enable_irq();
}

void App_GetFastRingSnapshotStatus(uint16_t *count,
                                   uint16_t *capacity,
                                   uint32_t *write_seq)
{
    __disable_irq();
    if (count != NULL) {
        *count = g_fastring_snapshot_count;
    }
    if (capacity != NULL) {
        *capacity = APP_FASTRING_SIZE;
    }
    if (write_seq != NULL) {
        *write_seq = g_fastring_snapshot_write_seq;
    }
    __enable_irq();
}

uint16_t App_CopyFastRingSnapshotChunk(uint32_t snapshot_write_seq,
                                       uint16_t start_idx,
                                       uint8_t max_samples,
                                       FastRingSample_t *out)
{
    uint16_t copied;
    uint16_t snapshot_count;
    uint16_t i;

    if ((out == NULL) || (max_samples == 0U)) {
        return 0U;
    }

    __disable_irq();
    if (snapshot_write_seq != g_fastring_snapshot_write_seq) {
        __enable_irq();
        return 0U;
    }

    snapshot_count = g_fastring_snapshot_count;
    if (start_idx >= snapshot_count) {
        __enable_irq();
        return 0U;
    }

    copied = (uint16_t)(snapshot_count - start_idx);
    if (copied > (uint16_t)max_samples) {
        copied = (uint16_t)max_samples;
    }

    for (i = 0U; i < copied; i++) {
        out[i] = g_fastring_snapshot_buf[start_idx + i];
    }
    __enable_irq();

    return copied;
}

void App_FastRingPushDual(void)
{
    FastRingSample_t *sample = &g_fastring_buf[g_fastring_head];

    sample->target_iq_l_ma = app_fastlog_scale_to_i16(g_current_loop_debug1.target_iq, 1000.0f);
    sample->iq_ref_l_ma = app_fastlog_scale_to_i16(g_current_loop_debug1.iq_ref, 1000.0f);
    sample->filtered_iq_l_ma = app_fastlog_scale_to_i16(g_current_loop_debug1.filtered_iq, 1000.0f);
    sample->raw_iq_l_ma = app_fastlog_scale_to_i16(g_current_loop_debug1.raw_iq, 1000.0f);
    sample->uq_final_l_mv = app_fastlog_scale_to_i16(g_current_loop_debug1.uq_final, 1000.0f);

    sample->target_iq_r_ma = app_fastlog_scale_to_i16(g_current_loop_debug2.target_iq, 1000.0f);
    sample->iq_ref_r_ma = app_fastlog_scale_to_i16(g_current_loop_debug2.iq_ref, 1000.0f);
    sample->filtered_iq_r_ma = app_fastlog_scale_to_i16(g_current_loop_debug2.filtered_iq, 1000.0f);
    sample->raw_iq_r_ma = app_fastlog_scale_to_i16(g_current_loop_debug2.raw_iq, 1000.0f);
    sample->uq_final_r_mv = app_fastlog_scale_to_i16(g_current_loop_debug2.uq_final, 1000.0f);

    sample->bus_mv = (uint16_t)app_fastlog_scale_to_i16(g_bus_voltage_filtered, 1000.0f);
    sample->sample_idx = (uint16_t)g_fastring_write_seq;
    sample->status_flags = app_fastring_status_flags_snapshot();

    g_fastring_head = (uint16_t)((g_fastring_head + 1U) % APP_FASTRING_SIZE);
    if (g_fastring_count < APP_FASTRING_SIZE) {
        g_fastring_count++;
    }
    g_fastring_write_seq++;
}
