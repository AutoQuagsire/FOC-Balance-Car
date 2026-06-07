#include "app_foc.h"
#include "foc_common.h"
#include "Filter.h"
#include "current_sense.h"
#include "BusVoltage.h"
#include "main.h"
#include "AS5047P_RW.h"
#include "sensor.h"
#include "driver.h"
#include "BLDCMotor.h"
#include "stm32g4xx_hal.h"
#include "sys.h"
#include "app_attitude.h"
#include <math.h>
#include <stdint.h>
#include "PID.h"

#if defined(__GNUC__)
#define APP_FOC_HOT __attribute__((optimize("O2,fast-math")))
#else
#define APP_FOC_HOT
#endif




static uint8_t App_InitBusVoltage(void);
static uint8_t App_InitMotor1Stack(void);
static uint8_t App_InitMotor2Stack(void);
static uint8_t App_InitFOCAlgorithm(void);
static void App_FOC_ForcePowerStageOff(void);
static int16_t app_fastlog_scale_to_i16(float value, float scale);

static volatile uint8_t g_foc_stack_ready = 0U;
static volatile uint8_t g_bus_telemetry_ready = 0U;
static volatile uint8_t g_foc_power_stage_enabled = 0U;
static uint32_t         g_last_bus_voltage_sample_tick_ms = 0U;

/* FOC timing state shared by the SimpleFOC compatibility layer. */
FocFrequency_t g_foc = {
    .g_foc_frequency = FOC_FREQUENCY_DEFAULT,
    .g_foc_period_s = FOC_PERIOD_S_DEFAULT,
    .g_foc_period_us = FOC_PERIOD_US_DEFAULT,
    .g_foc_it_timer = &htim5,
};





/**
 * @brief FOC 应用层对象初始化
 *
 * 职责：
 * - 初始化 Driver / Encoder / Sensor / CurrentSense；
 * - 装配 Motor 对象；
 * - 初始化速度环和电流环 PID；
 * - 不执行零位电角度校准。
 */
uint8_t App_FOCStack_Init(void)
{
    g_foc_stack_ready = 0U;
    g_bus_telemetry_ready = 0U;

    if(!App_InitBusVoltage()) {
        USB_Debug_Printf("Bus voltage init failed\r\n");
        return 0U;
    }
    HAL_Delay(1000);
    if(!App_InitMotor1Stack()) {
        USB_Debug_Printf("Motor1 stack init failed\r\n");
        return 0U;
    }
    if(!App_InitMotor2Stack()) {
        USB_Debug_Printf("Motor2 stack init failed\r\n");
        return 0U;
    }

    if (!App_InitFOCAlgorithm()) {
        USB_Debug_Printf("FOC algorithm init failed\r\n");
        return 0U;
    }
    App_FOC_ForcePowerStageOff();
    App_ResetFastRing();
    g_foc_stack_ready = 1U;
    g_last_bus_voltage_sample_tick_ms = HAL_GetTick();
    g_bus_telemetry_ready = 1U;
    USB_Debug_Printf("FOC stack init ok\r\n");
    return 1U;
}





/* 这些外设句柄由 CubeMX 生成并在别处定义
 * 这里用 extern 引用，供应用层初始化时使用 */
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;
/* =========================
 * 应用层 FOC 对象
 * =========================
 * 这一层的对象不属于某个单独模块，而是“把各模块组起来”
 * 所以统一放在 app_foc.c 内部静态保存
 */

static BusVoltage_t g_bus_voltage;
static volatile BusVoltageDebug_t g_bus_voltage_debug;
static LowPassFilter_t g_bus_voltage_lpf;   //母线电压的低通滤波器
static float g_bus_voltage_filtered = 0.0f;
static uint8_t g_bus_voltage_valid = 0U;

static Motor_t          g_motor1;   // 电机控制对象
static Driver_t        *g_driver1 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static AS5047P_Handle_t g_enc1;     // AS5047P 底层驱动句柄
static Sensor_t         g_sensor1;  // 传感器公共层对象
static CurrentSense_t  g_current_sense1; // 电流采样对象
static PID_t            g_speed_pid1;
LowPassFilter_t         g_speed_lpf1;
static PID_t            g_current_pid1;
static PID_t            g_current_pid1_Common;
LowPassFilter_t         g_current_lpf1;


static Motor_t          g_motor2;   // 电机控制对象
static Driver_t        *g_driver2 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static AS5047P_Handle_t g_enc2;     // AS5047P 底层驱动句柄
static Sensor_t         g_sensor2;  // 传感器公共层对象
static CurrentSense_t  g_current_sense2; // 电流采样对象
static PID_t            g_speed_pid2;
LowPassFilter_t         g_speed_lpf2;
static PID_t            g_current_pid2;
static PID_t            g_current_pid2_Common;
LowPassFilter_t         g_current_lpf2;


static uint32_t         g_last_loop_tick_ms = 0U;
static uint32_t         g_last_print_tick_ms = 0U;
static uint32_t         g_last_while_debug_tick_ms = 0U;
static volatile CurrentLoopDebugSnapshot_t g_current_loop_debug1;
static volatile CurrentLoopDebugSnapshot_t g_current_loop_debug2;
static volatile uint8_t g_foc_control_it_enabled = 0U;
static volatile uint32_t g_foc_loop_count = 0U;
static volatile uint32_t g_foc_last_loop_tick_ms = 0U;
static uint8_t g_current_i_unload_limit_ticks1 = 0U;
static uint8_t g_current_i_unload_limit_ticks2 = 0U;
static float g_current_iq_ref1 = 0.0f;
static float g_current_iq_ref2 = 0.0f;
static uint32_t g_last_current_debug_print_ms = 0U;

#if APP_BUS_VOLTAGE_ENABLE
static uint8_t App_BusVoltageStartupSample(void);
#endif

#define APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS (10U)
#define APP_BUS_VOLTAGE_LPF_CUTOFF_HZ (10.0f)
#define APP_BUS_VOLTAGE_STARTUP_SAMPLE_COUNT (20U)
#define APP_BUS_VOLTAGE_VALID_MIN_V (5.0f)
#define APP_BUS_VOLTAGE_VALID_MAX_V (30.0f)





static void App_ServiceBusVoltageSample(void)
{
#if APP_BUS_VOLTAGE_ENABLE
    if (!BusVoltage_SampleOnce(&g_bus_voltage)) {
        g_bus_voltage_valid = 0U;
        return;
    }

    g_bus_voltage_debug.bus_voltage = BusVoltage_GetBusVoltage(&g_bus_voltage);
    g_bus_voltage_debug.adc_pin_voltage = BusVoltage_GetAdcPinVoltage(&g_bus_voltage);
    g_bus_voltage_debug.raw_adc = BusVoltage_GetRawAdc(&g_bus_voltage);

    if ((g_bus_voltage_debug.bus_voltage < APP_BUS_VOLTAGE_VALID_MIN_V) ||
        (g_bus_voltage_debug.bus_voltage > APP_BUS_VOLTAGE_VALID_MAX_V)) {
        g_bus_voltage_valid = 0U;
        return;
    }

    g_bus_voltage_valid = 1U;
    g_bus_voltage_filtered = LowPassFilter_Update(&g_bus_voltage_lpf,
                                                  g_bus_voltage_debug.bus_voltage);

#if APP_BUS_VOLTAGE_FOC_ENABLE
    /* 使用滤波后的母线电压参与 FOC 调制，原始值保留用于调试对比。 */
    if (g_driver1 != NULL) {
        g_driver1->supply_voltage = g_bus_voltage_filtered;
    }
    if (g_driver2 != NULL) {
        g_driver2->supply_voltage = g_bus_voltage_filtered;
    }
#endif
#else
    g_bus_voltage_valid = 1U;
    g_bus_voltage_debug.raw_adc = 0U;
    g_bus_voltage_debug.adc_pin_voltage = 0.0f;
    g_bus_voltage_debug.bus_voltage = V_SUPPLY;
    g_bus_voltage_filtered = V_SUPPLY;
#endif
}



PID_t Left_Velocity_FOC_PID;
static float g_speed_target_radps = 0.3f;
volatile uint8_t g_current_pid_mode = 0U; /* 0=CurrentLoop_FFPI_V1, 1=Pure PI compare */
static volatile float g_current_i_sep_ratio_ff = CURRENT_LOOP_I_SEP_RATIO;
static volatile float g_current_i_sep_ratio_pure = CURRENT_LOOP_PURE_PI_I_SEP_RATIO;

float g_speed_fault2 = 0.0f;
float g_speed_fault1 = 0.0f;

#define LEFT_MOTOR_ENABLE 1U
#define RIGHT_MOTOR_ENABLE 1U

#define APP_LOOP_TEST_UQ_V        (1.0f)
#define APP_LOOP_PRINT_PERIOD_MS  (100U)

#define APP_SPEED_LOOP_ENABLE      (1U)
#define APP_SPEED_TARGET_RAD_S     (10.0f)
#define APP_SPEED_KP               (0.055f)
#define APP_SPEED_KI               (0.00035f)
#define APP_SPEED_KD               (0.0f)
#define APP_SPEED_UQ_LIMIT         (1.8f)
#define APP_SPEED_I_LIMIT          (0.5f)
#define APP_SPEED_I_ERR_MIN        (0.05f)
#define APP_SPEED_I_SEP_RATIO      (0.75f)
#define APP_SPEED_VEL_FAULT_ABS    (80.0f)


#define APP_CURRENT_LOOP_ENABLE      (1U)
#define APP_CURRENT_TARGET_A       (0.3f)
#define APP_CURRENT_KP             (2.5)
#define APP_CURRENT_KI             (0.2f)
#define APP_CURRENT_KD             (0.0f)

#define APP_LEFT_WHEEL_SPEED_SIGN   (1.0f)
#define APP_RIGHT_WHEEL_SPEED_SIGN  (-1.0f)

static volatile float g_iq_target_left = APP_CURRENT_TARGET_A;
static volatile float g_iq_target_right = APP_CURRENT_TARGET_A;

extern float vel_windowed_f1;
extern float vel_windowed_f2;

#define APP_CURRENT_I_LIMIT          (5.0f)
#define APP_CURRENT_PURE_PI_I_LIMIT  (6.0f)
#define APP_CURRENT_I_ERR_MIN        (0.05f)


#define APP_CURRENT_FF_KP             (2.8f)
#define APP_CURRENT_FF_KI             (0.35f)
#define APP_CURRENT_FF_KD             (0.0f)
#define APP_CURRENT_FF_I_LIMIT          (5.0f)


#define APP_CURRENT_I_ERR_MIN        (0.05f)
#define APP_CURRENT_OUT_LIMIT       (10.963f)
#define APP_CURRENT_DEBUG_PRINT_PERIOD_MS (100U)

#define APP_CS_SIGN_TEST_UQ_V         (0.8f)
#define APP_CS_SIGN_TEST_SETTLE_MS    (80U)
#define APP_CS_SIGN_TEST_SAMPLE_CNT   (120U)
#define APP_CS_SIGN_TEST_SAMPLE_DT_MS (1U)
#define APP_CS_SIGN_TEST_DEADBAND_A   (0.03f)

#define APP_SENSOR_DIR_TEST_UQ_V         (4.0f)
#define APP_SENSOR_DIR_TEST_SETTLE_MS    (500U)
#define APP_SENSOR_DIR_TEST_ELEC_STEP    (PI * 0.5f)
#define APP_SENSOR_DIR_TEST_DEADBAND_RAD (0.02f)
#define APP_SENSOR_DIR_TEST_MAX_STEP_MS  (2500U)



#define APP_MOVE_DOWNSAMPLE        (1U)


#if (APP_MOVE_DOWNSAMPLE < 1U)
#error "APP_MOVE_DOWNSAMPLE must be >= 1"
#endif

/* 重置 PID 运行状态：只清积分、上次误差和输出，不改 PID 参数 */
static void App_PIDResetRuntime(PID_t *pid)
{
    PID_Reset(pid);
}


/* 清空电流环调试快照，避免打印到上一次残留数据 */
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




/* 对外部 target_iq 做斜率限制，生成电流环内部目标 iq_ref。
 * 目的：把硬阶跃变成较平滑的内部参考，降低超调和下冲。
 */
APP_FOC_HOT
static float App_CurrentLoopSlewIqRef(float iq_ref, float target_iq_cmd)
{
    float delta = target_iq_cmd - iq_ref;

    if (delta > CURRENT_LOOP_IQ_REF_STEP_UP_MAX) {
        delta = CURRENT_LOOP_IQ_REF_STEP_UP_MAX;
    } else if (delta < -CURRENT_LOOP_IQ_REF_STEP_DOWN_MAX) {
        delta = -CURRENT_LOOP_IQ_REF_STEP_DOWN_MAX;
    }

    return iq_ref + delta;
}


/* 判断目标电流幅值是否正在下降。
 * 只处理同方向下降，例如 +0.9 -> +0.6 或 -0.9 -> -0.6。
 * 用于触发积分卸载限速，降低目标下降时的反向下冲。
 */
APP_FOC_HOT
static uint8_t App_CurrentLoopIsTargetMagnitudeFalling(float target_iq, float prev_target_iq)
{
    const float eps = CURRENT_LOOP_TARGET_STEP_EPS;
    float target_abs = fabsf(target_iq);
    float prev_abs = fabsf(prev_target_iq);
    uint8_t same_positive = (uint8_t)((target_iq > eps) && (prev_target_iq > eps));
    uint8_t same_negative = (uint8_t)((target_iq < -eps) && (prev_target_iq < -eps));

    return (uint8_t)((same_positive || same_negative) &&
                     ((target_abs + eps) < prev_abs));
}


/* 电流环核心计算：
 * 输入外部目标电流 target_iq_cmd 和当前 Iq 反馈，
 * 输出最终 q 轴电压 Uq。
 *
 * 前馈模式：
 *   iq_ref = slew(target_iq_cmd)
 *   Uq = PI(iq_ref - filtered_iq) + iq_ref * R * ff_coef
 *
 * 纯 PI 模式：
 *   iq_ref = target_iq_cmd
 *   Uq = PI(iq_ref - filtered_iq)
 */
APP_FOC_HOT
static float App_CurrentLoopComputeUq(Motor_t *motor,
                                      PID_t *pid,
                                      float target_iq_cmd,
                                      float filtered_iq,
                                      float raw_iq,
                                      uint8_t use_feedforward,
                                      volatile CurrentLoopDebugSnapshot_t *debug,
                                      uint8_t *i_unload_limit_ticks,
                                      float *iq_ref_state)
{
    float voltage_limit;
    float pi_out;
    float ff_term = 0.0f;
    float ff_coef = 0.0f;
    float integral_limit = 0.0f;
    float uq_final;
    float iq_ref = target_iq_cmd;
    uint8_t pid_flags = 0U;

    if ((motor == NULL) || (pid == NULL)) {
        return 0.0f;
    }

    /* 前馈 PI 下启用 iq_ref 斜率限制；纯 PI 下直接跟随外部目标 */
    if ((use_feedforward != 0U) && (iq_ref_state != NULL)) {
        iq_ref = App_CurrentLoopSlewIqRef(*iq_ref_state, target_iq_cmd);
        *iq_ref_state = iq_ref;
    } else if (iq_ref_state != NULL) {
        iq_ref = target_iq_cmd;
        *iq_ref_state = iq_ref;
    }

    /* 根据 |iq_ref| 插值得到前馈系数和积分限幅 */
    CurrentLoop_GetScheduledParams(iq_ref, &ff_coef, &integral_limit);
    pid->integral_limit = integral_limit;

    /* 前馈 PI 和纯 PI 使用不同积分分离阈值 */
    if (use_feedforward) {
        pid->I_SEP_RATIO = g_current_i_sep_ratio_ff;
    } else {
        pid->I_SEP_RATIO = g_current_i_sep_ratio_pure;
    }

    /* 目标电流幅值下降时，短时间启用积分卸载限速 */
    if ((use_feedforward != 0U) && (i_unload_limit_ticks != NULL)) {
        float prev_iq_ref = (debug != NULL) ? debug->iq_ref : iq_ref;

        if (App_CurrentLoopIsTargetMagnitudeFalling(iq_ref, prev_iq_ref)) {
            *i_unload_limit_ticks = CURRENT_LOOP_I_UNLOAD_LIMIT_TICKS;
        }

        if (*i_unload_limit_ticks > 0U) {
            pid_flags |= PID_CURRENT_LIMIT_I_UNLOAD;
            (*i_unload_limit_ticks)--;
        }
    } else if (i_unload_limit_ticks != NULL) {
        *i_unload_limit_ticks = 0U;
    }

    /* 电流 PI 主体 */
    PID_CalCurrent(pid, iq_ref, filtered_iq, pid_flags);
    pi_out = pid->output;

#if CURRENT_LOOP_USE_FEEDFORWARD
    /* 电压前馈项：用相电阻估算基础 Uq，PI 只修正残余误差 */
    if (use_feedforward) {
        ff_term = iq_ref * motor->param.phase_resistance * ff_coef;
    }
#else
    (void)use_feedforward;
#endif

    uq_final = pi_out + ff_term;

    /* 电压限幅优先级：Motor 配置 > Driver 限制 > PID 输出限制 */
    voltage_limit = motor->config.voltage_limit;
    if ((voltage_limit <= 0.0f) && (motor->driver != NULL)) {
        voltage_limit = motor->driver->voltage_limit;
    }
    if (voltage_limit <= 0.0f) {
        voltage_limit = pid->output_limit;
    }

    uq_final = constrain(uq_final, -voltage_limit, voltage_limit);

    /* 保存调试快照 */
    if (debug != NULL) {
        debug->target_iq = target_iq_cmd;
        debug->iq_ref = iq_ref;
        debug->filtered_iq = filtered_iq;
        debug->raw_iq = raw_iq;
        debug->error = iq_ref - filtered_iq;
        debug->pi_out = pi_out;
        debug->ff_term = ff_term;
        debug->uq_final = uq_final;
        debug->ff_coef = ff_coef;
        debug->integral_limit = pid->integral_limit;
        debug->pid_integral = pid->Ki * pid->error_integral;
    }

    return uq_final;
}


/* 周期性打印电流环调试信息。
 * 先关中断复制快照，再打印，避免打印过程中 ISR 正在改数据。
 */
static void App_PrintCurrentLoopDebugIfDue(void)
{
    CurrentLoopDebugSnapshot_t left_debug;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - g_last_current_debug_print_ms) < APP_CURRENT_DEBUG_PRINT_PERIOD_MS) {
        return;
    }
    g_last_current_debug_print_ms = now_ms;

    __disable_irq();
    left_debug = g_current_loop_debug1;
    __enable_irq();

#if LEFT_MOTOR_ENABLE
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
#endif
}



/* 应用一组电流环 PID 参数，并保留原有输出限幅/积分分离下限等配置 */
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

static void App_CurrentPIDSelectPairByMode(uint8_t mode, PID_t **pid1, PID_t **pid2)
{
    if ((pid1 == NULL) || (pid2 == NULL)) {
        return;
    }

    if (mode == 0U) {
        *pid1 = &g_current_pid1;
        *pid2 = &g_current_pid2;
    } else {
        *pid1 = &g_current_pid1_Common;
        *pid2 = &g_current_pid2_Common;
    }
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


/* 同时设置左右电机电流环 PID。
 * g_current_pid_mode = 0：前馈 PI 参数组；
 * g_current_pid_mode != 0：纯 PI 对照参数组。
 */
void App_CurrentPID_SetSame(float kp, float ki, float kd, float integral_limit)
{
    PID_t *pid1, *pid2;

    if (integral_limit <= 0.0f) {
        integral_limit = APP_CURRENT_I_LIMIT;
    }

    App_CurrentPIDSelectPairByMode(g_current_pid_mode, &pid1, &pid2);

    __disable_irq();
    App_CurrentPIDApplyOne(pid1, kp, ki, kd, integral_limit);
    App_CurrentPIDApplyOne(pid2, kp, ki, kd, integral_limit);
    __enable_irq();
}


/* 读取当前电流环 PID 参数，默认以左电机参数作为代表 */
void App_CurrentPID_GetSame(float *kp, float *ki, float *kd, float *integral_limit)
{
    float local_kp;
    float local_ki;
    float local_kd;
    float local_ilim;
    PID_t *pid;

    if (g_current_pid_mode == 0U) {
        pid = &g_current_pid1;
    } else {
        pid = &g_current_pid1_Common;
    }

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


/* 重置所有电流环运行状态。
 * 注意：这里也会重置 iq_ref，因此目标电流改变后不会沿用旧斜坡状态。
 */
uint8_t App_CurrentPID_SetMode(uint8_t mode)
{
    if (mode > 1U) {
        return 0U;
    }
    __disable_irq();
    g_current_pid_mode = mode;
    __enable_irq();
    App_ResetCurrentPIDs();
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

    if (g_current_pid_mode == 0U) {
        pid = &g_current_pid1;
    } else {
        pid = &g_current_pid1_Common;
    }

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

    if (g_current_pid_mode == 0U) {
        pid = &g_current_pid1;
    } else {
        pid = &g_current_pid1_Common;
    }

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

void App_ResetCurrentPIDs(void)
{
    __disable_irq();

    App_PIDResetRuntime(&g_current_pid1);
    App_PIDResetRuntime(&g_current_pid2);
    App_PIDResetRuntime(&g_current_pid1_Common);
    App_PIDResetRuntime(&g_current_pid2_Common);

    App_CurrentLoopDebugClear(&g_current_loop_debug1);
    App_CurrentLoopDebugClear(&g_current_loop_debug2);

    g_current_i_unload_limit_ticks1 = 0U;
    g_current_i_unload_limit_ticks2 = 0U;

    g_current_iq_ref1 = g_iq_target_left;
    g_current_iq_ref2 = g_iq_target_right;

    __enable_irq();
}

void App_FOC_SetIqTarget(float left_iq, float right_iq)
{
    __disable_irq();
    g_iq_target_left = left_iq;
    g_iq_target_right = right_iq;
    __enable_irq();
}

float App_FOC_GetAverageWheelSpeedRadps(void)
{
    float left_speed;
    float right_speed;

    __disable_irq();
    left_speed = vel_windowed_f1;
    right_speed = vel_windowed_f2;
    __enable_irq();

    return 0.5f * ((APP_LEFT_WHEEL_SPEED_SIGN * left_speed) +
                   (APP_RIGHT_WHEEL_SPEED_SIGN * right_speed));
}

float App_FOC_GetBusVoltageFiltered(void)
{
    return g_bus_voltage_filtered;
}

uint8_t App_FOC_BusTelemetryInit(void)
{
    if (g_foc_stack_ready != 0U) {
        g_last_bus_voltage_sample_tick_ms = HAL_GetTick();
        g_bus_telemetry_ready = 1U;
        return 1U;
    }

    g_bus_telemetry_ready = 0U;

    if (!App_InitBusVoltage()) {
        return 0U;
    }

    g_last_bus_voltage_sample_tick_ms = HAL_GetTick();
    g_bus_telemetry_ready = 1U;
    return 1U;
}

void App_FOC_BusTelemetryService(void)
{
    uint32_t now_ms;

    if (g_bus_telemetry_ready == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
#if APP_BUS_VOLTAGE_ENABLE
    if ((now_ms - g_last_bus_voltage_sample_tick_ms) >= APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS) {
        g_last_bus_voltage_sample_tick_ms = now_ms;
        App_ServiceBusVoltageSample();
    }
#endif
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
    snapshot.wheel_vel_left_radps = vel_windowed_f1;
    snapshot.wheel_vel_right_radps = vel_windowed_f2;
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
    if (g_foc_power_stage_enabled == 0U) {
        flags |= APP_FOC_STATUS_FLAG_POWER_STAGE_OFF;
    }
    if (App_Attitude_IsControlEnabled() != 0U) {
        flags |= APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON;
    }

    snapshot.status_flags = flags;
    *telemetry = snapshot;
}






/* 上电零位电角度校准。
 * 当前左右电机均使用 q 轴固定矢量吸附转子，再反算 zero_electrical_angle。
 */
uint8_t App_StartupCalibrate(void)
{
#if LEFT_MOTOR_ENABLE
    if (!Motor_CalibrateZeroElectricalAngle(&g_motor1, 4.0f, PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate1 failed\r\n");
        return 0U;
    }

    USB_Debug_Printf("zero_elec1 = %.6f\r\n", g_motor1.zero_electrical_angle);
#endif

#if RIGHT_MOTOR_ENABLE
    if (!Motor_CalibrateZeroElectricalAngle(&g_motor2, 4.0f, PI / 2.0f, 300)) {
        USB_Debug_Printf("Startup calibrate2 failed\r\n");
        return 0U;
    }

    USB_Debug_Printf("zero_elec2 = %.6f\r\n", g_motor2.zero_electrical_angle);
#endif

    return 1U;
}


/* 带死区的符号判断，用于电流采样正负号测试 */
static int8_t app_sign_with_deadband(float v, float deadband)
{
    if (v > deadband) {
        return 1;
    }
    if (v < -deadband) {
        return -1;
    }
    return 0;
}


/* 在指定电角度施加 uq，并对 ia/ib 多次采样取平均。
 * 用于判断电流采样方向是否正确。
 */
static PhaseCurrent_t app_measure_phase_current_avg(Motor_t *motor,
                                                    CurrentSense_t *cs,
                                                    float uq,
                                                    float elec_angle)
{
    uint32_t i;
    PhaseCurrent_t avg = {0.0f, 0.0f};

    if ((motor == NULL) || (cs == NULL)) {
        return avg;
    }

    Motor_SetPhaseVoltageQ(motor, uq, elec_angle);
    HAL_Delay(APP_CS_SIGN_TEST_SETTLE_MS);

    for (i = 0U; i < APP_CS_SIGN_TEST_SAMPLE_CNT; i++) {
        PhaseCurrent_t cur = CurrentSense_GetPhaseCurrent(cs);
        avg.ia += cur.ia;
        avg.ib += cur.ib;
        HAL_Delay(APP_CS_SIGN_TEST_SAMPLE_DT_MS);
    }

    avg.ia /= (float)APP_CS_SIGN_TEST_SAMPLE_CNT;
    avg.ib /= (float)APP_CS_SIGN_TEST_SAMPLE_CNT;

    return avg;
}


/* 单电机电流采样符号测试。
 * 根据 0、π、π/2、3π/2 四个电角度下 ia/ib 的符号，
 * 判断 A_SIGN / B_SIGN 是否需要翻转。
 */
static void app_current_sense_sign_test_one(const char *tag,
                                            Motor_t *motor,
                                            CurrentSense_t *cs)
{
    uint8_t was_enabled;
    PhaseCurrent_t s_0;
    PhaseCurrent_t s_pi;
    PhaseCurrent_t s_pi_2;
    PhaseCurrent_t s_3pi_2;
    int8_t a_score = 0;
    int8_t b_score = 0;
    float uq = APP_CS_SIGN_TEST_UQ_V;

    if ((motor == NULL) || (cs == NULL) || (motor->driver == NULL) || (!motor->driver->initialized)) {
        USB_Debug_Printf("[CS-SIGN][%s] skip (motor/cs not ready)\r\n", tag);
        return;
    }

    was_enabled = (uint8_t)motor->state.enabled;
    if (!was_enabled) {
        FOCMotor_enable(motor);
        HAL_Delay(10);
    }

    if (!cs->enabled) {
        CurrentSense_Enable(cs);
        HAL_Delay(10);
    }

    /* 限制测试电压，避免符号测试时输出过大 */
    if ((motor->driver->voltage_limit > 0.0f) && (uq > motor->driver->voltage_limit * 0.3f)) {
        uq = motor->driver->voltage_limit * 0.3f;
    }
    if (uq < 0.3f) {
        uq = 0.3f;
    }

    USB_Debug_Printf("[CS-SIGN][%s] test start A_SIGN=%d B_SIGN=%d uq=%.2fV\r\n",
                     tag, cs->CsParam.A_SIGN, cs->CsParam.B_SIGN, uq);

    s_0 = app_measure_phase_current_avg(motor, cs, uq, 0.0f);
    s_pi = app_measure_phase_current_avg(motor, cs, uq, PI);
    s_pi_2 = app_measure_phase_current_avg(motor, cs, uq, 0.5f * PI);
    s_3pi_2 = app_measure_phase_current_avg(motor, cs, uq, 1.5f * PI);

    Motor_SetPhaseVoltageQ(motor, 0.0f, 0.0f);

    /* 根据理论符号关系打分，负分表示建议翻转采样符号 */
    a_score += (app_sign_with_deadband(s_3pi_2.ia, APP_CS_SIGN_TEST_DEADBAND_A) == 1) ? 1 : -1;
    a_score += (app_sign_with_deadband(s_pi_2.ia, APP_CS_SIGN_TEST_DEADBAND_A) == -1) ? 1 : -1;
    b_score += (app_sign_with_deadband(s_0.ib, APP_CS_SIGN_TEST_DEADBAND_A) == 1) ? 1 : -1;
    b_score += (app_sign_with_deadband(s_pi.ib, APP_CS_SIGN_TEST_DEADBAND_A) == -1) ? 1 : -1;

    USB_Debug_Printf("[CS-SIGN][%s] theta0    : ia=% .4f ib=% .4f (expect ib>0)\r\n", tag, s_0.ia, s_0.ib);
    USB_Debug_Printf("[CS-SIGN][%s] thetaPI   : ia=% .4f ib=% .4f (expect ib<0)\r\n", tag, s_pi.ia, s_pi.ib);
    USB_Debug_Printf("[CS-SIGN][%s] thetaPI/2 : ia=% .4f ib=% .4f (expect ia<0)\r\n", tag, s_pi_2.ia, s_pi_2.ib);
    USB_Debug_Printf("[CS-SIGN][%s] theta3PI/2: ia=% .4f ib=% .4f (expect ia>0)\r\n", tag, s_3pi_2.ia, s_3pi_2.ib);

    USB_Debug_Printf("[CS-SIGN][%s] recommendation: A_SIGN %s (%d -> %d), B_SIGN %s (%d -> %d)\r\n",
                     tag,
                     (a_score >= 0) ? "KEEP" : "FLIP",
                     cs->CsParam.A_SIGN,
                     (a_score >= 0) ? cs->CsParam.A_SIGN : -cs->CsParam.A_SIGN,
                     (b_score >= 0) ? "KEEP" : "FLIP",
                     cs->CsParam.B_SIGN,
                     (b_score >= 0) ? cs->CsParam.B_SIGN : -cs->CsParam.B_SIGN);

    if (!was_enabled) {
        FOCMotor_disable(motor);
    }
}


/* 左右电机电流采样符号测试入口 */
void App_CurrentSenseSignTest(void)
{
    USB_Debug_Printf("[CS-SIGN] test begin (run with motor unloaded and hold rotor still)\r\n");

#if LEFT_MOTOR_ENABLE
    app_current_sense_sign_test_one("L", &g_motor1, &g_current_sense1);
#endif

#if RIGHT_MOTOR_ENABLE
    app_current_sense_sign_test_one("R", &g_motor2, &g_current_sense2);
#endif

    USB_Debug_Printf("[CS-SIGN] test end\r\n");
}


/* 计算两个角度之间的最短有符号差值，范围约为 [-π, π] */
static float app_angle_diff_signed(float from, float to)
{
    float d = to - from;

    while (d > PI) {
        d -= 2.0f * PI;
    }
    while (d < -PI) {
        d += 2.0f * PI;
    }

    return d;
}


/* 施加指定电角度电压矢量，等待转子稳定后读取机械角。
 * 用 DWT 做近似 ms 延时，同时持续刷新 Sensor。
 */
static float app_sensor_angle_after_settle(Motor_t *motor,
                                           Sensor_t *sensor,
                                           float uq,
                                           float elec_angle,
                                           uint32_t settle_ms)
{
    uint32_t i;
    uint32_t step_ms;
    uint32_t t0;
    uint32_t t1;
    uint32_t ticks_per_ms;

    Motor_SetPhaseVoltageQ(motor, uq, elec_angle);

    ticks_per_ms = SystemCoreClock / 1000U;
    if (ticks_per_ms == 0U) {
        ticks_per_ms = 1U;
    }

    step_ms = settle_ms;
    if (step_ms > APP_SENSOR_DIR_TEST_MAX_STEP_MS) {
        step_ms = APP_SENSOR_DIR_TEST_MAX_STEP_MS;
    }

    for (i = 0U; i < step_ms; i++) {
        Sensor_Update(sensor, 0.001f);

        t0 = DWT_GetTicks();
        do {
            t1 = DWT_GetElapsedTicks(t0);
        } while (t1 < ticks_per_ms);
    }

    Sensor_Update(sensor, 0.001f);
    return Sensor_GetAngle(sensor);
}


/* 单电机传感器方向测试。
 * 正向电角度步进时机械角应该按同一方向变化；
 * 反向电角度步进时机械角应该反向变化。
 */
static void app_sensor_direction_test_one(const char *tag, Motor_t *motor, Sensor_t *sensor)
{
    uint8_t was_enabled;
    float uq = APP_SENSOR_DIR_TEST_UQ_V;
    float a0;
    float ap;
    float an;
    float dp;
    float dn;
    int8_t score = 0;

    if ((motor == NULL) || (sensor == NULL) || (motor->driver == NULL) ||
        (!motor->driver->initialized) || (!sensor->initialized)) {
        USB_Debug_Printf("[DIR-TEST][%s] skip (motor/sensor not ready)\r\n", tag);
        return;
    }

    was_enabled = (uint8_t)motor->state.enabled;
    if (!was_enabled) {
        FOCMotor_enable(motor);
        HAL_Delay(10);
    }

    if ((motor->driver->voltage_limit > 0.0f) && (uq > motor->driver->voltage_limit * 0.3f)) {
        uq = motor->driver->voltage_limit * 0.3f;
    }
    if (uq < 0.3f) {
        uq = 0.3f;
    }

    USB_Debug_Printf("[DIR-TEST][%s] start uq=%.2fV elec_step=%.3f\r\n",
                     tag, uq, APP_SENSOR_DIR_TEST_ELEC_STEP);

    a0 = app_sensor_angle_after_settle(motor, sensor, uq, 0.0f, APP_SENSOR_DIR_TEST_SETTLE_MS);
    ap = app_sensor_angle_after_settle(motor, sensor, uq, APP_SENSOR_DIR_TEST_ELEC_STEP, APP_SENSOR_DIR_TEST_SETTLE_MS);
    an = app_sensor_angle_after_settle(motor, sensor, uq, -APP_SENSOR_DIR_TEST_ELEC_STEP, APP_SENSOR_DIR_TEST_SETTLE_MS);

    Motor_SetPhaseVoltageQ(motor, 0.0f, 0.0f);

    dp = app_angle_diff_signed(a0, ap);
    dn = app_angle_diff_signed(a0, an);

    if (dp > APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score++;
    } else if (dp < -APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score--;
    }

    if (dn < -APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score++;
    } else if (dn > APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score--;
    }

    USB_Debug_Printf("[DIR-TEST][%s] a0=%.4f ap=%.4f an=%.4f d_plus=%.4f d_minus=%.4f\r\n",
                     tag, a0, ap, an, dp, dn);

    if (score > 0) {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: sensor_direction_cw\r\n", tag);
    } else if (score < 0) {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: sensor_direction_ccw\r\n", tag);
    } else {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: inconclusive (increase uq/settle time and retest)\r\n", tag);
    }

    if (!was_enabled) {
        FOCMotor_disable(motor);
    }
}


/* 左右电机传感器方向测试入口 */
void App_SensorDirectionTest(void)
{
    USB_Debug_Printf("[DIR-TEST] begin (motor should be free to move, not held)\r\n");

#if LEFT_MOTOR_ENABLE
    app_sensor_direction_test_one("L", &g_motor1, &g_sensor1);
#endif

    HAL_Delay(500);

#if RIGHT_MOTOR_ENABLE
    app_sensor_direction_test_one("R", &g_motor2, &g_sensor2);
#endif

    USB_Debug_Printf("[DIR-TEST] end\r\n");
}


/* 非中断版应用循环：主要用于早期开环/速度环调试。
 * 正式 10kHz FOC 闭环目前走 App_LoopForIT()。
 */
void App_Loop(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (g_last_loop_tick_ms == 0U) {
        g_last_loop_tick_ms = now_ms;
        return;
    }

    uint32_t dt_ms = now_ms - g_last_loop_tick_ms;
    if (dt_ms == 0U) {
        return;
    }

    g_last_loop_tick_ms = now_ms;

    float dt = (float)dt_ms * 0.001f;

    if (!Motor_UpdateSensor(&g_motor1, dt)) {
        return;
    }

    float elec_angle = g_motor1.electrical_angle;
    float vel = Sensor_GetVelocityRaw(&g_sensor1);
    float uq_cmd = APP_LOOP_TEST_UQ_V;
    float vel_target = g_speed_target_radps;
    float vel_error = vel_target - vel;

#if APP_SPEED_LOOP_ENABLE
    if (fabsf(vel) > APP_SPEED_VEL_FAULT_ABS) {
        g_speed_fault1 = 1U;
    }

    if (!g_speed_fault1) {
        PID_Calculate(&g_speed_pid1, vel_target, vel, 0U);
        uq_cmd = g_speed_pid1.output;
    } else {
        uq_cmd = 0.0f;
    }
#endif

    float sin_e1 = 0.0f;
    float cos_e1 = 0.0f;

    Get_SinCos(elec_angle, &sin_e1, &cos_e1);
    Motor_SetPhaseVoltageQBySinCos(&g_motor1, uq_cmd, sin_e1, cos_e1);

    if ((now_ms - g_last_print_tick_ms) >= APP_LOOP_PRINT_PERIOD_MS) {
        g_last_print_tick_ms = now_ms;

#if APP_SPEED_LOOP_ENABLE
        USB_Debug_Printf("tgt=%.2f vel=%.3f err=%.3f uq=%.2f mech=%.4f elec=%.4f fault=%u\r\n",
                         vel_target,
                         vel,
                         vel_error,
                         uq_cmd,
                         Sensor_GetAngle(&g_sensor1),
                         g_motor1.electrical_angle,
                         (unsigned)g_speed_fault1);
#else
        USB_Debug_Printf("mech=%.4f elec=%.4f vel=%.3f uq=%.2f\r\n",
                         Sensor_GetAngle(&g_sensor1),
                         g_motor1.electrical_angle,
                         vel,
                         uq_cmd);
#endif
    }
}


/* 启动 FOC 控制定时器中断 */
void App_FOCControlIT_Enable(void)
{
    App_FOC_ForcePowerStageOff();

    if (HAL_TIM_Base_Start_IT(&htim5) == HAL_OK) {
        g_foc_control_it_enabled = 1U;
    } else {
        g_foc_control_it_enabled = 0U;
    }
}


/* 速度环调试变量 */
uint8_t App_FOC_SetPowerStageEnabled(uint8_t enable)
{
    uint8_t should_restore_tim5_irq = 0U;

    if (enable > 1U) {
        return 0U;
    }

    if (enable != 0U) {
        if ((g_foc_stack_ready == 0U) ||
            (g_foc_control_it_enabled == 0U) ||
            (g_bus_voltage_valid == 0U)) {
            return 0U;
        }
    }

    if (g_foc_control_it_enabled != 0U) {
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        should_restore_tim5_irq = 1U;
    }

    App_FOC_SetIqTarget(0.0f, 0.0f);
    App_ResetSpeedPIDs();
    App_ResetCurrentPIDs();

    if (enable == 0U) {
        (void)App_Attitude_SetControlEnabled(0U);
        App_FOC_ForcePowerStageOff();
    } else {
#if LEFT_MOTOR_ENABLE
        FOCMotor_enable(&g_motor1);
#endif
#if RIGHT_MOTOR_ENABLE
        FOCMotor_enable(&g_motor2);
#endif
        g_foc_power_stage_enabled = 1U;
    }

    if (should_restore_tim5_irq != 0U) {
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
    }

    return 1U;
}

uint8_t App_FOC_IsPowerStageEnabled(void)
{
    return g_foc_power_stage_enabled;
}

uint8_t App_FOC_SetDriverGateEnabled(uint8_t enable)
{
    uint8_t applied = 0U;

    if (enable > 1U) {
        return 0U;
    }

#if LEFT_MOTOR_ENABLE
    if (g_driver1 != NULL) {
        if (enable != 0U) {
            Driver_Enable(g_driver1);
        } else {
            Driver_Disable(g_driver1);
        }
        applied = 1U;
    }
#endif

#if RIGHT_MOTOR_ENABLE
    if (g_driver2 != NULL) {
        if (enable != 0U) {
            Driver_Enable(g_driver2);
        } else {
            Driver_Disable(g_driver2);
        }
        applied = 1U;
    }
#endif

    return applied;
}

float vel1 = 0;
float vel_windowed1 = 0;
float vel_windowed_f1 = 0;
float uq_cmd1 = APP_LOOP_TEST_UQ_V;

float vel2 = 0;
float vel_windowed2 = 0;
float vel_windowed_f2 = 0;
float uq_cmd2 = APP_LOOP_TEST_UQ_V;

/* move() 降采样计数器：速度环不必和 10kHz 电流环同频 */
static uint16_t g_move_downsample_cnt = 10U;


/* 电流环观测变量 */
PhaseCurrent_t Left_Current = {0.0f, 0.0f};
float Left_RawIq = 0.0f;
float Left_FilteredIq = 0.0f;

PhaseCurrent_t Right_Current = {0.0f, 0.0f};
float Right_RawIq = 0.0f;
float Right_FilteredIq = 0.0f;


/* 重置速度环 PID 运行状态 */
void App_ResetSpeedPIDs(void)
{
    __disable_irq();
    App_PIDResetRuntime(&g_speed_pid1);
    App_PIDResetRuntime(&g_speed_pid2);
    __enable_irq();
}


#if APP_BUS_VOLTAGE_ENABLE
static uint8_t App_BusVoltageStartupSample(void)
{
    uint32_t i;
    uint32_t valid_count = 0U;
    uint32_t raw_sum = 0U;
    float adc_pin_sum = 0.0f;
    float bus_sum = 0.0f;

    g_bus_voltage_valid = 0U;

    for (i = 0U; i < APP_BUS_VOLTAGE_STARTUP_SAMPLE_COUNT; i++) {
        if (BusVoltage_SampleOnce(&g_bus_voltage)) {
            const uint16_t raw_adc = BusVoltage_GetRawAdc(&g_bus_voltage);
            const float adc_pin_voltage = BusVoltage_GetAdcPinVoltage(&g_bus_voltage);
            const float bus_voltage = BusVoltage_GetBusVoltage(&g_bus_voltage);

            if ((bus_voltage >= APP_BUS_VOLTAGE_VALID_MIN_V) &&
                (bus_voltage <= APP_BUS_VOLTAGE_VALID_MAX_V)) {
                raw_sum += raw_adc;
                adc_pin_sum += adc_pin_voltage;
                bus_sum += bus_voltage;
                valid_count++;
            }
        }

        HAL_Delay(1U);
    }

    if (valid_count == 0U) {
        g_bus_voltage_debug.raw_adc = 0U;
        g_bus_voltage_debug.adc_pin_voltage = 0.0f;
        g_bus_voltage_debug.bus_voltage = 0.0f;
        g_bus_voltage_filtered = 0.0f;
        return 0U;
    }

    g_bus_voltage_debug.raw_adc =
        (uint16_t)((raw_sum + (valid_count / 2U)) / valid_count);
    g_bus_voltage_debug.adc_pin_voltage = adc_pin_sum / (float)valid_count;
    g_bus_voltage_debug.bus_voltage = bus_sum / (float)valid_count;
    g_bus_voltage_filtered = LowPassFilter_Update(&g_bus_voltage_lpf,
                                                  g_bus_voltage_debug.bus_voltage);
    g_bus_voltage_valid = 1U;

    return 1U;
}
#endif


static FastRingSample_t g_fastring_buf[APP_FASTRING_SIZE];
static FastRingSample_t g_fastring_snapshot_buf[APP_FASTRING_SIZE];
static volatile uint16_t g_fastring_head = 0U;
static volatile uint16_t g_fastring_count = 0U;
static volatile uint32_t g_fastring_write_seq = 0U;
static volatile uint16_t g_fastring_snapshot_count = 0U;
static volatile uint32_t g_fastring_snapshot_write_seq = 0U;




static uint16_t app_fastring_status_flags_snapshot(void)
{
    uint16_t flags = 0U;

    if (g_speed_fault1 > 0.5f) {
        flags |= APP_FOC_STATUS_FLAG_SPEED_FAULT_L;
    }
    if (g_speed_fault2 > 0.5f) {
        flags |= APP_FOC_STATUS_FLAG_SPEED_FAULT_R;
    }
    if (g_foc_stack_ready != 0U) {
        flags |= APP_FOC_STATUS_FLAG_STACK_READY;
    }
    if (g_foc_control_it_enabled != 0U) {
        flags |= APP_FOC_STATUS_FLAG_CONTROL_IT_ENABLED;
    }
    if (g_bus_voltage_valid != 0U) {
        flags |= APP_FOC_STATUS_FLAG_BUS_VALID;
    }
    flags |= APP_FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE;
#if APP_SPEED_LOOP_ENABLE
    flags |= APP_FOC_STATUS_FLAG_SPEED_LOOP_ENABLED;
#endif
#if APP_CURRENT_LOOP_ENABLE
    flags |= APP_FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED;
#endif
    if (g_foc_power_stage_enabled == 0U) {
        flags |= APP_FOC_STATUS_FLAG_POWER_STAGE_OFF;
    }
    if (App_Attitude_IsControlEnabled() != 0U) {
        flags |= APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON;
    }
    return flags;
}

static int16_t app_fastlog_scale_to_i16(float value, float scale)
{
    float scaled;

    if (scale == 0.0f) {
        return 0;
    }

    scaled = value * scale;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
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
    uint16_t local_count;
    uint16_t local_head;
    uint32_t local_write_seq;

    __disable_irq();
    local_count = g_fastring_count;
    local_head = g_fastring_head;
    local_write_seq = g_fastring_write_seq;
    __enable_irq();

    if (count != NULL) {
        *count = local_count;
    }
    if (capacity != NULL) {
        *capacity = APP_FASTRING_SIZE;
    }
    if (head != NULL) {
        *head = local_head;
    }
    if (write_seq != NULL) {
        *write_seq = local_write_seq;
    }
}

uint16_t App_CopyFastRingLatest(uint16_t start_idx,
                                uint8_t max_samples,
                                FastRingSample_t *out)
{
    uint16_t local_count;
    uint16_t local_head;
    uint16_t base_idx;
    uint16_t available;
    uint16_t i;

    if ((out == NULL) || (max_samples == 0U)) {
        return 0U;
    }

    __disable_irq();
    local_count = g_fastring_count;
    local_head = g_fastring_head;

    if (start_idx >= local_count) {
        __enable_irq();
        return 0U;
    }

    available = (uint16_t)(local_count - start_idx);
    if ((uint16_t)max_samples < available) {
        available = (uint16_t)max_samples;
    }

    base_idx = (uint16_t)((local_head + APP_FASTRING_SIZE - local_count) % APP_FASTRING_SIZE);
    for (i = 0U; i < available; i++) {
        uint16_t ring_idx = (uint16_t)((base_idx + start_idx + i) % APP_FASTRING_SIZE);
        out[i] = g_fastring_buf[ring_idx];
    }
    __enable_irq();

    return available;
}

void App_SnapshotFastRing(uint16_t *count,
                          uint16_t *capacity,
                          uint32_t *write_seq)
{
    uint16_t local_count;
    uint16_t local_head;
    uint16_t base_idx;
    uint16_t i;
    uint32_t local_write_seq;

    __disable_irq();
    local_count = g_fastring_count;
    local_head = g_fastring_head;
    local_write_seq = g_fastring_write_seq;
    __enable_irq();

    base_idx = (uint16_t)((local_head + APP_FASTRING_SIZE - local_count) % APP_FASTRING_SIZE);
    for (i = 0U; i < local_count; i++) {
        uint16_t ring_idx = (uint16_t)((base_idx + i) % APP_FASTRING_SIZE);
        g_fastring_snapshot_buf[i] = g_fastring_buf[ring_idx];
    }

    __disable_irq();
    g_fastring_snapshot_count = local_count;
    g_fastring_snapshot_write_seq = local_write_seq;
    __enable_irq();

    if (count != NULL) {
        *count = local_count;
    }
    if (capacity != NULL) {
        *capacity = APP_FASTRING_SIZE;
    }
    if (write_seq != NULL) {
        *write_seq = local_write_seq;
    }
}

void App_GetFastRingSnapshotStatus(uint16_t *count,
                                   uint16_t *capacity,
                                   uint32_t *write_seq)
{
    uint16_t local_count;
    uint32_t local_write_seq;

    __disable_irq();
    local_count = g_fastring_snapshot_count;
    local_write_seq = g_fastring_snapshot_write_seq;
    __enable_irq();

    if (count != NULL) {
        *count = local_count;
    }
    if (capacity != NULL) {
        *capacity = APP_FASTRING_SIZE;
    }
    if (write_seq != NULL) {
        *write_seq = local_write_seq;
    }
}

uint16_t App_CopyFastRingSnapshotChunk(uint32_t snapshot_write_seq,
                                       uint16_t start_idx,
                                       uint8_t max_samples,
                                       FastRingSample_t *out)
{
    uint16_t local_count;
    uint16_t available;
    uint16_t i;

    if ((out == NULL) || (max_samples == 0U)) {
        return 0U;
    }

    __disable_irq();
    if (snapshot_write_seq != g_fastring_snapshot_write_seq) {
        __enable_irq();
        return 0U;
    }

    local_count = g_fastring_snapshot_count;
    if (start_idx >= local_count) {
        __enable_irq();
        return 0U;
    }

    available = (uint16_t)(local_count - start_idx);
    if ((uint16_t)max_samples < available) {
        available = (uint16_t)max_samples;
    }

    for (i = 0U; i < available; i++) {
        out[i] = g_fastring_snapshot_buf[start_idx + i];
    }
    __enable_irq();

    return available;
}

APP_FOC_HOT
static void fastring_push_dual(void)
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


/* 10kHz FOC 主体：
 * 1. 更新传感器和电角度；
 * 2. 计算 sin/cos；
 * 3. 读取相电流并计算 Iq；
 * 4. 电流环计算 Uq；
 * 5. 输出 FVPWM；
 * 6. FastRing 记录。
 */
APP_FOC_HOT
static uint8_t loopFOC(void)
{
#if LEFT_MOTOR_ENABLE
    if (!Motor_UpdateSensor(&g_motor1, FOC_PERIOD_S)) {
        return 0U;
    }
#endif

#if RIGHT_MOTOR_ENABLE
    if (!Motor_UpdateSensor(&g_motor2, FOC_PERIOD_S)) {
        return 0U;
    }
#endif

    {
        float sin_e1 = 0.0f;
        float cos_e1 = 0.0f;
        float sin_e2 = 0.0f;
        float cos_e2 = 0.0f;

#if LEFT_MOTOR_ENABLE
        Get_SinCos(g_motor1.electrical_angle, &sin_e1, &cos_e1);
#endif

#if RIGHT_MOTOR_ENABLE
        Get_SinCos(g_motor2.electrical_angle, &sin_e2, &cos_e2);
#endif

        float Uq_cmd1 = 0.0f;
        float Uq_cmd2 = 0.0f;
        float iq_target_left;
        float iq_target_right;

        __disable_irq();
        iq_target_left = g_iq_target_left;
        iq_target_right = g_iq_target_right;
        __enable_irq();

#if LEFT_MOTOR_ENABLE && APP_CURRENT_LOOP_ENABLE
        Left_Current = CurrentSense_GetPhaseCurrent(&g_current_sense1);
        Left_RawIq = CurrentSense_CalcIq(&g_current_sense1, sin_e1, cos_e1);
        Left_FilteredIq = LowPassFilter_Update(&g_current_lpf1, Left_RawIq);

        if (g_current_pid_mode == 0U) {
            Uq_cmd1 = App_CurrentLoopComputeUq(&g_motor1,
                                               &g_current_pid1,
                                               iq_target_left,
                                               Left_FilteredIq,
                                               Left_RawIq,
                                               1U,
                                               &g_current_loop_debug1,
                                               &g_current_i_unload_limit_ticks1,
                                               &g_current_iq_ref1);
        } else {
            Uq_cmd1 = App_CurrentLoopComputeUq(&g_motor1,
                                               &g_current_pid1_Common,
                                               iq_target_left,
                                               Left_FilteredIq,
                                               Left_RawIq,
                                               0U,
                                               &g_current_loop_debug1,
                                               &g_current_i_unload_limit_ticks1,
                                               &g_current_iq_ref1);
        }
#endif

#if RIGHT_MOTOR_ENABLE && APP_CURRENT_LOOP_ENABLE
        Right_Current = CurrentSense_GetPhaseCurrent(&g_current_sense2);
        Right_RawIq = CurrentSense_CalcIq(&g_current_sense2, sin_e2, cos_e2);
        Right_FilteredIq = LowPassFilter_Update(&g_current_lpf2, Right_RawIq);

        if (g_current_pid_mode == 0U) {
            Uq_cmd2 = App_CurrentLoopComputeUq(&g_motor2,
                                               &g_current_pid2,
                                               iq_target_right,
                                               Right_FilteredIq,
                                               Right_RawIq,
                                               1U,
                                               &g_current_loop_debug2,
                                               &g_current_i_unload_limit_ticks2,
                                               &g_current_iq_ref2);
        } else {
            Uq_cmd2 = App_CurrentLoopComputeUq(&g_motor2,
                                               &g_current_pid2_Common,
                                               iq_target_right,
                                               Right_FilteredIq,
                                               Right_RawIq,
                                               0U,
                                               &g_current_loop_debug2,
                                               &g_current_i_unload_limit_ticks2,
                                               &g_current_iq_ref2);
        }
#endif

        if (g_foc_power_stage_enabled != 0U) {
            Motor_SetPhaseVoltageQBySinCos(&g_motor1, Uq_cmd1, sin_e1, cos_e1);
            Motor_SetPhaseVoltageQBySinCos(&g_motor2, Uq_cmd2, sin_e2, cos_e2);
        } else {
            Motor_SetPhaseVoltageQBySinCos(&g_motor1, 0.0f, sin_e1, cos_e1);
            Motor_SetPhaseVoltageQBySinCos(&g_motor2, 0.0f, sin_e2, cos_e2);
        }

        fastring_push_dual();
    }

    return 1U;
}


/* 速度环/上层控制降频执行。
 * 电流环 10kHz，速度环没必要同频，使用 APP_MOVE_DOWNSAMPLE 降采样。
 */
static void move(void)
{
    float vel_target1 = g_speed_target_radps;
    float vel_target2 = g_speed_target_radps;

    vel1 = Sensor_GetVelocityRaw(&g_sensor1);
    vel_windowed1 = Sensor_GetVelocityWindowed(&g_sensor1);
    vel_windowed_f1 = LowPassFilter_Update(&g_speed_lpf1, vel_windowed1);

    vel2 = Sensor_GetVelocityRaw(&g_sensor2);
    vel_windowed2 = Sensor_GetVelocityWindowed(&g_sensor2);
    vel_windowed_f2 = LowPassFilter_Update(&g_speed_lpf2, vel_windowed2);

#if APP_SPEED_LOOP_ENABLE
    PID_Calculate(&g_speed_pid1, vel_target1, vel_windowed_f1, 0U);
    uq_cmd1 = g_speed_pid1.output;

    PID_Calculate(&g_speed_pid2, vel_target2, vel_windowed_f2, 0U);
    uq_cmd2 = g_speed_pid2.output;
#else
    uq_cmd1 = APP_LOOP_TEST_UQ_V;
    uq_cmd2 = APP_LOOP_TEST_UQ_V;
#endif

    /* 保存左速度环调试数据 */
    Left_Velocity_FOC_PID = g_speed_pid1;
    pid_csv_data.timestamp_ms = HAL_GetTick();
    pid_csv_data.setpoint = vel_target1;
    pid_csv_data.input = vel_windowed_f1;
    pid_csv_data.pwm = uq_cmd1;
    pid_csv_data.error = vel_target1 - vel_windowed_f1;
    pid_csv_data.p_term = g_speed_pid1.Kp * pid_csv_data.error;
    pid_csv_data.i_term = g_speed_pid1.Ki * g_speed_pid1.error_integral;
    pid_csv_data.d_term = g_speed_pid1.Kd * (pid_csv_data.error - g_speed_pid1.last_error);
}


/* 10kHz 定时器中断入口。
 * loopFOC 每次执行；move() 按降采样周期执行。
 */
void App_LoopForIT(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    if (!loopFOC()) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        return;
    }

    g_foc_loop_count++;
    g_foc_last_loop_tick_ms = HAL_GetTick();

    g_move_downsample_cnt++;
    if (g_move_downsample_cnt >= APP_MOVE_DOWNSAMPLE) {
        g_move_downsample_cnt = 0U;
        move();
    }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}


/* 主循环调试输出。 */
void DebuginWhile(void)
{
    uint32_t now_ms;

    now_ms = HAL_GetTick();
#if APP_BUS_VOLTAGE_ENABLE
    if ((now_ms - g_last_bus_voltage_sample_tick_ms) >= APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS) {
        g_last_bus_voltage_sample_tick_ms = now_ms;
        App_ServiceBusVoltageSample();
    }
#endif

    if ((now_ms - g_last_while_debug_tick_ms) >= APP_LOOP_PRINT_PERIOD_MS) {
        g_last_while_debug_tick_ms = now_ms;
        /* 预留常规 while 调试输出入口，当前按需求先留空。 */
    }
}







static uint8_t App_InitBusVoltage(void)
{
    #if APP_BUS_VOLTAGE_ENABLE
        BusVoltage_Setup(&g_bus_voltage, &hadc3);
        BusVoltage_Enable(&g_bus_voltage);
        LowPassFilter_Init(&g_bus_voltage_lpf,
                        APP_BUS_VOLTAGE_LPF_CUTOFF_HZ,
                        1000.0f / (float)APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS);

        if (!App_BusVoltageStartupSample()) {
            USB_Debug_Printf("BusVoltage startup sample failed, PWM disabled\r\n");
            return 0U;
        }

        USB_Debug_Printf("BusVoltage: ADC,PinV,BusV\r\n");
        USB_Debug_Printf("%u,%.3f,%.3f\r\n",
                        (unsigned)g_bus_voltage_debug.raw_adc,
                        g_bus_voltage_debug.adc_pin_voltage,
                        g_bus_voltage_debug.bus_voltage);
    #else
        g_bus_voltage_valid = 1U;
        g_bus_voltage_debug.raw_adc = 0U;
        g_bus_voltage_debug.adc_pin_voltage = 0.0f;
        g_bus_voltage_debug.bus_voltage = V_SUPPLY;
        g_bus_voltage_filtered = V_SUPPLY;
        USB_Debug_Printf("BusVoltage disabled, use fixed V_SUPPLY=%.3f\r\n", V_SUPPLY);
    #endif
        return 1U;
}






static uint8_t App_InitMotor1Stack(void)
{
#if LEFT_MOTOR_ENABLE
    g_motor1.state.motor_status = motor_initializing;
    /* 左电机 Driver 初始化 */
    g_driver1 = Driver_GetInstance(DRIVER_LEFT);
    if (g_driver1 == NULL) {
        g_motor1.state.motor_status = motor_init_failed;
        USB_Debug_Printf("Driver_GetInstance1 failed\r\n");
        return 0U;
    }

    if (!Driver_Init(g_driver1,
                     &htim1,
                     TIM_CHANNEL_1,
                     TIM_CHANNEL_3,
                     TIM_CHANNEL_4,
                     Motor_EN_GPIO_Port,
                     Motor_EN_Pin,
                     1U,
                     19 * 0.577f)) {
        g_motor1.state.motor_status = motor_init_failed;
        USB_Debug_Printf("Driver1_Init failed\r\n");
        return 0U;
    }
    Driver_Disable(g_driver1);
#if APP_BUS_VOLTAGE_FOC_ENABLE
    g_driver1->supply_voltage = g_bus_voltage_filtered;
#else
    g_driver1->supply_voltage = V_SUPPLY;
#endif

    /* 左编码器底层驱动 + Sensor 层初始化 */
    if (!AS5047P_RW_Init(&g_enc1, &hspi3, EcdL_CS_GPIO_Port, EcdL_CS_Pin)) {
        g_motor1.state.motor_status = motor_init_failed;
        USB_Debug_Printf("AS5047P_RW_Init1 failed\r\n");
        return 0U;
    }

    Sensor_LinkAS5047P(&g_enc1, &g_sensor1);
    if (!Sensor_Init(&g_sensor1)) {
        g_motor1.state.motor_status = motor_init_failed;
        USB_Debug_Printf("Sensor_Init1 failed\r\n");
        return 0U;
    }

    /* 左电机电流采样初始化。
     * CurrentSense_Init 必须放在最前，因为它会清空 CurrentSense 对象。
     */
    CurrentSense_Init(&g_current_sense1);
    CurrentSense_Config(&g_current_sense1, &hadc1, &htim3, TIM_CHANNEL_4);
    CurrentSenseParam_Init(&g_current_sense1,
                           FOC_SHUNT_RESISTOR_OHM,
                           FOC_AMP_GAIN,
                           1,
                           1);
    CurrentSense_CalibrateOffsets(&g_current_sense1);

    /* 装配左 Motor 对象 */
    linkSensor(&g_sensor1, &g_motor1);
    linkDriver(g_driver1, &g_motor1);
    linkCurrentSense(&g_current_sense1, &g_motor1);

    MotorParam_Init(&g_motor1, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);
    g_motor1.zero_electrical_angle = 0.0f;
    if (!FOCMotor_ConfigureState(&g_motor1)) {
        g_motor1.state.motor_status = motor_init_failed;
        USB_Debug_Printf("FOCMotor_ConfigureState1 failed\r\n");
        return 0U;
    }
#endif
    return 1U;
}





static uint8_t App_InitMotor2Stack(void)
{
#if RIGHT_MOTOR_ENABLE
    g_motor2.state.motor_status = motor_initializing;
    /* 右电机 Driver 初始化 */
    g_driver2 = Driver_GetInstance(DRIVER_RIGHT);
    if (g_driver2 == NULL) {
        g_motor2.state.motor_status = motor_init_failed;
        USB_Debug_Printf("Driver_GetInstance2 failed\r\n");
        return 0U;
    }

    if (!Driver_Init(g_driver2,
                     &htim4,
                     TIM_CHANNEL_4,
                     TIM_CHANNEL_3,
                     TIM_CHANNEL_2,
                     Motor_EN_GPIO_Port,
                     Motor_EN_Pin,
                     1U,
                     19 * 0.577f)) {
        g_motor2.state.motor_status = motor_init_failed;
        USB_Debug_Printf("Driver2_Init failed\r\n");
        return 0U;
    }
    Driver_Disable(g_driver2);
#if APP_BUS_VOLTAGE_FOC_ENABLE
    g_driver2->supply_voltage = g_bus_voltage_filtered;
#else
    g_driver2->supply_voltage = V_SUPPLY;
#endif

    /* 右编码器底层驱动 + Sensor 层初始化 */
    if (!AS5047P_RW_Init(&g_enc2, &hspi1, EcdR_CS_GPIO_Port, EcdR_CS_Pin)) {
        g_motor2.state.motor_status = motor_init_failed;
        USB_Debug_Printf("AS5047P_RW_Init2 failed\r\n");
        return 0U;
    }

    Sensor_LinkAS5047P(&g_enc2, &g_sensor2);
    if (!Sensor_Init(&g_sensor2)) {
        g_motor2.state.motor_status = motor_init_failed;
        USB_Debug_Printf("Sensor_Init2 failed\r\n");
        return 0U;
    }

    /* 右电机电流采样初始化 */
    CurrentSense_Init(&g_current_sense2);
    CurrentSense_Config(&g_current_sense2, &hadc2, &htim2, TIM_CHANNEL_2);
    CurrentSenseParam_Init(&g_current_sense2,
                           FOC_SHUNT_RESISTOR_OHM,
                           FOC_AMP_GAIN,
                           1,
                           1);
    CurrentSense_CalibrateOffsets(&g_current_sense2);

    /* 装配右 Motor 对象 */
    linkSensor(&g_sensor2, &g_motor2);
    linkDriver(g_driver2, &g_motor2);
    linkCurrentSense(&g_current_sense2, &g_motor2);

    MotorParam_Init(&g_motor2, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);
    g_motor2.zero_electrical_angle = 0.0f;
    if (!FOCMotor_ConfigureState(&g_motor2)) {
        g_motor2.state.motor_status = motor_init_failed;
        USB_Debug_Printf("FOCMotor_ConfigureState2 failed\r\n");
        return 0U;
    }
#endif
    return 1U;
}


static uint8_t App_InitFOCAlgorithm(void)
{
    #if APP_SPEED_LOOP_ENABLE
        /* 速度环 PID 初始化。当前速度环仍属于 V0.1 验证阶段 */
        PID_ParameterInitEx(&g_speed_pid1,
                            APP_SPEED_KP,
                            APP_SPEED_KI,
                            APP_SPEED_KD,
                            APP_SPEED_I_LIMIT,
                            APP_SPEED_UQ_LIMIT,
                            APP_SPEED_I_ERR_MIN,
                            APP_SPEED_I_SEP_RATIO);

        PID_ParameterInitEx(&g_speed_pid2,
                            APP_SPEED_KP,
                            APP_SPEED_KI,
                            APP_SPEED_KD,
                            APP_SPEED_I_LIMIT,
                            APP_SPEED_UQ_LIMIT,
                            APP_SPEED_I_ERR_MIN,
                            APP_SPEED_I_SEP_RATIO);

        Left_Velocity_FOC_PID = g_speed_pid1;
        g_speed_target_radps = 0.3f;
        g_speed_fault1 = 0U;
        g_speed_fault2 = 0U;

        /* 速度反馈低通，当前截止频率 100Hz */
        LowPassFilter_Init(&g_speed_lpf1, 50.0f, FOC_FREQUENCY);
        LowPassFilter_Init(&g_speed_lpf2, 50.0f, FOC_FREQUENCY);
    #endif


    #if APP_CURRENT_LOOP_ENABLE
        /* 前馈 PI 电流环参数组 */
        PID_ParameterInitEx(&g_current_pid1,
                            APP_CURRENT_FF_KP,
                            APP_CURRENT_FF_KI,
                            APP_CURRENT_FF_KD,
                            APP_CURRENT_FF_I_LIMIT,
                            APP_CURRENT_OUT_LIMIT,
                            APP_CURRENT_I_ERR_MIN,
                            CURRENT_LOOP_I_SEP_RATIO);

        PID_ParameterInitEx(&g_current_pid2,
                            APP_CURRENT_FF_KP,
                            APP_CURRENT_FF_KI,
                            APP_CURRENT_FF_KD,
                            APP_CURRENT_FF_I_LIMIT,
                            APP_CURRENT_OUT_LIMIT,
                            APP_CURRENT_I_ERR_MIN,
                            CURRENT_LOOP_I_SEP_RATIO);

        /* 纯 PI 对照参数组，保留用于实验对比 */
        PID_ParameterInitEx(&g_current_pid1_Common,
                            5.3f,
                            0.62f,
                            0.0f,
                            APP_CURRENT_PURE_PI_I_LIMIT,
                            11.0f,
                            0.05f,
                            CURRENT_LOOP_PURE_PI_I_SEP_RATIO);

        PID_ParameterInitEx(&g_current_pid2_Common,
                            5.3f,
                            0.62f,
                            0.0f,
                            APP_CURRENT_PURE_PI_I_LIMIT,
                            11.0f,
                            0.05f,
                            CURRENT_LOOP_PURE_PI_I_SEP_RATIO);

        /* 电流反馈低通，当前截止频率 800Hz */
        LowPassFilter_Init(&g_current_lpf1, 800.0f, FOC_FREQUENCY);
        LowPassFilter_Init(&g_current_lpf2, 800.0f, FOC_FREQUENCY);

        App_ResetCurrentPIDs();
    #endif
        return 1U;
}

static void App_FOC_ForcePowerStageOff(void)
{
#if LEFT_MOTOR_ENABLE
    FOCMotor_disable(&g_motor1);
#endif
#if RIGHT_MOTOR_ENABLE
    FOCMotor_disable(&g_motor2);
#endif

    g_foc_power_stage_enabled = 0U;
}
