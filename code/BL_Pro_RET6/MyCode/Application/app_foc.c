#include "app_foc.h"
#include "app_foc_bus.h"
#include "app_foc_debug.h"
#include "app_foc_internal.h"
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
#include "usb_debug.h"

#if defined(__GNUC__)
#define APP_FOC_HOT __attribute__((optimize("O2,fast-math")))
#else
#define APP_FOC_HOT
#endif




static uint8_t App_InitMotor1Stack(void);
static uint8_t App_InitMotor2Stack(void);
static uint8_t App_InitFOCAlgorithm(App_FOCMotorControl_t *control);
static uint8_t App_ConfigureLoopController(App_FOCMotorControl_t *control,
                                           MotorOuterLoopMode_t outer_loop);
static void App_FOC_ForcePowerStageOff(void);

volatile uint8_t g_foc_stack_ready = 0U;
volatile uint8_t g_foc_power_stage_enabled = 0U;

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

    if(!App_FOC_BusInit()) {
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

    App_FOC_ForcePowerStageOff();
    App_ResetFastRing();
    g_foc_stack_ready = 1U;
    USB_Debug_Printf("FOC stack init ok\r\n");
    return 1U;
}





/* =========================
 * 应用层 FOC 对象
 * =========================
 * 这一层的对象不属于某个单独模块，而是“把各模块组起来”
 * 所以统一放在 app_foc.c 内部静态保存
 */

Motor_t                 g_motor1;   // 电机控制对象
Driver_t               *g_driver1 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static AS5047P_Handle_t g_enc1;     // AS5047P 底层驱动句柄
Sensor_t                g_sensor1;  // 传感器公共层对象
CurrentSense_t          g_current_sense1; // 电流采样对象
App_FOCMotorControl_t   g_foc_left_control = {
    .motor = &g_motor1,
    .velocity.speed_target_radps = 0.3f,
    .current.iq_target = 0.0f,
};


Motor_t                 g_motor2;   // 电机控制对象
Driver_t               *g_driver2 = NULL; // 三相驱动对象（由 Driver 模块提供实例）
static AS5047P_Handle_t g_enc2;     // AS5047P 底层驱动句柄
Sensor_t                g_sensor2;  // 传感器公共层对象
CurrentSense_t          g_current_sense2; // 电流采样对象
App_FOCMotorControl_t   g_foc_right_control = {
    .motor = &g_motor2,
    .velocity.speed_target_radps = 0.3f,
    .current.iq_target = 0.0f,
};




static uint32_t         g_last_while_debug_tick_ms = 0U;
volatile CurrentLoopDebugSnapshot_t g_current_loop_debug1;
volatile CurrentLoopDebugSnapshot_t g_current_loop_debug2;
volatile uint8_t g_foc_control_it_enabled = 0U;
volatile uint32_t g_foc_loop_count = 0U;
volatile uint32_t g_foc_last_loop_tick_ms = 0U;
uint8_t g_current_i_unload_limit_ticks1 = 0U;
uint8_t g_current_i_unload_limit_ticks2 = 0U;

volatile uint8_t g_current_pid_mode = 0U; /* 0=CurrentLoop_FFPI_V1, 1=Pure PI compare */
volatile float g_current_i_sep_ratio_ff = CURRENT_LOOP_I_SEP_RATIO;
volatile float g_current_i_sep_ratio_pure = CURRENT_LOOP_PURE_PI_I_SEP_RATIO;

float g_speed_fault2 = 0.0f;
float g_speed_fault1 = 0.0f;

#define LEFT_MOTOR_ENABLE 1U
#define RIGHT_MOTOR_ENABLE 1U

#define APP_LOOP_TEST_UQ_V        (1.0f)
#define APP_LOOP_PRINT_PERIOD_MS  (100U)

#define APP_SPEED_KP               (0.055f)
#define APP_SPEED_KI               (0.00035f)
#define APP_SPEED_KD               (0.0f)
#define APP_SPEED_UQ_LIMIT         (1.8f)
#define APP_SPEED_I_LIMIT          (0.5f)
#define APP_SPEED_I_ERR_MIN        (0.05f)
#define APP_SPEED_I_SEP_RATIO      (0.75f)


#define APP_CURRENT_TARGET_A       (0.0f)

#define APP_LEFT_WHEEL_SPEED_SIGN   (1.0f)
#define APP_RIGHT_WHEEL_SPEED_SIGN  (-1.0f)

#define APP_CURRENT_PURE_PI_I_LIMIT  (6.0f)

#define APP_CURRENT_FF_KP             (2.8f)
#define APP_CURRENT_FF_KI             (0.35f)
#define APP_CURRENT_FF_KD             (0.0f)
#define APP_CURRENT_FF_I_LIMIT          (5.0f)

#define APP_MOVE_DOWNSAMPLE        (10U)


#if (APP_MOVE_DOWNSAMPLE < 1U)
#error "APP_MOVE_DOWNSAMPLE must be >= 1"
#endif


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




float vel_windowed1 = 0;
float uq_cmd1 = APP_LOOP_TEST_UQ_V;

float vel_windowed2 = 0;
float uq_cmd2 = APP_LOOP_TEST_UQ_V;

/* move() 降采样计数器：速度环不必和 10kHz 电流环同频 */
static uint16_t g_move_downsample_cnt = 10U;


/* 电流环观测变量 */
PhaseCurrent_t Left_Current = {0.0f, 0.0f};
float Left_RawIq = 0.0f;
PhaseCurrent_t Right_Current = {0.0f, 0.0f};
float Right_RawIq = 0.0f;




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
    App_FOCMotorControl_t *left = &g_foc_left_control;
    App_FOCMotorControl_t *right = &g_foc_right_control;

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
        iq_target_left = left->current.iq_target;
        iq_target_right = right->current.iq_target;
        __enable_irq();

#if LEFT_MOTOR_ENABLE && APP_CURRENT_LOOP_ENABLE
        Left_Current = CurrentSense_GetPhaseCurrent(&g_current_sense1);
        Left_RawIq = CurrentSense_CalcIq(&g_current_sense1, sin_e1, cos_e1);
        left->current.iq_meas = LowPassFilter_Update(&left->current.current_lpf, Left_RawIq);

        if (g_current_pid_mode == 0U) {
            Uq_cmd1 = App_CurrentLoopComputeUq(&g_motor1,
                                               &left->current.pid_ff,
                                               iq_target_left,
                                               left->current.iq_meas,
                                               Left_RawIq,
                                               1U,
                                               &g_current_loop_debug1,
                                               &g_current_i_unload_limit_ticks1,
                                               &left->current.iq_ref);
        } else {
            Uq_cmd1 = App_CurrentLoopComputeUq(&g_motor1,
                                               &left->current.pid_pi,
                                               iq_target_left,
                                               left->current.iq_meas,
                                               Left_RawIq,
                                               0U,
                                               &g_current_loop_debug1,
                                               &g_current_i_unload_limit_ticks1,
                                               &left->current.iq_ref);
        }
#endif

#if RIGHT_MOTOR_ENABLE && APP_CURRENT_LOOP_ENABLE
        Right_Current = CurrentSense_GetPhaseCurrent(&g_current_sense2);
        Right_RawIq = CurrentSense_CalcIq(&g_current_sense2, sin_e2, cos_e2);
        right->current.iq_meas = LowPassFilter_Update(&right->current.current_lpf, Right_RawIq);

        if (g_current_pid_mode == 0U) {
            Uq_cmd2 = App_CurrentLoopComputeUq(&g_motor2,
                                               &right->current.pid_ff,
                                               iq_target_right,
                                               right->current.iq_meas,
                                               Right_RawIq,
                                               1U,
                                               &g_current_loop_debug2,
                                               &g_current_i_unload_limit_ticks2,
                                               &right->current.iq_ref);
        } else {
            Uq_cmd2 = App_CurrentLoopComputeUq(&g_motor2,
                                               &right->current.pid_pi,
                                               iq_target_right,
                                               right->current.iq_meas,
                                               Right_RawIq,
                                               0U,
                                               &g_current_loop_debug2,
                                               &g_current_i_unload_limit_ticks2,
                                               &right->current.iq_ref);
        }
#endif

        if (g_foc_power_stage_enabled != 0U) {
            Motor_SetPhaseVoltageQBySinCos(&g_motor1, Uq_cmd1, sin_e1, cos_e1);
            Motor_SetPhaseVoltageQBySinCos(&g_motor2, Uq_cmd2, sin_e2, cos_e2);
        } else {
            Motor_SetPhaseVoltageQBySinCos(&g_motor1, 0.0f, sin_e1, cos_e1);
            Motor_SetPhaseVoltageQBySinCos(&g_motor2, 0.0f, sin_e2, cos_e2);
        }

        App_FastRingPushDual();
    }

    return 1U;
}


/* 速度环/上层控制降频执行。
 * 电流环 10kHz，速度环没必要同频，使用 APP_MOVE_DOWNSAMPLE 降采样。
 */
static void move(void)
{
    App_FOCMotorControl_t *left = &g_foc_left_control;
    App_FOCMotorControl_t *right = &g_foc_right_control;
    float vel_target1 = left->velocity.speed_target_radps;
    float vel_target2 = right->velocity.speed_target_radps;

    vel_windowed1 = Sensor_GetVelocityWindowed(&g_sensor1);
    left->velocity.speed_meas_radps = LowPassFilter_Update(&left->velocity.speed_lpf, vel_windowed1);

    vel_windowed2 = Sensor_GetVelocityWindowed(&g_sensor2);
    right->velocity.speed_meas_radps = LowPassFilter_Update(&right->velocity.speed_lpf, vel_windowed2);

#if APP_SPEED_LOOP_ENABLE
    PID_Calculate(&left->velocity.speed_pid, vel_target1, left->velocity.speed_meas_radps, 0U);
    uq_cmd1 = left->velocity.speed_pid.output;

    PID_Calculate(&right->velocity.speed_pid, vel_target2, right->velocity.speed_meas_radps, 0U);
    uq_cmd2 = right->velocity.speed_pid.output;
#else
    uq_cmd1 = APP_LOOP_TEST_UQ_V;
    uq_cmd2 = APP_LOOP_TEST_UQ_V;
#endif

    pid_csv_data.timestamp_ms = HAL_GetTick();
    pid_csv_data.setpoint = vel_target1;
    pid_csv_data.input = left->velocity.speed_meas_radps;
    pid_csv_data.pwm = uq_cmd1;
    pid_csv_data.error = vel_target1 - left->velocity.speed_meas_radps;
    pid_csv_data.p_term = left->velocity.speed_pid.Kp * pid_csv_data.error;
    pid_csv_data.i_term = left->velocity.speed_pid.Ki * left->velocity.speed_pid.error_integral;
    pid_csv_data.d_term = left->velocity.speed_pid.Kd *
                          (pid_csv_data.error - left->velocity.speed_pid.last_error);
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
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - g_last_while_debug_tick_ms) >= APP_LOOP_PRINT_PERIOD_MS) {
        g_last_while_debug_tick_ms = now_ms;
        /* 预留常规 while 调试输出入口，当前按需求先留空。 */
    }
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
    linkDriver(g_driver1, &g_motor1);
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
    linkSensor(&g_sensor1, &g_motor1);

    /* 左电机电流采样初始化。
     * CurrentSense_Init 必须放在最前，因为它会清空 CurrentSense 对象。
     */
#if BLDCMOTOR_ENABLE_CURRENT_SENSE
    CurrentSense_Init(&g_current_sense1);
    CurrentSense_Config(&g_current_sense1, &hadc1, &htim3, TIM_CHANNEL_4);
    CurrentSenseParam_Init(&g_current_sense1,
                           FOC_SHUNT_RESISTOR_OHM,
                           FOC_AMP_GAIN,
                           1,
                           1);
    CurrentSense_CalibrateOffsets(&g_current_sense1);
    linkCurrentSense(&g_current_sense1, &g_motor1);
#endif /* BLDCMOTOR_ENABLE_CURRENT_SENSE */
    /* 装配左 Motor 对象 */




    MotorParam_Init(&g_motor1, 14.0f, 10.3f, 0.0f, 0.0f, 0.0f);
    g_motor1.zero_electrical_angle = 0.0f;
    if (!FOCMotor_ConfigureState(&g_motor1)) {
        g_motor1.state.motor_status = motor_init_failed;
        USB_Debug_Printf("FOCMotor_ConfigureState1 failed\r\n");
        return 0U;
    }
    if (!Motor_SetControlMode(&g_motor1,
                              motor_outer_torque,
                              motor_inner_current)) {
        USB_Debug_Printf("Motor_SetControlMode failed\r\n");
        return 0U;
    }
    if (!App_InitFOCAlgorithm(&g_foc_left_control)) {
        USB_Debug_Printf("App_InitFOCAlgorithm failed\r\n");
        return 0U;
    }
#endif /* LEFT_MOTOR_ENABLE */
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
    if (!Motor_SetControlMode(&g_motor2,
                              motor_outer_torque,
                              motor_inner_current)) {
        USB_Debug_Printf("Motor_SetControlMode failed\r\n");
        return 0U;
    }
    if (!App_InitFOCAlgorithm(&g_foc_right_control)) {
        USB_Debug_Printf("App_InitFOCAlgorithm failed\r\n");
        return 0U;
    }
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




static void App_ResetOuterLoopState(App_FOCMotorControl_t *control)
{
    if (control == NULL) {
        return;
    }

    PID_Reset(&control->velocity.speed_pid);
}


static uint8_t App_ConfigureLoopController(App_FOCMotorControl_t *control,
                                           MotorOuterLoopMode_t outer_loop)
{
    Motor_t *motor;

    if (control == NULL) {
        return 0U;
    }

    motor = control->motor;
    if (motor == NULL) {
        return 0U;
    }
    /* 速度反馈低通，当前截止频率 50Hz */
    LowPassFilter_Init(&control->velocity.speed_lpf, 50.0f, FOC_FREQUENCY);
    PID_Reset(&control->velocity.speed_pid);
    LowPassFilter_Reset(&control->velocity.speed_lpf);
    control->velocity.speed_meas_radps = 0.0f;
    switch (outer_loop)
    {

    case motor_outer_torque:
        App_ResetOuterLoopState(control);
        if (motor->config.inner_loop == motor_inner_voltage) 
        {

        }
        if (motor->config.inner_loop == motor_inner_current) 
        {

        }
        return 1U;

    case motor_outer_velocity:
        if (motor->sensor == NULL) 
        {
            USB_Debug_Printf("FOC velocity mode needs sensor\r\n");
            return 0U;
        }
#if FOC_ENABLE_VELOCITY_LOOP
        if (motor->config.inner_loop == motor_inner_current) 
        {

        /* 速度环 PID 初始化。当前速度环仍属于 V0.1 验证阶段 */
            PID_ParameterInitEx(&control->velocity.speed_pid,
                                APP_SPEED_KP,
                                APP_SPEED_KI,
                                APP_SPEED_KD,
                                APP_SPEED_I_LIMIT,
                                APP_SPEED_UQ_LIMIT,
                                APP_SPEED_I_ERR_MIN,
                                APP_SPEED_I_SEP_RATIO);

        if (control == &g_foc_left_control) {
            g_speed_fault1 = 0U;
        } else if (control == &g_foc_right_control) {
            g_speed_fault2 = 0U;
        }

        control->velocity.speed_target_radps = 0.0f;
        control->velocity.speed_meas_radps = 0.0f;

      
        return 1U;
        }
#endif
        USB_Debug_Printf("velocity current inner loop not implemented\r\n");
        return 0U;

    case motor_outer_position:

        if (motor->sensor == NULL) {
            USB_Debug_Printf("FOC position mode needs sensor\r\n");
            return 0U;
        }
#if FOC_ENABLE_POSITION_LOOP
        if (motor->config.inner_loop == motor_inner_voltage) {
            PID_ParameterInitEx(&g_position_pid1,
                                10.0f, 0.0f, 0.1f,
                                Uq_max,
                                Uq_max,
                                0.05f,
                                0.7f);
            PID_Reset(&g_position_pid1);
            return 1U;
        }
#endif
        USB_Debug_Printf("Position current inner loop not implemented\r\n");
        return 0U;


    case motor_outer_openloop_velocity:
        if (motor->config.inner_loop != motor_inner_voltage) {
            USB_Debug_Printf("Openloop velocity needs voltage inner mode\r\n");
            return 0U;
        }

        App_ResetOuterLoopState(control);
        return 1U;

    default:
        return 0U;
    }
}



static uint8_t App_InitFOCAlgorithm(App_FOCMotorControl_t *control)
{
    Motor_t *motor;

    if (control == NULL) {
        return 0U;
    }
    motor = control->motor;
    if (motor == NULL) {
        return 0U;
    }
    if (motor->driver == NULL) {
        return 0U;
    }

    switch (motor->config.inner_loop)
    {
    case motor_inner_voltage:
       
        
        break;

    case motor_inner_current:
#if BLDCMOTOR_ENABLE_CURRENT_SENSE
        if (motor->current_sense == NULL) {
            USB_Debug_Printf("FOC current mode needs current_sense\r\n");
            return 0U;
        }
        PID_ParameterInitEx(&control->current.pid_ff,
                            APP_CURRENT_FF_KP, APP_CURRENT_FF_KI, APP_CURRENT_FF_KD,
                            APP_CURRENT_FF_I_LIMIT,
                            APP_CURRENT_OUT_LIMIT,
                            APP_CURRENT_I_ERR_MIN,
                            CURRENT_LOOP_I_SEP_RATIO);
        PID_ParameterInitEx(&control->current.pid_pi,
                            5.3f,
                            0.62f,
                            0.0f,
                            APP_CURRENT_PURE_PI_I_LIMIT,
                            11.0f,
                            0.05f,
                            CURRENT_LOOP_PURE_PI_I_SEP_RATIO);
        PID_Reset(&control->current.pid_ff);
        PID_Reset(&control->current.pid_pi);
        LowPassFilter_Reset(&control->velocity.speed_lpf);
        LowPassFilter_Init(&control->current.current_lpf, 800.0f, FOC_FREQUENCY);
        control->current.iq_target = APP_CURRENT_TARGET_A;
        control->current.iq_ref = 0.0f;
        control->current.iq_meas = 0.0f;
#else
        USB_Debug_Printf("FOC current mode disabled by macro\r\n");
        return 0U;
#endif
        break;

    default:
        return 0U;
    }

#if APP_CURRENT_LOOP_ENABLE
    App_ResetCurrentPIDs(control);
#endif

    return App_ConfigureLoopController(control, motor->config.outer_loop);
}




void App_FOC_SetIqTarget(float left_iq, float right_iq)
{
    __disable_irq();
    g_foc_left_control.current.iq_target = left_iq;
    g_foc_right_control.current.iq_target = right_iq;
    __enable_irq();
}


float App_FOC_GetAverageWheelSpeedRadps(void)
{
    float left_speed;
    float right_speed;

    __disable_irq();
    left_speed = g_foc_left_control.velocity.speed_meas_radps;
    right_speed = g_foc_right_control.velocity.speed_meas_radps;
    __enable_irq();

    return 0.5f * ((APP_LEFT_WHEEL_SPEED_SIGN * left_speed) +
                   (APP_RIGHT_WHEEL_SPEED_SIGN * right_speed));
}

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
    App_ResetCurrentPIDs(&g_foc_left_control);
    App_ResetCurrentPIDs(&g_foc_right_control);

    if (enable == 0U) {
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



/* 重置 PID 运行状态：只清积分、上次误差和输出，不改 PID 参数 */
void App_PIDResetRuntime(PID_t *pid)
{
    PID_Reset(pid);
}


/* 重置速度环 PID 运行状态 */
void App_ResetSpeedPIDs(void)
{
    __disable_irq();
    App_PIDResetRuntime(&g_foc_left_control.velocity.speed_pid);
    App_PIDResetRuntime(&g_foc_right_control.velocity.speed_pid);
    __enable_irq();
}
