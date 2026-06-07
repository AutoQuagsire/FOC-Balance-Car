#include "./BLDCMotor.h"
#include "./driver.h"
#include "./current_sense.h"
#include "./platform.h"
#include "foc_common.h"
#include <math.h>
#include <stddef.h>

#if defined(__has_attribute)
  #if __has_attribute(optimize)
    #define ATTR_OPT_FAST __attribute__((optimize("O2,fast-math")))
  #else
    #define ATTR_OPT_FAST
  #endif
#else
  #define ATTR_OPT_FAST
#endif

#if defined(__has_attribute)
  #if __has_attribute(always_inline)
    #define ATTR_ALWAYS_INLINE __attribute__((always_inline)) inline
  #else
    #define ATTR_ALWAYS_INLINE inline
  #endif
#else
  #define ATTR_ALWAYS_INLINE inline
#endif

ATTR_OPT_FAST
void Get_SinCos(float angle_el, float *sint, float *cost)
{
    if ((sint == NULL) || (cost == NULL)) {
        return;
    }
#if defined(__GNUC__)
    __builtin_sincosf(angle_el, sint, cost);
#else
    *sint = sinf(angle_el);
    *cost = cosf(angle_el);
#endif
}





/* ============================================================
 * 电流环自适应参数调度表
 *
 * 作用：
 * 根据 |target_iq| 的大小，插值得到：
 * 1. ff_coef        ：前馈系数，用于计算电压前馈项
 * 2. integral_limit ：电流环积分限幅，用于限制积分输出
 *
 * 设计背景：
 * 小电流段需要更小的积分限幅，避免积分造成小电流超调；
 * 大电流段需要更大的积分限幅，保证有足够稳态补偿能力。
 *
 * 注意：
 * 这里的 iq_abs 必须按绝对值查表，因为正负电流只代表方向，
 * 参数调度主要取决于电流幅值。
 * ============================================================ */
static const CurrentLoopSchedulePoint_t current_loop_schedule_table[] = {
    {0.03f, 0.420f, 0.300f},
    {0.10f, 0.460f, 0.300f},
    {0.30f, 0.485f, 0.400f},
    {0.60f, 0.500f, 0.500f},
    {0.90f, 0.510f, 0.550f},
    {1.20f, 0.520f, 0.580f},
    {1.50f, 0.530f, 0.600f},
    {1.80f, 0.540f, 0.650f},
};


/* 一维线性插值函数
 *
 * a：区间左端点参数值
 * b：区间右端点参数值
 * t：插值比例，范围理论上为 0~1
 *
 * 返回：
 * a 和 b 之间按 t 比例插值得到的值
 */
static ATTR_ALWAYS_INLINE float current_loop_lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}


/* ============================================================
 * 根据目标电流幅值获取电流环调度参数
 *
 * 输入：
 * target_iq       ：目标 q 轴电流，可以为正或负
 *
 * 输出：
 * ff_coef         ：前馈系数
 * integral_limit  ：积分限幅
 *
 * 逻辑：
 * 1. 对 target_iq 取绝对值；
 * 2. 若小于表格最小点，则使用最小点参数；
 * 3. 若大于表格最大点，则使用最大点参数；
 * 4. 若位于表格中间，则寻找相邻两个点并线性插值。
 *
 * 这样做的好处：
 * 不需要为每一个电流值单独测参数，
 * 只需要测几个关键电流点，然后通过插值平滑过渡。
 * ============================================================ */
ATTR_OPT_FAST
void CurrentLoop_GetScheduledParams(float target_iq,
                                    float *ff_coef,
                                    float *integral_limit)
{
    const size_t table_size = sizeof(current_loop_schedule_table) / sizeof(current_loop_schedule_table[0]);
    const CurrentLoopSchedulePoint_t *last_point;
    float iq_abs;
    size_t idx;

    /* 参数合法性检查：
     * 如果输出指针为空，或者表格为空，直接返回。
     */
    if ((ff_coef == NULL) || (integral_limit == NULL) || (table_size == 0U)) {
        return;
    }

    /* 调度表按电流幅值工作，正负方向不影响参数选择 */
    iq_abs = fabsf(target_iq);
    last_point = &current_loop_schedule_table[table_size - 1U];

    /* 小于最小标定点：钳制到最小点参数 */
    if (iq_abs <= current_loop_schedule_table[0].iq_abs) {
        *ff_coef = current_loop_schedule_table[0].ff_coef;
        *integral_limit = current_loop_schedule_table[0].integral_limit;
        return;
    }

    /* 大于最大标定点：钳制到最大点参数 */
    if (iq_abs >= last_point->iq_abs) {
        *ff_coef = last_point->ff_coef;
        *integral_limit = last_point->integral_limit;
        return;
    }

    /* 在调度表中寻找 iq_abs 所在区间 */
    for (idx = 0U; idx < (table_size - 1U); idx++) {
        const CurrentLoopSchedulePoint_t *p0 = &current_loop_schedule_table[idx];
        const CurrentLoopSchedulePoint_t *p1 = &current_loop_schedule_table[idx + 1U];

        if (iq_abs <= p1->iq_abs) {
            const float span = p1->iq_abs - p0->iq_abs;

            /* ratio 表示 iq_abs 在 p0~p1 区间内的位置 */
            const float ratio = (span > 0.0f) ? ((iq_abs - p0->iq_abs) / span) : 0.0f;

            *ff_coef = current_loop_lerp(p0->ff_coef, p1->ff_coef, ratio);
            *integral_limit = current_loop_lerp(p0->integral_limit, p1->integral_limit, ratio);
            return;
        }
    }

    /* 理论上不会执行到这里，作为兜底保护 */
    *ff_coef = last_point->ff_coef;
    *integral_limit = last_point->integral_limit;
}


/* ============================================================
 * 绑定 Driver 到 Motor
 *
 * 作用：
 * Motor 层不直接持有具体 PWM 输出细节，
 * 只保存一个 Driver_t 指针，通过 Driver 层输出三相 PWM。
 * ============================================================ */
void linkDriver(Driver_t *driver, Motor_t *motor)
{
    if (!driver || !motor) return;

    motor->driver = driver;
}


/* ============================================================
 * 绑定 Sensor 到 Motor
 *
 * 作用：
 * 把传感器对象挂到电机对象上。
 * Motor 层之后只通过 Sensor_* 接口获取机械角、速度等信息，
 * 不直接访问 AS5047P / AS5600 等具体编码器驱动。
 * ============================================================ */
void linkSensor(Sensor_t *sensor, Motor_t *motor)
{
    if (!sensor || !motor) return;

    motor->sensor = sensor;
    motor->state.has_sensor = (sensor != NULL) ? 1U : 0U;
}


/* 绑定电流采样对象到 Motor */
void linkCurrentSense(CurrentSense_t *Current_Sense, Motor_t *motor)
{
    if (!Current_Sense || !motor) return;

    motor->current_sense = Current_Sense;
}


/* ============================================================
 * FOCMotor_init()
 *
 * 功能：
 * 完成 Motor 层的软件初始化检查和状态整理。
 *
 * 主要流程：
 * 1. 检查 Motor 指针是否合法；
 * 2. 检查 Driver 是否已经连接并初始化；
 * 3. 检查 Sensor 是否可用；
 * 4. 整理电压限制，保证 Motor 限制不超过 Driver 限制；
 * 5. 整理电机参数，例如 Ld/Lq；
 * 6. 对开环模式补默认方向；
 * 7. 使能电机并进入未校准状态。
 *
 * 注意：
 * 这个函数不是零位电角度校准。
 * 执行完 init 后，电机状态仍然是 motor_uncalibrated，
 * 后续还需要 Motor_CalibrateZeroElectricalAngle()。
 * ============================================================ */
uint8_t FOCMotor_init(Motor_t *FOC_Motor)
{
    if (!FOC_Motor) {
        return 0;
    }

    /* Driver 是 Motor 正常输出 PWM 的前提。
     * 如果 Driver 没有连接，或者 Driver_Init() 没有成功，
     * 则 Motor 初始化失败。
     */
    if (!FOC_Motor->driver || !(FOC_Motor->driver->initialized)) {
        FOC_Motor->state.motor_status = motor_init_failed;
        FOC_Motor->state.enabled = 0;
        return 0;
    }

    /* 检查传感器是否已经初始化。
     * 有 Sensor 才能进行闭环角度/速度/FOC 控制。
     */
    if (FOC_Motor->sensor && FOC_Motor->sensor->initialized) {
        FOC_Motor->state.has_sensor = 1U;
    } else {
        FOC_Motor->state.has_sensor = 0U;
    }

    FOC_Motor->state.motor_status = motor_initializing;

    /* 电压限制保护：
     * Motor 的 voltage_limit 不允许超过 Driver 的最大输出限制。
     */
    if (FOC_Motor->config.voltage_limit > FOC_Motor->driver->voltage_limit) {
        FOC_Motor->config.voltage_limit = FOC_Motor->driver->voltage_limit;
    }

    /* 零位校准用的对齐电压也不能超过 Motor 的电压限制 */
    if (FOC_Motor->config.voltage_sensor_align > FOC_Motor->config.voltage_limit) {
        FOC_Motor->config.voltage_sensor_align = FOC_Motor->config.voltage_limit;
    }

    /* 电感参数整理：
     * 如果只设置了 Ld 或 Lq 中的一个，则默认另一个相同。
     * 对表贴式永磁同步电机，Ld/Lq 接近时这样处理是可接受的。
     */
    if (FOC_Motor->param.Ld == 0.0f && FOC_Motor->param.Lq != 0.0f) {
        FOC_Motor->param.Ld = FOC_Motor->param.Lq;
    } else if (FOC_Motor->param.Lq == 0.0f && FOC_Motor->param.Ld != 0.0f) {
        FOC_Motor->param.Lq = FOC_Motor->param.Ld;
    }

    /* 开环控制模式下，如果没有传感器且方向未知，
     * 则给一个默认方向，避免后续控制逻辑没有方向信息。
     */
    if ((FOC_Motor->state.has_sensor == 0U) &&
        (FOC_Motor->config.outer_loop == motor_outer_openloop_velocity) &&
        (FOC_Motor->state.sensor_direction == sensor_direction_unknown)) {
        FOC_Motor->state.sensor_direction = sensor_direction_cw;
    }
    /* 使能前后给一点延时，给驱动芯片和 PWM 输出状态稳定的时间 */
    /* 初始化完成后，电机还没有做零位电角度校准 */
    FOC_Motor->state.motor_status = motor_uncalibrated;

    return 1;
}


/* ============================================================
 * 电机失能
 *
 * 顺序：
 * 1. 如果有电流采样模块，先关闭电流采样；
 * 2. PWM 输出清零；
 * 3. 关闭 Driver；
 * 4. 更新 enabled 状态。
 *
 * 设计原则：
 * 先去能量，再断执行链。
 * ============================================================ */
uint8_t FOCMotor_ConfigureState(Motor_t *motor)
{
    if (motor == NULL) {
        return 0U;
    }
    if ((motor->driver == NULL) || (motor->driver->initialized == 0U)) {
        return 0U;
    }

    if ((motor->sensor != NULL) && (motor->sensor->initialized != 0U)) {
        motor->state.has_sensor = 1U;
    } else {
        motor->state.has_sensor = 0U;
    }

    motor->state.motor_status = motor_initializing;

    if (motor->config.voltage_limit <= 0.0f) {
        motor->config.voltage_limit = motor->driver->voltage_limit;
    } else if (motor->config.voltage_limit > motor->driver->voltage_limit) {
        motor->config.voltage_limit = motor->driver->voltage_limit;
    }

    if (motor->config.voltage_sensor_align <= 0.0f) {
        motor->config.voltage_sensor_align = motor->config.voltage_limit;
    } else if (motor->config.voltage_sensor_align > motor->config.voltage_limit) {
        motor->config.voltage_sensor_align = motor->config.voltage_limit;
    }

    if ((motor->param.Ld == 0.0f) && (motor->param.Lq != 0.0f)) {
        motor->param.Ld = motor->param.Lq;
    } else if ((motor->param.Lq == 0.0f) && (motor->param.Ld != 0.0f)) {
        motor->param.Lq = motor->param.Ld;
    }

    if (motor->state.sensor_direction == sensor_direction_unknown) {
        motor->state.sensor_direction = sensor_direction_cw;
    }

    motor->state.motor_status = motor_uncalibrated;
    return 1U;
}

void FOCMotor_disable(Motor_t *motor)
{
    if (!motor || !motor->driver) return;
    if (!motor->driver->initialized) return;

    if (motor->current_sense) {
        CurrentSense_Disable(motor->current_sense);
    }

    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);
    Driver_Disable(motor->driver);

    motor->state.enabled = 0;
}


/* ============================================================
 * 电机使能
 *
 * 顺序：
 * 1. 使能 Driver；
 * 2. PWM 输出清零，避免一使能就输出未知占空比；
 * 3. 如果有电流采样模块，则使能电流采样；
 * 4. 更新 enabled 状态。
 * ============================================================ */
void FOCMotor_enable(Motor_t *motor)
{
    if (!motor || !motor->driver) return;
    if (!motor->driver->initialized) return;

    Driver_Enable(motor->driver);

    /* 使能后立即清零 PWM，保证安全 */
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);

    if (motor->current_sense) {
        CurrentSense_Enable(motor->current_sense);
    }

    motor->state.enabled = 1;
}


/* 初始化电机参数 */
void MotorParam_Init(Motor_t *motor, float pole_pairs, float phase_resistance,
                     float kv, float Ld, float Lq)
{
    if (!motor) return;

    motor->param.pole_pairs = pole_pairs;
    motor->param.phase_resistance = phase_resistance;
    motor->param.kv = kv;
    motor->param.Ld = Ld;
    motor->param.Lq = Lq;
}


/* ============================================================
 * 根据机械角计算电角度
 *
 * 电角度 = pole_pairs * mechanical_angle - zero_electrical_angle
 *
 * 如果编码器方向为 CCW，则等效极对数取负，
 * 这样可以统一处理正反方向。
 *
 * 该函数不做空指针检查，调用前必须保证 motor/sensor 合法，
 * 因此命名为 Unchecked。
 * ============================================================ */
ATTR_OPT_FAST
static ATTR_ALWAYS_INLINE float Motor_CalcElectricalAngleUnchecked(const Motor_t *motor,
                                                                   const Sensor_t *sensor)
{
    float mech_angle = sensor->data.shaft_angle;
    float pole_pairs = motor->param.pole_pairs;

    if (motor->state.sensor_direction == sensor_direction_ccw) {
        pole_pairs = -pole_pairs;
    }

    float elec_angle = pole_pairs * mech_angle - motor->zero_electrical_angle;
    return normalizeAngle(elec_angle);
}


/* 获取机械角 */
float Motor_GetMechanicalAngle(Motor_t *motor)
{
    if (!motor || !motor->sensor) {
        return 0.0f;
    }

    if (!(motor->sensor->initialized)) {
        return 0.0f;
    }

    return Sensor_GetAngle(motor->sensor);
}


/* 获取当前电角度 */
ATTR_OPT_FAST
float Motor_GetElectricalAngle(Motor_t *motor)
{
    if (!motor) {
        return 0.0f;
    }

    Sensor_t *sensor = motor->sensor;
    if (!sensor || !sensor->initialized) {
        return 0.0f;
    }

    return Motor_CalcElectricalAngleUnchecked(motor, sensor);
}


/* ============================================================
 * 更新传感器数据，并同步电角度
 *
 * 输入：
 * dt：本次传感器更新周期，用于 Sensor 层速度估计
 *
 * 作用：
 * 1. 调用 Sensor_Update() 更新机械角、跨圈角、速度；
 * 2. 根据最新机械角计算电角度；
 * 3. 存入 motor->electrical_angle。
 * ============================================================ */
ATTR_OPT_FAST
uint8_t Motor_UpdateSensor(Motor_t *motor, float dt)
{
    if (!motor || !motor->sensor) {
        return 0U;
    }

    if (!(motor->sensor->initialized)) {
        return 0U;
    }

    Sensor_Update(motor->sensor, dt);

    motor->electrical_angle = Motor_GetElectricalAngle(motor);

    return 1U;
}


/* 快速浮点限幅 */
static ATTR_OPT_FAST ATTR_ALWAYS_INLINE float clampf_fast(float x, float low, float high)
{
    return (x < low) ? low : ((x > high) ? high : x);
}


/* ============================================================
 * BLDC_SetFVPWM()
 *
 * 功能：
 * 根据 q 轴电压 uq 和电角度 sin/cos，计算三相 PWM。
 *
 * 输入：
 * uq：q 轴电压命令，正负决定转矩方向
 * st：sin(electrical_angle)
 * ct：cos(electrical_angle)
 *
 * 当前实现：
 * Ud = 0，只输出 q 轴电压。
 *
 * 流程：
 * 1. 对 uq 做限幅；
 * 2. 反 Park 变换得到 Ualpha/Ubeta；
 * 3. Clarke 逆变换得到三相电压 Ua/Ub/Uc；
 * 4. 使用零序注入，把三相电压平移到 [0, V_SUPPLY]；
 * 5. 转换为 TIM CCR 比较值；
 * 6. 写入 PWM。
 * ============================================================ */
ATTR_OPT_FAST
static void BLDC_SetFVPWM(Motor_t *motor, float uq, float st, float ct)
{
    float vbus;
    float uq_limit;
    float uq_hw_max;

    if (!motor || !motor->driver || !motor->driver->htim) {
        return;
    }

#if APP_BUS_VOLTAGE_FOC_ENABLE
    /* 优先使用实时母线电压；异常时回退到编译期默认值。 */
    vbus = motor->driver->supply_voltage;
    if (vbus <= 0.0f || vbus > 26.0f) {
        vbus = V_SUPPLY;
    }
#else
    vbus = V_SUPPLY;
#endif

    uq_hw_max = vbus * 0.577f;

    /* uq 限幅，既要满足控制层限制，也不能超过当前母线电压可实现范围。 */
    uq_limit = motor->driver->voltage_limit;
    if (uq_limit <= 0.0f || uq_limit > uq_hw_max) {
        uq_limit = uq_hw_max;
    }

    if (uq > uq_limit) {
        uq = uq_limit;
    } else if (uq < -uq_limit) {
        uq = -uq_limit;
    }

    /* 反 Park 变换：
     * Ud = 0
     * Ualpha = Ud*cos - Uq*sin = -Uq*sin
     * Ubeta  = Ud*sin + Uq*cos =  Uq*cos
     */
    float Ualpha = -uq * st;
    float Ubeta  =  uq * ct;

    /* 逆 Clarke，得到三相中心对称电压 */
    float t      = _SQRT3 * Ubeta;
    float Ua     = Ualpha;
    float Ub     = (-Ualpha + t) * 0.5f;
    float Uc     = (-Ualpha - t) * 0.5f;

    /* 零序注入：
     * 通过减去最大最小值的中点，把三相电压整体平移，
     * 使其落入 [0, V_SUPPLY]，提高母线电压利用率。
     */
    float Umax = Ua;
    if (Ub > Umax) Umax = Ub;
    if (Uc > Umax) Umax = Uc;

    float Umin = Ua;
    if (Ub < Umin) Umin = Ub;
    if (Uc < Umin) Umin = Uc;

    float Uzero = (vbus * 0.5f) - (Umax + Umin) * 0.5f;

    Ua = clampf_fast(Ua + Uzero, 0.0f, vbus);
    Ub = clampf_fast(Ub + Uzero, 0.0f, vbus);
    Uc = clampf_fast(Uc + Uzero, 0.0f, vbus);

    /* 电压映射到 PWM 比较值 */
    const float scale = (float)motor->driver->htim->Init.Period * (1.0f / vbus);
    const uint32_t ccr_a = (uint32_t)(Ua * scale + 0.5f);
    const uint32_t ccr_b = (uint32_t)(Ub * scale + 0.5f);
    const uint32_t ccr_c = (uint32_t)(Uc * scale + 0.5f);

    Driver_SetCompareFast(motor->driver, ccr_a, ccr_b, ccr_c);
}


/* 施加一个固定电角度的 q 轴电压矢量。
 * 主要用于零位电角度校准时吸住转子。
 */
static void Motor_ApplyAlignVector(Motor_t *motor, float uq, float elec_angle)
{
    float st = sinf(elec_angle);
    float ct = cosf(elec_angle);
    BLDC_SetFVPWM(motor, uq, st, ct);
}


/* 已经有 sin/cos 时直接输出 q 轴电压，避免重复计算三角函数 */
void Motor_SetPhaseVoltageQBySinCos(Motor_t *motor, float uq, float sin_el, float cos_el)
{
    BLDC_SetFVPWM(motor, uq, sin_el, cos_el);
}


/* 根据电角度输出 q 轴电压 */
void Motor_SetPhaseVoltageQ(Motor_t *motor, float uq, float elec_angle)
{
    float st = sinf(elec_angle);
    float ct = cosf(elec_angle);
    BLDC_SetFVPWM(motor, uq, st, ct);
}


/* ============================================================
 * 零位电角度校准
 *
 * 功能：
 * 施加一个固定电角度电压矢量，使转子吸附到已知电角位置，
 * 然后读取机械角，计算 zero_electrical_angle。
 *
 * 输入：
 * align_voltage：校准电压
 * align_angle  ：施加的对齐电角度
 * settle_ms    ：等待转子稳定的时间
 *
 * 核心公式：
 * zero_electrical_angle =
 *     dir * pole_pairs * mech_align - theta_field
 *
 * 其中：
 * mech_align  ：转子稳定后的机械角
 * theta_field ：实际施加的磁场方向
 *
 * 注意：
 * 这里对 mech_align 做圆均值，而不是普通平均。
 * 因为角度是周期量，接近 0/2π 边界时普通平均会出错。
 * ============================================================ */
uint8_t Motor_CalibrateZeroElectricalAngle(Motor_t *motor,
                                           float align_voltage,
                                           float align_angle,
                                           uint16_t settle_ms)
{
    uint8_t was_enabled;

    if (!motor || !motor->sensor || !motor->driver) {
        return 0U;
    }

    if (!motor->sensor->initialized) {
        return 0U;
    }

    if (!motor->driver->initialized) {
        return 0U;
    }

    was_enabled = motor->state.enabled;

    /* 1. 使能电机 */
    FOCMotor_enable(motor);

    /* 2. 施加固定电角度矢量，让转子吸附到指定位置 */
    Motor_ApplyAlignVector(motor, align_voltage, align_angle);

    /* 3. 等待机械转子稳定 */
    HAL_Delay(settle_ms);

    /* 4. 多次采样机械角，做圆均值 */
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    for (uint16_t i = 0; i < 32; i++) {
        /* 这里只是刷新 Sensor 层角度数据。
         * dt 给正值即可，不依赖这次速度估计。
         */
        Sensor_Update(motor->sensor, 0.001f);

        float a = Sensor_GetAngle(motor->sensor);

        sum_sin += sinf(a);
        sum_cos += cosf(a);

        HAL_Delay(2);
    }

    /* 圆均值角度 */
    float mech_align = atan2f(sum_sin, sum_cos);
    mech_align = normalizeAngle(mech_align);

    /* 根据传感器方向确定电角度正方向 */
    float dir = 1.0f;
    if (motor->state.sensor_direction == sensor_direction_ccw) {
        dir = -1.0f;
    }

    /* 注意：
     * 这里 theta_field 使用 align_angle + π/2。
     * 这是因为当前 Motor_ApplyAlignVector() 施加的是 q 轴电压，
     * q 轴电压矢量与转子磁链/d 轴存在 90°关系。
     */
    float theta_field = normalizeAngle(align_angle + 0.5f * PI);

    motor->zero_electrical_angle =
        normalizeAngle(dir * motor->param.pole_pairs * mech_align - theta_field);

    /* 6. 校准完成后关闭 PWM 输出 */
    Driver_SetPwm(motor->driver, 0.0f, 0.0f, 0.0f);
    if (was_enabled == 0U) {
        FOCMotor_disable(motor);
    }

    return 1U;
}
