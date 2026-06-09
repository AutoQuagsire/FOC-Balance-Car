/**
 * @file BLDCMotor.h
 * @brief BLDC 电机控制对象定义与 Motor 层接口声明
 *
 * 本文件属于 SimpleFOC_Learning 中的 Motor 层。
 *
 * Motor 层职责：
 * 1. 管理电机物理参数、控制配置和运行状态；
 * 2. 连接 Driver / Sensor / CurrentSense 等底层模块；
 * 3. 提供电角度计算、零位电角度校准、q轴电压输出等接口；
 * 4. 为后续 FOC 电流环、速度环、位置环提供统一的电机对象。
 *
 * 分层原则：
 * - Driver 层负责 PWM / 三相电压输出；
 * - Sensor 层负责机械角、跨圈角、速度估计；
 * - CurrentSense 层负责电流采样；
 * - Motor 层只通过这些模块的公共接口取数据或输出控制量，
 *   不直接操作具体编码器芯片或具体驱动芯片细节。
 */

#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#include <stdint.h>
#include "driver.h"
#include "current_sense.h"
#include "sensor.h"

/* ============================================================
 * 电流环编译期配置宏
 * ============================================================
 */
#define BLDCMOTOR_ENABLE_CURRENT_SENSE 1



/**
 * @brief 是否启用电流环前馈 PI 控制
 *
 * 1：启用前馈 + PI
 * 0：保留纯 PI 控制路径，用于对比实验或调试
 *
 * 当前工程建议：
 * 使用前馈 PI 作为正式方案。
 */
#ifndef CURRENT_LOOP_USE_FEEDFORWARD
#define CURRENT_LOOP_USE_FEEDFORWARD 1
#endif


/**
 * @brief 前馈 PI 模式下的积分分离比例
 *
 * 积分分离阈值通常形如：
 *
 *     i_band = CURRENT_LOOP_I_SEP_RATIO * |Iq_ref|
 *
 * 当误差绝对值小于该阈值时，允许正常积分。
 *
 * 作用：
 * - 大误差阶段主要依靠前馈和比例项快速响应；
 * - 接近目标后再允许积分补偿稳态误差；
 * - 避免积分在阶跃初期过早堆积，造成超调。
 *
 * 当前实验定稿候选值：
 * 0.2f
 */
#ifndef CURRENT_LOOP_I_SEP_RATIO
#define CURRENT_LOOP_I_SEP_RATIO 0.2f
#endif


/**
 * @brief 纯 PI 模式下的积分分离比例
 *
 * 纯 PI 没有前馈项提供基础电压，因此积分需要更早参与。
 * 如果阈值太小，比例项可能无法把误差压入积分区间，
 * 导致稳态误差难以消除。
 *
 * 当前仅用于纯 PI 对比实验。
 */
#ifndef CURRENT_LOOP_PURE_PI_I_SEP_RATIO
#define CURRENT_LOOP_PURE_PI_I_SEP_RATIO 0.75f
#endif


/**
 * @brief 单个控制周期内，积分卸载的最大变化量
 *
 * 用于限制积分项在目标阶跃后过快卸载。
 *
 * 背景：
 * 在反向阶跃，例如 0.8A -> 0.6A 时，
 * 目标下降会导致 error 变负，积分项如果快速卸载，
 * Uq_final 会被拉得过低，从而造成明显下冲。
 *
 * 该限速用于缓和积分项变化，使反向下冲更可控。
 */
#ifndef CURRENT_LOOP_I_UNLOAD_STEP_MAX
#define CURRENT_LOOP_I_UNLOAD_STEP_MAX 0.01f
#endif


/**
 * @brief 积分卸载限速持续的控制周期数
 *
 * 控制周期假设为 100us 时：
 *
 *     4 ticks = 0.4ms
 *
 * 作用：
 * 只在目标刚发生阶跃后的短时间内限制积分卸载，
 * 避免长期影响稳态积分补偿能力。
 */
#ifndef CURRENT_LOOP_I_UNLOAD_LIMIT_TICKS
#define CURRENT_LOOP_I_UNLOAD_LIMIT_TICKS 4U
#endif


/**
 * @brief 判断目标电流发生变化的最小阈值
 *
 * 如果：
 *
 *     |target_iq_new - target_iq_last| > CURRENT_LOOP_TARGET_STEP_EPS
 *
 * 则认为发生了一次目标阶跃。
 *
 * 该阈值用于避免由于浮点误差或微小抖动反复触发瞬态策略。
 */
#ifndef CURRENT_LOOP_TARGET_STEP_EPS
#define CURRENT_LOOP_TARGET_STEP_EPS 0.005f
#endif


/**
 * @brief 内部电流参考 iq_ref 的最大上升斜率
 *
 * 单位：
 * A / control_step
 *
 * 如果电流环周期为 100us：
 *
 *     0.025A / 100us = 250A/s
 *
 * 作用：
 * 把外部阶跃目标 target_iq_cmd 转换为内部斜坡目标 iq_ref，
 * 减少 FF、P、I、积分限幅在目标突变时同时跳变造成的冲击。
 */
#ifndef CURRENT_LOOP_IQ_REF_STEP_UP_MAX
#define CURRENT_LOOP_IQ_REF_STEP_UP_MAX 0.02f
#endif


/**
 * @brief 内部电流参考 iq_ref 的最大下降斜率
 *
 * 当前和上升斜率保持一致。
 *
 * 如果后续反向下冲仍然偏大，可以考虑：
 *
 *     down_step < up_step
 *
 * 但当前实验中 0.025A / 100us 对称斜坡已经可以作为工程定稿候选。
 */
#ifndef CURRENT_LOOP_IQ_REF_STEP_DOWN_MAX
#define CURRENT_LOOP_IQ_REF_STEP_DOWN_MAX 0.02f
#endif


/*
 * ============================================================
 * CurrentLoop_FFPI_V1
 *
 * 电流环控制结构：
 *
 *     Uq = PI(Iq_ref - Iq_meas)
 *          + Iq_ref * R_phase * ff_coef
 *
 * 其中：
 * - Iq_ref 是经过斜率限制后的内部电流目标；
 * - Iq_meas 是实际测得的 q 轴电流；
 * - ff_coef 是前馈修正系数；
 * - R_phase 是电机相电阻；
 * - PI 用于修正动态误差和残余稳态误差。
 *
 * 当前策略：
 * - ff_coef 根据 |Iq_ref| 查表插值；
 * - integral_limit 根据 |Iq_ref| 查表插值；
 * - 低电流段使用较小积分限幅，降低小电流超调；
 * - 高电流段使用较大积分限幅，保证稳态补偿能力。
 *
 * 纯 PI 路径仅用于对比实验和调试，不作为当前主方案。
 * ============================================================
 */


/**
 * @brief 电流环自适应调度表的单个标定点
 *
 * 通过多个标定点描述不同电流幅值下适合的：
 * - 前馈系数；
 * - 积分限幅。
 *
 * 实际运行时会根据 |Iq_ref| 在线性插值得到连续参数。
 */
typedef struct {
    float iq_abs;          /**< 电流幅值标定点，单位 A */
    float ff_coef;         /**< 前馈系数 */
    float integral_limit;  /**< 积分限幅，单位通常等效为 V 或 PI 输出量 */
} CurrentLoopSchedulePoint_t;


/**
 * @brief 电流环调试快照
 *
 * 用于高速记录或串口打印电流环运行状态。
 *
 * 推荐用途：
 * - 分析阶跃响应；
 * - 对比 filtered_iq / raw_iq；
 * - 观察 pi_out、ff_term、uq_final 的贡献；
 * - 验证自适应 FF / LLIMIT 是否按预期变化；
 * - 分析 pid_integral 是否过快堆积或卸载。
 */
typedef struct {
    float target_iq;       /**< 外部目标电流，速度环/上层控制给出的目标 */
    float iq_ref;          /**< 经过斜率限制后的内部电流目标 */
    float filtered_iq;     /**< 滤波后的 q 轴电流，通常作为控制反馈 */
    float raw_iq;          /**< 未滤波或轻滤波的 q 轴电流，用于观察真实瞬态 */
    float error;           /**< 电流误差：iq_ref - filtered_iq */
    float pi_out;          /**< PI 控制器输出 */
    float ff_term;         /**< 前馈电压项 */
    float uq_final;        /**< 最终 q 轴电压命令 */
    float ff_coef;         /**< 当前插值得到的前馈系数 */
    float integral_limit;  /**< 当前插值得到的积分限幅 */
    float pid_integral;    /**< 当前积分项状态 */
} CurrentLoopDebugSnapshot_t;



/* ============================================================
 * 电机物理参数
 * ============================================================
 *
 * 这部分描述电机本体参数，一般由规格书、测量或参数辨识得到。
 * 运行过程中通常不会频繁修改。
 */
typedef struct {
    float pole_pairs;         /**< 极对数 */
    float phase_resistance;   /**< 相电阻，单位 Ω */
    float kv;                 /**< 电机 KV 值，通常单位 rpm/V */
    float Lq;                 /**< q 轴电感，单位 H */
    float Ld;                 /**< d 轴电感，单位 H */
} MotorParam_t;


/* ============================================================
 * 电机外环模式
 * ============================================================
 *
 * 描述 Motor 层希望跟踪的上层目标。
 * 这里保留开环速度，暂不保留开环位置，因为它和闭环位置的语义容易混淆。
 */
typedef enum {
    motor_outer_torque=0,             /**< 力矩目标 */
    motor_outer_velocity,           /**< 闭环速度目标 */
    motor_outer_position,           /**< 闭环位置/角度目标 */
    motor_outer_openloop_velocity   /**< 开环速度 */
} MotorOuterLoopMode_t;


/* ============================================================
 * 电机内环模式
 * ============================================================
 *
 * 描述 Motor 层内部输出采用的执行方式。
 */
typedef enum {
    motor_inner_voltage=0,    /**< 电压模式 */
    motor_inner_current     /**< 电流环模式 */
} MotorInnerLoopMode_t;



/* ============================================================
 * 电机控制配置
 * ============================================================
 *
 * 这部分是控制策略配置，属于用户设定，
 * 不属于电机物理参数。
 */
typedef struct {
    MotorOuterLoopMode_t outer_loop; /**< 当前外环模式 */
    MotorInnerLoopMode_t inner_loop; /**< 当前内环模式 */
    float voltage_limit;             /**< Motor 层允许输出的最大电压 */
    float voltage_sensor_align;      /**< 传感器对齐/零电角校准时使用的电压 */
} MotorConfig_t;


/* ============================================================
 * 三相电压输出状态
 * ============================================================
 *
 * 保存当前三相目标输出电压。
 * 主要用于调试、状态观察或后续扩展。
 */
typedef struct {
    float Ua; /**< A 相电压 */
    float Ub; /**< B 相电压 */
    float Uc; /**< C 相电压 */
} PhaseVoltage_t;


/* ============================================================
 * 电机状态枚举
 * ============================================================
 *
 * 描述 Motor 对象当前处于哪个生命周期阶段。
 */
typedef enum {
    motor_init_failed = 0, /**< 初始化失败 */
    motor_initializing,    /**< 正在初始化 */
    motor_uncalibrated,    /**< 已初始化，但尚未完成零位电角度校准 */
    motor_ready,           /**< 已完成初始化/校准，可正常运行 */
    motor_error            /**< 运行过程中出现错误 */
} MotorStatus_t;


/* ============================================================
 * 传感器方向枚举
 * ============================================================
 *
 * 描述传感器角度增加方向与电机定义正方向之间的关系。
 *
 * 用途：
 * 电角度计算时，如果传感器方向与电机正方向相反，
 * 需要对极对数或机械角方向取反。
 */
typedef enum {
    sensor_direction_unknown = 0, /**< 方向未知，尚未校准 */
    sensor_direction_cw,          /**< 顺时针为正方向 */
    sensor_direction_ccw          /**< 逆时针为正方向 */
} SensorDirection_t;


/* ============================================================
 * 电机运行状态
 * ============================================================
 *
 * 存放运行过程中会变化的状态量。
 */
typedef struct {
    uint8_t enabled;                    /**< 电机是否已使能：1=已使能，0=未使能 */
    uint8_t has_sensor;                 /**< 是否连接并初始化传感器：1=有，0=无 */
    MotorStatus_t motor_status;         /**< 当前电机状态 */
    SensorDirection_t sensor_direction; /**< 传感器方向 */
    PhaseVoltage_t phase_voltage;       /**< 当前三相输出电压状态 */
} MotorState_t;


/* ============================================================
 * 电机总对象
 * ============================================================
 *
 * Motor_t 是 Motor 层的核心对象，包含：
 * - 电机物理参数；
 * - 控制配置；
 * - 运行状态；
 * - 与 Driver / CurrentSense / Sensor 的连接关系；
 * - 当前电角度和零位电角度。
 *
 * 设计原则：
 * Motor 层不直接操作底层硬件寄存器，
 * 而是通过 driver、current_sense、sensor 指针访问对应模块。
 */
typedef struct {
    MotorParam_t param;             /**< 电机物理参数 */
    MotorConfig_t config;           /**< 控制配置参数 */
    MotorState_t state;             /**< 运行状态信息 */

    Driver_t *driver;               /**< 指向电机驱动模块 */
    CurrentSense_t *current_sense;  /**< 指向电流采样模块 */
    Sensor_t *sensor;               /**< 指向角度/速度传感器模块 */

    float electrical_angle;         /**< 当前电角度，单位 rad */
    float zero_electrical_angle;    /**< 零位电角度，单位 rad */

} Motor_t;


/* ============================================================
 * 模块链接接口
 * ============================================================
 */

/**
 * @brief 将 Driver 模块绑定到 Motor 对象
 *
 * Motor 后续通过 driver 指针输出三相 PWM / 电压。
 */
void linkDriver(Driver_t *driver, Motor_t *motor);


/**
 * @brief 将 Sensor 模块绑定到 Motor 对象
 *
 * Motor 后续通过 Sensor_* 接口获取机械角、速度等数据。
 */
void linkSensor(Sensor_t *sensor, Motor_t *motor);


/**
 * @brief 将 CurrentSense 模块绑定到 Motor 对象
 *
 * Motor 后续通过 CurrentSense 模块获取相电流 / q轴电流等信息。
 */
void linkCurrentSense(CurrentSense_t *Current_Sense, Motor_t *motor);


/* ============================================================
 * Motor 生命周期接口
 * ============================================================
 */

/**
 * @brief Motor 层初始化
 *
 * 检查 Driver / Sensor 状态，整理配置参数，
 * 并使 Motor 进入可校准状态。
 *
 * 注意：
 * 该函数不等价于零位电角度校准。
 * 调用后仍需要 Motor_CalibrateZeroElectricalAngle()。
 */
uint8_t FOCMotor_ConfigureState(Motor_t *motor);


/**
 * @brief 初始化电机物理参数
 *
 * @param motor            Motor 对象
 * @param pole_pairs       极对数
 * @param phase_resistance 相电阻
 * @param kv               KV 值
 * @param Ld               d 轴电感
 * @param Lq               q 轴电感
 */
void MotorParam_Init(Motor_t *motor, float pole_pairs, float phase_resistance,
                     float kv, float Ld, float Lq);


/**
 * @brief 失能电机输出
 *
 * 通常会关闭 PWM 输出，并关闭 Driver。
 */
void FOCMotor_disable(Motor_t *FOC_Motor);


/**
 * @brief 使能电机输出
 *
 * 通常会使能 Driver，并将 PWM 初值清零。
 */
void FOCMotor_enable(Motor_t *FOC_Motor);


/* ============================================================
 * 角度与传感器接口
 * ============================================================
 */

/**
 * @brief 获取当前机械角
 *
 * 返回 Sensor 层提供的机械角，单位 rad。
 */
float Motor_GetMechanicalAngle(Motor_t *motor);


/**
 * @brief 获取当前电角度
 *
 * 电角度由机械角、极对数、传感器方向和零位电角度共同决定。
 */
float Motor_GetElectricalAngle(Motor_t *motor);


/**
 * @brief 更新传感器数据并同步 Motor 内部电角度
 *
 * @param dt 本次更新周期，单位 s
 *
 * 该函数会调用 Sensor_Update()，然后更新 motor->electrical_angle。
 */
uint8_t Motor_UpdateSensor(Motor_t *motor, float dt);
void Get_SinCos(float angle_el, float *sint, float *cost);


/* ============================================================
 * 相电压 / q轴电压输出接口
 * ============================================================
 */

/**
 * @brief 根据电角度输出 q 轴电压
 *
 * 内部会计算 sin/cos，然后调用底层 FVPWM / SVPWM 输出。
 */
void Motor_SetPhaseVoltageQ(Motor_t *motor, float uq, float elec_angle);


/**
 * @brief 使用已知 sin/cos 输出 q 轴电压
 *
 * 用于控制循环中已经计算过 sin/cos 的场景，
 * 避免重复调用 sinf/cosf。
 */
void Motor_SetPhaseVoltageQBySinCos(Motor_t *motor,
                                    float uq,
                                    float sin_el,
                                    float cos_el);


/* ============================================================
 * 电流环参数调度接口
 * ============================================================
 */

/**
 * @brief 根据目标电流获取前馈系数和积分限幅
 *
 * @param target_iq       目标 q 轴电流，可以为正或负
 * @param ff_coef         输出：前馈系数
 * @param integral_limit  输出：积分限幅
 *
 * 内部根据 |target_iq| 查表并线性插值。
 */
void CurrentLoop_GetScheduledParams(float target_iq,
                                    float *ff_coef,
                                    float *integral_limit);


/* ============================================================
 * 零位电角度校准接口
 * ============================================================
 */

/**
 * @brief 执行零位电角度校准
 *
 * @param motor          Motor 对象
 * @param align_voltage  对齐电压
 * @param align_angle    对齐电角度
 * @param settle_ms      等待转子稳定的时间，单位 ms
 *
 * 作用：
 * 施加固定电角度电压矢量，使转子吸附到已知位置，
 * 然后读取机械角并计算 zero_electrical_angle。
 */
uint8_t Motor_CalibrateZeroElectricalAngle(Motor_t *motor,
                                           float align_voltage,
                                           float align_angle,
                                           uint16_t settle_ms);
uint8_t Motor_SetControlMode(Motor_t *motor,
                             MotorOuterLoopMode_t outer_loop,
                             MotorInnerLoopMode_t inner_loop);
#endif /* BLDC_MOTOR_H */
