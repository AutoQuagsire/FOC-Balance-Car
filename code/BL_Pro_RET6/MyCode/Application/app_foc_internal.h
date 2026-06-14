#ifndef APP_FOC_INTERNAL_H
#define APP_FOC_INTERNAL_H

#include <stdint.h>

#include "BLDCMotor.h"
#include "BusVoltage.h"
#include "PID.h"
#include "app_foc_control.h"
#include "current_sense.h"
#include "driver.h"
#include "sensor.h"
#include "app_foc_debug.h"

/*
 * app_foc 模块内部共享头。
 *
 * 这个文件的定位，不是对外 API，而是 app_foc 模块族内部多个 .c
 * 文件之间的“共享声明汇总”：
 *
 * - app_foc.c            : 主控制逻辑、FOC 栈初始化、目标下发
 * - app_foc_bus.c        : 母线电压采样 / 滤波 / 状态维护
 * - app_foc_debug.c      : DebugLink / 调参 / Telemetry / FastRing
 * - app_foc_itTimer.c    : 高频控制中断入口与调度
 * - app_foc_test.c       : 测试 / 标定 / 方向检查等辅助逻辑
 *
 * 为什么需要这个文件：
 * 1. FOC 已经拆成多个编译单元，很多内部状态和底层对象需要跨文件共享
 * 2. 这些内容又不应该暴露到 app_foc.h 对外公共接口里
 * 3. 集中放在这里，能避免在各个 .c 文件里散落大量 extern 声明
 *
 * 放在这里的内容应满足：
 * - 仅供 app_foc 模块内部使用
 * - 不希望 main / debug_link / 其他应用层模块直接依赖
 *
 * 不应放在这里的内容：
 * - 对外公共 API
 * - 与 FOC 无关的跨模块通用定义
 * - 本来就只在单个 .c 文件内部使用的局部静态对象
 */

/*
 * Current-loop default tuning constants.
 *
 * 这些宏主要给 FOC 内部多个子模块共享，例如：
 * - app_foc.c            : 初始化 PID / 控制算法
 * - app_foc_debug.c      : 在线调参时的回退默认值
 *
 * 因为它们属于“FOC 内部实现常量”，而不是对外配置接口，所以放在
 * internal 头里而不是 app_foc.h。
 */
#define APP_CURRENT_I_LIMIT             (5.0f)
#define APP_CURRENT_I_ERR_MIN           (0.05f)
#define APP_CURRENT_OUT_LIMIT           (10.963f)
#define APP_CURRENT_DEBUG_PRINT_PERIOD_MS (100U)

/*
 * Internal feature gates.
 *
 * 这些开关影响 FOC 模块内部行为和状态标志生成。
 * 这里保留默认值，允许在更高层编译选项里覆盖。
 */
#ifndef APP_SPEED_LOOP_ENABLE
#define APP_SPEED_LOOP_ENABLE           (1U)
#endif

#ifndef APP_CURRENT_LOOP_ENABLE
#define APP_CURRENT_LOOP_ENABLE         (1U)
#endif

/*
 * FOC core runtime state.
 *
 * 这组变量描述“整个 FOC 子系统当前是否已经准备好、是否在运行”。
 * 它们会被多个子模块共同读取：
 * - app_foc.c        负责写入主状态
 * - app_foc_itTimer.c 读取/更新中断相关状态
 * - app_foc_debug.c  打包给上位机做状态展示
 */
extern volatile uint8_t g_foc_stack_init_ready;
extern volatile uint8_t g_foc_system_enabled;
extern uint8_t g_bus_voltage_valid;
extern float g_bus_voltage_filtered;
extern volatile uint8_t g_foc_control_it_enabled;
extern volatile uint32_t g_foc_loop_count;
extern volatile uint32_t g_foc_last_loop_tick_ms;

/*
 * Bus-voltage debug snapshot.
 *
 * 保留最近一次母线采样的原始/换算值，主要供调试打印和 DebugLink
 * Telemetry 使用。业务控制逻辑通常不直接消费这个结构体。
 */
extern volatile BusVoltageDebug_t g_bus_voltage_debug;

/*
 * Underlying motor-stack objects.
 *
 * 这些对象是 FOC 模块内部真正驱动硬件的底层实例：
 * - Motor_t         : 电机对象
 * - Driver_t        : 驱动器对象
 * - Sensor_t        : 位置传感器对象
 * - CurrentSense_t  : 电流采样对象
 *
 * 多个子模块都需要访问它们，因此统一从 internal 头共享。
 */
extern Motor_t g_motor1;
extern Motor_t g_motor2;
extern Driver_t *g_driver1;
extern Driver_t *g_driver2;
extern Sensor_t g_sensor1;
extern Sensor_t g_sensor2;
extern CurrentSense_t g_current_sense1;
extern CurrentSense_t g_current_sense2;

/*
 * Per-motor current-loop debug snapshots.
 *
 * 记录左右电机当前环最近一次关键量，用于：
 * - 实时 stream
 * - FastRing/FastCap 数据抓取
 * - 调试打印
 *
 * 这些属于内部可观测状态，不应该直接暴露给外部模块写入。
 */
extern volatile CurrentLoopDebugSnapshot_t g_current_loop_debug1;
extern volatile CurrentLoopDebugSnapshot_t g_current_loop_debug2;

/*
 * Current-loop runtime tuning selectors.
 *
 * g_current_pid_mode:
 *   选择当前上位机调参作用在哪一类 PID 上。
 *   例如 FF / PI 两套参数共用同一组读写入口时，需要一个模式位区分。
 *
 * g_current_i_sep_ratio_ff / g_current_i_sep_ratio_pure:
 *   当前环不同控制模式对应的积分分离比例。
 *   debug 调参和 PID 重建时都可能读取这些值。
 */
extern volatile uint8_t g_current_pid_mode;
extern volatile float g_current_i_sep_ratio_ff;
extern volatile float g_current_i_sep_ratio_pure;

/*
 * Per-motor auxiliary status.
 *
 * 这组量目前仍由多个 FOC 子文件共同使用，因此暂时保留在 internal 头：
 * - g_current_i_unload_limit_ticks1/2 :
 *     单侧电流环在某些工况下的卸载/限幅计数器
 * - g_speed_fault1/2 :
 *     单侧速度环故障/异常状态量，debug 状态打包时会用到
 *
 * 后续如果这些状态被完整收进 App_FOCMotorControl_t，再考虑从这里移除。
 */
extern uint8_t g_current_i_unload_limit_ticks1;
extern uint8_t g_current_i_unload_limit_ticks2;
extern float g_speed_fault1;
extern float g_speed_fault2;

/*
 * Internal helpers shared across FOC submodules.
 *
 * App_PIDResetRuntime():
 *   仅清 PID 运行时状态，不改参数，给重配置/复位逻辑复用。
 *
 * App_FastRingPushDual():
 *   将当前左右电机调试样本压入双侧同步 FastRing。
 *
 * App_FOC_BusInit():
 *   母线采样模块初始化，由主 FOC 初始化流程调用。
 */
void App_PIDResetRuntime(PID_t *pid);
void App_FastRingPushDual(void);
uint8_t App_FOC_BusInit(void);

#endif
