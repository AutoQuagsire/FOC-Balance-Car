#ifndef APP_ATTITUDE_INTERNAL_H
#define APP_ATTITUDE_INTERNAL_H

#include "app_attitude.h"
#include "app_attitude_control.h"
#include "PID.h"

/*
 * app_attitude 模块内部共享头。
 *
 * 用途：
 * 1. 让 app_attitude.c 与 app_attitude_debug.c 共享内部状态声明
 * 2. 避免把这些实现细节暴露到对外公共头 app_attitude.h
 *
 * 放在这里的内容应当满足：
 * - 只服务于 app_attitude 模块内部多个 .c 文件之间共享
 * - 不希望被 main/debug_link/其他应用层模块直接依赖
 *
 * 不应放在这里的内容：
 * - 对外公共 API
 * - 与 app_attitude 无关的跨模块通用定义
 */

/* Speed/attitude loop runtime state shared by core and debug helpers. */
extern volatile App_AttitudeTelemetry_t g_attitude_telemetry;
extern App_AttitudeControl_t g_attitude_control;

/*
 * 姿态模块内部参数同步入口。
 *
 * reset_runtime = 1:
 * - 装载姿态/速度环默认参数
 * - 同步速度环 PID 运行约束
 * - 清零速度环 PID 运行状态
 *
 * reset_runtime = 0:
 * - 保留当前调参结果
 * - 仅同步速度环 PID 运行约束
 */
void App_Attitude_ApplyPidParams(uint8_t reset_runtime);

#endif
