#ifndef APP_FOC_H
#define APP_FOC_H

#include <stdint.h>

#define APP_FOC_STATUS_FLAG_SPEED_FAULT_L        (1U << 0)
#define APP_FOC_STATUS_FLAG_SPEED_FAULT_R        (1U << 1)
#define APP_FOC_STATUS_FLAG_STACK_READY          (1U << 8)
#define APP_FOC_STATUS_FLAG_CONTROL_IT_ENABLED   (1U << 9)
#define APP_FOC_STATUS_FLAG_BUS_VALID            (1U << 10)
#define APP_FOC_STATUS_FLAG_CURRENT_LOOP_ACTIVE  (1U << 11)
#define APP_FOC_STATUS_FLAG_SPEED_LOOP_ENABLED   (1U << 12)
#define APP_FOC_STATUS_FLAG_CURRENT_LOOP_ENABLED (1U << 13)
#define APP_FOC_STATUS_FLAG_POWER_STAGE_OFF      (1U << 14)
#define APP_FOC_STATUS_FLAG_ATTITUDE_CONTROL_ON  (1U << 15)

#define APP_FASTRING_SIZE (512U)

typedef struct {
    int16_t target_iq_l_ma;
    int16_t iq_ref_l_ma;
    int16_t filtered_iq_l_ma;
    int16_t raw_iq_l_ma;
    int16_t uq_final_l_mv;
    int16_t target_iq_r_ma;
    int16_t iq_ref_r_ma;
    int16_t filtered_iq_r_ma;
    int16_t raw_iq_r_ma;
    int16_t uq_final_r_mv;
    uint16_t bus_mv;
    uint16_t sample_idx;
    uint16_t status_flags;
} FastRingSample_t;

typedef struct
{
    float wheel_vel_left_radps;
    float wheel_vel_right_radps;
    float filtered_iq_left_a;
    float filtered_iq_right_a;
    float uq_left_v;
    float uq_right_v;
    float bus_voltage_v;
    uint16_t status_flags;
} App_FOCTelemetry_t;

uint8_t App_FOCStack_Init(void);
uint8_t App_StartupCalibrate(void);
void App_Loop(void);
uint8_t App_FOC_BusTelemetryInit(void);
void App_FOC_BusTelemetryService(void);
void App_CurrentSenseSignTest(void);
void App_SensorDirectionTest(void);
void App_FOCControlIT_Enable(void);
void App_LoopForIT(void);
void DebuginWhile(void);
void App_ResetFastRing(void);
void App_GetFastRingStatus(uint16_t *count,
                           uint16_t *capacity,
                           uint16_t *head,
                           uint32_t *write_seq);
void App_SnapshotFastRing(uint16_t *count,
                          uint16_t *capacity,
                          uint32_t *write_seq);
void App_GetFastRingSnapshotStatus(uint16_t *count,
                                   uint16_t *capacity,
                                   uint32_t *write_seq);
uint16_t App_CopyFastRingLatest(uint16_t start_idx,
                                uint8_t max_samples,
                                FastRingSample_t *out);
uint16_t App_CopyFastRingSnapshotChunk(uint32_t snapshot_write_seq,
                                       uint16_t start_idx,
                                       uint8_t max_samples,
                                       FastRingSample_t *out);
void App_ResetSpeedPIDs(void);
void App_ResetCurrentPIDs(void);
void App_FOC_SetIqTarget(float left_iq, float right_iq);
float App_FOC_GetAverageWheelSpeedRadps(void);
void App_FOC_GetTelemetry(App_FOCTelemetry_t *telemetry);
uint8_t App_FOC_SetPowerStageEnabled(uint8_t enable);
uint8_t App_FOC_IsPowerStageEnabled(void);
uint8_t App_FOC_SetDriverGateEnabled(uint8_t enable);
void App_CurrentPID_SetSame(float kp, float ki, float kd, float integral_limit);
void App_CurrentPID_GetSame(float *kp, float *ki, float *kd, float *integral_limit);
uint8_t App_CurrentPID_SetMode(uint8_t mode);
uint8_t App_CurrentPID_GetMode(void);
uint8_t App_CurrentPID_SetOutputLimit(float output_limit);
uint8_t App_CurrentPID_GetOutputLimit(float *output_limit);
uint8_t App_CurrentPID_SetIErrMin(float i_err_min);
uint8_t App_CurrentPID_GetIErrMin(float *i_err_min);
uint8_t App_CurrentPID_SetISepRatio(float i_sep_ratio);
uint8_t App_CurrentPID_GetISepRatio(float *i_sep_ratio);

extern volatile uint8_t g_current_pid_mode; /* 0=CurrentLoop_FFPI_V1, 1=Pure PI compare */

/* DebugLink 用的状态变量 */
extern float vel_windowed_f1;
extern float vel_windowed_f2;
extern float Left_FilteredIq;
extern float Right_FilteredIq;
extern float uq_cmd1;
extern float uq_cmd2;
extern float g_speed_fault1;
extern float g_speed_fault2;

float App_FOC_GetBusVoltageFiltered(void);

#endif
