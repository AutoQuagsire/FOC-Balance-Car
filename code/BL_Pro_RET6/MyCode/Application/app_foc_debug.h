#ifndef APP_FOC_DEBUG_H
#define APP_FOC_DEBUG_H

#include <stdint.h>

#define APP_FASTRING_SIZE (512U)

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

/*
 * Current-loop runtime tuning and diagnostics.
 *
 * This interface is used by USB debug / DebugLink parameter access and keeps
 * upper-computer tuning helpers out of the main app_foc.h control surface.
 */
void App_FOC_GetTelemetry(App_FOCTelemetry_t *telemetry);
void App_PrintCurrentLoopDebugIfDue(void);

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

void App_ResetCurrentPIDs(void);

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

#endif
