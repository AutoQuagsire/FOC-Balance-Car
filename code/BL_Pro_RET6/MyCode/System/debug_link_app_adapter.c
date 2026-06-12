#include "debug_link_app_adapter.h"

#include "app_attitude.h"
#include "app_foc.h"
#include "app_foc_debug.h"

#include <string.h>

#define DL_APP_PARAM_CURRENT_KP            0x01U
#define DL_APP_PARAM_CURRENT_KI            0x02U
#define DL_APP_PARAM_CURRENT_KD            0x03U
#define DL_APP_PARAM_CURRENT_ILIM          0x04U
#define DL_APP_PARAM_CURRENT_OUTPUT_LIMIT  0x05U
#define DL_APP_PARAM_CURRENT_I_ERR_MIN     0x06U
#define DL_APP_PARAM_CURRENT_I_SEP_RATIO   0x07U
#define DL_APP_PARAM_CURRENT_MODE          0x08U
#define DL_APP_PARAM_SPEED_KP              0x10U
#define DL_APP_PARAM_SPEED_KI              0x11U
#define DL_APP_PARAM_SPEED_PITCH_LIMIT     0x12U
#define DL_APP_PARAM_SPEED_UNWIND_GAIN     0x13U
#define DL_APP_PARAM_ATTITUDE_KP           0x20U
#define DL_APP_PARAM_ATTITUDE_KD           0x21U
#define DL_APP_PARAM_ATTITUDE_IQ_LIMIT     0x22U
#define DL_APP_PARAM_ATTITUDE_SHUTDOWN_RAD 0x23U

static void DebugLinkApp_WriteFloat(float value, uint8_t raw_bytes[4])
{
    memcpy(raw_bytes, &value, sizeof(value));
}

static float DebugLinkApp_ReadFloat(const uint8_t raw_bytes[4])
{
    float value;
    memcpy(&value, raw_bytes, sizeof(value));
    return value;
}

uint8_t DebugLinkApp_SetPowerStageEnabled(uint8_t enable)
{
    if (enable == 0U) {
        (void)App_Attitude_SetControlEnabled(0U);
    }

    return (App_FOC_SetSystemEnabled(enable) != 0U) ?
        DEBUG_LINK_APP_RESULT_OK : DEBUG_LINK_APP_RESULT_BUSY;
}

uint8_t DebugLinkApp_SetAttitudeControlEnabled(uint8_t enable)
{
    return (App_Attitude_SetControlEnabled(enable) != 0U) ?
        DEBUG_LINK_APP_RESULT_OK : DEBUG_LINK_APP_RESULT_BUSY;
}

uint8_t DebugLinkApp_GetParam(uint8_t param_id,
                              uint8_t *data_type,
                              uint8_t raw_bytes[4])
{
    float value_f = 0.0f;
    uint8_t value_u8 = 0U;

    if ((data_type == NULL) || (raw_bytes == NULL)) {
        return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
    }

    switch (param_id)
    {
    case DL_APP_PARAM_CURRENT_KP:
    case DL_APP_PARAM_CURRENT_KI:
    case DL_APP_PARAM_CURRENT_KD:
    case DL_APP_PARAM_CURRENT_ILIM:
    {
        float kp;
        float ki;
        float kd;
        float ilim;

        App_CurrentPID_GetSame(&kp, &ki, &kd, &ilim);
        if (param_id == DL_APP_PARAM_CURRENT_KP) {
            value_f = kp;
        } else if (param_id == DL_APP_PARAM_CURRENT_KI) {
            value_f = ki;
        } else if (param_id == DL_APP_PARAM_CURRENT_KD) {
            value_f = kd;
        } else {
            value_f = ilim;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;
    }

    case DL_APP_PARAM_CURRENT_OUTPUT_LIMIT:
        if (App_CurrentPID_GetOutputLimit(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_CURRENT_I_ERR_MIN:
        if (App_CurrentPID_GetIErrMin(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_CURRENT_I_SEP_RATIO:
        if (App_CurrentPID_GetISepRatio(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_CURRENT_MODE:
        value_u8 = App_CurrentPID_GetMode();
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_UINT8;
        raw_bytes[0] = value_u8;
        raw_bytes[1] = 0U;
        raw_bytes[2] = 0U;
        raw_bytes[3] = 0U;
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_KP:
        if (App_Attitude_GetSpeedKp(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_KI:
        if (App_Attitude_GetSpeedKi(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_PITCH_LIMIT:
        if (App_Attitude_GetSpeedPitchLimitRad(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_UNWIND_GAIN:
        if (App_Attitude_GetSpeedUnwindGain(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_KP:
        if (App_Attitude_GetAttitudeKp(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_KD:
        if (App_Attitude_GetAttitudeKd(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_IQ_LIMIT:
        if (App_Attitude_GetAttitudeIqLimit(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_SHUTDOWN_RAD:
        if (App_Attitude_GetAttitudeShutdownRad(&value_f) == 0U) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
        }
        *data_type = DEBUG_LINK_APP_PARAM_TYPE_FLOAT;
        DebugLinkApp_WriteFloat(value_f, raw_bytes);
        return DEBUG_LINK_APP_RESULT_OK;

    default:
        return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
    }
}

uint8_t DebugLinkApp_SetParam(uint8_t param_id,
                              uint8_t data_type,
                              const uint8_t raw_bytes[4])
{
    float value_f;
    uint8_t value_u8;

    if (raw_bytes == NULL) {
        return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
    }

    value_f = DebugLinkApp_ReadFloat(raw_bytes);
    value_u8 = raw_bytes[0];

    switch (param_id)
    {
    case DL_APP_PARAM_CURRENT_KP:
    case DL_APP_PARAM_CURRENT_KI:
    case DL_APP_PARAM_CURRENT_KD:
    case DL_APP_PARAM_CURRENT_ILIM:
    {
        float kp;
        float ki;
        float kd;
        float ilim;

        if (data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }

        App_CurrentPID_GetSame(&kp, &ki, &kd, &ilim);
        if (param_id == DL_APP_PARAM_CURRENT_KP) {
            kp = value_f;
        } else if (param_id == DL_APP_PARAM_CURRENT_KI) {
            ki = value_f;
        } else if (param_id == DL_APP_PARAM_CURRENT_KD) {
            kd = value_f;
        } else {
            ilim = value_f;
        }
        App_CurrentPID_SetSame(kp, ki, kd, ilim);
        return DEBUG_LINK_APP_RESULT_OK;
    }

    case DL_APP_PARAM_CURRENT_OUTPUT_LIMIT:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_CurrentPID_SetOutputLimit(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_CURRENT_I_ERR_MIN:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_CurrentPID_SetIErrMin(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_CURRENT_I_SEP_RATIO:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_CurrentPID_SetISepRatio(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_CURRENT_MODE:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_UINT8) ||
            (App_CurrentPID_SetMode(value_u8) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_KP:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetSpeedKp(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_KI:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetSpeedKi(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_PITCH_LIMIT:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetSpeedPitchLimitRad(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_SPEED_UNWIND_GAIN:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetSpeedUnwindGain(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_KP:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetAttitudeKp(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_KD:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetAttitudeKd(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_IQ_LIMIT:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetAttitudeIqLimit(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    case DL_APP_PARAM_ATTITUDE_SHUTDOWN_RAD:
        if ((data_type != DEBUG_LINK_APP_PARAM_TYPE_FLOAT) ||
            (App_Attitude_SetAttitudeShutdownRad(value_f) == 0U)) {
            return DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE;
        }
        return DEBUG_LINK_APP_RESULT_OK;

    default:
        return DEBUG_LINK_APP_RESULT_BAD_PARAM_ID;
    }
}

void DebugLinkApp_GetFastRingStatus(DebugLinkApp_FastRingStatus_t *status)
{
    if (status == NULL) {
        return;
    }
    App_GetFastRingStatus(&status->count,
                          &status->capacity,
                          &status->head,
                          &status->write_seq);
}

void DebugLinkApp_SnapshotFastRing(DebugLinkApp_FastRingStatus_t *status)
{
    if (status == NULL) {
        return;
    }
    status->head = 0U;
    App_SnapshotFastRing(&status->count,
                         &status->capacity,
                         &status->write_seq);
}

void DebugLinkApp_GetFastRingSnapshotStatus(DebugLinkApp_FastRingStatus_t *status)
{
    if (status == NULL) {
        return;
    }
    status->head = 0U;
    App_GetFastRingSnapshotStatus(&status->count,
                                  &status->capacity,
                                  &status->write_seq);
}

uint16_t DebugLinkApp_CopyFastRingSnapshotChunk(
    uint32_t snapshot_write_seq,
    uint16_t start_idx,
    uint8_t max_samples,
    DebugLinkApp_FastRingSample_t *out)
{
    FastRingSample_t app_chunk[8U];
    uint16_t copied;
    uint16_t i;

    if (out == NULL) {
        return 0U;
    }
    if (max_samples > 8U) {
        max_samples = 8U;
    }

    copied = App_CopyFastRingSnapshotChunk(snapshot_write_seq,
                                          start_idx,
                                          max_samples,
                                          app_chunk);
    for (i = 0U; i < copied; i++) {
        out[i].target_iq_l_ma = app_chunk[i].target_iq_l_ma;
        out[i].iq_ref_l_ma = app_chunk[i].iq_ref_l_ma;
        out[i].filtered_iq_l_ma = app_chunk[i].filtered_iq_l_ma;
        out[i].raw_iq_l_ma = app_chunk[i].raw_iq_l_ma;
        out[i].uq_final_l_mv = app_chunk[i].uq_final_l_mv;
        out[i].target_iq_r_ma = app_chunk[i].target_iq_r_ma;
        out[i].iq_ref_r_ma = app_chunk[i].iq_ref_r_ma;
        out[i].filtered_iq_r_ma = app_chunk[i].filtered_iq_r_ma;
        out[i].raw_iq_r_ma = app_chunk[i].raw_iq_r_ma;
        out[i].uq_final_r_mv = app_chunk[i].uq_final_r_mv;
        out[i].bus_mv = app_chunk[i].bus_mv;
        out[i].sample_idx = app_chunk[i].sample_idx;
        out[i].status_flags = app_chunk[i].status_flags;
    }
    return copied;
}
