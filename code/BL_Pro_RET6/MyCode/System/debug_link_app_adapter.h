#ifndef DEBUG_LINK_APP_ADAPTER_H
#define DEBUG_LINK_APP_ADAPTER_H

#include <stdint.h>

#define DEBUG_LINK_APP_PARAM_TYPE_FLOAT 0x01U
#define DEBUG_LINK_APP_PARAM_TYPE_UINT8 0x02U

#define DEBUG_LINK_APP_RESULT_OK              0U
#define DEBUG_LINK_APP_RESULT_BAD_PARAM_ID    1U
#define DEBUG_LINK_APP_RESULT_BAD_PARAM_VALUE 2U
#define DEBUG_LINK_APP_RESULT_BUSY            3U

typedef struct {
    uint16_t count;
    uint16_t capacity;
    uint16_t head;
    uint32_t write_seq;
} DebugLinkApp_FastRingStatus_t;

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
} DebugLinkApp_FastRingSample_t;

uint8_t DebugLinkApp_SetPowerStageEnabled(uint8_t enable);
uint8_t DebugLinkApp_SetAttitudeControlEnabled(uint8_t enable);

uint8_t DebugLinkApp_GetParam(uint8_t param_id,
                              uint8_t *data_type,
                              uint8_t raw_bytes[4]);
uint8_t DebugLinkApp_SetParam(uint8_t param_id,
                              uint8_t data_type,
                              const uint8_t raw_bytes[4]);

void DebugLinkApp_GetFastRingStatus(DebugLinkApp_FastRingStatus_t *status);
void DebugLinkApp_SnapshotFastRing(DebugLinkApp_FastRingStatus_t *status);
void DebugLinkApp_GetFastRingSnapshotStatus(DebugLinkApp_FastRingStatus_t *status);
uint16_t DebugLinkApp_CopyFastRingSnapshotChunk(
    uint32_t snapshot_write_seq,
    uint16_t start_idx,
    uint8_t max_samples,
    DebugLinkApp_FastRingSample_t *out);

#endif /* DEBUG_LINK_APP_ADAPTER_H */
