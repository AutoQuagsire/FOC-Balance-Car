#include "debug_link_app_codec.h"

#include "debug_link_app_adapter.h"
#include "debug_link_protocol.h"

#include <stddef.h>

static uint16_t DebugLinkCodec_BuildFastRingStatusLikePayload(
    uint8_t op,
    uint8_t take_snapshot,
    uint8_t *payload,
    uint16_t payload_size)
{
    DebugLinkApp_FastRingStatus_t status;

    if ((payload == NULL) || (payload_size < DEBUG_LINK_CODEC_FASTRING_STATUS_PAYLOAD_SIZE)) {
        return 0U;
    }

    if (take_snapshot != 0U) {
        DebugLinkApp_SnapshotFastRing(&status);
    } else {
        DebugLinkApp_GetFastRingStatus(&status);
    }

    payload[0] = op;
    DL_WriteU16LE(&payload[1], status.count);
    DL_WriteU16LE(&payload[3], status.capacity);
    DL_WriteU16LE(&payload[5], status.head);
    DL_WriteU32LE(&payload[7], status.write_seq);

    return DEBUG_LINK_CODEC_FASTRING_STATUS_PAYLOAD_SIZE;
}

uint8_t DebugLinkCodec_ClampFastRingChunkSamples(uint8_t requested_samples)
{
    if (requested_samples > DEBUG_LINK_CODEC_FASTRING_MAX_SAMPLES_PER_CHUNK) {
        return DEBUG_LINK_CODEC_FASTRING_MAX_SAMPLES_PER_CHUNK;
    }
    return requested_samples;
}

uint16_t DebugLinkCodec_BuildFastRingStatusPayload(uint8_t op,
                                                   uint8_t *payload,
                                                   uint16_t payload_size)
{
    return DebugLinkCodec_BuildFastRingStatusLikePayload(op, 0U, payload, payload_size);
}

uint16_t DebugLinkCodec_BuildFastRingSnapshotPayload(uint8_t op,
                                                     uint8_t *payload,
                                                     uint16_t payload_size)
{
    return DebugLinkCodec_BuildFastRingStatusLikePayload(op, 1U, payload, payload_size);
}

uint16_t DebugLinkCodec_BuildFastRingChunkPayload(uint8_t op,
                                                  uint32_t snapshot_write_seq,
                                                  uint16_t start_idx,
                                                  uint8_t max_samples,
                                                  uint8_t *payload,
                                                  uint16_t payload_size)
{
    DebugLinkApp_FastRingStatus_t status;
    DebugLinkApp_FastRingSample_t chunk[DEBUG_LINK_CODEC_FASTRING_MAX_SAMPLES_PER_CHUNK];
    uint16_t sample_count;
    uint16_t total_size;
    uint16_t i;

    if ((payload == NULL) || (payload_size < DEBUG_LINK_CODEC_FASTRING_CHUNK_HEADER_SIZE)) {
        return 0U;
    }

    max_samples = DebugLinkCodec_ClampFastRingChunkSamples(max_samples);

    DebugLinkApp_GetFastRingSnapshotStatus(&status);
    sample_count = DebugLinkApp_CopyFastRingSnapshotChunk(snapshot_write_seq,
                                                          start_idx,
                                                          max_samples,
                                                          chunk);

    total_size = (uint16_t)(DEBUG_LINK_CODEC_FASTRING_CHUNK_HEADER_SIZE +
                            sample_count * DEBUG_LINK_CODEC_FASTRING_CHUNK_SAMPLE_SIZE);
    if (payload_size < total_size) {
        return 0U;
    }

    payload[0] = op;
    DL_WriteU16LE(&payload[1], status.count);
    DL_WriteU16LE(&payload[3], status.capacity);
    DL_WriteU32LE(&payload[5], status.write_seq);
    DL_WriteU16LE(&payload[9], start_idx);
    payload[11] = (uint8_t)sample_count;

    for (i = 0U; i < sample_count; i++) {
        uint8_t *p = &payload[DEBUG_LINK_CODEC_FASTRING_CHUNK_HEADER_SIZE +
                              i * DEBUG_LINK_CODEC_FASTRING_CHUNK_SAMPLE_SIZE];

        DL_WriteU16LE(&p[0], (uint16_t)chunk[i].target_iq_l_ma);
        DL_WriteU16LE(&p[2], (uint16_t)chunk[i].iq_ref_l_ma);
        DL_WriteU16LE(&p[4], (uint16_t)chunk[i].filtered_iq_l_ma);
        DL_WriteU16LE(&p[6], (uint16_t)chunk[i].raw_iq_l_ma);
        DL_WriteU16LE(&p[8], (uint16_t)chunk[i].uq_final_l_mv);
        DL_WriteU16LE(&p[10], (uint16_t)chunk[i].target_iq_r_ma);
        DL_WriteU16LE(&p[12], (uint16_t)chunk[i].iq_ref_r_ma);
        DL_WriteU16LE(&p[14], (uint16_t)chunk[i].filtered_iq_r_ma);
        DL_WriteU16LE(&p[16], (uint16_t)chunk[i].raw_iq_r_ma);
        DL_WriteU16LE(&p[18], (uint16_t)chunk[i].uq_final_r_mv);
        DL_WriteU16LE(&p[20], chunk[i].bus_mv);
        DL_WriteU16LE(&p[22], chunk[i].sample_idx);
        DL_WriteU16LE(&p[24], chunk[i].status_flags);
    }

    return total_size;
}
