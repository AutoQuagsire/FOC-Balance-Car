#ifndef DEBUG_LINK_APP_CODEC_H
#define DEBUG_LINK_APP_CODEC_H

#include <stdint.h>

#define DEBUG_LINK_CODEC_FASTRING_STATUS_PAYLOAD_SIZE      11U
#define DEBUG_LINK_CODEC_FASTRING_CHUNK_HEADER_SIZE        12U
#define DEBUG_LINK_CODEC_FASTRING_CHUNK_SAMPLE_SIZE        26U
#define DEBUG_LINK_CODEC_FASTRING_MAX_SAMPLES_PER_CHUNK    8U

uint8_t DebugLinkCodec_ClampFastRingChunkSamples(uint8_t requested_samples);

uint16_t DebugLinkCodec_BuildFastRingStatusPayload(uint8_t op,
                                                   uint8_t *payload,
                                                   uint16_t payload_size);

uint16_t DebugLinkCodec_BuildFastRingSnapshotPayload(uint8_t op,
                                                     uint8_t *payload,
                                                     uint16_t payload_size);

uint16_t DebugLinkCodec_BuildFastRingChunkPayload(uint8_t op,
                                                  uint32_t snapshot_write_seq,
                                                  uint16_t start_idx,
                                                  uint8_t max_samples,
                                                  uint8_t *payload,
                                                  uint16_t payload_size);

#endif
