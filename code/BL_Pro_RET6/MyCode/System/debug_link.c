#include "debug_link.h"
#include "debug_link_app_adapter.h"
#include "debug_link_app_codec.h"
#include "debug_link_protocol.h"
#include "usart.h"

#include <stddef.h>
#include <string.h>

static volatile uint32_t s_rx_frame_count = 0U;
static volatile uint32_t s_rx_error_count = 0U;
static volatile uint32_t s_tx_frame_count = 0U;
static volatile uint32_t s_nack_count = 0U;

static DL_ProtocolParser_t s_parser;
static volatile DL_Frame_t s_rx_frame;
static volatile uint8_t s_frame_ready = 0U;
static uint8_t s_rx_dma_buf[DL_RX_BUF_SIZE];
static uint16_t s_rx_dma_read_idx = 0U;
static uint8_t s_tx_buf[DL_TX_BUF_SIZE];

static uint8_t s_stream_enabled = 0U;
static uint16_t s_stream_rate_hz = 100U;
static uint32_t s_stream_period_ms = 10U;
static uint32_t s_last_stream_tick_ms = 0U;

static volatile DebugLink_StatusSnapshot_t s_status;

static void DebugLink_ResetRxState(void)
{
    (void)HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
    s_rx_dma_read_idx = 0U;
}

static void DebugLink_StartRxDma(void)
{
    if (HAL_UART_Receive_DMA(&huart1, s_rx_dma_buf, sizeof(s_rx_dma_buf)) == HAL_OK) {
        if (huart1.hdmarx != NULL) {
            /*
             * The DMA runs as a circular byte sink. We poll the write pointer
             * from the main loop, so half/full-transfer IRQs are unnecessary.
             */
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
        }
    }
}

static void DebugLink_ServiceRxDma(void)
{
    uint16_t write_idx;

    if (huart1.hdmarx == NULL) {
        return;
    }

    write_idx = (uint16_t)(sizeof(s_rx_dma_buf) - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (write_idx >= sizeof(s_rx_dma_buf)) {
        write_idx = 0U;
    }

    while (s_rx_dma_read_idx != write_idx) {
        DL_Frame_t tmp;
        DL_ParseResult_t result;
        uint8_t byte = s_rx_dma_buf[s_rx_dma_read_idx];

        s_rx_dma_read_idx++;
        if (s_rx_dma_read_idx >= sizeof(s_rx_dma_buf)) {
            s_rx_dma_read_idx = 0U;
        }

        result = DL_Protocol_InputByte(&s_parser, byte, &tmp);
        if (result == DL_PARSE_FRAME_OK) {
            s_rx_frame_count++;
            s_rx_frame.version = tmp.version;
            s_rx_frame.msg_type = tmp.msg_type;
            s_rx_frame.seq = tmp.seq;
            s_rx_frame.payload_len = tmp.payload_len;
            if (tmp.payload_len > 0U) {
                memcpy((void *)s_rx_frame.payload, tmp.payload, tmp.payload_len);
            }
            s_frame_ready = 1U;
        } else if (result == DL_PARSE_FRAME_ERROR) {
            s_rx_error_count++;
        }
    }
}

static uint8_t DebugLink_TryTransmit(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0U;
    }

    /*
     * DebugLink responses are short and are sent from the main loop. A short
     * blocking TX keeps bring-up simple and avoids a stuck TX busy state when
     * the host repeatedly opens and closes the COM port.
     */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 10U) != HAL_OK) {
        return 0U;
    }

    s_tx_frame_count++;
    return 1U;
}

static void DebugLink_SendAck(uint8_t req_msg, uint8_t status)
{
    uint8_t payload[2];
    uint16_t len;

    payload[0] = req_msg;
    payload[1] = status;

    len = DL_Protocol_BuildFrame(DL_MSG_ACK, 0U, payload, sizeof(payload),
                                 s_tx_buf, sizeof(s_tx_buf));
    if (len > 0U) {
        (void)DebugLink_TryTransmit(s_tx_buf, len);
    }
}

static void DebugLink_SendNack(uint8_t req_msg, uint8_t reason)
{
    uint8_t payload[2];
    uint16_t len;

    payload[0] = req_msg;
    payload[1] = reason;

    len = DL_Protocol_BuildFrame(DL_MSG_NACK, 0U, payload, sizeof(payload),
                                 s_tx_buf, sizeof(s_tx_buf));
    if (len > 0U) {
        (void)DebugLink_TryTransmit(s_tx_buf, len);
        s_nack_count++;
    }
}

static void DebugLink_SendParamValueRsp(const DL_Frame_t *frame,
                                        uint8_t param_id,
                                        uint8_t data_type,
                                        const uint8_t raw_bytes[4])
{
    uint8_t payload[6];
    uint16_t len;

    payload[0] = param_id;
    payload[1] = data_type;
    memcpy(&payload[2], raw_bytes, 4U);

    len = DL_Protocol_BuildFrame(DL_MSG_PARAM_VALUE_RSP, frame->seq,
                                 payload, sizeof(payload),
                                 s_tx_buf, sizeof(s_tx_buf));
    if (len > 0U) {
        (void)DebugLink_TryTransmit(s_tx_buf, len);
    }
}

static void DebugLink_HandleGetDeviceInfo(const DL_Frame_t *frame)
{
    uint8_t payload[9];
    uint16_t len;

    if (frame->payload_len != 0U) {
        DebugLink_SendNack(DL_MSG_GET_DEVICE_INFO_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    payload[0] = DL_DEVICE_TYPE_BL_PRO_RET6;
    payload[1] = DL_PROTO_VERSION;
    payload[2] = DL_FW_MAJOR;
    payload[3] = DL_FW_MINOR;
    payload[4] = DL_FW_PATCH;
    DL_WriteU16LE(&payload[5], DL_CAP_STATUS_STREAM | DL_CAP_PARAM_READ | DL_CAP_PARAM_WRITE |
                               DL_CAP_POWER_STAGE_CONTROL | DL_CAP_ATTITUDE_CONTROL |
                               DL_CAP_FAST_RING);
    DL_WriteU16LE(&payload[7], DL_PROTO_MAX_PAYLOAD);

    len = DL_Protocol_BuildFrame(DL_MSG_DEVICE_INFO_RSP, frame->seq,
                                 payload, sizeof(payload),
                                 s_tx_buf, sizeof(s_tx_buf));
    if (len > 0U) {
        (void)DebugLink_TryTransmit(s_tx_buf, len);
    }
}

static void DebugLink_HandleStreamControl(const DL_Frame_t *frame)
{
    uint8_t enable;
    uint8_t stream_id;
    uint16_t rate_hz;

    if (frame->payload_len != 8U) {
        DebugLink_SendNack(DL_MSG_STREAM_CONTROL_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    enable = frame->payload[0];
    stream_id = frame->payload[1];
    rate_hz = DL_ReadU16LE(&frame->payload[2]);

    if (stream_id != 0U) {
        DebugLink_SendNack(DL_MSG_STREAM_CONTROL_REQ, DL_NACK_UNSUPPORTED);
        return;
    }

    if ((rate_hz == 0U) || (rate_hz > 200U)) {
        DebugLink_SendNack(DL_MSG_STREAM_CONTROL_REQ, DL_NACK_BAD_PARAM_VALUE);
        return;
    }

    s_stream_enabled = enable ? 1U : 0U;
    s_stream_rate_hz = rate_hz;
    s_stream_period_ms = 1000U / rate_hz;
    s_last_stream_tick_ms = HAL_GetTick();

    DebugLink_SendAck(DL_MSG_STREAM_CONTROL_REQ, DL_ACK_STATUS_OK);
}

static void DebugLink_HandlePowerStageControl(const DL_Frame_t *frame)
{
    uint8_t enable;

    if (frame->payload_len != 1U) {
        DebugLink_SendNack(DL_MSG_POWER_STAGE_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    enable = frame->payload[0];
    if (enable > 1U) {
        DebugLink_SendNack(DL_MSG_POWER_STAGE_REQ, DL_NACK_BAD_PARAM_VALUE);
        return;
    }

    if (DebugLinkApp_SetPowerStageEnabled(enable) != DEBUG_LINK_APP_RESULT_OK) {
        DebugLink_SendNack(DL_MSG_POWER_STAGE_REQ, DL_NACK_BUSY);
        return;
    }

    DebugLink_SendAck(DL_MSG_POWER_STAGE_REQ, DL_ACK_STATUS_OK);
}

static void DebugLink_HandleAttitudeControl(const DL_Frame_t *frame)
{
    uint8_t enable;

    if (frame->payload_len != 1U) {
        DebugLink_SendNack(DL_MSG_ATTITUDE_CONTROL_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    enable = frame->payload[0];
    if (enable > 1U) {
        DebugLink_SendNack(DL_MSG_ATTITUDE_CONTROL_REQ, DL_NACK_BAD_PARAM_VALUE);
        return;
    }

    if (DebugLinkApp_SetAttitudeControlEnabled(enable) != DEBUG_LINK_APP_RESULT_OK) {
        DebugLink_SendNack(DL_MSG_ATTITUDE_CONTROL_REQ, DL_NACK_BUSY);
        return;
    }

    DebugLink_SendAck(DL_MSG_ATTITUDE_CONTROL_REQ, DL_ACK_STATUS_OK);
}

static void DebugLink_HandleGetParam(const DL_Frame_t *frame)
{
    uint8_t param_id;
    uint8_t raw_bytes[4] = {0U, 0U, 0U, 0U};
    uint8_t data_type = 0U;
    uint8_t result;

    if (frame->payload_len != 1U) {
        DebugLink_SendNack(DL_MSG_GET_PARAM_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    param_id = frame->payload[0];
    result = DebugLinkApp_GetParam(param_id, &data_type, raw_bytes);
    if (result == DEBUG_LINK_APP_RESULT_BAD_PARAM_ID) {
        DebugLink_SendNack(DL_MSG_GET_PARAM_REQ, DL_NACK_BAD_PARAM_ID);
        return;
    }
    if (result != DEBUG_LINK_APP_RESULT_OK) {
        DebugLink_SendNack(DL_MSG_GET_PARAM_REQ, DL_NACK_BAD_PARAM_VALUE);
        return;
    }

    DebugLink_SendParamValueRsp(frame, param_id, data_type, raw_bytes);
}

static void DebugLink_HandleSetParam(const DL_Frame_t *frame)
{
    uint8_t param_id;
    uint8_t data_type;
    uint8_t result;

    if (frame->payload_len != 6U) {
        DebugLink_SendNack(DL_MSG_SET_PARAM_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    param_id = frame->payload[0];
    data_type = frame->payload[1];
    result = DebugLinkApp_SetParam(param_id, data_type, &frame->payload[2]);
    if (result == DEBUG_LINK_APP_RESULT_BAD_PARAM_ID) {
        DebugLink_SendNack(DL_MSG_SET_PARAM_REQ, DL_NACK_BAD_PARAM_ID);
        return;
    }
    if (result != DEBUG_LINK_APP_RESULT_OK) {
        DebugLink_SendNack(DL_MSG_SET_PARAM_REQ, DL_NACK_BAD_PARAM_VALUE);
        return;
    }

    DebugLink_SendAck(DL_MSG_SET_PARAM_REQ, DL_ACK_STATUS_OK);
}

#define DL_FASTRING_OP_STATUS     0x01U
#define DL_FASTRING_OP_SNAPSHOT   0x02U
#define DL_FASTRING_OP_READ_CHUNK 0x03U

static void DebugLink_HandleFastRing(const DL_Frame_t *frame)
{
    uint8_t op;
    uint16_t len;

    if (frame->payload_len < 1U) {
        DebugLink_SendNack(DL_MSG_FAST_RING_REQ, DL_NACK_BAD_LENGTH);
        return;
    }

    op = frame->payload[0];

    switch (op)
    {
    case DL_FASTRING_OP_STATUS:
    {
        uint8_t payload[DEBUG_LINK_CODEC_FASTRING_STATUS_PAYLOAD_SIZE];

        len = DebugLinkCodec_BuildFastRingStatusPayload(DL_FASTRING_OP_STATUS,
                                                        payload,
                                                        sizeof(payload));

        if (len > 0U) {
            len = DL_Protocol_BuildFrame(DL_MSG_FAST_RING_DATA, frame->seq,
                                         payload, len,
                                         s_tx_buf, sizeof(s_tx_buf));
            if (len > 0U) {
                (void)DebugLink_TryTransmit(s_tx_buf, len);
            }
        }
        break;
    }

    case DL_FASTRING_OP_SNAPSHOT:
    {
        uint8_t payload[DEBUG_LINK_CODEC_FASTRING_STATUS_PAYLOAD_SIZE];

        len = DebugLinkCodec_BuildFastRingSnapshotPayload(DL_FASTRING_OP_SNAPSHOT,
                                                          payload,
                                                          sizeof(payload));

        if (len > 0U) {
            len = DL_Protocol_BuildFrame(DL_MSG_FAST_RING_DATA, frame->seq,
                                         payload, len,
                                         s_tx_buf, sizeof(s_tx_buf));
            if (len > 0U) {
                (void)DebugLink_TryTransmit(s_tx_buf, len);
            }
        }
        break;
    }

    case DL_FASTRING_OP_READ_CHUNK:
    {
        uint16_t start_idx;
        uint8_t max_samples;
        uint32_t snapshot_write_seq;
        uint8_t payload[DL_PROTO_MAX_PAYLOAD];

        if (frame->payload_len < 8U) {
            DebugLink_SendNack(DL_MSG_FAST_RING_REQ, DL_NACK_BAD_LENGTH);
            return;
        }

        snapshot_write_seq = DL_ReadU32LE(&frame->payload[1]);
        start_idx = DL_ReadU16LE(&frame->payload[5]);
        max_samples = DebugLinkCodec_ClampFastRingChunkSamples(frame->payload[7]);

        len = DebugLinkCodec_BuildFastRingChunkPayload(DL_FASTRING_OP_READ_CHUNK,
                                                       snapshot_write_seq,
                                                       start_idx,
                                                       max_samples,
                                                       payload,
                                                       sizeof(payload));

        if (len > 0U) {
            len = DL_Protocol_BuildFrame(DL_MSG_FAST_RING_DATA, frame->seq,
                                         payload,
                                         len,
                                         s_tx_buf, sizeof(s_tx_buf));
            if (len > 0U) {
                (void)DebugLink_TryTransmit(s_tx_buf, len);
            }
        }
        break;
    }

    default:
        DebugLink_SendNack(DL_MSG_FAST_RING_REQ, DL_NACK_UNSUPPORTED);
        break;
    }
}

static void DebugLink_SendStatusStream(void)
{
    uint8_t payload[44];
    uint16_t len;
    DebugLink_StatusSnapshot_t st;

    __disable_irq();
    st = s_status;
    __enable_irq();

    DL_WriteU32LE(&payload[0], st.tick_ms);
    DL_WriteU16LE(&payload[4], (uint16_t)st.pitch_target_deg_x100);
    DL_WriteU16LE(&payload[6], (uint16_t)st.speed_p_term_deg_x100);
    DL_WriteU16LE(&payload[8], (uint16_t)st.speed_i_term_deg_x100);
    DL_WriteU16LE(&payload[10], (uint16_t)st.pitch_meas_deg_x100);
    DL_WriteU16LE(&payload[12], (uint16_t)st.pitch_rate_dps_x100);
    DL_WriteU16LE(&payload[14], (uint16_t)st.speed_target_radps_x100);
    DL_WriteU16LE(&payload[16], (uint16_t)st.speed_meas_radps_x100);
    DL_WriteU16LE(&payload[18], (uint16_t)st.attitude_p_term_ma);
    DL_WriteU16LE(&payload[20], (uint16_t)st.attitude_d_term_ma);
    DL_WriteU16LE(&payload[22], (uint16_t)st.iq_cmd_ma);
    DL_WriteU16LE(&payload[24], (uint16_t)st.iq_cmd_clamped_ma);
    DL_WriteU16LE(&payload[26], (uint16_t)st.speed_output_limit_deg_x100);
    DL_WriteU16LE(&payload[28], (uint16_t)st.attitude_output_limit_ma);
    DL_WriteU16LE(&payload[30], (uint16_t)st.iq_l_x1000);
    DL_WriteU16LE(&payload[32], (uint16_t)st.iq_r_x1000);
    DL_WriteU16LE(&payload[34], (uint16_t)st.uq_l_mv);
    DL_WriteU16LE(&payload[36], (uint16_t)st.uq_r_mv);
    DL_WriteU16LE(&payload[38], st.bus_mv);
    DL_WriteU16LE(&payload[40], st.fault_flags);
    DL_WriteU16LE(&payload[42], (uint16_t)st.speed_raw_radps_x100);

    len = DL_Protocol_BuildFrame(DL_MSG_STATUS_STREAM, 0U,
                                 payload, sizeof(payload),
                                 s_tx_buf, sizeof(s_tx_buf));
    if (len > 0U) {
        (void)DebugLink_TryTransmit(s_tx_buf, len);
    }
}

static void DebugLink_ProcessRxFrame(void)
{
    DL_Frame_t frame;

    if (s_frame_ready == 0U) {
        return;
    }

    frame.version = s_rx_frame.version;
    frame.msg_type = s_rx_frame.msg_type;
    frame.seq = s_rx_frame.seq;
    frame.payload_len = s_rx_frame.payload_len;
    if (frame.payload_len > 0U) {
        memcpy(frame.payload, (const void *)s_rx_frame.payload, frame.payload_len);
    }

    s_frame_ready = 0U;

    switch (frame.msg_type)
    {
    case DL_MSG_PING_REQ:
        DebugLink_SendAck(DL_MSG_PING_REQ, DL_ACK_STATUS_OK);
        break;

    case DL_MSG_GET_DEVICE_INFO_REQ:
        DebugLink_HandleGetDeviceInfo(&frame);
        break;

    case DL_MSG_STREAM_CONTROL_REQ:
        DebugLink_HandleStreamControl(&frame);
        break;

    case DL_MSG_GET_PARAM_REQ:
        DebugLink_HandleGetParam(&frame);
        break;

    case DL_MSG_SET_PARAM_REQ:
        DebugLink_HandleSetParam(&frame);
        break;

    case DL_MSG_SAVE_PARAMS_REQ:
        DebugLink_SendNack(DL_MSG_SAVE_PARAMS_REQ, DL_NACK_UNSUPPORTED);
        break;

    case DL_MSG_POWER_STAGE_REQ:
        DebugLink_HandlePowerStageControl(&frame);
        break;

    case DL_MSG_ATTITUDE_CONTROL_REQ:
        DebugLink_HandleAttitudeControl(&frame);
        break;

    case DL_MSG_FAST_RING_REQ:
        DebugLink_HandleFastRing(&frame);
        break;

    default:
        DebugLink_SendNack(frame.msg_type, DL_NACK_UNSUPPORTED);
        break;
    }
}

void DebugLink_Init(void)
{
    DebugLink_ResetRxState();
    DL_Protocol_InitParser(&s_parser);
    s_frame_ready = 0U;
    s_rx_frame_count = 0U;
    s_rx_error_count = 0U;
    s_tx_frame_count = 0U;
    s_nack_count = 0U;
    s_stream_enabled = 0U;
    s_stream_rate_hz = 100U;
    s_stream_period_ms = 10U;
    s_last_stream_tick_ms = 0U;
    memset((void *)&s_status, 0, sizeof(s_status));
    memset(s_rx_dma_buf, 0, sizeof(s_rx_dma_buf));

    DebugLink_StartRxDma();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) {
        return;
    }

    s_rx_error_count++;

    DebugLink_ResetRxState();
    DL_Protocol_InitParser(&s_parser);
    DebugLink_StartRxDma();
}

void DebugLink_Process(void)
{
    DebugLink_ServiceRxDma();
    DebugLink_ProcessRxFrame();

    if (s_stream_enabled != 0U) {
        uint32_t now = HAL_GetTick();
        if ((now - s_last_stream_tick_ms) >= s_stream_period_ms) {
            s_last_stream_tick_ms += s_stream_period_ms;
            if ((now - s_last_stream_tick_ms) >= s_stream_period_ms) {
                s_last_stream_tick_ms = now;
            }
            DebugLink_SendStatusStream();
        }
    }
}

void DebugLink_UpdateStatusSnapshot(const DebugLink_StatusSnapshot_t *status)
{
    if (status == NULL) {
        return;
    }

    __disable_irq();
    s_status = *status;
    __enable_irq();
}
