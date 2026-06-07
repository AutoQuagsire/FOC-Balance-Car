#ifndef DEBUG_LINK_H
#define DEBUG_LINK_H

#include <stdint.h>

#define DL_RX_BUF_SIZE   256U
#define DL_TX_BUF_SIZE   256U

/* 设备信息 */
#define DL_DEVICE_TYPE_BL_PRO_RET6   0x01U
#define DL_FW_MAJOR   0
#define DL_FW_MINOR   1
#define DL_FW_PATCH   0

/* 能力标志 */
#define DL_CAP_STATUS_STREAM   (1U << 0)
#define DL_CAP_PARAM_READ      (1U << 2)
#define DL_CAP_PARAM_WRITE     (1U << 3)
#define DL_CAP_PARAM_SAVE      (1U << 4)
#define DL_CAP_POWER_STAGE_CONTROL  (1U << 5)
#define DL_CAP_ATTITUDE_CONTROL     (1U << 6)
#define DL_CAP_FAST_RING            (1U << 7)

/* STATUS_STREAM 状态快照结构体 — 24 字节 */
typedef struct
{
    uint32_t tick_ms;
    int16_t  pitch_target_deg_x100;
    int16_t  speed_p_term_deg_x100;
    int16_t  speed_i_term_deg_x100;
    int16_t  pitch_meas_deg_x100;
    int16_t  pitch_rate_dps_x100;
    int16_t  speed_target_radps_x100;
    int16_t  speed_meas_radps_x100;
    int16_t  attitude_p_term_ma;
    int16_t  attitude_d_term_ma;
    int16_t  iq_cmd_ma;
    int16_t  iq_cmd_clamped_ma;
    int16_t  speed_output_limit_deg_x100;
    int16_t  attitude_output_limit_ma;
    int16_t  iq_l_x1000;
    int16_t  iq_r_x1000;
    int16_t  uq_l_mv;
    int16_t  uq_r_mv;
    uint16_t bus_mv;
    uint16_t fault_flags;
    int16_t  speed_raw_radps_x100;
} DebugLink_StatusSnapshot_t;

void DebugLink_Init(void);
void DebugLink_Process(void);
void DebugLink_UpdateStatusSnapshot(const DebugLink_StatusSnapshot_t *status);

#endif /* DEBUG_LINK_H */
