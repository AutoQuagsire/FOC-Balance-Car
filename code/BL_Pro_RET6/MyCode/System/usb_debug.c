/**
 ******************************************************************************
 * @file    usb_debug.c
 * @brief   USB 虚拟串口调试发送封装
 ******************************************************************************
 */

#include "usb_debug.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "PID.h"
#include "app_foc.h"
#include "INT.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

extern float Left_Target;

#define TUNE_TARGET        Left_Target
#define TUNE_SIDE_NAME     "CURRENT_LR"
#define TUNE_TARGET_NAME   "LEFT_SPEED"

/* PID参数接收范围（与参考firmware.cpp一致） */
#define PID_KP_MAX 100.0f
#define PID_KI_MAX 50.0f
#define PID_KD_MAX 50.0f
#define PID_ILIM_MAX 20.0f
#define PID_UPDATE_MOTOR_HOLDOFF_MS 1000U

static uint8_t pid_update_motor_holdoff_active = 0U;
static uint32_t pid_update_motor_reenable_tick = 0U;
static uint8_t pid_update_ack_pending = 0U;
static float pid_update_ack_kp = 0.0f;
static float pid_update_ack_ki = 0.0f;
static float pid_update_ack_kd = 0.0f;
static float pid_update_ack_ilim = 0.0f;

static void process_pid_update_motor_holdoff(void)
{
    if (!pid_update_motor_holdoff_active)
        return;

    if ((int32_t)(HAL_GetTick() - pid_update_motor_reenable_tick) >= 0)
    {
        if (App_FOC_SetDriverGateEnabled(1U) == 0U) {
            pid_update_motor_reenable_tick = HAL_GetTick() + 50U;
            USB_Debug_Printf("# WARN: driver gate re-enable deferred\r\n");
            return;
        }

        pid_update_motor_holdoff_active = 0U;
        USB_Debug_Printf("# Motor driver re-enabled after command update holdoff\r\n");
        App_ResetSpeedPIDs();
        App_ResetCurrentPIDs();

        if (pid_update_ack_pending)
        {
            USB_Debug_Printf("# Current PID(L/R) Updated: P=%.4f I=%.6f D=%.6f ILim=%.4f\r\n",
                             pid_update_ack_kp,
                             pid_update_ack_ki,
                             
                             pid_update_ack_kd,
                             pid_update_ack_ilim);
            USB_Debug_Printf("# PID update ack delayed by %lu ms\r\n",
                             (uint32_t)PID_UPDATE_MOTOR_HOLDOFF_MS);
            pid_update_ack_pending = 0U;
        }


    }
}

static uint8_t trigger_pid_update_motor_holdoff(void)
{
    if (App_FOC_SetDriverGateEnabled(0U) == 0U) {
        USB_Debug_Printf("# WARN: failed to disable driver gate for holdoff\r\n");
        return 0U;
    }

    pid_update_motor_holdoff_active = 1U;
    pid_update_motor_reenable_tick = HAL_GetTick() + PID_UPDATE_MOTOR_HOLDOFF_MS;
    return 1U;
}

static void apply_pid_and_ack(float kp, float ki, float kd, float ilim)
{
    float final_ilim = 0.0f;

    App_CurrentPID_GetSame(NULL, NULL, NULL, &final_ilim);

    if (kp < -PID_KP_MAX || kp > PID_KP_MAX ||
        ki < -PID_KI_MAX || ki > PID_KI_MAX ||
        kd < -PID_KD_MAX || kd > PID_KD_MAX)
    {
        USB_Debug_Printf("# ERROR: PID params rejected (range: -100<=Kp<=100, -50<=Ki<=50, -50<=Kd<=50)\r\n");
        return;
    }

    if (ilim >= 0.0f)
    {
        if (ilim <= 0.0f || ilim > PID_ILIM_MAX)
        {
            USB_Debug_Printf("# ERROR: ILim rejected (range: 0<ILim<=%.2f)\r\n", PID_ILIM_MAX);
            return;
        }
        final_ilim = ilim;
    }

    /* 支持可选更新积分限幅；并清状态避免突变 */
    App_CurrentPID_SetSame(kp, ki, kd, final_ilim);
    if (trigger_pid_update_motor_holdoff() == 0U) {
        USB_Debug_Printf("# WARN: PID applied, but driver-gate holdoff not entered\r\n");
        return;
    }
    pid_update_ack_kp = kp;
    pid_update_ack_ki = ki;
    pid_update_ack_kd = kd;
    pid_update_ack_ilim = final_ilim;
    pid_update_ack_pending = 1U;

    USB_Debug_Printf("# Motor driver disabled for %lu ms after PID update\r\n", (uint32_t)PID_UPDATE_MOTOR_HOLDOFF_MS);
}

static void apply_target_and_ack(float target)
{
    TUNE_TARGET = target;

    USB_Debug_Printf("# Target(%s) updated %.4f (PID state kept; use PID RESET to clear)\r\n",
                     TUNE_TARGET_NAME,
                     TUNE_TARGET);
}

static uint8_t parse_pid_values(const char *payload, float *kp, float *ki, float *kd, float *ilim)
{
    char tmp[64];
    uint32_t i;
    uint32_t j;
    char *p;
    char *endp;
    float vals[3];
    uint32_t cnt;

    if (payload == NULL || kp == NULL || ki == NULL || kd == NULL || ilim == NULL)
        return 0U;

    /*
     * 规范化输入：把标签字符与分隔符都转为空格，
     * 最终只保留数字/符号/指数相关字符，便于 strtof 逐个提取。
     */
    j = 0U;
    for (i = 0U; payload[i] != '\0' && j < (sizeof(tmp) - 1U); i++)
    {
        char c = payload[i];
        if ((c >= '0' && c <= '9') ||
            c == '+' || c == '-' || c == '.' ||
            c == 'e' || c == 'E')
        {
            tmp[j++] = c;
        }
        else
        {
            c = ' ';
            tmp[j++] = c;
        }
    }
    tmp[j] = '\0';

    p = tmp;
    cnt = 0U;
    while (*p != '\0' && cnt < 3U)
    {
        while (*p == ' ')
            p++;

        if (*p == '\0')
            break;

        vals[cnt] = strtof(p, &endp);
        if (endp == p)
        {
            /* 当前字符不是数字起点，继续向后找 */
            p++;
            continue;
        }

        cnt++;
        p = endp;
    }

    if (cnt == 3U)
    {
        *kp = vals[0];
        *ki = vals[1];
        *kd = vals[2];
        *ilim = -1.0f;

        /* 可选第4个数字作为积分限幅（支持 L/ILIM 标记后的数值） */
        while (*p == ' ')
            p++;
        if (*p != '\0')
        {
            float ilim_v = strtof(p, &endp);
            if (endp != p)
                *ilim = ilim_v;
        }
        return 1U;
    }

    return 0U;
}

static void handle_one_command(const char *cmd)
{
    uint32_t i;
    uint32_t j;
    float kp;
    float ki;
    float kd;
    float ilim;

    if (cmd == NULL || cmd[0] == '\0')
        return;


    /* ---------- PID参数协议 ----------
     * 1) SET P:1.5 I:0.2 D:0.05
     * 2) SET KP:1.5 KI:0.2 KD:0.05
     * 3) PID 1.5 0.2 0.05
     */
    if (strncmp(cmd, "SET ", 4) == 0)
    {
        if (parse_pid_values(cmd + 4, &kp, &ki, &kd, &ilim))
        {
            apply_pid_and_ack(kp, ki, kd, ilim);
            return;
        }

        USB_Debug_Printf("# ERROR: PID format invalid in SET payload: [%s]\r\n", cmd + 4);
        USB_Debug_Printf("# EXPECT: SET P:x I:y D:z | SET KP:x KI:y KD:z | SET x y z\r\n");
        return;
    }

    if (strncmp(cmd, "PID MODE", 8) == 0)
    {
        g_current_pid_mode = (g_current_pid_mode == 0U) ? 1U : 0U;
        USB_Debug_Printf("# Current PID mode switched to: %s\r\n",
                         (g_current_pid_mode == 0U) ? "CurrentLoop_FFPI_V1" : "Pure PI compare");
        return;
    }

    if (strncmp(cmd, "PID ", 4) == 0)
    {
        if (parse_pid_values(cmd + 4, &kp, &ki, &kd, &ilim))
        {
            apply_pid_and_ack(kp, ki, kd, ilim);
            return;
        }

        USB_Debug_Printf("# ERROR: PID format invalid payload: [%s]\r\n", cmd + 4);
        USB_Debug_Printf("# EXPECT: PID Kp Ki Kd | PID P:x I:y D:z\r\n");
        return;
    }

    if (strncmp(cmd, "STATUS", 6) == 0)
    {
        float cur_kp;
        float cur_ki;
        float cur_kd;
        float cur_ilim;
        float sched_ff = 0.0f;
        float sched_ilim = 0.0f;

        App_CurrentPID_GetSame(&cur_kp, &cur_ki, &cur_kd, &cur_ilim);
        CurrentLoop_GetScheduledParams(TUNE_TARGET, &sched_ff, &sched_ilim);
        USB_Debug_Printf("# STATUS(%s): Kp=%.4f Ki=%.6f Kd=%.6f ILim=%.4f FFMode=%u FFcoef=%.4f SchedILim=%.4f TargetIq=%.3f\r\n",
                         TUNE_SIDE_NAME,
                         cur_kp, cur_ki, cur_kd,
                         cur_ilim,
                         (unsigned)(g_current_pid_mode == 0U),
                         sched_ff,
                         sched_ilim,
                         TUNE_TARGET);
        return;
    }

    /* 检查格式 "KFEED:" */
    if (strncmp(cmd, "KFEED:", 6) == 0)
    {
        (void)strtof(cmd + 6, NULL);
        USB_Debug_Printf("# KFEED is deprecated in CurrentLoop_FFPI_V1; ff_coef is scheduled by |Iq_ref|\r\n");
        return;
    }

    if (strncmp(cmd, "PID RESET", 9) == 0)
    {
        App_ResetCurrentPIDs();
        App_ResetSpeedPIDs();
        USB_Debug_Printf("# PID runtime state reset\r\n");
        return;
    }

    /* 检查格式 "tgt:" */
    if (cmd[0] == 't' && cmd[1] == 'g' && cmd[2] == 't' && cmd[3] == ':')
    {
        /* 提取并转换数值 */
        char value_str[32];
        i = 4U;
        j = 0U;
        while (cmd[i] != '\0' && j < (sizeof(value_str) - 1U))
        {
            value_str[j++] = cmd[i];
            i++;
        }
        value_str[j] = '\0';

        apply_target_and_ack(strtof((const char *)value_str, NULL));
        return;
    }

    USB_Debug_Printf("Unknown cmd: %s\r\n", cmd);
    USB_Debug_Printf("Cmd list: tgt:<value> | SET/PID tunes current PID L/R same: SET P:x I:y D:z [L:l] | PID x y z [l] | STATUS | PID MODE | PID RESET\r\n");
}

/* 诊断计数：记录 CDC_Transmit_FS 返回 USBD_BUSY 的次数 */
static volatile uint32_t usb_debug_busy_count = 0;

/* USB 接收流缓冲（非阻塞，支持多包累积） */
static uint8_t usb_rx_stream[256];
static volatile uint32_t usb_rx_stream_len = 0;
static volatile uint8_t usb_rx_overflow = 0;

/* ── 内部发送（单次尝试，绝不阻塞主循环）────────────────────────── */
static void send_blocking(const uint8_t *buf, uint16_t len)
{
    uint32_t t0 = HAL_GetTick();
    int ret;

    if (buf == NULL || len == 0U)
        return;

    do
    {
        ret = CDC_Transmit_FS((uint8_t *)buf, len);
        if (ret == USBD_OK)
            return;

        if (ret == USBD_BUSY)
        {
            /* 记录 BUSY 次数，便于诊断 */
            usb_debug_busy_count++;
        }
        else
        {
            /* 其他错误直接退出，避免无意义重试 */
            return;
        }
    }
    while ((HAL_GetTick() - t0) < USB_DEBUG_TIMEOUT_MS);
}

/* ── 公开函数 ────────────────────────────────────────────────────── */

void USB_Debug_Send(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
        return;
    send_blocking(buf, len);
}

void USB_Debug_SendStr(const char *str)
{
    if (str == NULL)
        return;
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0)
        return;
    send_blocking((const uint8_t *)str, len);
}

void USB_Debug_Printf(const char *fmt, ...)
{
    static char buf[USB_DEBUG_BUF_SIZE];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
        send_blocking((uint8_t *)buf, (uint16_t)len);
}

uint32_t USB_Debug_GetBusyCount(void)
{
    return (uint32_t)usb_debug_busy_count;
}

/**
 * @brief  USB 接收回调（在 CDC_Receive_FS 中调用，仅记录数据）
 */
void USB_RxCallback(uint8_t* buf, uint32_t len)
{
    uint32_t i;

    if (buf == NULL || len == 0U)
        return;

    for (i = 0U; i < len; i++)
    {
        if (usb_rx_stream_len < (sizeof(usb_rx_stream) - 1U))
        {
            usb_rx_stream[usb_rx_stream_len++] = buf[i];
        }
        else
        {
            /* 溢出时丢弃后续输入，等待主循环处理并提示 */
            usb_rx_overflow = 1U;
            break;
        }
    }
}

/**
 * @brief  处理 USB 接收的命令（在主循环中调用，非阻塞）
 *         格式：tgt:数值\n
 */
void Process_USB_Command(void)
{
    char cmd[256];
    uint32_t i;
    uint32_t j;
    uint32_t len;
    uint8_t overflow;

    /* 非阻塞处理：PID改参后到时自动恢复电机驱动 */
    process_pid_update_motor_holdoff();

    if (usb_rx_stream_len == 0U)
        return;

    /* 原子搬运，避免回调在解析过程中改写缓冲 */
    __disable_irq();
    len = usb_rx_stream_len;
    if (len > (sizeof(cmd) - 1U))
        len = sizeof(cmd) - 1U;
    memcpy(cmd, usb_rx_stream, len);
    usb_rx_stream_len = 0U;
    overflow = usb_rx_overflow;
    usb_rx_overflow = 0U;
    __enable_irq();

    if (len == 0U)
        return;

    cmd[len] = '\0';
    j = len;

    if (overflow)
    {
        USB_Debug_Printf("# ERROR: command buffer overflow, some input dropped\r\n");
    }

    /* 单包可能包含多行命令：逐行处理，避免 STATUS 与 SET 粘包时丢命令 */
    {
        char line[64];
        uint32_t li = 0U;
        for (i = 0U; i < j; i++)
        {
            char c = cmd[i];
            if (c == '\r')
                continue;

            if (c == '\n')
            {
                line[li] = '\0';
                if (li > 0U)
                    handle_one_command(line);
                li = 0U;
                continue;
            }

            if (li < (sizeof(line) - 1U))
                line[li++] = c;
        }

        line[li] = '\0';
        if (li > 0U)
            handle_one_command(line);
    }
}
