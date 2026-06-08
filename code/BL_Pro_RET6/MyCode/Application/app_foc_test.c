#include "app_foc_test.h"

#include "app_foc_internal.h"
#include "stm32g4xx_hal.h"
#include "sys.h"
#include "usb_debug.h"

#include <stdint.h>

#define APP_CS_SIGN_TEST_UQ_V         (0.8f)
#define APP_CS_SIGN_TEST_SETTLE_MS    (80U)
#define APP_CS_SIGN_TEST_SAMPLE_CNT   (120U)
#define APP_CS_SIGN_TEST_SAMPLE_DT_MS (1U)
#define APP_CS_SIGN_TEST_DEADBAND_A   (0.03f)

#define APP_SENSOR_DIR_TEST_UQ_V         (4.0f)
#define APP_SENSOR_DIR_TEST_SETTLE_MS    (500U)
#define APP_SENSOR_DIR_TEST_ELEC_STEP    (PI * 0.5f)
#define APP_SENSOR_DIR_TEST_DEADBAND_RAD (0.02f)
#define APP_SENSOR_DIR_TEST_MAX_STEP_MS  (2500U)

#ifndef LEFT_MOTOR_ENABLE
#define LEFT_MOTOR_ENABLE 1U
#endif

#ifndef RIGHT_MOTOR_ENABLE
#define RIGHT_MOTOR_ENABLE 1U
#endif

static int8_t app_sign_with_deadband(float v, float deadband)
{
    if (v > deadband) {
        return 1;
    }
    if (v < -deadband) {
        return -1;
    }
    return 0;
}

static PhaseCurrent_t app_measure_phase_current_avg(Motor_t *motor,
                                                    CurrentSense_t *cs,
                                                    float uq,
                                                    float elec_angle)
{
    uint32_t i;
    PhaseCurrent_t avg = {0.0f, 0.0f};

    if ((motor == NULL) || (cs == NULL)) {
        return avg;
    }

    Motor_SetPhaseVoltageQ(motor, uq, elec_angle);
    HAL_Delay(APP_CS_SIGN_TEST_SETTLE_MS);

    for (i = 0U; i < APP_CS_SIGN_TEST_SAMPLE_CNT; i++) {
        PhaseCurrent_t cur = CurrentSense_GetPhaseCurrent(cs);
        avg.ia += cur.ia;
        avg.ib += cur.ib;
        HAL_Delay(APP_CS_SIGN_TEST_SAMPLE_DT_MS);
    }

    avg.ia /= (float)APP_CS_SIGN_TEST_SAMPLE_CNT;
    avg.ib /= (float)APP_CS_SIGN_TEST_SAMPLE_CNT;

    return avg;
}

static void app_current_sense_sign_test_one(const char *tag,
                                            Motor_t *motor,
                                            CurrentSense_t *cs)
{
    uint8_t was_enabled;
    PhaseCurrent_t s_0;
    PhaseCurrent_t s_pi;
    PhaseCurrent_t s_pi_2;
    PhaseCurrent_t s_3pi_2;
    int8_t a_score = 0;
    int8_t b_score = 0;
    float uq = APP_CS_SIGN_TEST_UQ_V;

    if ((motor == NULL) || (cs == NULL) || (motor->driver == NULL) || (!motor->driver->initialized)) {
        USB_Debug_Printf("[CS-SIGN][%s] skip (motor/cs not ready)\r\n", tag);
        return;
    }

    was_enabled = (uint8_t)motor->state.enabled;
    if (!was_enabled) {
        FOCMotor_enable(motor);
        HAL_Delay(10);
    }

    if (!cs->enabled) {
        CurrentSense_Enable(cs);
        HAL_Delay(10);
    }

    if ((motor->driver->voltage_limit > 0.0f) && (uq > motor->driver->voltage_limit * 0.3f)) {
        uq = motor->driver->voltage_limit * 0.3f;
    }
    if (uq < 0.3f) {
        uq = 0.3f;
    }

    USB_Debug_Printf("[CS-SIGN][%s] test start A_SIGN=%d B_SIGN=%d uq=%.2fV\r\n",
                     tag, cs->CsParam.A_SIGN, cs->CsParam.B_SIGN, uq);

    s_0 = app_measure_phase_current_avg(motor, cs, uq, 0.0f);
    s_pi = app_measure_phase_current_avg(motor, cs, uq, PI);
    s_pi_2 = app_measure_phase_current_avg(motor, cs, uq, 0.5f * PI);
    s_3pi_2 = app_measure_phase_current_avg(motor, cs, uq, 1.5f * PI);

    Motor_SetPhaseVoltageQ(motor, 0.0f, 0.0f);

    a_score += (app_sign_with_deadband(s_3pi_2.ia, APP_CS_SIGN_TEST_DEADBAND_A) == 1) ? 1 : -1;
    a_score += (app_sign_with_deadband(s_pi_2.ia, APP_CS_SIGN_TEST_DEADBAND_A) == -1) ? 1 : -1;
    b_score += (app_sign_with_deadband(s_0.ib, APP_CS_SIGN_TEST_DEADBAND_A) == 1) ? 1 : -1;
    b_score += (app_sign_with_deadband(s_pi.ib, APP_CS_SIGN_TEST_DEADBAND_A) == -1) ? 1 : -1;

    USB_Debug_Printf("[CS-SIGN][%s] theta0    : ia=% .4f ib=% .4f (expect ib>0)\r\n", tag, s_0.ia, s_0.ib);
    USB_Debug_Printf("[CS-SIGN][%s] thetaPI   : ia=% .4f ib=% .4f (expect ib<0)\r\n", tag, s_pi.ia, s_pi.ib);
    USB_Debug_Printf("[CS-SIGN][%s] thetaPI/2 : ia=% .4f ib=% .4f (expect ia<0)\r\n", tag, s_pi_2.ia, s_pi_2.ib);
    USB_Debug_Printf("[CS-SIGN][%s] theta3PI/2: ia=% .4f ib=% .4f (expect ia>0)\r\n", tag, s_3pi_2.ia, s_3pi_2.ib);

    USB_Debug_Printf("[CS-SIGN][%s] recommendation: A_SIGN %s (%d -> %d), B_SIGN %s (%d -> %d)\r\n",
                     tag,
                     (a_score >= 0) ? "KEEP" : "FLIP",
                     cs->CsParam.A_SIGN,
                     (a_score >= 0) ? cs->CsParam.A_SIGN : -cs->CsParam.A_SIGN,
                     (b_score >= 0) ? "KEEP" : "FLIP",
                     cs->CsParam.B_SIGN,
                     (b_score >= 0) ? cs->CsParam.B_SIGN : -cs->CsParam.B_SIGN);

    if (!was_enabled) {
        FOCMotor_disable(motor);
    }
}

void App_CurrentSenseSignTest(void)
{
    USB_Debug_Printf("[CS-SIGN] test begin (run with motor unloaded and hold rotor still)\r\n");

#if LEFT_MOTOR_ENABLE
    app_current_sense_sign_test_one("L", &g_motor1, &g_current_sense1);
#endif

#if RIGHT_MOTOR_ENABLE
    app_current_sense_sign_test_one("R", &g_motor2, &g_current_sense2);
#endif

    USB_Debug_Printf("[CS-SIGN] test end\r\n");
}

static float app_angle_diff_signed(float from, float to)
{
    float d = to - from;

    while (d > PI) {
        d -= 2.0f * PI;
    }
    while (d < -PI) {
        d += 2.0f * PI;
    }

    return d;
}

static float app_sensor_angle_after_settle(Motor_t *motor,
                                           Sensor_t *sensor,
                                           float uq,
                                           float elec_angle,
                                           uint32_t settle_ms)
{
    uint32_t i;
    uint32_t step_ms;
    uint32_t t0;
    uint32_t t1;
    uint32_t ticks_per_ms;

    Motor_SetPhaseVoltageQ(motor, uq, elec_angle);

    ticks_per_ms = SystemCoreClock / 1000U;
    if (ticks_per_ms == 0U) {
        ticks_per_ms = 1U;
    }

    step_ms = settle_ms;
    if (step_ms > APP_SENSOR_DIR_TEST_MAX_STEP_MS) {
        step_ms = APP_SENSOR_DIR_TEST_MAX_STEP_MS;
    }

    for (i = 0U; i < step_ms; i++) {
        Sensor_Update(sensor, 0.001f);

        t0 = DWT_GetTicks();
        do {
            t1 = DWT_GetElapsedTicks(t0);
        } while (t1 < ticks_per_ms);
    }

    Sensor_Update(sensor, 0.001f);
    return Sensor_GetAngle(sensor);
}

static void app_sensor_direction_test_one(const char *tag, Motor_t *motor, Sensor_t *sensor)
{
    uint8_t was_enabled;
    float uq = APP_SENSOR_DIR_TEST_UQ_V;
    float a0;
    float ap;
    float an;
    float dp;
    float dn;
    int8_t score = 0;

    if ((motor == NULL) || (sensor == NULL) || (motor->driver == NULL) ||
        (!motor->driver->initialized) || (!sensor->initialized)) {
        USB_Debug_Printf("[DIR-TEST][%s] skip (motor/sensor not ready)\r\n", tag);
        return;
    }

    was_enabled = (uint8_t)motor->state.enabled;
    if (!was_enabled) {
        FOCMotor_enable(motor);
        HAL_Delay(10);
    }

    if ((motor->driver->voltage_limit > 0.0f) && (uq > motor->driver->voltage_limit * 0.3f)) {
        uq = motor->driver->voltage_limit * 0.3f;
    }
    if (uq < 0.3f) {
        uq = 0.3f;
    }

    USB_Debug_Printf("[DIR-TEST][%s] start uq=%.2fV elec_step=%.3f\r\n",
                     tag, uq, APP_SENSOR_DIR_TEST_ELEC_STEP);

    a0 = app_sensor_angle_after_settle(motor, sensor, uq, 0.0f, APP_SENSOR_DIR_TEST_SETTLE_MS);
    ap = app_sensor_angle_after_settle(motor, sensor, uq, APP_SENSOR_DIR_TEST_ELEC_STEP, APP_SENSOR_DIR_TEST_SETTLE_MS);
    an = app_sensor_angle_after_settle(motor, sensor, uq, -APP_SENSOR_DIR_TEST_ELEC_STEP, APP_SENSOR_DIR_TEST_SETTLE_MS);

    Motor_SetPhaseVoltageQ(motor, 0.0f, 0.0f);

    dp = app_angle_diff_signed(a0, ap);
    dn = app_angle_diff_signed(a0, an);

    if (dp > APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score++;
    } else if (dp < -APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score--;
    }

    if (dn < -APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score++;
    } else if (dn > APP_SENSOR_DIR_TEST_DEADBAND_RAD) {
        score--;
    }

    USB_Debug_Printf("[DIR-TEST][%s] a0=%.4f ap=%.4f an=%.4f d_plus=%.4f d_minus=%.4f\r\n",
                     tag, a0, ap, an, dp, dn);

    if (score > 0) {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: sensor_direction_cw\r\n", tag);
    } else if (score < 0) {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: sensor_direction_ccw\r\n", tag);
    } else {
        USB_Debug_Printf("[DIR-TEST][%s] recommendation: inconclusive (increase uq/settle time and retest)\r\n", tag);
    }

    if (!was_enabled) {
        FOCMotor_disable(motor);
    }
}

void App_SensorDirectionTest(void)
{
    USB_Debug_Printf("[DIR-TEST] begin (motor should be free to move, not held)\r\n");

#if LEFT_MOTOR_ENABLE
    app_sensor_direction_test_one("L", &g_motor1, &g_sensor1);
#endif

    HAL_Delay(500);

#if RIGHT_MOTOR_ENABLE
    app_sensor_direction_test_one("R", &g_motor2, &g_sensor2);
#endif

    USB_Debug_Printf("[DIR-TEST] end\r\n");
}
