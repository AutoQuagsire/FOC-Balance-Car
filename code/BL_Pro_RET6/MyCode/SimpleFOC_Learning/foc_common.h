#ifndef FOC_COMMON_H
#define FOC_COMMON_H

#include <stdint.h>
#include "main.h"

typedef struct {
    float g_foc_frequency;
    float g_foc_period_s;
    uint32_t g_foc_period_us;
    TIM_HandleTypeDef *g_foc_it_timer;
} FocFrequency_t;

/* Control-loop timing defaults.
 * The runtime values can be updated from an interrupt timer by app_foc_new.c.
 */
#ifndef FOC_FREQUENCY_DEFAULT
#define FOC_FREQUENCY_DEFAULT 10000.0f
#endif

#ifndef FOC_PERIOD_S_DEFAULT
#define FOC_PERIOD_S_DEFAULT (1.0f / FOC_FREQUENCY_DEFAULT)
#endif

#ifndef FOC_PERIOD_US_DEFAULT
#define FOC_PERIOD_US_DEFAULT 100U
#endif

extern FocFrequency_t g_foc;

#ifndef FOC_FREQUENCY
#define FOC_FREQUENCY (g_foc.g_foc_frequency)
#endif

#ifndef FOC_PERIOD_S
#define FOC_PERIOD_S (g_foc.g_foc_period_s)
#endif

#ifndef FOC_PERIOD_US
#define FOC_PERIOD_US (g_foc.g_foc_period_us)
#endif

/* Math / modulation constants */
#ifndef PI
#define PI 3.14159265359f
#endif

static inline float normalizeAngle(float angle)
{
    float a = angle - (float)(int32_t)(angle * (1.0f / (2.0f * PI))) * (2.0f * PI);
    return (a >= 0.0f) ? a : (a + 2.0f * PI);
}

#ifndef V_SUPPLY
#define V_SUPPLY 12.0f
#endif

#ifndef Uq_max
#define Uq_max (V_SUPPLY * 0.577f)
#endif

/* Bus-voltage measurement / compensation switches.
 *
 * APP_BUS_VOLTAGE_ENABLE:
 *   1 = enable ADC3 VBUS sampling, startup validity check, and debug values
 *   0 = bypass VBUS sampling and use fixed V_SUPPLY as the bus voltage
 *
 * APP_BUS_VOLTAGE_FOC_ENABLE:
 *   1 = use measured/filtered VBUS for FOC PWM modulation
 *   0 = keep FOC PWM modulation on fixed V_SUPPLY
 */
#ifndef APP_BUS_VOLTAGE_ENABLE
#define APP_BUS_VOLTAGE_ENABLE 1U
#endif

#ifndef APP_BUS_VOLTAGE_FOC_ENABLE
#define APP_BUS_VOLTAGE_FOC_ENABLE APP_BUS_VOLTAGE_ENABLE
#endif

#ifndef _SQRT3
#define _SQRT3 1.73205080757f
#endif

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif /* FOC_COMMON_H */
