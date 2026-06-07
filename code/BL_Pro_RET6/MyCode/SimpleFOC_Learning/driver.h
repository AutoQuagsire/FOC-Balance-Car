#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "sys.h"

typedef enum {
    DRIVER_LEFT = 0,
    DRIVER_RIGHT = 1,
} DriverSide_t;

typedef struct {
    uint8_t initialized;
    uint8_t enabled;

    TIM_HandleTypeDef *htim;
    uint32_t chA;
    uint32_t chB;
    uint32_t chC;

    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    uint8_t enable_active_low;   // 1=低电平使能，0=高电平使能

    float voltage_limit;
    float supply_voltage;
} Driver_t;

/* Fast-path compare write.
 * This relies on TIM_CHANNEL_x encoding so that (TIM_CHANNEL_x >> 2U)
 * maps to CCR register index (CCR1..CCR4). */
static inline void Driver_SetCompareFast(const Driver_t *driver,
                                         uint32_t ccr_a,
                                         uint32_t ccr_b,
                                         uint32_t ccr_c)
{
    volatile uint32_t *ccr;

    if ((driver == NULL) || (driver->htim == NULL)) {
        return;
    }

    ccr = &driver->htim->Instance->CCR1;
    ccr[(driver->chA >> 2U)] = ccr_a;
    ccr[(driver->chB >> 2U)] = ccr_b;
    ccr[(driver->chC >> 2U)] = ccr_c;
}


/* NOTE: remove any top-level function calls from headers. */


/* Public driver API */
void Driver_SetPwm(Driver_t *driver, float ua, float ub, float uc);
void Driver_Disable(Driver_t *driver);
void Driver_Enable(Driver_t *driver);
/* Driver initialization and accessor */
uint8_t Driver_Init(Driver_t *driver, TIM_HandleTypeDef *htim,
                    uint32_t chA, uint32_t chB, uint32_t chC,
                    GPIO_TypeDef *en_port, uint16_t en_pin,
                    uint8_t enable_active_low,
                    float voltage_limit);
Driver_t* Driver_GetInstance(DriverSide_t side);


#endif /* DRIVER_H */
