#include "driver.h"
#include "BLDCMotor.h"
#include "stm32g4xx_hal_tim.h"
#include <stdbool.h>
#include <stdint.h>

/* Global driver instances */
static Driver_t g_drivers[2] = {0};  /* [0]=LEFT, [1]=RIGHT */
Driver_t *driver = NULL;  /* Legacy pointer, use Driver_GetInstance instead */

static void DriverTIM_WriteCompare(Driver_t *m, uint32_t phA, uint32_t phB, uint32_t phC)
{
    if ((m == NULL) || (m->htim == NULL)) {
        return;
    }

    __HAL_TIM_SET_COMPARE(m->htim, m->chA, phA);
    __HAL_TIM_SET_COMPARE(m->htim, m->chB, phB);
    __HAL_TIM_SET_COMPARE(m->htim, m->chC, phC);
}

void Driver_SetPwm(Driver_t *driver, float ua, float ub, float uc)
{
    if (driver == NULL) {
        return;
    }

    DriverTIM_WriteCompare(driver, (uint32_t)ua, (uint32_t)ub, (uint32_t)uc);
}

void Driver_Enable(Driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    if (driver->en_port != NULL) {
        GPIO_PinState state =
            driver->enable_active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(driver->en_port, driver->en_pin, state);
    }

    driver->enabled = 1U;
}

void Driver_Disable(Driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    Driver_SetPwm(driver, 0.0f, 0.0f, 0.0f);

    if (driver->en_port != NULL) {
        GPIO_PinState state =
            driver->enable_active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(driver->en_port, driver->en_pin, state);
    }

    driver->enabled = 0U;
}

uint8_t Driver_Init(Driver_t *driver, TIM_HandleTypeDef *htim,
                    uint32_t chA, uint32_t chB, uint32_t chC,
                    GPIO_TypeDef *en_port, uint16_t en_pin,
                    uint8_t enable_active_low,
                    float voltage_limit)
{
    if ((driver == NULL) || (htim == NULL)) {
        return 0U;
    }

    driver->initialized = 0U;
    driver->enabled = 0U;

    driver->htim = htim;
    driver->chA = chA;
    driver->chB = chB;
    driver->chC = chC;
    driver->en_port = en_port;
    driver->en_pin = en_pin;
    driver->enable_active_low = enable_active_low;
    driver->voltage_limit = voltage_limit;
    driver->supply_voltage = voltage_limit;

    if (HAL_TIM_PWM_Start(driver->htim, driver->chA) != HAL_OK) {
        return 0U;
    }
    if (HAL_TIM_PWM_Start(driver->htim, driver->chB) != HAL_OK) {
        return 0U;
    }
    if (HAL_TIM_PWM_Start(driver->htim, driver->chC) != HAL_OK) {
        return 0U;
    }

    __HAL_TIM_SET_COMPARE(driver->htim, driver->chA, 0U);
    __HAL_TIM_SET_COMPARE(driver->htim, driver->chB, 0U);
    __HAL_TIM_SET_COMPARE(driver->htim, driver->chC, 0U);

    driver->initialized = 1U;
    return 1U;
}



Driver_t* Driver_GetInstance(DriverSide_t side)
{
    if (side >= 2U) {
        return NULL;
    }
    return &g_drivers[side];
}
