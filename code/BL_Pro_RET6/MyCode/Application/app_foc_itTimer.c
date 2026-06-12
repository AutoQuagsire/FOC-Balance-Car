#include "app_foc_itTimer.h"
#include "app_foc_internal.h"



uint8_t App_FOCControlIT_Enable(TIM_HandleTypeDef *htim)
{
    if (htim != NULL) {
        if (g_foc.g_foc_it_timer != htim) {
            if (!App_FOCControlIT_ConfigFromTimer(htim)) {
                return 0U;
            }
        }
    }

    if (g_foc.g_foc_it_timer == NULL) {
        return 0U;
    }

    if (HAL_TIM_Base_Start_IT(g_foc.g_foc_it_timer) != HAL_OK) {
        g_foc_control_it_enabled = 0U;
        return 0U;
    }

    g_foc_control_it_enabled = 1U;
    return 1U;
}

void App_FOCControlIT_Disable(void)
{
    if (g_foc.g_foc_it_timer != NULL) {
        (void)HAL_TIM_Base_Stop_IT(g_foc.g_foc_it_timer);
    }
    g_foc_control_it_enabled = 0U;
}






static uint32_t App_GetTimerClockHz(const TIM_HandleTypeDef *htim)
{
    RCC_ClkInitTypeDef clk_config = {0};
    uint32_t latency = 0U;
    uint32_t pclk = 0U;

    if (htim == NULL) {
        return 0U;
    }

    HAL_RCC_GetClockConfig(&clk_config, &latency);

    if (htim->Instance == TIM1) {
        pclk = HAL_RCC_GetPCLK2Freq();
        if (clk_config.APB2CLKDivider != RCC_HCLK_DIV1) {
            pclk *= 2U;
        }
    } else {
        pclk = HAL_RCC_GetPCLK1Freq();
        if (clk_config.APB1CLKDivider != RCC_HCLK_DIV1) {
            pclk *= 2U;
        }
    }

    return pclk;
}

uint8_t App_FOCControlIT_ConfigFromTimer(TIM_HandleTypeDef *htim)
{
    uint32_t tim_clk_hz;
    uint32_t psc;
    uint32_t arr;
    double frequency_hz;

    if (htim == NULL) {
        return 0U;
    }

    tim_clk_hz = App_GetTimerClockHz(htim);
    psc = (uint32_t)htim->Init.Prescaler + 1U;
    arr = (uint32_t)__HAL_TIM_GET_AUTORELOAD(htim) + 1U;

    if ((tim_clk_hz == 0U) || (psc == 0U) || (arr == 0U)) {
        return 0U;
    }

    frequency_hz = (double)tim_clk_hz / ((double)psc * (double)arr);
    if (frequency_hz <= 0.0) {
        return 0U;
    }

    g_foc.g_foc_frequency = (float)frequency_hz;
    g_foc.g_foc_period_s = 1.0f / g_foc.g_foc_frequency;
    g_foc.g_foc_period_us = (uint32_t)((g_foc.g_foc_period_s * 1000000.0f) + 0.5f);
    g_foc.g_foc_it_timer = htim;

    USB_Debug_Printf("FOC IT cfg: f=%.2f Hz, Ts=%lu us\r\n",
                 g_foc.g_foc_frequency,
                 (unsigned long)g_foc.g_foc_period_us);
    return 1U;
}



