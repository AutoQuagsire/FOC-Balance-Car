#include "app_foc_bus.h"

#include "app_foc_internal.h"
#include "Filter.h"
#include "adc.h"
#include "stm32g4xx_hal.h"
#include "usb_debug.h"

#define APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS    (10U)
#define APP_BUS_VOLTAGE_LPF_CUTOFF_HZ       (10.0f)
#define APP_BUS_VOLTAGE_STARTUP_SAMPLE_COUNT (20U)
#define APP_BUS_VOLTAGE_VALID_MIN_V         (5.0f)
#define APP_BUS_VOLTAGE_VALID_MAX_V         (30.0f)

static BusVoltage_t g_bus_voltage;
volatile BusVoltageDebug_t g_bus_voltage_debug;
static LowPassFilter_t g_bus_voltage_lpf;
float g_bus_voltage_filtered = 0.0f;
uint8_t g_bus_voltage_valid = 0U;

static uint8_t s_bus_telemetry_ready = 0U;
static uint32_t s_last_bus_voltage_sample_tick_ms = 0U;

static void App_BusApplyFilteredVoltageToDrivers(void)
{
#if APP_BUS_VOLTAGE_FOC_ENABLE
    if (g_driver1 != NULL) {
        g_driver1->supply_voltage = g_bus_voltage_filtered;
    }
    if (g_driver2 != NULL) {
        g_driver2->supply_voltage = g_bus_voltage_filtered;
    }
#endif
}

static void App_ServiceBusVoltageSample(void)
{
#if APP_BUS_VOLTAGE_ENABLE
    if (!BusVoltage_SampleOnce(&g_bus_voltage)) {
        g_bus_voltage_valid = 0U;
        return;
    }

    g_bus_voltage_debug.bus_voltage = BusVoltage_GetBusVoltage(&g_bus_voltage);
    g_bus_voltage_debug.adc_pin_voltage = BusVoltage_GetAdcPinVoltage(&g_bus_voltage);
    g_bus_voltage_debug.raw_adc = BusVoltage_GetRawAdc(&g_bus_voltage);

    if ((g_bus_voltage_debug.bus_voltage < APP_BUS_VOLTAGE_VALID_MIN_V) ||
        (g_bus_voltage_debug.bus_voltage > APP_BUS_VOLTAGE_VALID_MAX_V)) {
        g_bus_voltage_valid = 0U;
        return;
    }

    g_bus_voltage_valid = 1U;
    g_bus_voltage_filtered = LowPassFilter_Update(&g_bus_voltage_lpf,
                                                  g_bus_voltage_debug.bus_voltage);
    App_BusApplyFilteredVoltageToDrivers();
#else
    g_bus_voltage_valid = 1U;
    g_bus_voltage_debug.raw_adc = 0U;
    g_bus_voltage_debug.adc_pin_voltage = 0.0f;
    g_bus_voltage_debug.bus_voltage = V_SUPPLY;
    g_bus_voltage_filtered = V_SUPPLY;
#endif
}

#if APP_BUS_VOLTAGE_ENABLE
static uint8_t App_BusVoltageStartupSample(void)
{
    uint32_t i;
    uint32_t valid_count = 0U;
    uint32_t raw_sum = 0U;
    float adc_pin_sum = 0.0f;
    float bus_sum = 0.0f;

    g_bus_voltage_valid = 0U;

    for (i = 0U; i < APP_BUS_VOLTAGE_STARTUP_SAMPLE_COUNT; i++) {
        if (BusVoltage_SampleOnce(&g_bus_voltage)) {
            const uint16_t raw_adc = BusVoltage_GetRawAdc(&g_bus_voltage);
            const float adc_pin_voltage = BusVoltage_GetAdcPinVoltage(&g_bus_voltage);
            const float bus_voltage = BusVoltage_GetBusVoltage(&g_bus_voltage);

            if ((bus_voltage >= APP_BUS_VOLTAGE_VALID_MIN_V) &&
                (bus_voltage <= APP_BUS_VOLTAGE_VALID_MAX_V)) {
                raw_sum += raw_adc;
                adc_pin_sum += adc_pin_voltage;
                bus_sum += bus_voltage;
                valid_count++;
            }
        }

        HAL_Delay(1U);
    }

    if (valid_count == 0U) {
        g_bus_voltage_debug.raw_adc = 0U;
        g_bus_voltage_debug.adc_pin_voltage = 0.0f;
        g_bus_voltage_debug.bus_voltage = 0.0f;
        g_bus_voltage_filtered = 0.0f;
        return 0U;
    }

    g_bus_voltage_debug.raw_adc =
        (uint16_t)((raw_sum + (valid_count / 2U)) / valid_count);
    g_bus_voltage_debug.adc_pin_voltage = adc_pin_sum / (float)valid_count;
    g_bus_voltage_debug.bus_voltage = bus_sum / (float)valid_count;
    g_bus_voltage_filtered = LowPassFilter_Update(&g_bus_voltage_lpf,
                                                  g_bus_voltage_debug.bus_voltage);
    g_bus_voltage_valid = 1U;

    return 1U;
}
#endif

uint8_t App_FOC_BusInit(void)
{
#if APP_BUS_VOLTAGE_ENABLE
    BusVoltage_Setup(&g_bus_voltage, &hadc3);
    BusVoltage_Enable(&g_bus_voltage);
    LowPassFilter_Init(&g_bus_voltage_lpf,
                       APP_BUS_VOLTAGE_LPF_CUTOFF_HZ,
                       1000.0f / (float)APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS);

    if (!App_BusVoltageStartupSample()) {
        s_bus_telemetry_ready = 0U;
        USB_Debug_Printf("BusVoltage startup sample failed, PWM disabled\r\n");
        return 0U;
    }

    USB_Debug_Printf("BusVoltage: ADC,PinV,BusV\r\n");
    USB_Debug_Printf("%u,%.3f,%.3f\r\n",
                     (unsigned)g_bus_voltage_debug.raw_adc,
                     g_bus_voltage_debug.adc_pin_voltage,
                     g_bus_voltage_debug.bus_voltage);
#else
    g_bus_voltage_valid = 1U;
    g_bus_voltage_debug.raw_adc = 0U;
    g_bus_voltage_debug.adc_pin_voltage = 0.0f;
    g_bus_voltage_debug.bus_voltage = V_SUPPLY;
    g_bus_voltage_filtered = V_SUPPLY;
    USB_Debug_Printf("BusVoltage disabled, use fixed V_SUPPLY=%.3f\r\n", V_SUPPLY);
#endif

    s_last_bus_voltage_sample_tick_ms = HAL_GetTick();
    s_bus_telemetry_ready = 1U;
    return 1U;
}

float App_FOC_GetBusVoltageFiltered(void)
{
    return g_bus_voltage_filtered;
}

uint8_t App_FOC_BusTelemetryInit(void)
{
    if (g_foc_stack_ready != 0U) {
        s_last_bus_voltage_sample_tick_ms = HAL_GetTick();
        s_bus_telemetry_ready = 1U;
        return 1U;
    }

    s_bus_telemetry_ready = 0U;
    return App_FOC_BusInit();
}

void App_FOC_BusTelemetryService(void)
{
    uint32_t now_ms;

    if (s_bus_telemetry_ready == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
#if APP_BUS_VOLTAGE_ENABLE
    if ((now_ms - s_last_bus_voltage_sample_tick_ms) >= APP_BUS_VOLTAGE_SAMPLE_PERIOD_MS) {
        s_last_bus_voltage_sample_tick_ms = now_ms;
        App_ServiceBusVoltageSample();
    }
#endif
}
