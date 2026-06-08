#ifndef APP_FOC_BUS_H
#define APP_FOC_BUS_H

#include <stdint.h>

uint8_t App_FOC_BusTelemetryInit(void);
void App_FOC_BusTelemetryService(void);
float App_FOC_GetBusVoltageFiltered(void);

#endif
