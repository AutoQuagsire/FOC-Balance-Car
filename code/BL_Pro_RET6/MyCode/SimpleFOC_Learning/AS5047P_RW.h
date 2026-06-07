#ifndef AS5047P_RW_H
#define AS5047P_RW_H

#include "main.h"
#include <stdint.h>

#define AS5047P_CPR              16384.0f
#define AS5047P_ANGLE_RAD_SCALE  (2.0f * 3.14159265358979f / AS5047P_CPR)

#define AS5047P_REG_NOP          0x0000
#define AS5047P_REG_ERRFL        0x0001
#define AS5047P_REG_DIAAGC       0x3FFC
#define AS5047P_REG_MAG          0x3FFD
#define AS5047P_REG_ANGLEUNC     0x3FFE
#define AS5047P_REG_ANGLECOM     0x3FFF

typedef enum {
    AS5047P_OK = 0,
    AS5047P_ERROR,
    AS5047P_TIMEOUT
} AS5047P_Status_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    uint8_t initialized;
    uint8_t errorflag;

    uint16_t raw_angle;
    float angle_rad;
} AS5047P_Handle_t;
 



uint8_t AS5047P_RW_Init(AS5047P_Handle_t *dev,
                     SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port,
                     uint16_t cs_pin);

AS5047P_Status_t AS5047P_ReadRawAngle(AS5047P_Handle_t *dev, uint16_t *raw);
AS5047P_Status_t AS5047P_ReadMagnitude(AS5047P_Handle_t *dev, uint16_t *mag);
AS5047P_Status_t AS5047P_ReadErrfl(AS5047P_Handle_t *dev, uint16_t *errfl);
AS5047P_Status_t AS5047P_ClearErrorFlag(AS5047P_Handle_t *dev);


#endif
