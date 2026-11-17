#ifndef HL5300_H
#define HL5300_H

#include "stm32g0xx_hal.h"

#include <stdint.h>

#define HL5300_MIN_DISTANCE_MM 25
#define HL5300_MAX_DISTANCE_MM 3600
#define HL5300_DISTANCE_FRESH_TIMEOUT_MS 300U

typedef enum {
    HL5300_NO_DATA,
    HL5300_IO_ERROR,
    HL5300_OUT_OF_RANGE,
    HL5300_VALID,
} HL5300ReadResult;

/* NULL on success, otherwise a short MAVLink diagnostic for the failed stage. */
const char *hl5300_init(I2C_HandleTypeDef *bus);

/* Poll once. A decoded sample is returned even when it falls outside the valid range. */
HL5300ReadResult hl5300_read_distance(I2C_HandleTypeDef *bus, int16_t *raw_mm);

#endif
