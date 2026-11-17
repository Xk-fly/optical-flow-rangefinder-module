#ifndef HL5300_H
#define HL5300_H

#include "stm32g0xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define HL5300_DISTANCE_FRESH_TIMEOUT_MS 300U

/* NULL on success, otherwise a short MAVLink diagnostic for the failed stage. */
const char *hl5300_init(I2C_HandleTypeDef *bus);

/* Poll once. Only replace distance_mm with a newly read, in-range sample. */
bool hl5300_read_distance(I2C_HandleTypeDef *bus, uint16_t *distance_mm);

#endif
