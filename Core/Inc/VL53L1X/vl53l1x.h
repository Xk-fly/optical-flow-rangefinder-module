#ifndef VL53L1X_H
#define VL53L1X_H

#include "VL53L1X/core/VL53L1X_api.h"
#include "VL53L1X/vl53_types.h"
#include "main.h"

#define VL53_DISTANCE_FRESH_TIMEOUT_MS 300U

uint8_t vl53_Init(void);
VL53ReadResult vl53_GetDistance(uint16_t *distance_mm);

#endif
