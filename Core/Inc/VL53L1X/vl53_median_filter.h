#ifndef VL53_MEDIAN_FILTER_H
#define VL53_MEDIAN_FILTER_H

#include <stdint.h>

typedef struct {
    uint16_t samples[3];
    uint32_t last_update_ms;
    uint8_t count;
    uint8_t index;
    uint8_t has_update;
} VL53Median3Filter;

void VL53Median3Filter_Reset(VL53Median3Filter *filter);
uint16_t VL53Median3Filter_Update(VL53Median3Filter *filter,
                                  uint16_t sample,
                                  uint32_t now_ms,
                                  uint32_t reset_timeout_ms);

#endif
