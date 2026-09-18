#include "vl53_median_filter.h"

#include <stddef.h>

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) {
        uint16_t temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c) {
        uint16_t temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b) {
        uint16_t temporary = a;
        a = b;
        b = temporary;
    }
    return b;
}

void VL53Median3Filter_Reset(VL53Median3Filter *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->samples[0] = 0U;
    filter->samples[1] = 0U;
    filter->samples[2] = 0U;
    filter->last_update_ms = 0U;
    filter->count = 0U;
    filter->index = 0U;
    filter->has_update = 0U;
}

uint16_t VL53Median3Filter_Update(VL53Median3Filter *filter,
                                  uint16_t sample,
                                  uint32_t now_ms,
                                  uint32_t reset_timeout_ms)
{
    if (filter == NULL) {
        return sample;
    }

    if ((filter->has_update != 0U) &&
        ((uint32_t)(now_ms - filter->last_update_ms) > reset_timeout_ms)) {
        VL53Median3Filter_Reset(filter);
    }

    filter->samples[filter->index] = sample;
    filter->index = (uint8_t)((filter->index + 1U) % 3U);
    if (filter->count < 3U) {
        filter->count++;
    }
    filter->last_update_ms = now_ms;
    filter->has_update = 1U;

    if (filter->count < 3U) {
        return sample;
    }

    return median3(filter->samples[0], filter->samples[1], filter->samples[2]);
}
