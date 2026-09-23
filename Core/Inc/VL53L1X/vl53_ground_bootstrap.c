#include "vl53_ground_bootstrap.h"

#include <stddef.h>

void VL53GroundBootstrap_Reset(VL53GroundBootstrap *state)
{
    if (state == NULL) {
        return;
    }

    state->mode = VL53_RANGE_BOOTSTRAP;
    state->real_confirm_count = 0U;
}

uint8_t VL53GroundBootstrap_Update(VL53GroundBootstrap *state,
                                   VL53ReadResult result,
                                   uint16_t measured_mm,
                                   uint16_t *published_mm,
                                   uint8_t *using_bootstrap)
{
    if ((state == NULL) || (published_mm == NULL) || (using_bootstrap == NULL)) {
        return 0U;
    }

    *using_bootstrap = 0U;

    if (state->mode == VL53_RANGE_REAL_LOCKED) {
        if (result != VL53_READ_VALID) {
            return 0U;
        }

        *published_mm = measured_mm;
        return 1U;
    }

    switch (result) {
    case VL53_READ_TOO_CLOSE:
        state->real_confirm_count = 0U;
        *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
        *using_bootstrap = 1U;
        return 1U;

    case VL53_READ_VALID:
        if (measured_mm < VL53_GROUND_BOOTSTRAP_EXIT_MM) {
            state->real_confirm_count = 0U;
            *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
            *using_bootstrap = 1U;
            return 1U;
        }

        if (state->real_confirm_count < VL53_GROUND_BOOTSTRAP_CONFIRM_COUNT) {
            state->real_confirm_count++;
        }

        if (state->real_confirm_count >= VL53_GROUND_BOOTSTRAP_CONFIRM_COUNT) {
            state->mode = VL53_RANGE_REAL_LOCKED;
            *published_mm = measured_mm;
            return 1U;
        }

        *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
        *using_bootstrap = 1U;
        return 1U;

    case VL53_READ_NOT_READY:
        return 0U;

    case VL53_READ_TOO_FAR:
    case VL53_READ_ERROR:
    default:
        state->real_confirm_count = 0U;
        return 0U;
    }
}
