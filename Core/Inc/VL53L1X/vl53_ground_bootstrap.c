#include "vl53_ground_bootstrap.h"

#include <stddef.h>

static void enter_bootstrap(VL53GroundBootstrap *state, uint8_t evidence_seen)
{
    state->mode = VL53_RANGE_BOOTSTRAP;
    state->real_confirm_count = 0U;
    state->landing_confirm_count = 0U;
    state->nodata_confirm_count = 0U;
    state->last_valid_real_present = 0U;
    state->last_valid_real_mm = 0U;
    state->bootstrap_evidence_seen = evidence_seen;
}

static void note_valid_real(VL53GroundBootstrap *state, uint16_t measured_mm)
{
    state->last_valid_real_present = 1U;
    state->last_valid_real_mm = measured_mm;
}

static uint8_t near_ground_nodata_allowed(const VL53GroundBootstrap *state)
{
    return (state->last_valid_real_present != 0U) &&
           (state->last_valid_real_mm <= VL53_GROUND_NODATA_LAST_VALID_MAX_MM);
}

void VL53GroundBootstrap_Reset(VL53GroundBootstrap *state)
{
    if (state == NULL) {
        return;
    }

    enter_bootstrap(state, 0U);
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

    if (state->mode == VL53_RANGE_BOOTSTRAP) {
        switch (result) {
        case VL53_READ_TOO_CLOSE:
            state->bootstrap_evidence_seen = 1U;
            state->real_confirm_count = 0U;
            *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
            *using_bootstrap = 1U;
            return 1U;

        case VL53_READ_VALID:
            if (measured_mm < VL53_GROUND_BOOTSTRAP_EXIT_MM) {
                state->bootstrap_evidence_seen = 1U;
                state->real_confirm_count = 0U;
                *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
                *using_bootstrap = 1U;
                return 1U;
            }

            if (state->real_confirm_count < VL53_GROUND_REAL_CONFIRM_COUNT) {
                state->real_confirm_count++;
            }

            if (state->real_confirm_count >= VL53_GROUND_REAL_CONFIRM_COUNT) {
                state->mode = VL53_RANGE_REAL_FLIGHT;
                state->real_confirm_count = 0U;
                state->landing_confirm_count = 0U;
                state->nodata_confirm_count = 0U;
                state->bootstrap_evidence_seen = 0U;
                note_valid_real(state, measured_mm);
                *published_mm = measured_mm;
                return 1U;
            }

            *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
            *using_bootstrap = 1U;
            return 1U;

        case VL53_READ_NOT_READY:
            state->real_confirm_count = 0U;
            if (state->bootstrap_evidence_seen != 0U) {
                *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
                *using_bootstrap = 1U;
                return 1U;
            }
            return 0U;

        case VL53_READ_TOO_FAR:
        case VL53_READ_ERROR:
        default:
            state->real_confirm_count = 0U;
            return 0U;
        }
    }

    /* VL53_RANGE_REAL_FLIGHT */
    switch (result) {
    case VL53_READ_VALID:
        note_valid_real(state, measured_mm);
        state->nodata_confirm_count = 0U;

        if (measured_mm <= VL53_GROUND_LANDING_NEAR_MM) {
            if (state->landing_confirm_count < VL53_GROUND_LANDING_CONFIRM_COUNT) {
                state->landing_confirm_count++;
            }
            if (state->landing_confirm_count >= VL53_GROUND_LANDING_CONFIRM_COUNT) {
                enter_bootstrap(state, 1U);
                *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
                *using_bootstrap = 1U;
                return 1U;
            }
        } else {
            state->landing_confirm_count = 0U;
        }

        *published_mm = measured_mm;
        return 1U;

    case VL53_READ_TOO_CLOSE:
    case VL53_READ_NOT_READY:
        state->landing_confirm_count = 0U;

        if (near_ground_nodata_allowed(state) != 0U) {
            if (state->nodata_confirm_count < VL53_GROUND_NODATA_CONFIRM_COUNT) {
                state->nodata_confirm_count++;
            }
            if (state->nodata_confirm_count >= VL53_GROUND_NODATA_CONFIRM_COUNT) {
                enter_bootstrap(state, 1U);
                *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
                *using_bootstrap = 1U;
                return 1U;
            }
        } else {
            state->nodata_confirm_count = 0U;
        }
        return 0U;

    case VL53_READ_TOO_FAR:
    case VL53_READ_ERROR:
    default:
        state->landing_confirm_count = 0U;
        state->nodata_confirm_count = 0U;
        return 0U;
    }
}
