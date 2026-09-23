#include "vl53_ground_bootstrap.h"

#include <stddef.h>

static void enter_bootstrap(VL53GroundBootstrap *state)
{
    state->mode = VL53_RANGE_BOOTSTRAP;
    state->real_confirm_count = 0U;
    state->landing_confirm_count = 0U;
    state->airborne_clear_seen = 0U;
    state->landing_armed = 0U;
    state->updates_since_valid = 0U;
    state->last_valid_real_mm = 0U;
}

static void note_nonvalid_update(VL53GroundBootstrap *state)
{
    if (state->updates_since_valid < 255U) {
        state->updates_since_valid++;
    }
}

static void note_valid_update(VL53GroundBootstrap *state, uint16_t measured_mm)
{
    state->updates_since_valid = 0U;
    state->last_valid_real_mm = measured_mm;
}

static uint8_t recent_low_altitude_evidence(const VL53GroundBootstrap *state)
{
    return (state->airborne_clear_seen != 0U) &&
           (state->landing_armed != 0U) &&
           (state->updates_since_valid <= VL53_GROUND_LAST_VALID_MAX_AGE_UPDATES);
}

void VL53GroundBootstrap_Reset(VL53GroundBootstrap *state)
{
    if (state == NULL) {
        return;
    }

    enter_bootstrap(state);
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

    if (result == VL53_READ_VALID) {
        note_valid_update(state, measured_mm);
    } else {
        note_nonvalid_update(state);
    }

    if (state->mode == VL53_RANGE_BOOTSTRAP) {
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

            if (state->real_confirm_count < VL53_GROUND_REAL_CONFIRM_COUNT) {
                state->real_confirm_count++;
            }

            if (state->real_confirm_count >= VL53_GROUND_REAL_CONFIRM_COUNT) {
                state->mode = VL53_RANGE_REAL_FLIGHT;
                state->real_confirm_count = 0U;
                state->airborne_clear_seen =
                    (measured_mm >= VL53_GROUND_AIRBORNE_CLEAR_MM) ? 1U : 0U;
                state->landing_armed = 0U;
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

    if (state->mode == VL53_RANGE_REAL_FLIGHT) {
        switch (result) {
        case VL53_READ_VALID:
            if (measured_mm >= VL53_GROUND_AIRBORNE_CLEAR_MM) {
                state->airborne_clear_seen = 1U;
                state->landing_armed = 0U;
            } else if (state->airborne_clear_seen != 0U) {
                state->landing_armed =
                    (measured_mm <= VL53_GROUND_LANDING_ARM_MM) ? 1U : 0U;
            }

            if ((state->landing_armed != 0U) &&
                (measured_mm <= VL53_GROUND_LANDING_NEAR_MM)) {
                state->mode = VL53_RANGE_LANDING_CONFIRM;
                state->landing_confirm_count = 1U;
            }

            *published_mm = measured_mm;
            return 1U;

        case VL53_READ_TOO_CLOSE:
            if (recent_low_altitude_evidence(state) != 0U) {
                state->mode = VL53_RANGE_LANDING_CONFIRM;
                state->landing_confirm_count = 1U;
            }
            return 0U;

        case VL53_READ_NOT_READY:
        case VL53_READ_TOO_FAR:
        case VL53_READ_ERROR:
        default:
            return 0U;
        }
    }

    /* VL53_RANGE_LANDING_CONFIRM */
    switch (result) {
    case VL53_READ_VALID:
        if (measured_mm >= VL53_GROUND_AIRBORNE_CLEAR_MM) {
            state->mode = VL53_RANGE_REAL_FLIGHT;
            state->landing_confirm_count = 0U;
            state->airborne_clear_seen = 1U;
            state->landing_armed = 0U;
            *published_mm = measured_mm;
            return 1U;
        }

        if (measured_mm > VL53_GROUND_LANDING_NEAR_MM) {
            state->mode = VL53_RANGE_REAL_FLIGHT;
            state->landing_confirm_count = 0U;
            state->landing_armed =
                (measured_mm <= VL53_GROUND_LANDING_ARM_MM) ? 1U : 0U;
            *published_mm = measured_mm;
            return 1U;
        }

        if (state->landing_confirm_count < VL53_GROUND_LANDING_CONFIRM_COUNT) {
            state->landing_confirm_count++;
        }
        if (state->landing_confirm_count >= VL53_GROUND_LANDING_CONFIRM_COUNT) {
            enter_bootstrap(state);
            *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
            *using_bootstrap = 1U;
            return 1U;
        }

        *published_mm = measured_mm;
        return 1U;

    case VL53_READ_TOO_CLOSE:
        if (recent_low_altitude_evidence(state) == 0U) {
            state->mode = VL53_RANGE_REAL_FLIGHT;
            state->landing_confirm_count = 0U;
            state->landing_armed = 0U;
            return 0U;
        }

        if (state->landing_confirm_count < VL53_GROUND_LANDING_CONFIRM_COUNT) {
            state->landing_confirm_count++;
        }
        if (state->landing_confirm_count >= VL53_GROUND_LANDING_CONFIRM_COUNT) {
            enter_bootstrap(state);
            *published_mm = VL53_GROUND_BOOTSTRAP_DISTANCE_MM;
            *using_bootstrap = 1U;
            return 1U;
        }
        return 0U;

    case VL53_READ_NOT_READY:
        if (state->updates_since_valid > VL53_GROUND_LAST_VALID_MAX_AGE_UPDATES) {
            state->mode = VL53_RANGE_REAL_FLIGHT;
            state->landing_confirm_count = 0U;
            state->landing_armed = 0U;
        }
        return 0U;

    case VL53_READ_TOO_FAR:
    case VL53_READ_ERROR:
    default:
        if (state->updates_since_valid > VL53_GROUND_LAST_VALID_MAX_AGE_UPDATES) {
            state->mode = VL53_RANGE_REAL_FLIGHT;
            state->landing_confirm_count = 0U;
            state->landing_armed = 0U;
        }
        return 0U;
    }
}
