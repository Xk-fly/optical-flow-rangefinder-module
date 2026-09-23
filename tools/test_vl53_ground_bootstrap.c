#include <stdint.h>
#include <stdio.h>

#include "vl53_ground_bootstrap.h"

#define CHECK_EQ(expected, actual)                                                   \
    do {                                                                              \
        uint32_t expected_value = (uint32_t)(expected);                               \
        uint32_t actual_value = (uint32_t)(actual);                                   \
        if (expected_value != actual_value) {                                         \
            fprintf(stderr, "%s:%d expected %lu, got %lu\n",                       \
                    __FILE__, __LINE__,                                               \
                    (unsigned long)expected_value, (unsigned long)actual_value);       \
            return 1;                                                                 \
        }                                                                             \
    } while (0)

static uint8_t update(VL53GroundBootstrap *state,
                      VL53ReadResult result,
                      uint16_t measured_mm,
                      uint16_t *out,
                      uint8_t *synthetic)
{
    return VL53GroundBootstrap_Update(state, result, measured_mm, out, synthetic);
}

static int enter_real_flight(VL53GroundBootstrap *state, uint16_t *out, uint8_t *synthetic)
{
    CHECK_EQ(1U, update(state, VL53_READ_VALID, 100U, out, synthetic));
    CHECK_EQ(1U, *synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state->mode);

    CHECK_EQ(1U, update(state, VL53_READ_VALID, 110U, out, synthetic));
    CHECK_EQ(0U, *synthetic);
    CHECK_EQ(110U, *out);
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state->mode);
    return 0;
}

static int mark_airborne_and_arm_landing(VL53GroundBootstrap *state,
                                         uint16_t *out,
                                         uint8_t *synthetic)
{
    CHECK_EQ(1U, update(state, VL53_READ_VALID, 500U, out, synthetic));
    CHECK_EQ(1U, state->airborne_clear_seen);
    CHECK_EQ(0U, state->landing_armed);

    CHECK_EQ(1U, update(state, VL53_READ_VALID, 180U, out, synthetic));
    CHECK_EQ(1U, state->landing_armed);
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state->mode);
    return 0;
}

static int test_powerup_near_field_bootstraps_immediately(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 25U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 60U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    return 0;
}

static int test_hard_faults_never_create_synthetic_range(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 123U;
    uint8_t synthetic = 1U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(0U, update(&state, VL53_READ_ERROR, 0U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(0U, update(&state, VL53_READ_TOO_FAR, 4000U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    return 0;
}

static int test_two_real_samples_exit_bootstrap(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;
    return 0;
}

static int test_midair_too_close_cannot_false_bootstrap(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 1000U, &out, &synthetic));
    CHECK_EQ(1U, state.airborne_clear_seen);
    CHECK_EQ(0U, state.landing_armed);

    CHECK_EQ(0U, update(&state, VL53_READ_TOO_CLOSE, 0U, &out, &synthetic));
    CHECK_EQ(0U, update(&state, VL53_READ_TOO_CLOSE, 0U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
    CHECK_EQ(0U, synthetic);
    return 0;
}

static int test_landing_reenters_bootstrap(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;
    if (mark_airborne_and_arm_landing(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 70U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_LANDING_CONFIRM, state.mode);
    CHECK_EQ(1U, state.landing_confirm_count);
    CHECK_EQ(70U, out);
    CHECK_EQ(0U, synthetic);

    CHECK_EQ(0U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_LANDING_CONFIRM, state.mode);

    CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 25U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    return 0;
}

static int test_landing_confirmation_can_cancel(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;
    if (mark_airborne_and_arm_landing(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 70U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_LANDING_CONFIRM, state.mode);

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 150U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
    CHECK_EQ(1U, state.landing_armed);
    CHECK_EQ(0U, state.landing_confirm_count);

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 230U, &out, &synthetic));
    CHECK_EQ(0U, state.landing_armed);
    return 0;
}

static int test_stale_low_altitude_evidence_cannot_bootstrap(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;
    uint8_t i;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;
    if (mark_airborne_and_arm_landing(&state, &out, &synthetic) != 0) return 1;

    for (i = 0U; i < 6U; i++) {
        CHECK_EQ(0U, update(&state, VL53_READ_ERROR, 0U, &out, &synthetic));
    }
    CHECK_EQ(0U, update(&state, VL53_READ_TOO_CLOSE, 0U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
    CHECK_EQ(0U, state.landing_armed);
    return 0;
}

static int test_two_complete_takeoff_landing_cycles_without_reboot(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;
    uint8_t cycle;

    VL53GroundBootstrap_Reset(&state);

    for (cycle = 0U; cycle < 2U; cycle++) {
        CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 25U, &out, &synthetic));
        CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
        CHECK_EQ(50U, out);
        CHECK_EQ(1U, synthetic);

        if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;
        if (mark_airborne_and_arm_landing(&state, &out, &synthetic) != 0) return 1;

        CHECK_EQ(0U, update(&state, VL53_READ_TOO_CLOSE, 25U, &out, &synthetic));
        CHECK_EQ(VL53_RANGE_LANDING_CONFIRM, state.mode);
        CHECK_EQ(1U, state.landing_confirm_count);

        CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 25U, &out, &synthetic));
        CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
        CHECK_EQ(50U, out);
        CHECK_EQ(1U, synthetic);
    }

    return 0;
}

int main(void)
{
    if (test_powerup_near_field_bootstraps_immediately() != 0) return 1;
    if (test_hard_faults_never_create_synthetic_range() != 0) return 1;
    if (test_two_real_samples_exit_bootstrap() != 0) return 1;
    if (test_midair_too_close_cannot_false_bootstrap() != 0) return 1;
    if (test_landing_reenters_bootstrap() != 0) return 1;
    if (test_landing_confirmation_can_cancel() != 0) return 1;
    if (test_stale_low_altitude_evidence_cannot_bootstrap() != 0) return 1;
    if (test_two_complete_takeoff_landing_cycles_without_reboot() != 0) return 1;

    puts("VL53_GROUND_BOOTSTRAP_V2_TEST_PASS");
    return 0;
}
