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

static int enter_real_flight(VL53GroundBootstrap *state,
                             uint16_t *out,
                             uint8_t *synthetic)
{
    CHECK_EQ(1U, update(state, VL53_READ_VALID, 150U, out, synthetic));
    CHECK_EQ(1U, *synthetic);
    CHECK_EQ(50U, *out);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state->mode);

    CHECK_EQ(1U, update(state, VL53_READ_VALID, 160U, out, synthetic));
    CHECK_EQ(0U, *synthetic);
    CHECK_EQ(160U, *out);
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state->mode);
    return 0;
}

static int test_powerup_near_field_bootstraps(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);

    CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 25U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 120U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    return 0;
}

static int test_bootstrap_not_ready_requires_prior_ground_evidence(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;
    uint8_t i;

    VL53GroundBootstrap_Reset(&state);

    CHECK_EQ(0U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);

    CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 20U, &out, &synthetic));
    CHECK_EQ(1U, synthetic);

    for (i = 0U; i < 5U; i++) {
        CHECK_EQ(1U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
        CHECK_EQ(50U, out);
        CHECK_EQ(1U, synthetic);
        CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    }
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

    CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 20U, &out, &synthetic));
    CHECK_EQ(0U, update(&state, VL53_READ_ERROR, 0U, &out, &synthetic));
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
    return enter_real_flight(&state, &out, &synthetic);
}

static int test_single_near_real_sample_does_not_land(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 90U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
    CHECK_EQ(90U, out);
    CHECK_EQ(0U, synthetic);

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 180U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
    CHECK_EQ(0U, state.landing_confirm_count);
    return 0;
}

static int test_two_near_real_samples_return_bootstrap(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 95U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
    CHECK_EQ(95U, out);

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 85U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    return 0;
}

static int test_near_ground_nodata_recovers_bootstrap(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 250U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);

    CHECK_EQ(0U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
    CHECK_EQ(0U, update(&state, VL53_READ_TOO_CLOSE, 0U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);

    CHECK_EQ(1U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);

    CHECK_EQ(1U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    return 0;
}

static int test_midair_nodata_never_fakes_ground(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;
    uint8_t i;

    VL53GroundBootstrap_Reset(&state);
    if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;

    CHECK_EQ(1U, update(&state, VL53_READ_VALID, 800U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);

    for (i = 0U; i < 6U; i++) {
        CHECK_EQ(0U, update(&state,
                           (i & 1U) ? VL53_READ_TOO_CLOSE : VL53_READ_NOT_READY,
                           0U, &out, &synthetic));
        CHECK_EQ(VL53_RANGE_REAL_FLIGHT, state.mode);
        CHECK_EQ(0U, synthetic);
    }
    return 0;
}

static int test_two_complete_cycles_without_reboot(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;
    uint8_t cycle;

    VL53GroundBootstrap_Reset(&state);

    for (cycle = 0U; cycle < 2U; cycle++) {
        CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 20U, &out, &synthetic));
        CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
        CHECK_EQ(50U, out);
        CHECK_EQ(1U, synthetic);

        if (enter_real_flight(&state, &out, &synthetic) != 0) return 1;

        CHECK_EQ(1U, update(&state, VL53_READ_VALID, 200U, &out, &synthetic));
        CHECK_EQ(0U, update(&state, VL53_READ_TOO_CLOSE, 0U, &out, &synthetic));
        CHECK_EQ(0U, update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
        CHECK_EQ(1U, update(&state, VL53_READ_TOO_CLOSE, 0U, &out, &synthetic));
        CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
        CHECK_EQ(50U, out);
        CHECK_EQ(1U, synthetic);
    }

    return 0;
}

int main(void)
{
    if (test_powerup_near_field_bootstraps() != 0) return 1;
    if (test_bootstrap_not_ready_requires_prior_ground_evidence() != 0) return 1;
    if (test_hard_faults_never_create_synthetic_range() != 0) return 1;
    if (test_two_real_samples_exit_bootstrap() != 0) return 1;
    if (test_single_near_real_sample_does_not_land() != 0) return 1;
    if (test_two_near_real_samples_return_bootstrap() != 0) return 1;
    if (test_near_ground_nodata_recovers_bootstrap() != 0) return 1;
    if (test_midair_nodata_never_fakes_ground() != 0) return 1;
    if (test_two_complete_cycles_without_reboot() != 0) return 1;

    puts("VL53_GROUND_BOOTSTRAP_V3_TEST_PASS");
    return 0;
}
