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

static int test_too_close_bootstraps_at_5cm(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_TOO_CLOSE, 28U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    return 0;
}

static int test_faults_are_never_disguised_as_ground(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 123U;
    uint8_t synthetic = 1U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(0U, VL53GroundBootstrap_Update(&state, VL53_READ_ERROR, 0U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(0U, VL53GroundBootstrap_Update(&state, VL53_READ_TOO_FAR, 4000U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);
    return 0;
}

static int test_real_range_requires_two_confirmations(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);

    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 70U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(VL53_RANGE_BOOTSTRAP, state.mode);

    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 80U, &out, &synthetic));
    CHECK_EQ(80U, out);
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(VL53_RANGE_REAL_LOCKED, state.mode);
    return 0;
}

static int test_near_boundary_does_not_lock_real(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 55U, &out, &synthetic));
    CHECK_EQ(50U, out);
    CHECK_EQ(1U, synthetic);
    CHECK_EQ(0U, state.real_confirm_count);
    return 0;
}

static int test_real_mode_never_falls_back_to_synthetic(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 70U, &out, &synthetic));
    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 80U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_LOCKED, state.mode);

    CHECK_EQ(0U, VL53GroundBootstrap_Update(&state, VL53_READ_TOO_CLOSE, 20U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(VL53_RANGE_REAL_LOCKED, state.mode);

    CHECK_EQ(0U, VL53GroundBootstrap_Update(&state, VL53_READ_ERROR, 0U, &out, &synthetic));
    CHECK_EQ(0U, synthetic);
    CHECK_EQ(VL53_RANGE_REAL_LOCKED, state.mode);

    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 900U, &out, &synthetic));
    CHECK_EQ(900U, out);
    CHECK_EQ(0U, synthetic);
    return 0;
}

static int test_not_ready_preserves_confirmation_streak(void)
{
    VL53GroundBootstrap state;
    uint16_t out = 0U;
    uint8_t synthetic = 0U;

    VL53GroundBootstrap_Reset(&state);
    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 70U, &out, &synthetic));
    CHECK_EQ(1U, state.real_confirm_count);
    CHECK_EQ(0U, VL53GroundBootstrap_Update(&state, VL53_READ_NOT_READY, 0U, &out, &synthetic));
    CHECK_EQ(1U, state.real_confirm_count);
    CHECK_EQ(1U, VL53GroundBootstrap_Update(&state, VL53_READ_VALID, 75U, &out, &synthetic));
    CHECK_EQ(VL53_RANGE_REAL_LOCKED, state.mode);
    return 0;
}

int main(void)
{
    if (test_too_close_bootstraps_at_5cm() != 0) return 1;
    if (test_faults_are_never_disguised_as_ground() != 0) return 1;
    if (test_real_range_requires_two_confirmations() != 0) return 1;
    if (test_near_boundary_does_not_lock_real() != 0) return 1;
    if (test_real_mode_never_falls_back_to_synthetic() != 0) return 1;
    if (test_not_ready_preserves_confirmation_streak() != 0) return 1;

    puts("VL53_GROUND_BOOTSTRAP_TEST_PASS");
    return 0;
}
