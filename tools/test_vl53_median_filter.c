#include <stdint.h>
#include <stdio.h>

#include "vl53_median_filter.h"

#define CHECK_EQ(expected, actual)                                                   \
    do {                                                                              \
        uint16_t expected_value = (uint16_t)(expected);                               \
        uint16_t actual_value = (uint16_t)(actual);                                   \
        if (expected_value != actual_value) {                                         \
            fprintf(stderr, "%s:%d expected %u, got %u\n",                         \
                    __FILE__, __LINE__, expected_value, actual_value);                 \
            return 1;                                                                 \
        }                                                                             \
    } while (0)

static int test_third_sample_is_filtered(void)
{
    VL53Median3Filter filter;

    VL53Median3Filter_Reset(&filter);
    CHECK_EQ(1000U, VL53Median3Filter_Update(&filter, 1000U, 0U, 300U));
    CHECK_EQ(3000U, VL53Median3Filter_Update(&filter, 3000U, 100U, 300U));
    CHECK_EQ(1010U, VL53Median3Filter_Update(&filter, 1010U, 200U, 300U));
    CHECK_EQ(1020U, VL53Median3Filter_Update(&filter, 1020U, 300U, 300U));
    return 0;
}

static int test_expired_history_is_not_reused(void)
{
    VL53Median3Filter filter;

    VL53Median3Filter_Reset(&filter);
    CHECK_EQ(1000U, VL53Median3Filter_Update(&filter, 1000U, 0U, 300U));
    CHECK_EQ(1010U, VL53Median3Filter_Update(&filter, 1010U, 100U, 300U));
    CHECK_EQ(1000U, VL53Median3Filter_Update(&filter, 990U, 200U, 300U));

    /* The 401 ms gap exceeds freshness. The first recovered sample must not be
       voted down by the old 1 m history. */
    CHECK_EQ(2000U, VL53Median3Filter_Update(&filter, 2000U, 601U, 300U));
    CHECK_EQ(2100U, VL53Median3Filter_Update(&filter, 2100U, 701U, 300U));
    CHECK_EQ(2100U, VL53Median3Filter_Update(&filter, 5000U, 801U, 300U));
    return 0;
}

static int test_timeout_boundary_and_tick_wrap(void)
{
    VL53Median3Filter filter;

    VL53Median3Filter_Reset(&filter);
    CHECK_EQ(1000U, VL53Median3Filter_Update(&filter, 1000U, UINT32_MAX - 100U, 300U));
    CHECK_EQ(1100U, VL53Median3Filter_Update(&filter, 1100U, 50U, 300U));
    CHECK_EQ(1100U, VL53Median3Filter_Update(&filter, 1200U, 199U, 300U));

    /* Exactly 300 ms remains part of the same window; 301 ms resets it. */
    CHECK_EQ(1200U, VL53Median3Filter_Update(&filter, 1300U, 499U, 300U));
    CHECK_EQ(2000U, VL53Median3Filter_Update(&filter, 2000U, 800U, 300U));
    return 0;
}

int main(void)
{
    if (test_third_sample_is_filtered() != 0) {
        return 1;
    }
    if (test_expired_history_is_not_reused() != 0) {
        return 1;
    }
    if (test_timeout_boundary_and_tick_wrap() != 0) {
        return 1;
    }

    puts("VL53_MEDIAN_FILTER_TEST_PASS");
    return 0;
}
