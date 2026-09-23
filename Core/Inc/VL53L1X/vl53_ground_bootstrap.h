#ifndef VL53_GROUND_BOOTSTRAP_H
#define VL53_GROUND_BOOTSTRAP_H

#include <stdint.h>
#include "vl53_types.h"

/*
 * Use exactly the first physically trusted VL53 distance (5 cm) as the
 * synthetic ground-start value.  This matches the MAVLink DISTANCE_SENSOR
 * advertised minimum and ArduPilot RNGFND1_MIN_CM=5.
 */
#define VL53_GROUND_BOOTSTRAP_DISTANCE_MM 50U
#define VL53_GROUND_BOOTSTRAP_EXIT_MM 60U
#define VL53_GROUND_BOOTSTRAP_CONFIRM_COUNT 2U

typedef enum {
    VL53_RANGE_BOOTSTRAP = 0,
    VL53_RANGE_REAL_LOCKED = 1
} VL53RangeMode;

typedef struct {
    VL53RangeMode mode;
    uint8_t real_confirm_count;
} VL53GroundBootstrap;

void VL53GroundBootstrap_Reset(VL53GroundBootstrap *state);

/*
 * Returns 1 when a distance should be published to MAVLink.
 *
 * Before real range is locked:
 *   - TOO_CLOSE publishes the synthetic 50 mm ground value.
 *   - VALID readings >= 60 mm must be seen twice consecutively before the
 *     state permanently locks to real range.  Until then, 50 mm is published.
 *   - NOT_READY publishes nothing and does not destroy a confirmation streak.
 *   - TOO_FAR/ERROR publish nothing and clear a confirmation streak.
 *
 * After real range is locked, only VALID readings are published.  The state
 * never falls back to synthetic range until the module is rebooted.
 */
uint8_t VL53GroundBootstrap_Update(VL53GroundBootstrap *state,
                                   VL53ReadResult result,
                                   uint16_t measured_mm,
                                   uint16_t *published_mm,
                                   uint8_t *using_bootstrap);

#endif
