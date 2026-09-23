#ifndef VL53_GROUND_BOOTSTRAP_H
#define VL53_GROUND_BOOTSTRAP_H

#include <stdint.h>
#include "vl53_types.h"

/*
 * Ground Bootstrap V2
 *
 * The physical VL53 lens is only about 20-30 mm above the landing surface,
 * below the 50 mm trusted measurement minimum.  Publish 50 mm while on the
 * ground so ArduPilot can establish optical-flow relative aiding, then move
 * to real range after takeoff.  Unlike V1, V2 deliberately supports repeated
 * takeoff -> flight -> landing -> bootstrap cycles without rebooting.
 */
#define VL53_GROUND_BOOTSTRAP_DISTANCE_MM       50U
#define VL53_GROUND_BOOTSTRAP_EXIT_MM           80U
#define VL53_GROUND_REAL_CONFIRM_COUNT           2U

/* A real flight must first clear 25 cm before a later landing can re-arm. */
#define VL53_GROUND_AIRBORNE_CLEAR_MM          250U

/*
 * On descent, a trusted real sample <=20 cm arms landing detection.
 * Two near-ground observations at <=8 cm (or TOO_CLOSE) then return to the
 * 5 cm bootstrap state.  At the 10 Hz range loop this is about 0.2 s.
 */
#define VL53_GROUND_LANDING_ARM_MM             200U
#define VL53_GROUND_LANDING_NEAR_MM             80U
#define VL53_GROUND_LANDING_CONFIRM_COUNT        2U

/*
 * TOO_CLOSE is allowed to confirm landing only while a recent real low-altitude
 * sample exists.  Five 10 Hz updates correspond to about 0.5 s.
 */
#define VL53_GROUND_LAST_VALID_MAX_AGE_UPDATES   5U

typedef enum {
    VL53_RANGE_BOOTSTRAP = 0,
    VL53_RANGE_REAL_FLIGHT = 1,
    VL53_RANGE_LANDING_CONFIRM = 2
} VL53RangeMode;

typedef struct {
    VL53RangeMode mode;
    uint8_t real_confirm_count;
    uint8_t landing_confirm_count;
    uint8_t airborne_clear_seen;
    uint8_t landing_armed;
    uint8_t updates_since_valid;
    uint16_t last_valid_real_mm;
} VL53GroundBootstrap;

void VL53GroundBootstrap_Reset(VL53GroundBootstrap *state);

/*
 * Returns 1 when a range should be published to MAVLink.
 *
 * BOOTSTRAP:
 *   - TOO_CLOSE and VALID readings below 80 mm publish synthetic 50 mm.
 *   - two VALID readings >=80 mm switch to REAL_FLIGHT and publish real range.
 *   - hard ERROR / TOO_FAR never create synthetic data.
 *
 * REAL_FLIGHT:
 *   - real ranges publish normally.
 *   - once >=250 mm has been seen, a later valid range <=200 mm arms landing.
 *   - near-ground readings then enter LANDING_CONFIRM.
 *   - mid-air TOO_CLOSE without recent low-altitude evidence cannot bootstrap.
 *
 * LANDING_CONFIRM:
 *   - two near-ground observations (<=80 mm or TOO_CLOSE) within recent-valid
 *     evidence switch back to BOOTSTRAP and immediately publish 50 mm.
 *   - climbing back above the near-ground window cancels confirmation.
 *
 * This cycle can repeat indefinitely during one power-on session.
 */
uint8_t VL53GroundBootstrap_Update(VL53GroundBootstrap *state,
                                   VL53ReadResult result,
                                   uint16_t measured_mm,
                                   uint16_t *published_mm,
                                   uint8_t *using_bootstrap);

#endif
