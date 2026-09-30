#ifndef VL53_GROUND_BOOTSTRAP_H
#define VL53_GROUND_BOOTSTRAP_H

#include <stdint.h>
#include "vl53_types.h"

/*
 * Ground Bootstrap V3
 *
 * The VL53 lens is only about 20-30 mm above the landing surface, below the
 * trusted 50 mm real-range minimum.  On the ground, publish a synthetic 50 mm
 * range so ArduPilot can establish optical-flow relative aiding.
 *
 * V3 deliberately uses only two states:
 *   BOOTSTRAP -> REAL_FLIGHT
 *
 * The transition out of BOOTSTRAP is conservative (>=15 cm for two samples),
 * while returning to BOOTSTRAP is permissive only when there is recent
 * near-ground evidence.  This prevents the observed failure where the module
 * entered real mode while still on the ground and then remained NoData until
 * it was physically moved, without allowing a mid-air range dropout to fake a
 * 5 cm ground range.
 */
#define VL53_GROUND_BOOTSTRAP_DISTANCE_MM       50U
#define VL53_GROUND_BOOTSTRAP_EXIT_MM          150U
#define VL53_GROUND_REAL_CONFIRM_COUNT           2U

/* Two trusted real samples <=10 cm are enough to declare ground contact. */
#define VL53_GROUND_LANDING_NEAR_MM            100U
#define VL53_GROUND_LANDING_CONFIRM_COUNT        2U

/*
 * If REAL_FLIGHT loses usable samples after the last trusted real distance was
 * <=30 cm, three consecutive TOO_CLOSE / NOT_READY updates (~0.3 s at 10 Hz)
 * return to BOOTSTRAP.  Hard transport/API errors and TOO_FAR never fabricate
 * a ground range.
 */
#define VL53_GROUND_NODATA_LAST_VALID_MAX_MM    300U
#define VL53_GROUND_NODATA_CONFIRM_COUNT          3U

typedef enum {
    VL53_RANGE_BOOTSTRAP = 0,
    VL53_RANGE_REAL_FLIGHT = 1
} VL53RangeMode;

typedef struct {
    VL53RangeMode mode;
    uint8_t real_confirm_count;
    uint8_t landing_confirm_count;
    uint8_t nodata_confirm_count;
    uint8_t last_valid_real_present;
    uint8_t bootstrap_evidence_seen;
    uint16_t last_valid_real_mm;
} VL53GroundBootstrap;

void VL53GroundBootstrap_Reset(VL53GroundBootstrap *state);

/*
 * Returns 1 when a range should be published to MAVLink.
 *
 * BOOTSTRAP:
 *   - TOO_CLOSE and VALID readings below 15 cm publish synthetic 5 cm.
 *   - once ground evidence has been seen, transient NOT_READY also keeps the
 *     synthetic 5 cm alive instead of dropping to NoData.
 *   - two VALID readings >=15 cm switch to REAL_FLIGHT and publish real range.
 *   - hard ERROR / TOO_FAR never create synthetic data.
 *
 * REAL_FLIGHT:
 *   - valid ranges publish normally.
 *   - two valid ranges <=10 cm return directly to BOOTSTRAP.
 *   - if the last valid real range was <=30 cm, three consecutive TOO_CLOSE or
 *     NOT_READY updates return to BOOTSTRAP and immediately publish 5 cm.
 *   - if the last valid range was >30 cm, NoData remains NoData; this protects
 *     against sunlight, edges, vegetation and other mid-air dropouts.
 *
 * The cycle can repeat indefinitely without rebooting.
 */
uint8_t VL53GroundBootstrap_Update(VL53GroundBootstrap *state,
                                   VL53ReadResult result,
                                   uint16_t measured_mm,
                                   uint16_t *published_mm,
                                   uint8_t *using_bootstrap);

#endif
