#include "vl53l1x.h"
#include "vl53_median_filter.h"

#define VL53_I2C_ADDRESS      0x52U
#define VL53_BOOT_TIMEOUT_MS  1000U

static VL53Median3Filter vl53_distance_filter;

uint8_t vl53_Init(void)
{
    uint8_t sensor_state = 0U;
    uint32_t started_ms = HAL_GetTick();

    VL53Median3Filter_Reset(&vl53_distance_filter);

    while (sensor_state == 0U) {
        if (VL53L1X_BootState(VL53_I2C_ADDRESS, &sensor_state) != VL53L1X_ERROR_NONE) {
            return 0U;
        }
        if ((HAL_GetTick() - started_ms) >= VL53_BOOT_TIMEOUT_MS) {
            return 0U;
        }
        HAL_Delay(2U);
    }

    if (VL53L1X_SensorInit(VL53_I2C_ADDRESS) != VL53L1X_ERROR_NONE) {
        return 0U;
    }
    if (VL53L1X_SetDistanceMode(VL53_I2C_ADDRESS, 2U) != VL53L1X_ERROR_NONE) {
        return 0U;
    }
    if (VL53L1X_SetTimingBudgetInMs(VL53_I2C_ADDRESS, 100U) != VL53L1X_ERROR_NONE) {
        return 0U;
    }
    if (VL53L1X_SetInterMeasurementInMs(VL53_I2C_ADDRESS, 100U) != VL53L1X_ERROR_NONE) {
        return 0U;
    }
    if (VL53L1X_SetInterruptPolarity(VL53_I2C_ADDRESS, 0U) != VL53L1X_ERROR_NONE) {
        return 0U;
    }
    if (VL53L1X_StartRanging(VL53_I2C_ADDRESS) != VL53L1X_ERROR_NONE) {
        return 0U;
    }

    return 1U;
}

VL53ReadResult vl53_GetDistance(uint16_t *distance_mm)
{
    uint8_t data_ready = 0U;
    uint16_t measurement_mm = 0U;

    if (distance_mm == NULL) {
        return VL53_READ_ERROR;
    }
    if (VL53L1X_CheckForDataReady(VL53_I2C_ADDRESS, &data_ready) != VL53L1X_ERROR_NONE) {
        return VL53_READ_ERROR;
    }
    if (data_ready == 0U) {
        return VL53_READ_NOT_READY;
    }
    if (VL53L1X_GetDistance(VL53_I2C_ADDRESS, &measurement_mm) != VL53L1X_ERROR_NONE) {
        (void)VL53L1X_ClearInterrupt(VL53_I2C_ADDRESS);
        return VL53_READ_ERROR;
    }
    if (VL53L1X_ClearInterrupt(VL53_I2C_ADDRESS) != VL53L1X_ERROR_NONE) {
        return VL53_READ_ERROR;
    }

    /*
     * Preserve the known-good policy of not gating on RangeStatus, but keep
     * "too close", "too far" and transport/API errors distinct.  This lets
     * the ground-bootstrap layer substitute a value only for the physically
     * expected near-field case and never hide a sensor/communication fault.
     */
    *distance_mm = measurement_mm;
    if (measurement_mm < VL53_REAL_MIN_DISTANCE_MM) {
        return VL53_READ_TOO_CLOSE;
    }
    if (measurement_mm > VL53_REAL_MAX_DISTANCE_MM) {
        return VL53_READ_TOO_FAR;
    }

    *distance_mm = VL53Median3Filter_Update(&vl53_distance_filter,
                                            measurement_mm,
                                            HAL_GetTick(),
                                            VL53_DISTANCE_FRESH_TIMEOUT_MS);
    return VL53_READ_VALID;
}
