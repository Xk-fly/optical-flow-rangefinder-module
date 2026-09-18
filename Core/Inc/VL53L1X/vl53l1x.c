#include "vl53l1x.h"

#define VL53_I2C_ADDRESS      0x52U
#define VL53_BOOT_TIMEOUT_MS  1000U
#define VL53_MIN_DISTANCE_MM  50U
#define VL53_MAX_DISTANCE_MM  3600U

static uint16_t vl53_distance_window[3] = {0U, 0U, 0U};
static uint8_t vl53_distance_window_count = 0U;
static uint8_t vl53_distance_window_index = 0U;

static uint16_t vl53_median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) {
        uint16_t t = a;
        a = b;
        b = t;
    }
    if (b > c) {
        uint16_t t = b;
        b = c;
        c = t;
    }
    if (a > b) {
        uint16_t t = a;
        a = b;
        b = t;
    }
    return b;
}

uint8_t vl53_Init(void)
{
    uint8_t sensor_state = 0U;
    uint32_t started_ms = HAL_GetTick();

    vl53_distance_window_count = 0U;
    vl53_distance_window_index = 0U;
    vl53_distance_window[0] = 0U;
    vl53_distance_window[1] = 0U;
    vl53_distance_window[2] = 0U;

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

uint8_t vl53_GetDistance(uint16_t *distance_mm)
{
    uint8_t data_ready = 0U;
    uint16_t measurement_mm = 0U;

    if (distance_mm == NULL) {
        return 0U;
    }
    if (VL53L1X_CheckForDataReady(VL53_I2C_ADDRESS, &data_ready) != VL53L1X_ERROR_NONE) {
        return 0U;
    }
    if (data_ready == 0U) {
        return 0U;
    }
    if (VL53L1X_GetDistance(VL53_I2C_ADDRESS, &measurement_mm) != VL53L1X_ERROR_NONE) {
        (void)VL53L1X_ClearInterrupt(VL53_I2C_ADDRESS);
        return 0U;
    }
    if (VL53L1X_ClearInterrupt(VL53_I2C_ADDRESS) != VL53L1X_ERROR_NONE) {
        return 0U;
    }

    /* Keep the Known-Good behavior: do not gate on RangeStatus.
       Only reject values outside the physical range advertised to ArduPilot. */
    if ((measurement_mm < VL53_MIN_DISTANCE_MM) ||
        (measurement_mm > VL53_MAX_DISTANCE_MM)) {
        return 0U;
    }

    vl53_distance_window[vl53_distance_window_index] = measurement_mm;
    vl53_distance_window_index = (uint8_t)((vl53_distance_window_index + 1U) % 3U);

    if (vl53_distance_window_count < 3U) {
        vl53_distance_window_count++;
        *distance_mm = measurement_mm;
    } else {
        *distance_mm = vl53_median3(vl53_distance_window[0],
                                    vl53_distance_window[1],
                                    vl53_distance_window[2]);
    }

    return 1U;
}
