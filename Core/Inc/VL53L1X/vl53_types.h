#ifndef VL53_TYPES_H
#define VL53_TYPES_H

#include <stdint.h>

#define VL53_REAL_MIN_DISTANCE_MM 50U
#define VL53_REAL_MAX_DISTANCE_MM 3600U


typedef enum {
    VL53_READ_NOT_READY = 0,
    VL53_READ_VALID = 1,
    VL53_READ_TOO_CLOSE = 2,
    VL53_READ_TOO_FAR = 3,
    VL53_READ_ERROR = 4
} VL53ReadResult;

#endif
