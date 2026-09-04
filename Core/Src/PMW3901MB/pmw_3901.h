#pragma once

#include "main.h"

#define PMW3901_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define PMW3901_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define PMW3901_ACQ_BURST  0U
#define PMW3901_ACQ_DIRECT 1U

#ifndef PMW3901_ACQUISITION_MODE
#define PMW3901_ACQUISITION_MODE PMW3901_ACQ_BURST
#endif

#if (PMW3901_ACQUISITION_MODE != PMW3901_ACQ_BURST) && \
    (PMW3901_ACQUISITION_MODE != PMW3901_ACQ_DIRECT)
#error "Unsupported PMW3901 acquisition mode"
#endif

typedef enum {
	PMW3901_STATUS_OK = 0U,
	PMW3901_STATUS_SPI_ERROR = 1U,
	PMW3901_STATUS_BAD_ID = 2U,
	PMW3901_STATUS_CONFIG_ERROR = 3U,
	PMW3901_STATUS_NOT_READY = 4U,
	PMW3901_STATUS_DISCARDED = 5U
} PMW3901_Status;

typedef struct {
	uint8_t motion;
	uint8_t observation;
	int16_t delta_x;
	int16_t delta_y;
	uint8_t squal;
	uint8_t raw_sum;
	uint8_t raw_max;
	uint8_t raw_min;
	uint16_t shutter;
} PMW3901_Motion;

typedef struct {
	PMW3901_Motion last_raw;
	PMW3901_Motion last_direct;
	uint8_t last_status;
	uint8_t direct_status;
	uint8_t config_status;
	uint8_t config_bank0_4d;
	uint8_t config_bank14_65;
	uint8_t config_bank15_48;
	uint8_t config_mismatch_count;
	uint32_t sample_count;
	uint32_t direct_sample_count;
	uint32_t motion_count;
	uint32_t squal_nonzero_count;
	uint32_t direct_motion_count;
	uint32_t direct_squal_nonzero_count;
	uint32_t spi_error_count;
	uint32_t reject_count;
} PMW3901_Diagnostics;

uint8_t PMW3901_ReadReg(uint8_t reg);
void PMW3901_WriteReg(uint8_t reg, uint8_t data);
uint8_t PMW3901_init(SPI_HandleTypeDef spi_ch,uint8_t delay_time);
uint8_t  PMW3901_Performance_Optimization();
uint8_t PMW3901_ReadMotionBurst(PMW3901_Motion *motion);
uint8_t PMW3901_ReadDirectSnapshot(void);
uint32_t PMW3901_GetSpiErrorCount(void);
void PMW3901_GetDiagnostics(PMW3901_Diagnostics *diagnostics);
void pollingMotion(int * x, int * y, uint8_t * r07,uint16_t scale_per_mille);
void pollingMotionDiag(int *x, int *y, uint8_t *q, uint8_t *motion_reg, uint8_t *squal_reg, uint16_t scale_per_mille);
void PMW3901_ReadMotionBurstDiag(int *x, int *y, uint8_t *q, uint8_t *motion_reg, uint8_t *squal_reg, uint16_t scale_per_mille);
