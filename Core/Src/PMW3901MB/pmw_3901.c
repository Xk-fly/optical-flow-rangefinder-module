/*
 * PMW3901 driver for the STM32G030 optical-flow module.
 *
 * The initialization sequence and bring-up flow are based on the PX4
 * PMW3901 driver (BSD-3-Clause), commit
 * d5839e2dd52ca81e7d15051212f17b7eacce39be. The SPI timing and 0x16
 * Motion Burst transaction are cross-checked against Bitcraze and ArduPilot.
 * See docs/references/PMW3901_OPEN_SOURCE_REVIEW.md.
 */

#include "pmw_3901.h"

#include <stddef.h>

extern UART_HandleTypeDef huart2;
void send_status_text(UART_HandleTypeDef *huart, uint8_t severity, const char *text);

#define PMW3901_PRODUCT_ID_REG        0x00U
#define PMW3901_INVERSE_ID_REG        0x5FU
#define PMW3901_EXPECTED_PRODUCT_ID   0x49U
#define PMW3901_EXPECTED_INVERSE_ID   0xB6U
#define PMW3901_POWER_RESET_REG       0x3AU
#define PMW3901_MOTION_BURST_REG      0x16U
#define PMW3901_REG_MOTION            0x02U
#define PMW3901_REG_DELTA_X_L         0x03U
#define PMW3901_REG_DELTA_X_H         0x04U
#define PMW3901_REG_DELTA_Y_L         0x05U
#define PMW3901_REG_DELTA_Y_H         0x06U
#define PMW3901_REG_SQUAL             0x07U
#define PMW3901_REG_RAW_SUM           0x08U
#define PMW3901_REG_RAW_MAX           0x09U
#define PMW3901_REG_RAW_MIN           0x0AU
#define PMW3901_REG_SHUTTER_LOWER     0x0BU
#define PMW3901_REG_SHUTTER_UPPER     0x0CU
#define PMW3901_REG_OBSERVATION       0x15U
#define PMW3901_BANK_SELECT_REG       0x7FU
#define PMW3901_WRITE_FLAG            0x80U
#define PMW3901_MOTION_VALID_MASK     0x80U
#define PMW3901_MAX_REASONABLE_DELTA  240
#define PMW3901_MAV_SEVERITY_INFO     6U
#define PMW3901_MAV_SEVERITY_ERROR    3U

typedef struct {
    uint8_t reg;
    uint8_t value;
} PMW3901_RegWrite;

static SPI_HandleTypeDef pmw_spi;
static uint8_t pmw_initialized;
static uint8_t pmw_discard_next_burst;
static uint32_t pmw_spi_error_count;
static uint32_t pmw_sample_count;
static uint32_t pmw_motion_count;
static uint32_t pmw_squal_nonzero_count;
static uint32_t pmw_reject_count;
static uint32_t pmw_direct_sample_count;
static uint32_t pmw_direct_motion_count;
static uint32_t pmw_direct_squal_nonzero_count;
static uint8_t pmw_last_status;
static uint8_t pmw_direct_status;
static uint8_t pmw_config_status;
static uint8_t pmw_config_bank0_4d;
static uint8_t pmw_config_bank14_65;
static uint8_t pmw_config_bank15_48;
static uint8_t pmw_config_mismatch_count;
static PMW3901_Motion pmw_last_raw;
static PMW3901_Motion pmw_last_direct;

static const PMW3901_RegWrite pmw_fixed_stage_1[] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
    {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
    {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
    {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
    {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
    {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
    {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
    {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x21},
    {0x75, 0x1F}, {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08},
    {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F},
    {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70},
    {0x7F, 0x15}, {0x48, 0x48},
    {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E},
    {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
    {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}
};

static const PMW3901_RegWrite pmw_fixed_stage_2[] = {
    {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06},
    {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0},
    {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8},
    {0x5A, 0x50}, {0x40, 0x80}, {0x7F, 0x00}
};

static void PMW3901_DelayUs(uint32_t microseconds)
{
    uint32_t ticks_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    uint32_t reload = SysTick->LOAD + 1U;
    uint32_t previous = SysTick->VAL;
    uint32_t elapsed = 0U;
    uint32_t target = microseconds * ticks_per_us;

    if ((ticks_per_us == 0U) || (target == 0U)) {
        return;
    }

    while (elapsed < target) {
        uint32_t current = SysTick->VAL;
        if (previous >= current) {
            elapsed += previous - current;
        } else {
            elapsed += previous + reload - current;
        }
        previous = current;
    }
}

static HAL_StatusTypeDef PMW3901_ReadRegChecked(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;

    if (value == NULL) {
        return HAL_ERROR;
    }

    reg &= (uint8_t)~PMW3901_WRITE_FLAG;
    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);

    status = HAL_SPI_Transmit(&pmw_spi, &reg, 1U, 100U);
    if (status == HAL_OK) {
        PMW3901_DelayUs(50U);
        status = HAL_SPI_Receive(&pmw_spi, value, 1U, 100U);
    }

    PMW3901_DelayUs(20U);
    PMW3901_CS_HIGH();
    PMW3901_DelayUs(200U);

    if (status != HAL_OK) {
        pmw_spi_error_count++;
    }
    return status;
}

static HAL_StatusTypeDef PMW3901_WriteRegChecked(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;

    reg |= PMW3901_WRITE_FLAG;
    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);

    status = HAL_SPI_Transmit(&pmw_spi, &reg, 1U, 100U);
    if (status == HAL_OK) {
        PMW3901_DelayUs(50U);
        status = HAL_SPI_Transmit(&pmw_spi, &value, 1U, 100U);
    }

    PMW3901_DelayUs(20U);
    PMW3901_CS_HIGH();
    PMW3901_DelayUs(200U);

    if (status != HAL_OK) {
        pmw_spi_error_count++;
    }
    return status;
}

static uint8_t PMW3901_WriteSequence(const PMW3901_RegWrite *sequence, size_t count)
{
    size_t index;

    for (index = 0U; index < count; index++) {
        if (PMW3901_WriteRegChecked(sequence[index].reg, sequence[index].value) != HAL_OK) {
            return PMW3901_STATUS_SPI_ERROR;
        }
    }
    return PMW3901_STATUS_OK;
}

static void PMW3901_CaptureConfigReadback(void)
{
    pmw_config_status = PMW3901_STATUS_SPI_ERROR;
    pmw_config_bank0_4d = 0xFFU;
    pmw_config_bank14_65 = 0xFFU;
    pmw_config_bank15_48 = 0xFFU;
    pmw_config_mismatch_count = 0U;

    if ((PMW3901_WriteRegChecked(PMW3901_BANK_SELECT_REG, 0x00U) != HAL_OK) ||
        (PMW3901_ReadRegChecked(0x4DU, &pmw_config_bank0_4d) != HAL_OK) ||
        (PMW3901_WriteRegChecked(PMW3901_BANK_SELECT_REG, 0x14U) != HAL_OK) ||
        (PMW3901_ReadRegChecked(0x65U, &pmw_config_bank14_65) != HAL_OK) ||
        (PMW3901_WriteRegChecked(PMW3901_BANK_SELECT_REG, 0x15U) != HAL_OK) ||
        (PMW3901_ReadRegChecked(0x48U, &pmw_config_bank15_48) != HAL_OK)) {
        (void)PMW3901_WriteRegChecked(PMW3901_BANK_SELECT_REG, 0x00U);
        return;
    }

    if (PMW3901_WriteRegChecked(PMW3901_BANK_SELECT_REG, 0x00U) != HAL_OK) {
        return;
    }
    if (pmw_config_bank0_4d != 0x11U) {
        pmw_config_mismatch_count++;
    }
    if (pmw_config_bank14_65 != 0x67U) {
        pmw_config_mismatch_count++;
    }
    if (pmw_config_bank15_48 != 0x48U) {
        pmw_config_mismatch_count++;
    }
    pmw_config_status = PMW3901_STATUS_OK;
}

static char PMW3901_HexDigit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char)('0' + value) : (char)('A' + value - 10U);
}

static void PMW3901_SendInitStatus(uint8_t product_id, uint8_t inverse_id, uint8_t status)
{
    char message[] = "PMW ID=00 INV=00 RET=0";

    message[7] = PMW3901_HexDigit(product_id >> 4);
    message[8] = PMW3901_HexDigit(product_id);
    message[14] = PMW3901_HexDigit(inverse_id >> 4);
    message[15] = PMW3901_HexDigit(inverse_id);
    message[21] = (char)('0' + (status % 10U));
    send_status_text(&huart2,
                     (status == PMW3901_STATUS_OK) ? PMW3901_MAV_SEVERITY_INFO : PMW3901_MAV_SEVERITY_ERROR,
                     message);
}

uint8_t PMW3901_ReadReg(uint8_t reg)
{
    uint8_t value = 0xFFU;
    (void)PMW3901_ReadRegChecked(reg, &value);
    return value;
}

void PMW3901_WriteReg(uint8_t reg, uint8_t data)
{
    (void)PMW3901_WriteRegChecked(reg, data);
}

uint8_t PMW3901_Performance_Optimization(void)
{
    uint8_t value67;
    uint8_t value73;
    uint8_t c1;
    uint8_t c2;
    uint16_t scaled_c2;
    uint8_t status;

    const PMW3901_RegWrite calibration_prefix[] = {
        {0x7F, 0x00}, {0x55, 0x01}, {0x50, 0x07},
        {0x7F, 0x0E}, {0x43, 0x10}
    };
    const PMW3901_RegWrite calibration_middle[] = {
        {0x7F, 0x00}, {0x51, 0x7B}, {0x50, 0x00},
        {0x55, 0x00}, {0x7F, 0x0E}
    };

    status = PMW3901_WriteSequence(calibration_prefix,
                                   sizeof(calibration_prefix) / sizeof(calibration_prefix[0]));
    if (status != PMW3901_STATUS_OK) {
        return status;
    }

    if (PMW3901_ReadRegChecked(0x67U, &value67) != HAL_OK) {
        return PMW3901_STATUS_SPI_ERROR;
    }
    if (PMW3901_WriteRegChecked(0x48U, (value67 & 0x80U) ? 0x04U : 0x02U) != HAL_OK) {
        return PMW3901_STATUS_SPI_ERROR;
    }

    status = PMW3901_WriteSequence(calibration_middle,
                                   sizeof(calibration_middle) / sizeof(calibration_middle[0]));
    if (status != PMW3901_STATUS_OK) {
        return status;
    }

    if (PMW3901_ReadRegChecked(0x73U, &value73) != HAL_OK) {
        return PMW3901_STATUS_SPI_ERROR;
    }

    if (value73 == 0U) {
        if ((PMW3901_ReadRegChecked(0x70U, &c1) != HAL_OK) ||
            (PMW3901_ReadRegChecked(0x71U, &c2) != HAL_OK)) {
            return PMW3901_STATUS_SPI_ERROR;
        }

        c1 = (c1 <= 28U) ? (uint8_t)(c1 + 14U) : (uint8_t)(c1 + 11U);
        if (c1 > 0x3FU) {
            c1 = 0x3FU;
        }
        scaled_c2 = ((uint16_t)c2 * 45U) / 100U;
        c2 = (uint8_t)scaled_c2;

        if ((PMW3901_WriteRegChecked(0x7FU, 0x00U) != HAL_OK) ||
            (PMW3901_WriteRegChecked(0x61U, 0xADU) != HAL_OK) ||
            (PMW3901_WriteRegChecked(0x51U, 0x70U) != HAL_OK) ||
            (PMW3901_WriteRegChecked(0x7FU, 0x0EU) != HAL_OK) ||
            (PMW3901_WriteRegChecked(0x70U, c1) != HAL_OK) ||
            (PMW3901_WriteRegChecked(0x71U, c2) != HAL_OK)) {
            return PMW3901_STATUS_SPI_ERROR;
        }
    }

    status = PMW3901_WriteSequence(pmw_fixed_stage_1,
                                   sizeof(pmw_fixed_stage_1) / sizeof(pmw_fixed_stage_1[0]));
    if (status != PMW3901_STATUS_OK) {
        return status;
    }

    HAL_Delay(10U);
    return PMW3901_WriteSequence(pmw_fixed_stage_2,
                                 sizeof(pmw_fixed_stage_2) / sizeof(pmw_fixed_stage_2[0]));
}

uint8_t PMW3901_init(SPI_HandleTypeDef spi_ch, uint8_t delay_time)
{
    uint8_t product_id = 0xFFU;
    uint8_t inverse_id = 0xFFU;
    uint8_t discard;
    uint8_t status;
    uint8_t reg;

    (void)delay_time;
    pmw_spi = spi_ch;
    pmw_initialized = 0U;
    pmw_discard_next_burst = 0U;
    pmw_spi_error_count = 0U;
    pmw_sample_count = 0U;
    pmw_motion_count = 0U;
    pmw_squal_nonzero_count = 0U;
    pmw_reject_count = 0U;
    pmw_direct_sample_count = 0U;
    pmw_direct_motion_count = 0U;
    pmw_direct_squal_nonzero_count = 0U;
    pmw_last_status = PMW3901_STATUS_NOT_READY;
    pmw_direct_status = PMW3901_STATUS_NOT_READY;
    pmw_config_status = PMW3901_STATUS_NOT_READY;
    pmw_config_bank0_4d = 0xFFU;
    pmw_config_bank14_65 = 0xFFU;
    pmw_config_bank15_48 = 0xFFU;
    pmw_config_mismatch_count = 0U;
    pmw_last_raw = (PMW3901_Motion){0U};
    pmw_last_direct = (PMW3901_Motion){0U};

    HAL_Delay(40U);
    PMW3901_CS_HIGH();
    HAL_Delay(2U);
    PMW3901_CS_LOW();
    HAL_Delay(2U);
    PMW3901_CS_HIGH();
    HAL_Delay(2U);

    if (PMW3901_WriteRegChecked(PMW3901_POWER_RESET_REG, 0x5AU) != HAL_OK) {
        pmw_last_status = PMW3901_STATUS_SPI_ERROR;
        PMW3901_SendInitStatus(product_id, inverse_id, PMW3901_STATUS_SPI_ERROR);
        return PMW3901_STATUS_SPI_ERROR;
    }
    HAL_Delay(50U);

    if ((PMW3901_ReadRegChecked(PMW3901_PRODUCT_ID_REG, &product_id) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_INVERSE_ID_REG, &inverse_id) != HAL_OK)) {
        pmw_last_status = PMW3901_STATUS_SPI_ERROR;
        PMW3901_SendInitStatus(product_id, inverse_id, PMW3901_STATUS_SPI_ERROR);
        return PMW3901_STATUS_SPI_ERROR;
    }

    if ((product_id != 0x49U) || (inverse_id != 0xB6U)) {
        pmw_last_status = PMW3901_STATUS_BAD_ID;
        PMW3901_SendInitStatus(product_id, inverse_id, PMW3901_STATUS_BAD_ID);
        return PMW3901_STATUS_BAD_ID;
    }

    for (reg = 0x02U; reg <= 0x06U; reg++) {
        if (PMW3901_ReadRegChecked(reg, &discard) != HAL_OK) {
            pmw_last_status = PMW3901_STATUS_SPI_ERROR;
            PMW3901_SendInitStatus(product_id, inverse_id, PMW3901_STATUS_SPI_ERROR);
            return PMW3901_STATUS_SPI_ERROR;
        }
    }
    HAL_Delay(1U);

    status = PMW3901_Performance_Optimization();
    if (status == PMW3901_STATUS_OK) {
        PMW3901_CaptureConfigReadback();
        pmw_initialized = 1U;
        pmw_discard_next_burst = 1U;
    }
    pmw_last_status = status;
    PMW3901_SendInitStatus(product_id, inverse_id, status);
    return status;
}

uint8_t PMW3901_ReadMotionBurst(PMW3901_Motion *motion)
{
    uint8_t burst_address = 0x16U;
    uint8_t raw[12] = {0U};
    HAL_StatusTypeDef status;

    if ((motion == NULL) || (pmw_initialized == 0U)) {
        pmw_last_status = PMW3901_STATUS_NOT_READY;
        return PMW3901_STATUS_NOT_READY;
    }

    PMW3901_CS_LOW();
    PMW3901_DelayUs(5U);
    status = HAL_SPI_Transmit(&pmw_spi, &burst_address, 1U, 100U);
    if (status == HAL_OK) {
        PMW3901_DelayUs(150U);
        status = HAL_SPI_Receive(&pmw_spi, raw, sizeof(raw), 100U);
    }
    PMW3901_DelayUs(20U);
    PMW3901_CS_HIGH();
    PMW3901_DelayUs(50U);

    if (status != HAL_OK) {
        pmw_spi_error_count++;
        pmw_last_status = PMW3901_STATUS_SPI_ERROR;
        return PMW3901_STATUS_SPI_ERROR;
    }

    motion->motion = raw[0];
    motion->observation = raw[1];
    motion->delta_x = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    motion->delta_y = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    motion->squal = raw[6];
    motion->raw_sum = raw[7];
    motion->raw_max = raw[8];
    motion->raw_min = raw[9];
    motion->shutter = (uint16_t)(((uint16_t)raw[10] << 8) | raw[11]);

    if (pmw_discard_next_burst != 0U) {
        pmw_discard_next_burst = 0U;
        pmw_last_raw = *motion;
        pmw_sample_count++;
        motion->delta_x = 0;
        motion->delta_y = 0;
        motion->squal = 0U;
        pmw_last_status = PMW3901_STATUS_DISCARDED;
        return PMW3901_STATUS_DISCARDED;
    }

    pmw_last_raw = *motion;
    pmw_sample_count++;
    if ((motion->motion & PMW3901_MOTION_VALID_MASK) != 0U) {
        pmw_motion_count++;
    }
    if (motion->squal != 0U) {
        pmw_squal_nonzero_count++;
    }

    if ((motion->delta_x > PMW3901_MAX_REASONABLE_DELTA) ||
        (motion->delta_x < -PMW3901_MAX_REASONABLE_DELTA) ||
        (motion->delta_y > PMW3901_MAX_REASONABLE_DELTA) ||
        (motion->delta_y < -PMW3901_MAX_REASONABLE_DELTA)) {
        motion->delta_x = 0;
        motion->delta_y = 0;
        motion->squal = 0U;
        pmw_reject_count++;
        pmw_last_status = PMW3901_STATUS_CONFIG_ERROR;
        return PMW3901_STATUS_CONFIG_ERROR;
    }

    pmw_last_status = PMW3901_STATUS_OK;
    return PMW3901_STATUS_OK;
}

uint8_t PMW3901_ReadDirectSnapshot(void)
{
    PMW3901_Motion direct = {0U};
    uint8_t x_l;
    uint8_t x_h;
    uint8_t y_l;
    uint8_t y_h;
    uint8_t shutter_l;
    uint8_t shutter_h;

    if (pmw_initialized == 0U) {
        pmw_direct_status = PMW3901_STATUS_NOT_READY;
        return pmw_direct_status;
    }

    if ((PMW3901_WriteRegChecked(PMW3901_BANK_SELECT_REG, 0x00U) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_MOTION, &direct.motion) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_DELTA_X_L, &x_l) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_DELTA_X_H, &x_h) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_DELTA_Y_L, &y_l) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_DELTA_Y_H, &y_h) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_SQUAL, &direct.squal) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_RAW_SUM, &direct.raw_sum) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_RAW_MAX, &direct.raw_max) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_RAW_MIN, &direct.raw_min) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_SHUTTER_LOWER, &shutter_l) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_SHUTTER_UPPER, &shutter_h) != HAL_OK) ||
        (PMW3901_ReadRegChecked(PMW3901_REG_OBSERVATION, &direct.observation) != HAL_OK)) {
        pmw_direct_status = PMW3901_STATUS_SPI_ERROR;
        return pmw_direct_status;
    }

    direct.delta_x = (int16_t)((uint16_t)x_l | ((uint16_t)x_h << 8));
    direct.delta_y = (int16_t)((uint16_t)y_l | ((uint16_t)y_h << 8));
    direct.shutter = (uint16_t)(((uint16_t)shutter_h << 8) | shutter_l);
    pmw_last_direct = direct;
    pmw_direct_sample_count++;
    if (((direct.motion & PMW3901_MOTION_VALID_MASK) != 0U) ||
        (direct.delta_x != 0) || (direct.delta_y != 0)) {
        pmw_direct_motion_count++;
    }
    if (direct.squal != 0U) {
        pmw_direct_squal_nonzero_count++;
    }
    pmw_direct_status = PMW3901_STATUS_OK;
    return pmw_direct_status;
}

uint32_t PMW3901_GetSpiErrorCount(void)
{
    return pmw_spi_error_count;
}

void PMW3901_GetDiagnostics(PMW3901_Diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->last_raw = pmw_last_raw;
    diagnostics->last_direct = pmw_last_direct;
    diagnostics->last_status = pmw_last_status;
    diagnostics->direct_status = pmw_direct_status;
    diagnostics->config_status = pmw_config_status;
    diagnostics->config_bank0_4d = pmw_config_bank0_4d;
    diagnostics->config_bank14_65 = pmw_config_bank14_65;
    diagnostics->config_bank15_48 = pmw_config_bank15_48;
    diagnostics->config_mismatch_count = pmw_config_mismatch_count;
    diagnostics->sample_count = pmw_sample_count;
    diagnostics->direct_sample_count = pmw_direct_sample_count;
    diagnostics->motion_count = pmw_motion_count;
    diagnostics->squal_nonzero_count = pmw_squal_nonzero_count;
    diagnostics->direct_motion_count = pmw_direct_motion_count;
    diagnostics->direct_squal_nonzero_count = pmw_direct_squal_nonzero_count;
    diagnostics->spi_error_count = pmw_spi_error_count;
    diagnostics->reject_count = pmw_reject_count;
}

void pollingMotionDiag(int *x,
                       int *y,
                       uint8_t *q,
                       uint8_t *motion_reg,
                       uint8_t *squal_reg,
                       uint16_t scale_per_mille)
{
    PMW3901_Motion motion = {0U};
    uint8_t status;

    if ((x == NULL) || (y == NULL) || (q == NULL)) {
        return;
    }

    status = PMW3901_ReadMotionBurst(&motion);
    if ((status == PMW3901_STATUS_OK) && ((motion.motion & PMW3901_MOTION_VALID_MASK) != 0U)) {
        *x = (int32_t)motion.delta_x * scale_per_mille / 1000;
        *y = (int32_t)motion.delta_y * scale_per_mille / 1000;
    } else {
        *x = 0;
        *y = 0;
    }

    *q = (status == PMW3901_STATUS_OK) ? motion.squal : 0U;
    if (motion_reg != NULL) {
        *motion_reg = motion.motion;
    }
    if (squal_reg != NULL) {
        *squal_reg = motion.squal;
    }
}

void PMW3901_ReadMotionBurstDiag(int *x,
                                 int *y,
                                 uint8_t *q,
                                 uint8_t *motion_reg,
                                 uint8_t *squal_reg,
                                 uint16_t scale_per_mille)
{
    pollingMotionDiag(x, y, q, motion_reg, squal_reg, scale_per_mille);
}

void pollingMotion(int *x, int *y, uint8_t *q, uint16_t scale_per_mille)
{
    PMW3901_Motion motion = {0U};
    uint8_t status;

    if ((x == NULL) || (y == NULL) || (q == NULL)) {
        return;
    }

#if PMW3901_ACQUISITION_MODE == PMW3901_ACQ_DIRECT
    status = PMW3901_ReadDirectSnapshot();
    motion = pmw_last_direct;
#else
    status = PMW3901_ReadMotionBurst(&motion);
#endif

    if ((status == PMW3901_STATUS_OK) &&
        ((motion.motion & PMW3901_MOTION_VALID_MASK) != 0U)) {
        *x = (int32_t)motion.delta_x * scale_per_mille / 1000;
        *y = (int32_t)motion.delta_y * scale_per_mille / 1000;
    } else {
        *x = 0;
        *y = 0;
    }
    *q = (status == PMW3901_STATUS_OK) ? motion.squal : 0U;
}