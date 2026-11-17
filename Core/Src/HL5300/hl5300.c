#include "HL5300/hl5300.h"

#include <stdbool.h>

#define HL5300_ADDRESS (0x6CU << 1)
#define HL5300_IO_TIMEOUT_MS 6U
#define HL5300_FIRMWARE_SIZE 7679U
#define HL5300_PAGE_SIZE 32U

#define ARRAY_COUNT(a) (sizeof(a) / sizeof((a)[0]))

enum {
    REG_MCU_CONFIG = 0x00,
    REG_SYSTEM_CONFIG = 0x01,
    REG_CPU_STATUS = 0x02,
    REG_INTERRUPT_STATUS = 0x03,
    REG_DEVICE_ADDRESS = 0x06,
    REG_POWER = 0x07,
    REG_FIRMWARE_STATUS = 0x08,
    REG_COMMAND = 0x0A,
    REG_SIZE = 0x0B,
    REG_SCRATCH = 0x0C,
    REG_AO_DOMAIN = 0x3B,
    COMMAND_WRITE_FIRMWARE = 0x03,
    COMMAND_WRITE_CONFIG = 0x09,
    COMMAND_CK_CALIBRATION = 0x0B,
    COMMAND_CONTINUOUS_RANGE = 0x0F,
    INTERRUPT_DATA_READY = 0x01,
};

typedef struct {
    uint8_t reg;
    uint8_t value;
} RegisterWrite;

extern const uint8_t hl5300_firmware[HL5300_FIRMWARE_SIZE];

static bool read_registers(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(bus, HL5300_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                            data, length, HL5300_IO_TIMEOUT_MS) == HAL_OK;
}

static bool write_registers(I2C_HandleTypeDef *bus, uint8_t reg,
                            const uint8_t *data, uint16_t length)
{
    /* STM32 HAL's synchronous transmit API does not modify the source buffer. */
    return HAL_I2C_Mem_Write(bus, HL5300_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data, length, HL5300_IO_TIMEOUT_MS) == HAL_OK;
}

static bool write_register(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t value)
{
    return write_registers(bus, reg, &value, 1U);
}

static bool write_sequence(I2C_HandleTypeDef *bus, const RegisterWrite *sequence, size_t count)
{
    for (size_t index = 0; index < count; index++) {
        if (!write_register(bus, sequence[index].reg, sequence[index].value)) {
            return false;
        }
    }
    return true;
}

static bool wake_clock(I2C_HandleTypeDef *bus)
{
    if (!write_register(bus, REG_POWER, 0x0F) ||
        !write_register(bus, REG_POWER, 0x0E)) {
        return false;
    }
    HAL_Delay(5);
    return true;
}

static bool init_registers(I2C_HandleTypeDef *bus)
{
    /* Register values from the vendor's F030 7 m demo, internal oscillator and power management. */
    static const RegisterWrite sequence[] = {
        {REG_SYSTEM_CONFIG, 0x0C}, {REG_POWER, 0x00}, {REG_POWER, 0x01},
        {REG_POWER, 0x00}, {0x04, 0x21}, {0x05, 0x0E}, {REG_FIRMWARE_STATUS, 0x00},
        {0x37, 0x80}, {0x38, 0x30}, {0x39, 0x00}, {0x3A, 0x30}, {REG_AO_DOMAIN, 0x80},
        {0x3C, 0x80}, {0x3D, 0x80}, {0x3E, 0x00}, {0x3F, 0x00},
        {REG_POWER, 0x0E}, {REG_POWER, 0x0F},
    };
    if (!write_sequence(bus, sequence, ARRAY_COUNT(sequence))) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < 20U; attempt++) {
        uint8_t status;
        HAL_Delay(1);
        if (!read_registers(bus, REG_CPU_STATUS, &status, 1U)) {
            return false;
        }
        if ((status & 0x01U) == 0U) {
            return true;
        }
    }
    return false;
}

static bool firmware_ready(I2C_HandleTypeDef *bus)
{
    uint8_t status;
    return read_registers(bus, REG_FIRMWARE_STATUS, &status, 1U) &&
           (status == 0x66U || status == 0xAAU);
}

static bool download_firmware(I2C_HandleTypeDef *bus)
{
    static const RegisterWrite before_download[] = {
        {REG_POWER, 0x08}, {REG_POWER, 0x0A}, {REG_MCU_CONFIG, 0x06},
    };
    static const RegisterWrite after_download[] = {
        {REG_SYSTEM_CONFIG, 0x0C}, {REG_MCU_CONFIG, 0x06},
        {REG_AO_DOMAIN, 0xA0}, {REG_AO_DOMAIN, 0x80}, {REG_MCU_CONFIG, 0x07},
        {REG_POWER, 0x02}, {REG_POWER, 0x00},
    };
    uint8_t system_config;
    if (!wake_clock(bus) ||
        !write_sequence(bus, before_download, ARRAY_COUNT(before_download)) ||
        !read_registers(bus, REG_SYSTEM_CONFIG, &system_config, 1U) ||
        !write_register(bus, REG_SYSTEM_CONFIG, system_config | 0x01U) ||
        !write_register(bus, REG_COMMAND, 0x01) || !write_register(bus, REG_SIZE, 2U) ||
        !write_register(bus, REG_SCRATCH, 0U) || !write_register(bus, REG_SCRATCH + 1U, 0U)) {
        return false;
    }

    for (uint16_t offset = 0; offset < HL5300_FIRMWARE_SIZE; offset += HL5300_PAGE_SIZE) {
        const uint16_t remaining = HL5300_FIRMWARE_SIZE - offset;
        const uint8_t length = remaining < HL5300_PAGE_SIZE ? remaining : HL5300_PAGE_SIZE;
        const uint8_t command[] = {COMMAND_WRITE_FIRMWARE, length};
        if (!write_registers(bus, REG_COMMAND, command, sizeof(command)) ||
            !write_registers(bus, REG_SCRATCH, &hl5300_firmware[offset], length)) {
            return false;
        }
        /* The vendor demo leaves a 1 ms gap after each full page. */
        if (length == HL5300_PAGE_SIZE) {
            HAL_Delay(1);
        }
    }

    if (!write_sequence(bus, after_download, ARRAY_COUNT(after_download))) {
        return false;
    }
    HAL_Delay(100);
    return firmware_ready(bus);
}

/* This payload contains direction, length, parameter offset, then parameter bytes. */
static bool set_config(I2C_HandleTypeDef *bus, const uint8_t *data, uint16_t length)
{
    return wake_clock(bus) && write_registers(bus, REG_SCRATCH, data, length) &&
           write_register(bus, REG_COMMAND, COMMAND_WRITE_CONFIG);
}

static bool calibrate_clock(I2C_HandleTypeDef *bus)
{
    const uint8_t disable_temperature[] = {0x01, 0x01, 0x0E, 0x00};
    uint8_t status;
    if (!set_config(bus, disable_temperature, sizeof(disable_temperature))) {
        return false;
    }
    HAL_Delay(5);
    if (!wake_clock(bus) || !read_registers(bus, REG_INTERRUPT_STATUS, &status, 1U) ||
        !write_register(bus, REG_COMMAND, COMMAND_CK_CALIBRATION)) {
        return false;
    }

    for (uint16_t attempt = 0; attempt < 300U; attempt++) {
        HAL_Delay(10);
        if (!read_registers(bus, REG_INTERRUPT_STATUS, &status, 1U)) {
            return false;
        }
        if ((status & INTERRUPT_DATA_READY) != 0U) {
            uint8_t phase[2];
            HAL_Delay(10);
            if (!read_registers(bus, REG_SCRATCH, phase, sizeof(phase)) || phase[1] != 0xBBU) {
                return false;
            }
            const uint8_t clock_phase[] = {0x01, 0x01, 0x1B, phase[0]};
            return set_config(bus, clock_phase, sizeof(clock_phase));
        }
    }
    return false;
}

const char *hl5300_init(I2C_HandleTypeDef *bus)
{
    uint8_t identity;
    HAL_Delay(10);
    if (!read_registers(bus, REG_DEVICE_ADDRESS, &identity, 1U) || identity != HL5300_ADDRESS) {
        return "HL5300 probe failed (I2C 0x6C)";
    }
    if (!init_registers(bus)) {
        return "HL5300 register init failed";
    }
    if (!download_firmware(bus)) {
        return "HL5300 firmware download failed";
    }
    if (!calibrate_clock(bus)) {
        return "HL5300 CK calibration failed";
    }

    /* Vendor demo: 50 Hz and 120000 integration counts. Delay count is 0x0014. */
    const uint8_t integration[] = {0x01, 0x03, 0x01, 0xC0, 0xD4, 0x01};
    const uint8_t frame_delay[] = {0x01, 0x02, 0x04, 0x00, 0x14};
    uint8_t status;
    if (!set_config(bus, integration, sizeof(integration)) ||
        !set_config(bus, frame_delay, sizeof(frame_delay)) ||
        !read_registers(bus, REG_INTERRUPT_STATUS, &status, 1U) ||
        !firmware_ready(bus) || !wake_clock(bus) ||
        !write_register(bus, REG_COMMAND, COMMAND_CONTINUOUS_RANGE)) {
        return "HL5300 continuous start failed";
    }
    return NULL;
}

HL5300ReadResult hl5300_read_distance(I2C_HandleTypeDef *bus, int16_t *raw_mm)
{
    uint8_t status;
    if (!read_registers(bus, REG_INTERRUPT_STATUS, &status, 1U)) {
        return HL5300_IO_ERROR;
    }
    if ((status & INTERRUPT_DATA_READY) == 0U) {
        return HL5300_NO_DATA;
    }

    uint8_t sample[32];
    if (!read_registers(bus, REG_SCRATCH, sample, sizeof(sample))) {
        return HL5300_IO_ERROR;
    }

    /* Vendor Get_Measure1_Data: bytes 12 and 13 contain signed raw ToF in mm. */
    const uint16_t encoded = (uint16_t)sample[12] | ((uint16_t)sample[13] << 8U);
    *raw_mm = (int16_t)encoded;
    if (*raw_mm < HL5300_MIN_DISTANCE_MM || *raw_mm > HL5300_MAX_DISTANCE_MM) {
        return HL5300_OUT_OF_RANGE;
    }
    return HL5300_VALID;
}
