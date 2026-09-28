#include "mpu6050.h"
#include <math.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

MPU6050::~MPU6050()
{
    if (device_) i2c_master_bus_rm_device(device_);
}

esp_err_t MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    if (!device_) return ESP_ERR_INVALID_STATE;
    const uint8_t data[2] = {reg, value};
    return i2c_master_transmit(device_, data, sizeof(data), 100);
}

esp_err_t MPU6050::readRegisters(uint8_t reg, uint8_t *data, size_t size)
{
    if (!device_) return ESP_ERR_INVALID_STATE;
    if (!data || size == 0) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(device_, &reg, 1, data, size, 100);
}

bool MPU6050::isConnected()
{
    uint8_t who_am_i = 0;
    if (readRegisters(0x75, &who_am_i, 1) != ESP_OK) return false;
    return who_am_i == 0x68 || who_am_i == 0x70 ||
           who_am_i == 0x71 || who_am_i == 0x72;
}

bool MPU6050::begin(i2c_master_bus_handle_t bus)
{
    if (!bus) return false;
    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address_,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &config, &device_) != ESP_OK) return false;
    writeRegister(0x6b, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!isConnected()) return false;
    if (writeRegister(0x6b, 0x01) != ESP_OK) return false;
    writeRegister(0x19, 0x07);
    writeRegister(0x1a, 0x02);
    writeRegister(0x1b, 0x00);
    writeRegister(0x1c, 0x00);
    initialized_ = true;
    last_us_ = esp_timer_get_time();
    return true;
}

bool MPU6050::calibrate(uint16_t samples)
{
    if (!initialized_ || samples == 0) return false;
    double sum[3] = {};
    uint8_t data[14];
    for (uint16_t i = 0; i < samples; ++i) {
        if (readRegisters(0x3b, data, sizeof(data)) != ESP_OK) return false;
        for (int axis = 0; axis < 3; ++axis) {
            const int offset = 8 + axis * 2;
            sum[axis] += (int16_t)((data[offset] << 8) | data[offset + 1]);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    for (int axis = 0; axis < 3; ++axis)
        gyro_offset_[axis] = sum[axis] / samples;
    return true;
}

bool MPU6050::update()
{
    if (!initialized_) return false;
    uint8_t data[14];
    if (readRegisters(0x3b, data, sizeof(data)) != ESP_OK) return false;
    auto sample = [&](int offset) {
        return (int16_t)((data[offset] << 8) | data[offset + 1]);
    };
    const float ax = sample(0) / 16384.0f;
    const float ay = sample(2) / 16384.0f;
    const float az = sample(4) / 16384.0f;
    const float gx = (sample(8) - gyro_offset_[0]) / 131.0f;
    const float gy = (sample(10) - gyro_offset_[1]) / 131.0f;
    const float pitch_accel = atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2957795f;
    const float roll_accel = atan2f(-ax, az) * 57.2957795f;
    const int64_t now_us = esp_timer_get_time();
    const float dt = (now_us - last_us_) / 1000000.0f;
    last_us_ = now_us;
    if (!orientation_ready_ || dt <= 0.0f || dt > 1.0f) {
        pitch_ = pitch_accel;
        roll_ = roll_accel;
        orientation_ready_ = true;
    } else {
        pitch_ = 0.99f * (pitch_ + gx * dt) + 0.01f * pitch_accel;
        roll_ = 0.99f * (roll_ + gy * dt) + 0.01f * roll_accel;
    }
    return true;
}

bool MPU6050::getTilt(float *pitch, float *roll) const
{
    if (!initialized_ || !pitch || !roll) return false;
    *pitch = pitch_;
    *roll = roll_;
    return true;
}
