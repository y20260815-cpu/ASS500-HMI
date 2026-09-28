#include "Pca9555Driver.h"

Pca9555Driver::~Pca9555Driver() { end(); }

esp_err_t Pca9555Driver::begin(i2c_master_bus_handle_t bus, uint8_t address)
{
    if (initialized_) return ESP_OK;
    if (!bus) return ESP_ERR_INVALID_ARG;
    address_ = address;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address_,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &device_);
    if (err != ESP_OK) return err;
    uint8_t value = 0;
    err = readRegister(INPUT_PORT_0, &value);
    if (err != ESP_OK) {
        end();
        return err;
    }
    initialized_ = true;
    return ESP_OK;
}

void Pca9555Driver::end()
{
    if (device_) {
        i2c_master_bus_rm_device(device_);
        device_ = nullptr;
    }
    initialized_ = false;
}

esp_err_t Pca9555Driver::writeRegister(uint8_t reg, uint8_t value)
{
    if (!device_) return ESP_ERR_INVALID_STATE;
    const uint8_t data[2] = {reg, value};
    return i2c_master_transmit(device_, data, sizeof(data), 100);
}

esp_err_t Pca9555Driver::readRegister(uint8_t reg, uint8_t *value)
{
    if (!device_) return ESP_ERR_INVALID_STATE;
    if (!value) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(device_, &reg, 1, value, 1, 100);
}

esp_err_t Pca9555Driver::configurePort(uint8_t port, uint8_t config)
{
    if (port >= 2) return ESP_ERR_INVALID_ARG;
    return writeRegister(port ? CONFIG_PORT_1 : CONFIG_PORT_0, config);
}

esp_err_t Pca9555Driver::configurePorts(uint16_t config)
{
    esp_err_t err = configurePort(0, config & 0xff);
    return err == ESP_OK ? configurePort(1, config >> 8) : err;
}

esp_err_t Pca9555Driver::readPort(uint8_t port, uint8_t *value)
{
    if (port >= 2) return ESP_ERR_INVALID_ARG;
    return readRegister(port ? INPUT_PORT_1 : INPUT_PORT_0, value);
}

esp_err_t Pca9555Driver::readPorts(uint16_t *value)
{
    if (!value) return ESP_ERR_INVALID_ARG;
    uint8_t port0 = 0, port1 = 0;
    esp_err_t err = readPort(0, &port0);
    if (err == ESP_OK) err = readPort(1, &port1);
    if (err == ESP_OK) *value = port0 | (uint16_t(port1) << 8);
    return err;
}

uint16_t Pca9555Driver::readPortsValue()
{
    uint16_t value = 0xffff;
    return readPorts(&value) == ESP_OK ? value : 0xffff;
}

esp_err_t Pca9555Driver::writePort(uint8_t port, uint8_t value)
{
    if (port >= 2) return ESP_ERR_INVALID_ARG;
    return writeRegister(port ? OUTPUT_PORT_1 : OUTPUT_PORT_0, value);
}

esp_err_t Pca9555Driver::writePorts(uint16_t value)
{
    esp_err_t err = writePort(0, value & 0xff);
    return err == ESP_OK ? writePort(1, value >> 8) : err;
}

esp_err_t Pca9555Driver::setPinMode(uint8_t pin, bool input)
{
    if (pin >= 16) return ESP_ERR_INVALID_ARG;
    const uint8_t reg = pin / 8 ? CONFIG_PORT_1 : CONFIG_PORT_0;
    uint8_t value = 0;
    esp_err_t err = readRegister(reg, &value);
    if (err != ESP_OK) return err;
    if (input) value |= 1u << (pin % 8);
    else value &= ~(1u << (pin % 8));
    return writeRegister(reg, value);
}

bool Pca9555Driver::readPin(uint8_t pin)
{
    uint8_t value = 0;
    return pin < 16 && readPort(pin / 8, &value) == ESP_OK &&
           (value & (1u << (pin % 8)));
}

esp_err_t Pca9555Driver::writePin(uint8_t pin, bool on)
{
    if (pin >= 16) return ESP_ERR_INVALID_ARG;
    const uint8_t reg = pin / 8 ? OUTPUT_PORT_1 : OUTPUT_PORT_0;
    uint8_t value = 0;
    esp_err_t err = readRegister(reg, &value);
    if (err != ESP_OK) return err;
    if (on) value |= 1u << (pin % 8);
    else value &= ~(1u << (pin % 8));
    return writeRegister(reg, value);
}

esp_err_t Pca9555Driver::setPolarityInversion(uint8_t port, uint8_t mask)
{
    if (port >= 2) return ESP_ERR_INVALID_ARG;
    return writeRegister(port ? POLARITY_INV_1 : POLARITY_INV_0, mask);
}
