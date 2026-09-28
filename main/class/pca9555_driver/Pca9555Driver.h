#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

class Pca9555Driver {
public:
    enum Register : uint8_t { INPUT_PORT_0=0x00, INPUT_PORT_1=0x01,
        OUTPUT_PORT_0=0x02, OUTPUT_PORT_1=0x03, POLARITY_INV_0=0x04,
        POLARITY_INV_1=0x05, CONFIG_PORT_0=0x06, CONFIG_PORT_1=0x07 };
    explicit Pca9555Driver(uint8_t address=0x21) : address_(address) {}
    ~Pca9555Driver();
    esp_err_t begin(i2c_master_bus_handle_t bus, uint8_t address=0x21);
    void end();
    esp_err_t configurePort(uint8_t port,uint8_t config);
    esp_err_t configurePorts(uint16_t config);
    esp_err_t readPort(uint8_t port,uint8_t *value);
    esp_err_t readPorts(uint16_t *value);
    uint16_t readPortsValue();
    esp_err_t writePort(uint8_t port,uint8_t value);
    esp_err_t writePorts(uint16_t value);
    esp_err_t setPinMode(uint8_t pin,bool input);
    bool readPin(uint8_t pin);
    esp_err_t writePin(uint8_t pin,bool value);
    esp_err_t setPolarityInversion(uint8_t port,uint8_t mask);
    bool isInitialized() const { return initialized_; }
private:
    esp_err_t writeRegister(uint8_t reg,uint8_t value);
    esp_err_t readRegister(uint8_t reg,uint8_t *value);
    i2c_master_dev_handle_t device_=nullptr;
    uint8_t address_=0x21;
    bool initialized_=false;
};
