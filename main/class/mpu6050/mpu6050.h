#pragma once
#include <stdint.h>
#include "driver/i2c_master.h"

struct mpu6050_orientation_t { float pitch=0,roll=0,yaw=0; };
class MPU6050 {
public:
    explicit MPU6050(uint8_t address=0x68):address_(address){}
    ~MPU6050();
    bool begin(i2c_master_bus_handle_t bus);
    bool calibrate(uint16_t samples=500);
    bool update();
    bool getTilt(float *pitch,float *roll) const;
    bool isConnected();
private:
    esp_err_t writeRegister(uint8_t reg,uint8_t value);
    esp_err_t readRegisters(uint8_t reg,uint8_t *data,size_t size);
    i2c_master_dev_handle_t device_=nullptr;
    uint8_t address_=0x68; bool initialized_=false,orientation_ready_=false;
    float gyro_offset_[3]={}; float pitch_=0,roll_=0; int64_t last_us_=0;
};
