#pragma once

#include <stdint.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

/** LDROBOT LD06 UART receiver and motor-PWM controller. */
class Lidar {
public:
    static constexpr int kPointCount = 12;
    static constexpr int kAngleBins = 360;
    struct Point { float angle_deg = 0; uint16_t distance_mm = 0; uint8_t confidence = 0; };
    struct Frame {
        uint16_t speed_dps = 0;
        uint16_t timestamp_ms = 0;
        Point points[kPointCount] = {};
        bool valid = false;
    };
    struct Obstacle {
        bool detected = false;
        float angle_deg = 0;
        uint16_t distance_mm = 0;
        uint8_t confidence = 0;
    };

    Lidar() = default;
    ~Lidar();
    bool init(int lidar_txd_gpio, int pwm_gpio, uart_port_t port = UART_NUM_2,
              uint32_t pwm_frequency_hz = 30000, float pwm_duty_percent = 0.0f);
    void stop();
    Frame poll(TickType_t wait_ticks = 0);
    bool setMotorDuty(float duty_percent);
    bool isReady() const { return m_ready; }
    uint32_t crcErrorCount() const { return m_crc_errors; }
    Obstacle closestObstacle(float center_deg = 0.0f, float half_width_deg = 180.0f,
                             uint16_t max_distance_mm = 300, uint8_t min_confidence = 20,
                             uint32_t max_age_ms = 300) const;
    // Copy one recent representative measurement per equal angular sector.
    int copyScan(uint16_t *distance_mm, uint8_t *confidence, int sector_count,
                 uint32_t max_age_ms = 300) const;

private:
    static constexpr int kPacketSize = 47;
    struct Bin { uint16_t distance_mm = 0; uint8_t confidence = 0; int64_t updated_us = 0; };
    uart_port_t m_port = UART_NUM_2;
    bool m_ready = false;
    bool m_pwm_ready = false;
    uint8_t m_packet[kPacketSize] = {};
    uint8_t m_packet_len = 0;
    uint32_t m_crc_errors = 0;
    Bin m_bins[kAngleBins] = {};
    bool consume(uint8_t byte, Frame &out);
    bool decodePacket(Frame &out);
    static uint8_t crc8(const uint8_t *data, int length);
    static float normalizeAngle(float angle_deg);
};
