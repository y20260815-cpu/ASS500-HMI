/*
 * as500_can.h
 * ─────────────────────────────────────────────────────────────
 *  CAN (TWAI) 통신 캡슐화 — AS500Can 클래스 선언
 *  Receives vehicle/interface status and transmits the 0x701 drive command.
 * ─────────────────────────────────────────────────────────────
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "can_protocol.h"

class AS500Can {
public:
    struct Status {
        uint16_t battery_decivolts = 0;
        uint16_t current_deciamps = 0;
        uint64_t current_received_us = 0;
        int16_t motor1_rpm = 0; // decoded directly from STM32 0x5B8
        int16_t motor2_rpm = 0; // decoded directly from STM32 0x5B8
        uint16_t motion_age_ms = 65535;
        bool motion_valid = false;
        bool motion_brake_applied = true;
        bool motion_emergency = false;
        uint8_t relay_status = 0;  // bit 0..7: RY1..RY6, WUP, WDN
        uint8_t switch_status = 0; // bit 0..7: A+,A-,B+,B-,C+,C-,D+,D-
        uint8_t error_code = 0;
        int8_t fet_temperature_c = 0;
        bool brake_applied = true; // RY3 low means spring brake applied
        uint16_t steering_angle_raw = 0;
        float steering_angle_deg = 0.0f;
        float steering_speed_dps = 0.0f;
        int16_t steering_turns = 0;
        float steering_temperature_c = 0.0f;
        uint16_t steering_age_ms = 65535;
        bool steering_valid = false;
        float interface_pitch_deg = 0.0f;
        float interface_roll_deg = 0.0f;
        uint16_t interface_age_ms = 65535;
        bool interface_imu_valid = false;
        uint8_t interface_fnr = AS500_FNR_NEUTRAL;
        uint16_t interface_digital_age_ms = 65535;
        bool interface_fnr_valid = false;
        bool interface_cruise_pressed = false;
        uint16_t interface_gpio_input_state = 0;
        uint8_t interface_gpio_output_state = 0;
        uint8_t interface_throttle1 = 0;
        uint8_t interface_throttle2 = 0;
        uint8_t interface_vr1 = 0;
        uint8_t interface_vr2 = 0;
        uint16_t interface_analog_age_ms = 65535;
        bool interface_analog_valid = false;
        bool valid = false;
    };

    struct Diagnostics {
        int state = -1;
        uint32_t tx_queue = 0;
        uint32_t rx_queue = 0;
        uint32_t tx_errors = 0;
        uint32_t rx_errors = 0;
        uint32_t tx_failed = 0;
        uint32_t bus_errors = 0;
        bool valid = false;
    };

    AS500Can() = default;
    ~AS500Can() = default;

    /**
     * @brief TWAI 드라이버 초기화 및 시작
     * @param tx_gpio  CAN TX GPIO 번호
     * @param rx_gpio  CAN RX GPIO 번호
     * @param baud_kbps 속도 (현재 500 kbps 고정)
     * @return true: 성공 / false: 실패 (배선 확인)
     */
    bool init(int tx_gpio, int rx_gpio, int baud_kbps = 500,
              bool suppress_tx_test_mode = false);

    /** Receive and decode the newest STM32 I/O status frame (ID 0x5A0). */
    Status pollStatus();

    bool sendDriveCommand(int16_t left, int16_t right,
                          uint8_t accel_raw, uint8_t decel_raw,
                          uint8_t fnr, bool emergency,
                          uint8_t input_port, uint8_t relay_port);
    bool sendInterfaceOutputCommand(uint8_t output_control);

    /** Snapshot the legacy TWAI driver's live error counters. */
    Diagnostics diagnostics() const;

    /** @return 초기화 성공 여부 */
    bool isReady() const { return m_ready; }

private:
    bool ensureTransmitReady();
    bool m_ready = false;
    bool m_tx_suppressed = false;
    uint64_t m_last_any_rx_us = 0;
    uint64_t m_last_forced_restart_us = 0;
    uint16_t m_steering_angle_raw = 0;
    float m_steering_angle_deg = 0.0f;
    float m_steering_speed_dps = 0.0f;
    int16_t m_steering_turns = 0;
    float m_steering_temperature_c = 0.0f;
    uint64_t m_steering_received_us = 0;
    float m_interface_pitch_deg = 0.0f;
    float m_interface_roll_deg = 0.0f;
    uint64_t m_interface_received_us = 0;
    uint8_t m_interface_fnr = AS500_FNR_NEUTRAL;
    bool m_interface_cruise_pressed = false;
    uint16_t m_interface_gpio_input_state = 0;
    uint8_t m_interface_gpio_output_state = 0;
    uint64_t m_interface_digital_received_us = 0;
    uint8_t m_interface_throttle1 = 0;
    uint8_t m_interface_throttle2 = 0;
    uint8_t m_interface_vr1 = 0;
    uint8_t m_interface_vr2 = 0;
    uint64_t m_interface_analog_received_us = 0;
    int16_t m_motor1_rpm = 0;
    int16_t m_motor2_rpm = 0;
    bool m_motion_brake_applied = true;
    bool m_motion_emergency = false;
    uint64_t m_motion_received_us = 0;

    static constexpr uint32_t kIdStatus = AS500_CAN_ID_DRIVER_STATUS;
    static constexpr uint32_t kIdSteeringEncoder = AS500_CAN_ID_STEERING_ENCODER;
};
