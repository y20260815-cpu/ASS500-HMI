/*
 * as500_can.h
 * ─────────────────────────────────────────────────────────────
 *  CAN (TWAI) 통신 캡슐화 — AS500Can 클래스 선언
 *  Receive-only motor-driver status interface. AS500 does not transmit
 *  direct motor commands.
 * ─────────────────────────────────────────────────────────────
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "current_telemetry.h"

class AS500Can {
public:
    struct Status {
        uint16_t battery_decivolts = 0;
        uint16_t current_deciamps = 0;
        uint64_t current_received_us = 0;
        int16_t motor1_rpm = 0;
        int16_t motor2_rpm = 0;
        uint8_t motor_temperature_c = 0;
        uint8_t fet_temperature_c = 0;
        bool brake_applied = false; // STM32 relay command, not a physical sensor
        bool emergency = false;
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

    /** Receive and decode the newest STM32 status frame (ID 0x5B8). */
    Status pollStatus();

    /** Snapshot the legacy TWAI driver's live error counters. */
    Diagnostics diagnostics() const;

    /** @return 초기화 성공 여부 */
    bool isReady() const { return m_ready; }

private:
    CurrentTelemetry m_current;
    bool m_ready = false;
    bool m_tx_suppressed = false;

    static constexpr uint32_t kIdStatus = 0x5B8u;
};
