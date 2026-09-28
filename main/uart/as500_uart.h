/*
 * as500_uart.h
 * ─────────────────────────────────────────────────────────────
 *  A0 앵커 ↔ ESP32-P4 UART 통신 — AS500Uart 클래스 선언
 *
 *  receive formats:
 *    "D,{dL},{dR},{mask},{adc0},{adc1}\n"
 *    "M,{left},{right},{enable}\n" (-1000..1000 normalized motor commands)
 *     dL  : 좌측 앵커(A0) 거리 (cm)
 *     dR  : 우측 앵커(A1) 거리 (cm, A0가 대신 전달)
 *     mask: 앵커 유효 비트  bit0=A0(L), bit1=A1(R)
 *
 *  배선 (JC1060P470C_I_W Expand IO)
 *    GPIO45 RX ← A0 GPIO17 TX  (FPC3 pin-3)
 *    GPIO46 TX → A0 GPIO16 RX  (FPC3 pin-4, 예비)
 * ─────────────────────────────────────────────────────────────
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

class AS500Uart {
public:
    /* ── 수신 프레임 ──────────────────────────────────── */
    struct Frame {
        enum class Type : uint8_t { None, Distance, Motor, ConfigGet, ConfigSet, TurnSet };

        Type    type  = Type::None;
        int16_t dL    = 0;      /* 좌측 거리 (cm) */
        int16_t dR    = 0;      /* 우측 거리 (cm) */
        uint8_t mask  = 0;      /* 앵커 유효 비트  */
        uint16_t adc0 = 0;      /* Tag GPIO35 ADC raw (0..4095) */
        uint16_t adc1 = 0;      /* Tag GPIO36 ADC raw (0..4095) */
        int16_t motor_left  = 0; /* normalized FM command (-1000..1000) */
        int16_t motor_right = 0; /* normalized FM command (-1000..1000) */
        bool    motor_enable = false;
        int16_t turn_rpm = 500;
        float   config_max_speed_m_s = 0.0f;
        float   config_stop_distance_mm = 0.0f;
        float   config_max_estimate_distance_m = 0.0f;
        float   config_near_speed_m_s = 0.0f;
        float   config_decel_time_s = 0.0f;
        float   config_accel_time_s = 0.0f;
        float   config_turn_gain = 0.0f;
        float   config_turn_pwm_percent = 0.0f;
        bool    valid = false;  /* 파싱 성공 여부  */
    };

    AS500Uart() = default;
    ~AS500Uart() = default;

    /**
     * @brief UART 드라이버 초기화
     * @param rx_gpio  수신 GPIO (A0 TX → P4 RX)
     * @param tx_gpio  송신 GPIO (P4 TX → A0 RX, 예비)
     * @param baud     통신 속도 (기본 115200)
     * @param port     UART 포트 번호 (기본 UART_NUM_1)
     * @return true: 성공
     */
    bool init(int rx_gpio, int tx_gpio,
              int baud = 115200,
              uart_port_t port = UART_NUM_1);

    /**
     * @brief Read available bytes and return the newest complete line
     *        robot_task 루프에서 매 반복 호출 (non-blocking)
     * @param wait_ticks  uart_read_bytes 대기 틱 (기본 10 ms)
     * @return Frame  valid=true 이면 완전한 데이터
     */
    Frame poll(TickType_t wait_ticks = pdMS_TO_TICKS(10));

    /** Send one already formatted ASCII message to the connected host. */
    bool writeLine(const char *line);

    /** Parse one complete ASCII command without its line ending. */
    static bool parseLine(const char *line, Frame &out);

    /** @return 초기화 성공 여부 */
    bool isReady() const { return m_ready; }

private:
    uart_port_t m_port  = UART_NUM_1;
    bool        m_ready = false;
    char        m_line[64] = {};
    uint8_t     m_idx   = 0;

};
