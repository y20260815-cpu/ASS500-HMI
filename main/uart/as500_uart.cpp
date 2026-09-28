/*
 * as500_uart.cpp
 * ─────────────────────────────────────────────────────────────
 *  A0 앵커 ↔ ESP32-P4 UART 통신 — AS500Uart 구현
 *
 *  프로토콜
 *    수신 : "D,{dL},{dR},{mask},{adc0},{adc1}\n"
 *    예시 : "D,123,167,3\n"
 *           dL=123cm  dR=167cm  mask=0x03(양쪽 유효)
 * ─────────────────────────────────────────────────────────────
 */

#include "as500_uart.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "AS500Uart";

/* ═══════════════════════════════════════════════════════════
 *  AS500Uart::init()
 * ═══════════════════════════════════════════════════════════ */
bool AS500Uart::init(int rx_gpio, int tx_gpio, int baud, uart_port_t port)
{
    m_port = port;

    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(m_port, 512 * 2, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        m_ready = false;
        return false;
    }
    if (uart_param_config(m_port, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        m_ready = false;
        return false;
    }
    if (uart_set_pin(m_port, tx_gpio, rx_gpio,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        m_ready = false;
        return false;
    }

    m_idx   = 0;
    m_ready = true;
    ESP_LOGI(TAG, "UART%d ready: GPIO%d RX / GPIO%d TX @ %d baud",
             (int)m_port, rx_gpio, tx_gpio, baud);
    return true;
}

/* ═══════════════════════════════════════════════════════════
 *  AS500Uart::parseLine()  [private static]
 *  형식 : "D,{dL},{dR},{mask},{adc0},{adc1}"  (개행 문자 제거 후 호출)
 * ═══════════════════════════════════════════════════════════ */
bool AS500Uart::parseLine(const char *line, Frame &out)
{
    if (strncmp(line, "TURN,", 5) == 0) {
        int rpm; char trailing;
        if (sscanf(line + 5, "%d%c", &rpm, &trailing) != 1 ||
            rpm < 100 || rpm > 2500) return false;
        out.type = Frame::Type::TurnSet;
        out.turn_rpm = (int16_t)rpm;
        out.valid = true;
        return true;
    }
    if (strcmp(line, "CFG?") == 0) {
        out.type = Frame::Type::ConfigGet;
        out.valid = true;
        return true;
    }

    if (strncmp(line, "CFG,", 4) == 0) {
        float max_speed, stop_distance, max_estimate_distance, near_speed;
        float decel_time, accel_time, turn_gain, turn_pwm_percent;
        char trailing;
        if (sscanf(line + 4, "%f,%f,%f,%f,%f,%f,%f,%f%c",
                   &max_speed, &stop_distance, &max_estimate_distance,
                   &near_speed, &decel_time, &accel_time, &turn_gain,
                   &turn_pwm_percent, &trailing) != 8) return false;
        out.type = Frame::Type::ConfigSet;
        out.config_max_speed_m_s = max_speed;
        out.config_stop_distance_mm = stop_distance;
        out.config_max_estimate_distance_m = max_estimate_distance;
        out.config_near_speed_m_s = near_speed;
        out.config_decel_time_s = decel_time;
        out.config_accel_time_s = accel_time;
        out.config_turn_gain = turn_gain;
        out.config_turn_pwm_percent = turn_pwm_percent;
        out.valid = true;
        return true;
    }

    if (line[1] != ',') return false;

    if (line[0] == 'M') {
        int left, right, enable;
        char trailing;
        if (sscanf(line + 2, "%d,%d,%d%c", &left, &right, &enable,
                   &trailing) != 3) return false;
        if (left < -1000 || left > 1000 ||
            right < -1000 || right > 1000 ||
            (enable != 0 && enable != 1)) return false;
        out.type = Frame::Type::Motor;
        out.motor_left = (int16_t)left;
        out.motor_right = (int16_t)right;
        out.motor_enable = enable != 0;
        out.valid = true;
        return true;
    }

    if (line[0] != 'D') return false;

    int dL, dR, mask, adc0, adc1;
    if (sscanf(line + 2, "%d,%d,%d,%d,%d", &dL, &dR, &mask, &adc0, &adc1) != 5) return false;

    out.dL    = (int16_t)(dL   > 3000 ? 3000 : dL   < 0 ? 0 : dL);
    out.dR    = (int16_t)(dR   > 3000 ? 3000 : dR   < 0 ? 0 : dR);
    out.mask  = (uint8_t)(mask & 0xFF);
    out.adc0  = (uint16_t)(adc0 > 4095 ? 4095 : adc0 < 0 ? 0 : adc0);
    out.adc1  = (uint16_t)(adc1 > 4095 ? 4095 : adc1 < 0 ? 0 : adc1);
    out.type  = Frame::Type::Distance;
    out.valid = true;
    return true;
}

/* ═══════════════════════════════════════════════════════════
 *  AS500Uart::poll()
 *  1바이트씩 읽어 '\n' 완성 시 파싱 → Frame 반환
 *  미완성이면 valid=false 반환 (non-blocking)
 * ═══════════════════════════════════════════════════════════ */
AS500Uart::Frame AS500Uart::poll(TickType_t wait_ticks)
{
    Frame result;   /* valid=false 기본값 */

    if (!m_ready) return result;

    uint8_t bytes[64];
    int n = uart_read_bytes(m_port, bytes, sizeof(bytes), wait_ticks);
    if (n <= 0) return result;

    for (int i = 0; i < n; ++i) {
      const uint8_t ch = bytes[i];
      if (ch == '\n') {
        /* 라인 완성 → 파싱 */
        m_line[m_idx] = '\0';
        Frame parsed;
        parseLine(m_line, parsed);

        if (!parsed.valid) {
            ESP_LOGD(TAG, "parse failed: \"%s\"", m_line);
        } else {
            result = parsed;
        }
        m_idx = 0;
      } else if (ch == '\r') {
        /* CR 무시 */
      } else if (m_idx < (uint8_t)(sizeof(m_line) - 1)) {
        m_line[m_idx++] = (char)ch;
      } else {
        /* 버퍼 오버플로 → 리셋 */
        ESP_LOGW(TAG, "line buffer overflow — reset");
        m_idx = 0;
      }
    }

    return result;
}

bool AS500Uart::writeLine(const char *line)
{
    if (!m_ready || line == nullptr) return false;
    const size_t length = strlen(line);
    return uart_write_bytes(m_port, line, length) == (int)length;
}
