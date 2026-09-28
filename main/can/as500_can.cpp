/*
 * as500_can.cpp
 * ─────────────────────────────────────────────────────────────
 *  CAN (TWAI) 통신 — AS500Can 구현
 * ─────────────────────────────────────────────────────────────
 */

#include "as500_can.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG = "AS500Can";

/* ═══════════════════════════════════════════════════════════
 *  AS500Can::init()
 * ═══════════════════════════════════════════════════════════ */
bool AS500Can::init(int tx_gpio, int rx_gpio, int /*baud_kbps*/,
                      bool suppress_tx_test_mode)
{
    m_current = {};
    m_tx_suppressed = suppress_tx_test_mode;
    if (m_tx_suppressed) {
        m_ready = true;
        ESP_LOGI(TAG, "CAN TX suppressed for disconnected test mode");
        return true;
    }

    twai_general_config_t gc = {
        .mode           = TWAI_MODE_NORMAL,
        .tx_io          = (gpio_num_t)tx_gpio,
        .rx_io          = (gpio_num_t)rx_gpio,
        .clkout_io      = TWAI_IO_UNUSED,
        .bus_off_io     = TWAI_IO_UNUSED,
        .tx_queue_len   = 10,
        .rx_queue_len   = 10,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    twai_timing_config_t tc = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t fc = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&gc, &tc, &fc) != ESP_OK) {
        ESP_LOGE(TAG, "driver_install failed — check GPIO wiring");
        m_ready = false;
        return false;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed");
        m_ready = false;
        return false;
    }

    m_ready = true;
    ESP_LOGI(TAG, "500 kbps ready (GPIO%d TX / GPIO%d RX, NORMAL ACK)",
             tx_gpio, rx_gpio);
    return true;
}

/* ═══════════════════════════════════════════════════════════
 *  Status receive
 * ═══════════════════════════════════════════════════════════ */
AS500Can::Status AS500Can::pollStatus()
{
    Status newest;
    if (!m_ready || m_tx_suppressed) return newest;

    twai_message_t message = {};
    while (twai_receive(&message, 0) == ESP_OK) {
        char data_hex[3 * 8 + 1] = {};
        size_t offset = 0;
        const unsigned dlc = message.data_length_code > 8 ? 8 :
                             message.data_length_code;
        for (unsigned i = 0; i < dlc && offset < sizeof(data_hex); ++i) {
            const int written = snprintf(data_hex + offset,
                                         sizeof(data_hex) - offset,
                                         i == 0 ? "%02X" : " %02X",
                                         message.data[i]);
            if (written <= 0) break;
            offset += (size_t)written;
        }
        ESP_LOGI(TAG, "CAN_RX ID=0x%lX%s DLC=%u DATA=%s",
                 (unsigned long)message.identifier,
                 message.extd ? " EXT" : "", dlc,
                 message.rtr ? "RTR" : data_hex);

        if (!message.extd && !message.rtr && message.identifier == 0x5B9u) {
            m_current.receive(message.data, message.data_length_code, esp_timer_get_time());
            continue;
        }
        if (!message.extd && !message.rtr &&
            message.identifier == kIdStatus && message.data_length_code == 8) {
            const uint16_t encoded_m1 =
                ((uint16_t)message.data[0] << 4) |
                ((uint16_t)message.data[1] >> 4);
            const uint16_t encoded_m2 =
                ((uint16_t)(message.data[1] & 0x0F) << 8) |
                (uint16_t)message.data[2];
            newest.motor1_rpm = (int16_t)(
                ((int32_t)encoded_m1 * 7200L) / 4095L - 3600L);
            newest.motor2_rpm = (int16_t)(
                ((int32_t)encoded_m2 * 7200L) / 4095L - 3600L);
            if (newest.motor1_rpm >= -3 && newest.motor1_rpm <= 3)
                newest.motor1_rpm = 0;
            if (newest.motor2_rpm >= -3 && newest.motor2_rpm <= 3)
                newest.motor2_rpm = 0;
            newest.battery_decivolts =
                ((uint16_t)message.data[4] << 2) |
                ((uint16_t)message.data[5] >> 6);
            /* Same packed 7-bit temperatures as AS48:
               motor = byte5[5:0] + byte6[7], FET = byte6[6:0]. */
            newest.motor_temperature_c = (uint8_t)(
                ((message.data[5] & 0x3Fu) << 1) |
                ((message.data[6] >> 7) & 0x01u));
            newest.fet_temperature_c = message.data[6] & 0x7Fu;
            /* STM32 0x5B8 byte 3 is current in whole amperes. */
            newest.current_deciamps = (uint16_t)message.data[3] * 10u;
            newest.current_received_us = esp_timer_get_time();
            newest.brake_applied = (message.data[7] & 0x08u) != 0;
            newest.emergency = (message.data[7] & 0x04u) != 0;
            newest.valid = true;
        }
    }
    if (newest.valid && m_current.fresh(esp_timer_get_time())) {
        newest.current_deciamps = m_current.value;
        newest.current_received_us = m_current.received_us;
    }
    return newest;
}

AS500Can::Diagnostics AS500Can::diagnostics() const
{
    Diagnostics result;
    if (!m_ready || m_tx_suppressed) return result;
    twai_status_info_t status = {};
    if (twai_get_status_info(&status) != ESP_OK) return result;
    result.state = (int)status.state;
    result.tx_queue = status.msgs_to_tx;
    result.rx_queue = status.msgs_to_rx;
    result.tx_errors = status.tx_error_counter;
    result.rx_errors = status.rx_error_counter;
    result.tx_failed = status.tx_failed_count;
    result.bus_errors = status.bus_error_count;
    result.valid = true;
    return result;
}
