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

static int16_t legacy12ToRpm(uint16_t encoded)
{
    encoded &= 0x0FFFu;
    return (int16_t)(((int32_t)encoded * 7200L) / 4095L - 3600L);
}

/* ═══════════════════════════════════════════════════════════
 *  AS500Can::init()
 * ═══════════════════════════════════════════════════════════ */
bool AS500Can::init(int tx_gpio, int rx_gpio, int /*baud_kbps*/,
                      bool suppress_tx_test_mode)
{
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
        m_last_any_rx_us = esp_timer_get_time();
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

        if (!message.extd && !message.rtr &&
            message.identifier == AS500_CAN_ID_IO_STATE &&
            message.data_length_code == AS500_IO_STATE_DLC) {
            const uint16_t inputs =
                ((uint16_t)(message.data[AS500_IO_BYTE_INPUT_STATE_HIGH] & 0x03u) << 8) |
                message.data[AS500_IO_BYTE_INPUT_STATE_LOW];
            const bool forward = (inputs & AS500_IO_INPUT_FORWARD_MASK) != 0;
            const bool reverse = (inputs & AS500_IO_INPUT_REVERSE_MASK) != 0;
            m_interface_fnr = forward == reverse ? AS500_FNR_NEUTRAL :
                (forward ? AS500_FNR_FORWARD : AS500_FNR_REVERSE);
            m_interface_cruise_pressed =
                (inputs & AS500_IO_INPUT_CRUISE_MASK) != 0;
            m_interface_gpio_input_state = inputs;
            m_interface_gpio_output_state =
                message.data[AS500_IO_BYTE_OUTPUT_STATE] & AS500_IO_OUTPUT_STATE_MASK;
            m_interface_throttle1 =
                message.data[AS500_IO_BYTE_THROTTLE1];
            m_interface_throttle2 =
                message.data[AS500_IO_BYTE_THROTTLE2];
            m_interface_vr1 = message.data[AS500_IO_BYTE_VR1];
            m_interface_vr2 = message.data[AS500_IO_BYTE_VR2];
            const uint64_t received_us = esp_timer_get_time();
            m_interface_digital_received_us = received_us;
            m_interface_analog_received_us = received_us;
            continue;
        }

        if (!message.extd && !message.rtr &&
            message.identifier == AS500_CAN_ID_LEGACY_STATUS &&
            message.data_length_code == AS500_LEGACY_STATUS_DLC) {
            const uint16_t encoded1 =
                ((uint16_t)message.data[0] << 4) |
                ((message.data[1] >> 4) & 0x0Fu);
            const uint16_t encoded2 =
                ((uint16_t)(message.data[1] & 0x0Fu) << 8) |
                message.data[2];
            m_motor1_rpm = legacy12ToRpm(encoded1);
            m_motor2_rpm = legacy12ToRpm(encoded2);
            m_motion_brake_applied =
                (message.data[7] & AS500_LEGACY_STATUS_EMB_APPLIED_MASK) != 0;
            m_motion_emergency =
                (message.data[7] & AS500_LEGACY_STATUS_ESTOP_MASK) != 0;
            m_motion_received_us = esp_timer_get_time();
            continue;
        }

        if (!message.extd && !message.rtr &&
            message.identifier == kIdSteeringEncoder &&
            message.data_length_code == AS500_STEERING_DLC &&
            message.data[0] == AS500_STEERING_HEADER) {
            if (message.data[1] == AS500_STEERING_TYPE_MEASUREMENT) {
                m_steering_angle_raw = as500_can_u16_le(&message.data[2]);
                const int16_t speed_raw = as500_can_i16_le(&message.data[4]);
                m_steering_turns = as500_can_i16_le(&message.data[6]);
                m_steering_angle_deg =
                    (float)m_steering_angle_raw / 32768.0f * 360.0f;
                m_steering_speed_dps =
                    (float)speed_raw / 32768.0f * 3600.0f;
                m_steering_received_us = esp_timer_get_time();
            } else if (message.data[1] == AS500_STEERING_TYPE_TEMPERATURE) {
                const int16_t temperature_raw = as500_can_i16_le(&message.data[2]);
                m_steering_temperature_c = (float)temperature_raw / 100.0f;
            }
            continue;
        }

        if (!message.extd && !message.rtr &&
            message.identifier == AS500_CAN_ID_IO_TILT &&
            message.data_length_code == AS500_IO_TILT_DLC) {
            const int16_t pitch_centidegrees = as500_can_i16_be(
                &message.data[AS500_IO_TILT_BYTE_PITCH_H]);
            const int16_t roll_centidegrees = as500_can_i16_be(
                &message.data[AS500_IO_TILT_BYTE_ROLL_H]);
            m_interface_pitch_deg = (float)pitch_centidegrees / 100.0f;
            m_interface_roll_deg = (float)roll_centidegrees / 100.0f;
            m_interface_received_us = esp_timer_get_time();
            continue;
        }

        if (!message.extd && !message.rtr &&
            message.identifier == kIdStatus &&
            message.data_length_code == AS500_DRIVER_STATUS_DLC) {
            newest.switch_status =
                message.data[AS500_DRIVER_STATUS_BYTE_INPUTS];
            newest.relay_status = message.data[AS500_DRIVER_STATUS_BYTE_RELAYS];
            newest.error_code =
                message.data[AS500_DRIVER_STATUS_BYTE_IO_ERROR] &
                AS500_DRIVER_ERROR_MASK;
            newest.fet_temperature_c =
                (int8_t)message.data[AS500_DRIVER_STATUS_BYTE_FET_TEMP];
            newest.battery_decivolts = as500_can_u16_be(
                &message.data[AS500_DRIVER_STATUS_BYTE_VOLTAGE_H]);
            newest.current_deciamps = as500_can_u16_be(
                &message.data[AS500_DRIVER_STATUS_BYTE_CURRENT_H]);
            newest.current_received_us = esp_timer_get_time();
            newest.brake_applied =
                (newest.relay_status & AS500_RELAY_RY3_EMB_MASK) == 0;
            newest.valid = true;
        }
    }
    const uint64_t now_us = esp_timer_get_time();
    const uint64_t steering_age_us =
        m_steering_received_us != 0 && now_us >= m_steering_received_us
            ? now_us - m_steering_received_us : UINT64_MAX;
    newest.steering_angle_raw = m_steering_angle_raw;
    newest.steering_angle_deg = m_steering_angle_deg;
    newest.steering_speed_dps = m_steering_speed_dps;
    newest.steering_turns = m_steering_turns;
    newest.steering_temperature_c = m_steering_temperature_c;
    newest.steering_age_ms = steering_age_us / 1000u > 65535u
        ? 65535u : (uint16_t)(steering_age_us / 1000u);
    newest.steering_valid =
        steering_age_us <= (uint64_t)AS500_CAN_STEERING_TIMEOUT_MS * 1000u;
    const uint64_t interface_age_us =
        m_interface_received_us != 0 && now_us >= m_interface_received_us
            ? now_us - m_interface_received_us : UINT64_MAX;
    newest.interface_pitch_deg = m_interface_pitch_deg;
    newest.interface_roll_deg = m_interface_roll_deg;
    newest.interface_age_ms = interface_age_us / 1000u > UINT16_MAX
        ? UINT16_MAX : (uint16_t)(interface_age_us / 1000u);
    newest.interface_imu_valid =
        m_interface_received_us != 0 &&
        interface_age_us <= (uint64_t)AS500_CAN_IO_TIMEOUT_MS * 1000u;
    const uint64_t digital_age_us =
        m_interface_digital_received_us != 0 && now_us >= m_interface_digital_received_us
            ? now_us - m_interface_digital_received_us : UINT64_MAX;
    newest.interface_digital_age_ms = digital_age_us / 1000u > UINT16_MAX
        ? UINT16_MAX : (uint16_t)(digital_age_us / 1000u);
    newest.interface_fnr_valid =
        m_interface_digital_received_us != 0 &&
        digital_age_us <= (uint64_t)AS500_CAN_IO_TIMEOUT_MS * 1000u;
    newest.interface_fnr = newest.interface_fnr_valid
        ? m_interface_fnr : AS500_FNR_NEUTRAL;
    newest.interface_cruise_pressed = newest.interface_fnr_valid &&
        m_interface_cruise_pressed;
    newest.interface_gpio_input_state = m_interface_gpio_input_state;
    newest.interface_gpio_output_state = m_interface_gpio_output_state;
    const uint64_t analog_age_us =
        m_interface_analog_received_us != 0 && now_us >= m_interface_analog_received_us
            ? now_us - m_interface_analog_received_us : UINT64_MAX;
    newest.interface_throttle1 = m_interface_throttle1;
    newest.interface_throttle2 = m_interface_throttle2;
    newest.interface_vr1 = m_interface_vr1;
    newest.interface_vr2 = m_interface_vr2;
    newest.interface_analog_age_ms = analog_age_us / 1000u > UINT16_MAX
        ? UINT16_MAX : (uint16_t)(analog_age_us / 1000u);
    newest.interface_analog_valid = m_interface_analog_received_us != 0 &&
        analog_age_us <= (uint64_t)AS500_CAN_IO_TIMEOUT_MS * 1000u;
    const uint64_t motion_age_us =
        m_motion_received_us != 0 && now_us >= m_motion_received_us
            ? now_us - m_motion_received_us : UINT64_MAX;
    newest.motor1_rpm = m_motor1_rpm;
    newest.motor2_rpm = m_motor2_rpm;
    newest.motion_brake_applied = m_motion_brake_applied;
    newest.motion_emergency = m_motion_emergency;
    newest.motion_age_ms = motion_age_us / 1000u > UINT16_MAX
        ? UINT16_MAX : (uint16_t)(motion_age_us / 1000u);
    newest.motion_valid = m_motion_received_us != 0 &&
        motion_age_us <= (uint64_t)AS500_CAN_MOTION_TIMEOUT_MS * 1000u;
    return newest;
}

bool AS500Can::sendDriveCommand(int16_t left, int16_t right,
                                uint8_t accel_raw, uint8_t decel_raw,
                                uint8_t fnr, bool emergency,
                                uint8_t input_port, uint8_t relay_port)
{
    if (!ensureTransmitReady()) return false;
    if (fnr != AS500_FNR_FORWARD && fnr != AS500_FNR_REVERSE) {
        fnr = AS500_FNR_NEUTRAL;
        left = right = 0;
    }
    if (emergency) {
        left = right = 0;
        fnr = AS500_FNR_NEUTRAL;
        input_port |= AS500_DRIVE_INPUT_ESTOP_MASK;
    }

    twai_message_t message = {};
    message.identifier = AS500_CAN_ID_DRIVE_COMMAND;
    message.data_length_code = AS500_DRIVE_COMMAND_DLC;
    message.data[AS500_DRIVE_BYTE_LEFT_H] = (uint8_t)((uint16_t)left >> 8);
    message.data[AS500_DRIVE_BYTE_LEFT_L] = (uint8_t)left;
    message.data[AS500_DRIVE_BYTE_BRAKE_DELAY] = accel_raw;
    message.data[AS500_DRIVE_BYTE_BATTERY] = (uint8_t)(0x80u | decel_raw);
    message.data[AS500_DRIVE_BYTE_RIGHT_H] = (uint8_t)((uint16_t)right >> 8);
    message.data[AS500_DRIVE_BYTE_RIGHT_L] = (uint8_t)right;
    message.data[AS500_DRIVE_BYTE_INPUT_PORT] = input_port;
    /* RY1/RY2 are owned by STM32 power sequencing and RY3 by its EMB safety
       state machine. Never place commands for those relays on the bus. */
    message.data[AS500_DRIVE_BYTE_RELAY_PORT] =
        relay_port & AS500_RELAY_P4_COMMAND_MASK;
    const esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    if (result != ESP_OK) {
        static uint32_t failure_count = 0;
        ++failure_count;
        if (failure_count == 1 || failure_count % 50 == 0) {
            ESP_LOGE(TAG, "CAN_TX 0x%03lX failed: %s (count=%lu)",
                     (unsigned long)message.identifier, esp_err_to_name(result),
                     (unsigned long)failure_count);
        }
        return false;
    }
    return true;
}

bool AS500Can::sendInterfaceOutputCommand(uint8_t output_control)
{
    if (!ensureTransmitReady()) return false;
    twai_message_t message = {};
    message.identifier = AS500_CAN_ID_IO_COMMAND;
    message.data_length_code = AS500_IO_COMMAND_DLC;
    message.data[AS500_IO_COMMAND_BYTE_CONTROL] = output_control;
    const esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "CAN_TX 0x%03lX failed: %s",
                 (unsigned long)message.identifier, esp_err_to_name(result));
        return false;
    }
    return true;
}

bool AS500Can::ensureTransmitReady()
{
    if (!m_ready || m_tx_suppressed) return false;

    twai_status_info_t status = {};
    if (twai_get_status_info(&status) != ESP_OK) return false;
    if (status.state == TWAI_STATE_BUS_OFF) {
        const esp_err_t result = twai_initiate_recovery();
        ESP_LOGE(TAG, "CAN bus-off: recovery %s", esp_err_to_name(result));
        return false;
    }
    if (status.state == TWAI_STATE_RECOVERING) return false;
    if (status.state == TWAI_STATE_STOPPED) {
        twai_clear_transmit_queue();
        twai_clear_receive_queue();
        const esp_err_t result = twai_start();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "CAN restart failed: %s", esp_err_to_name(result));
            return false;
        }
        ESP_LOGW(TAG, "CAN recovered and restarted");
    }

    /* USB can keep the P4 alive while main power removes the external CAN
       transceiver and all peers. In that case the legacy driver may remain
       RUNNING with a saturated TX queue/error counter instead of completing
       a normal bus-off cycle. Periodically reset only this stalled state so
       CAN reconnects after main power returns without rebooting the P4. */
    if (status.state == TWAI_STATE_RUNNING) {
        const uint64_t now_us = esp_timer_get_time();
        const bool no_recent_rx = m_last_any_rx_us == 0 ||
            now_us - m_last_any_rx_us >= 1000000ULL;
        const bool tx_stalled = status.msgs_to_tx >= 8 ||
                                status.tx_error_counter >= 96;
        const bool restart_due = m_last_forced_restart_us == 0 ||
            now_us - m_last_forced_restart_us >= 1000000ULL;
        if (no_recent_rx && tx_stalled && restart_due) {
            m_last_forced_restart_us = now_us;
            const esp_err_t stop_result = twai_stop();
            if (stop_result != ESP_OK) {
                ESP_LOGE(TAG, "CAN forced stop failed: %s",
                         esp_err_to_name(stop_result));
                return false;
            }
            twai_clear_transmit_queue();
            twai_clear_receive_queue();
            const esp_err_t start_result = twai_start();
            if (start_result != ESP_OK) {
                ESP_LOGE(TAG, "CAN forced restart failed: %s",
                         esp_err_to_name(start_result));
                return false;
            }
            ESP_LOGW(TAG,
                     "CAN peer-power reset: restarted (txq=%lu txerr=%lu)",
                     (unsigned long)status.msgs_to_tx,
                     (unsigned long)status.tx_error_counter);
        }
    }
    return true;
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
