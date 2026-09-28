#include "lidar.h"
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace {
constexpr uint8_t kHeader = 0x54;
constexpr uint8_t kVersionAndLength = 0x2C;
constexpr int kPacketSize = 47;
constexpr int kUartBufferSize = 2048;
constexpr uint32_t kPwmMaxDuty = 1023;
const char *TAG = "Lidar";
uint16_t readLe16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
}

Lidar::~Lidar() { stop(); }

bool Lidar::init(int lidar_txd_gpio, int pwm_gpio, uart_port_t port,
                 uint32_t pwm_frequency_hz, float pwm_duty_percent)
{
    stop();
    m_port = port;
    const uart_config_t config = {
        .baud_rate = 230400, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(m_port, kUartBufferSize, 0, 0, nullptr, 0);
    if (err == ESP_OK) err = uart_param_config(m_port, &config);
    if (err == ESP_OK) err = uart_set_pin(m_port, UART_PIN_NO_CHANGE, lidar_txd_gpio,
                                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
        uart_driver_delete(m_port);
        return false;
    }

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK, .deconfigure = false,
    };
    const ledc_channel_config_t channel = {
        .gpio_num = pwm_gpio, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD, .flags = {},
    };
    err = ledc_timer_config(&timer);
    if (err == ESP_OK) err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWM init failed: %s", esp_err_to_name(err));
        uart_driver_delete(m_port);
        return false;
    }

    m_pwm_ready = m_ready = true;
    m_packet_len = 0;
    m_crc_errors = 0;
    for (Bin &bin : m_bins) bin = {};
    if (!setMotorDuty(pwm_duty_percent)) { stop(); return false; }
    if (pwm_duty_percent <= 0.0f) {
        ESP_LOGI(TAG, "LD06 ready: GPIO%d RX, GPIO%d held LOW (internal 10 Hz control)",
                 lidar_txd_gpio, pwm_gpio);
    } else {
        ESP_LOGI(TAG, "LD06 ready: GPIO%d RX, GPIO%d PWM @ %lu Hz (%.1f%%)",
                 lidar_txd_gpio, pwm_gpio, (unsigned long)pwm_frequency_hz,
                 (double)pwm_duty_percent);
    }
    return true;
}

void Lidar::stop()
{
    if (m_pwm_ready) { ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); m_pwm_ready = false; }
    if (m_ready) uart_driver_delete(m_port);
    m_ready = false;
    m_packet_len = 0;
}

bool Lidar::setMotorDuty(float duty_percent)
{
    if (!m_pwm_ready) return false;
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;
    uint32_t duty = (uint32_t)lroundf(duty_percent * kPwmMaxDuty / 100.0f);
    return ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty) == ESP_OK &&
           ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) == ESP_OK;
}

Lidar::Frame Lidar::poll(TickType_t wait_ticks)
{
    Frame newest;
    if (!m_ready) return newest;
    uint8_t data[256];
    int count = uart_read_bytes(m_port, data, sizeof(data), wait_ticks);
    for (int i = 0; i < count; ++i) { Frame f; if (consume(data[i], f)) newest = f; }
    return newest;
}

bool Lidar::consume(uint8_t byte, Frame &out)
{
    if (m_packet_len == 0) { if (byte == kHeader) m_packet[m_packet_len++] = byte; return false; }
    if (m_packet_len == 1 && byte != kVersionAndLength) {
        m_packet_len = (byte == kHeader) ? 1 : 0;
        return false;
    }
    m_packet[m_packet_len++] = byte;
    if (m_packet_len < kPacketSize) return false;
    m_packet_len = 0;
    if (decodePacket(out)) return true;
    ++m_crc_errors;
    return false;
}

bool Lidar::decodePacket(Frame &out)
{
    if (crc8(m_packet, kPacketSize - 1) != m_packet[kPacketSize - 1]) return false;
    out.speed_dps = readLe16(&m_packet[2]);
    float start = readLe16(&m_packet[4]) * 0.01f;
    float end = readLe16(&m_packet[42]) * 0.01f;
    float span = end - start;
    if (span < 0) span += 360;
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < kPointCount; ++i) {
        int offset = 6 + i * 3;
        Point &p = out.points[i];
        p.angle_deg = normalizeAngle(start + span * i / (kPointCount - 1));
        p.distance_mm = readLe16(&m_packet[offset]);
        p.confidence = m_packet[offset + 2];
        int bin = (int)lroundf(p.angle_deg) % kAngleBins;
        m_bins[bin] = { p.distance_mm, p.confidence, now };
    }
    out.timestamp_ms = readLe16(&m_packet[44]);
    out.valid = true;
    return true;
}

Lidar::Obstacle Lidar::closestObstacle(float center, float half_width, uint16_t max_distance,
                                       uint8_t min_confidence, uint32_t max_age_ms) const
{
    Obstacle result;
    center = normalizeAngle(center);
    if (half_width < 0) half_width = 0;
    if (half_width > 180) half_width = 180;
    int64_t now = esp_timer_get_time();
    for (int angle = 0; angle < kAngleBins; ++angle) {
        float delta = fabsf(angle - center);
        if (delta > 180) delta = 360 - delta;
        const Bin &b = m_bins[angle];
        if (delta > half_width || !b.distance_mm || b.distance_mm > max_distance ||
            b.confidence < min_confidence || now - b.updated_us > (int64_t)max_age_ms * 1000) continue;
        if (!result.detected || b.distance_mm < result.distance_mm)
            result = { true, (float)angle, b.distance_mm, b.confidence };
    }
    return result;
}

int Lidar::copyScan(uint16_t *distance, uint8_t *confidence, int count,
                    uint32_t max_age_ms) const
{
    if (!distance || !confidence || count <= 0) return 0;
    int valid = 0;
    int64_t now = esp_timer_get_time();
    for (int sector = 0; sector < count; ++sector) {
        int begin = sector * kAngleBins / count;
        int end = (sector + 1) * kAngleBins / count;
        const Bin *best = nullptr;
        for (int angle = begin; angle < end; ++angle) {
            const Bin &bin = m_bins[angle];
            if (!bin.distance_mm || now - bin.updated_us > (int64_t)max_age_ms * 1000) continue;
            if (!best || bin.distance_mm < best->distance_mm) best = &bin;
        }
        if (best) {
            distance[sector] = best->distance_mm;
            confidence[sector] = best->confidence;
            ++valid;
        } else {
            distance[sector] = 0;
            confidence[sector] = 0;
        }
    }
    return valid;
}

uint8_t Lidar::crc8(const uint8_t *data, int length)
{
    uint8_t crc = 0;
    for (int i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
    }
    return crc;
}

float Lidar::normalizeAngle(float angle)
{
    while (angle >= 360) angle -= 360;
    while (angle < 0) angle += 360;
    return angle;
}
