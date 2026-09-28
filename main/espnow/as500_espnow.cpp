#include "as500_espnow.h"

#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
extern "C" {
#include "esp_hosted_misc.h"
}

namespace {
const char *TAG = "as500_espnow";
}

AS500EspNow *AS500EspNow::s_instance = nullptr;

uint16_t AS500EspNow::crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t AS500EspNow::packetCrc(const Packet &packet)
{
    Packet copy = packet;
    copy.crc16 = 0;
    return crc16Ccitt(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));
}

void AS500EspNow::receiveCallback(uint32_t msg_id, const uint8_t *data,
                                    size_t length)
{
    struct __attribute__((packed)) RelayFrame {
        uint8_t source_mac[6];
        int8_t rssi;
        Packet packet;
    };
    AS500EspNow *self = s_instance;
    if (!self || !self->m_queue || !data || msg_id != kHostedMessageId ||
        length != sizeof(RelayFrame)) {
        if (self) ++self->m_rejected;
        return;
    }

    QueueItem item = {};
    const auto *relay = reinterpret_cast<const RelayFrame *>(data);
    item.packet = relay->packet;
    if (item.packet.magic != kMagic || item.packet.version != kVersion ||
        (item.packet.valid_mask & ~0x03U) != 0 ||
        (item.packet.key_state & ~KEY_MASK) != 0 ||
        item.packet.crc16 != packetCrc(item.packet)) {
        ++self->m_rejected;
        return;
    }

    memcpy(item.source_mac, relay->source_mac, sizeof(item.source_mac));
    item.rssi = relay->rssi;
    xQueueOverwrite(self->m_queue, &item);
}

bool AS500EspNow::init()
{
    if (m_ready) return true;

    m_queue = xQueueCreate(1, sizeof(QueueItem));
    if (!m_queue) {
        ESP_LOGE(TAG, "receive queue allocation failed");
        return false;
    }
    s_instance = this;

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return false;
    }
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK ||
        (err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK ||
        (err = esp_wifi_start()) != ESP_OK ||
        (err = esp_wifi_set_channel(kChannel, WIFI_SECOND_CHAN_NONE)) != ESP_OK) {
        ESP_LOGE(TAG, "remote Wi-Fi start failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t mac[6] = {};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        ESP_LOGI(TAG, "C6 STA %02X:%02X:%02X:%02X:%02X:%02X channel=%u",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], kChannel);
    }

    if ((err = esp_hosted_register_custom_callback(kHostedMessageId,
                                                    &receiveCallback)) != ESP_OK) {
        ESP_LOGE(TAG, "C6 relay callback failed: %s", esp_err_to_name(err));
        return false;
    }

    m_ready = true;
    ESP_LOGI(TAG, "C6 ESP-NOW relay ready (EZRT 20-byte packets with ADC)");
    return true;
}

AS500EspNow::Frame AS500EspNow::poll(TickType_t wait_ticks)
{
    Frame frame;
    QueueItem item = {};
    if (!m_queue || xQueueReceive(m_queue, &item, wait_ticks) != pdTRUE) {
        return frame;
    }
    frame.valid = true;
    frame.packet = item.packet;
    memcpy(frame.source_mac, item.source_mac, sizeof(frame.source_mac));
    frame.rssi = item.rssi;
    return frame;
}
