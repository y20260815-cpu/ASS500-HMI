#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class AS500EspNow {
public:
    static constexpr uint32_t kMagic = 0x455A5254UL; // "EZRT"
    static constexpr uint8_t kVersion = 1;
    static constexpr uint8_t kChannel = 1;
    static constexpr uint32_t kHostedMessageId = 0x455A5258UL; // "EZRX"

    enum Key : uint8_t {
        KEY_FORWARD = 1U << 0,
        KEY_REVERSE = 1U << 1,
        KEY_LEFT    = 1U << 2,
        KEY_RIGHT   = 1U << 3,
        KEY_STOP    = 1U << 4,
        KEY_ESTOP   = 1U << 5,
        KEY_MASK    = 0x3F,
    };

    struct __attribute__((packed)) Packet {
        uint32_t magic;
        uint16_t sequence;
        int16_t distance_a0_cm;
        int16_t distance_a1_cm;
        uint16_t adc0_raw;
        uint16_t adc1_raw;
        uint16_t crc16;
        uint8_t version;
        uint8_t tag_id;
        uint8_t valid_mask;
        uint8_t key_state;
    };

    struct Frame {
        bool valid = false;
        Packet packet = {};
        uint8_t source_mac[6] = {};
        int8_t rssi = 0;
    };

    bool init();
    Frame poll(TickType_t wait_ticks = 0);
    bool isReady() const { return m_ready; }
    uint32_t rejectedCount() const { return m_rejected; }

    static uint16_t packetCrc(const Packet &packet);

private:
    struct QueueItem {
        Packet packet;
        uint8_t source_mac[6];
        int8_t rssi;
    };

    static void receiveCallback(uint32_t msg_id, const uint8_t *data,
                                size_t length);
    static uint16_t crc16Ccitt(const uint8_t *data, size_t length);

    static AS500EspNow *s_instance;
    QueueHandle_t m_queue = nullptr;
    bool m_ready = false;
    volatile uint32_t m_rejected = 0;
};

static_assert(sizeof(AS500EspNow::Packet) == 20,
              "TagRemotePacket must stay C6-compatible at 20 bytes");
