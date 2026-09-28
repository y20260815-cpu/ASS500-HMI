#pragma once
#include <stdint.h>

// Standard CAN 0x5B9, DLC 8: [0:1] unsigned A*10, big endian;
// [2] version=1, [3] bit0=valid, [4:7] reserved=0.
struct CurrentTelemetry {
    static constexpr uint16_t encode(float amperes) {
        return !(amperes >= 0.0f) ? 0u : amperes >= 6553.4f ? 65534u :
               (uint16_t)(amperes * 10.0f + 0.5f);
    }
    static constexpr bool decode(const uint8_t *data, unsigned length, uint16_t &value) {
        if (length != 8 || data[2] != 1 || data[3] != 1) return false;
        value = ((uint16_t)data[0] << 8) | data[1];
        return value != 65535u;
    }
    uint16_t value = 0;
    uint64_t received_us = 0;
    bool valid = false;
    constexpr void receive(const uint8_t *data, unsigned length, uint64_t now) {
        valid = decode(data, length, value);
        received_us = now;
    }
    constexpr bool fresh(uint64_t now) const {
        return valid && now >= received_us && now - received_us <= 500000u;
    }
};
