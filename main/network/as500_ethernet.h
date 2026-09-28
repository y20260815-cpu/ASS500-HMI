#pragma once

#include <cstdint>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_eth.h"

class AS500Ethernet {
public:
    esp_err_t begin();
    bool hasIp() const { return m_has_ip; }

private:
    static void eventHandler(void *arg, esp_event_base_t base, int32_t id, void *data);

    esp_eth_handle_t m_eth_handle = nullptr;
    volatile bool m_has_ip = false;
};
