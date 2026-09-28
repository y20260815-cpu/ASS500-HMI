#pragma once

#include "esp_err.h"
#include "mqtt_client.h"

class AS500Mqtt {
public:
    esp_err_t begin();
    bool connected() const { return m_connected; }
    int publishTest();

private:
    static void eventHandler(void *arg, esp_event_base_t base, int32_t id, void *data);
    void makeIdentity();

    esp_mqtt_client_handle_t m_client = nullptr;
    volatile bool m_connected = false;
    char m_client_id[64] = {};
    char m_subscribe_topic[64] = {};
};
