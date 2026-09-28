#include "as500_mqtt.h"

#include <cstdio>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"

// Reuse the AWS IoT device identity supplied in the user's reference project.
// PROGMEM is an Arduino storage qualifier and is unnecessary in ESP-IDF.
#ifndef PROGMEM
#define PROGMEM
#endif
#include "aws_certs.h"

namespace {
constexpr char AWS_ENDPOINT[] = "mqtts://a14xbcv33glzp0-ats.iot.ap-northeast-2.amazonaws.com:8883";
constexpr char PUBLISH_TOPIC[] = "skb";
const char *TAG = "as500_mqtt";
}

void AS500Mqtt::makeIdentity()
{
    uint8_t mac[6] = {};
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_ETH));
    char cpuid[25] = {};
    std::snprintf(cpuid, sizeof(cpuid),
                  "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    std::snprintf(m_client_id, sizeof(m_client_id), "ESP32_AS500_%s", cpuid);
    std::snprintf(m_subscribe_topic, sizeof(m_subscribe_topic), "Hskb/%s", cpuid);
}

void AS500Mqtt::eventHandler(void *arg, esp_event_base_t, int32_t id, void *data)
{
    auto *self = static_cast<AS500Mqtt *>(arg);
    auto *event = static_cast<esp_mqtt_event_handle_t>(data);
    switch (static_cast<esp_mqtt_event_id_t>(id)) {
    case MQTT_EVENT_CONNECTED:
        self->m_connected = true;
        ESP_LOGI(TAG, "AWS IoT connected; client=%s", self->m_client_id);
        esp_mqtt_client_subscribe(event->client, self->m_subscribe_topic, 1);
        self->publishTest();
        break;
    case MQTT_EVENT_DISCONNECTED:
        self->m_connected = false;
        ESP_LOGW(TAG, "AWS IoT disconnected (automatic reconnect enabled)");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Subscribed: %s", self->m_subscribe_topic);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "RX topic=%.*s payload=%.*s",
                 event->topic_len, event->topic, event->data_len, event->data);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "Test message acknowledged, id=%d", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        self->m_connected = false;
        ESP_LOGE(TAG, "MQTT/TLS error type=%d", event->error_handle->error_type);
        break;
    default:
        break;
    }
}

esp_err_t AS500Mqtt::begin()
{
    makeIdentity();
    const esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {.uri = AWS_ENDPOINT},
            .verification = {.certificate = AWS_CERT_CA},
        },
        .credentials = {
            .client_id = m_client_id,
            .authentication = {
                .certificate = AWS_CERT_CRT,
                .key = AWS_CERT_PRIVATE,
            },
        },
        .session = {.keepalive = 60},
        .network = {.reconnect_timeout_ms = 5000},
    };
    m_client = esp_mqtt_client_init(&cfg);
    if (m_client == nullptr) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(
                                                       m_client,
                                                       static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
                                                       &eventHandler, this), TAG, "event registration failed");
    ESP_LOGI(TAG, "Starting AWS IoT; subscribe=%s", m_subscribe_topic);
    return esp_mqtt_client_start(m_client);
}

int AS500Mqtt::publishTest()
{
    if (m_client == nullptr || !m_connected) return -1;
    const char payload[] = "{\"device\":\"esp32p4_as500\",\"test\":\"mqtt_over_ethernet\",\"status\":\"online\"}";
    const int id = esp_mqtt_client_publish(m_client, PUBLISH_TOPIC, payload, 0, 1, 0);
    ESP_LOGI(TAG, "TX topic=%s id=%d payload=%s", PUBLISH_TOPIC, id, payload);
    return id;
}
