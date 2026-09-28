#include "as500_ethernet.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_log.h"
#include "esp_netif.h"

namespace {
constexpr gpio_num_t PHY_POWER_GPIO = GPIO_NUM_51;
constexpr int PHY_ADDRESS = 1;
const char *TAG = "as500_eth";
}

void AS500Ethernet::eventHandler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    auto *self = static_cast<AS500Ethernet *>(arg);
    if (base == ETH_EVENT) {
        if (id == ETHERNET_EVENT_CONNECTED) ESP_LOGI(TAG, "Ethernet link up");
        if (id == ETHERNET_EVENT_DISCONNECTED) {
            self->m_has_ip = false;
            ESP_LOGW(TAG, "Ethernet link down");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        const auto *event = static_cast<ip_event_got_ip_t *>(data);
        self->m_has_ip = true;
        ESP_LOGI(TAG, "IPv4: " IPSTR ", gateway: " IPSTR,
                 IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.gw));
    }
}

esp_err_t AS500Ethernet::begin()
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    gpio_config_t power_cfg = {};
    power_cfg.pin_bit_mask = 1ULL << PHY_POWER_GPIO;
    power_cfg.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&power_cfg), TAG, "PHY power GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(PHY_POWER_GPIO, 1), TAG, "PHY power on failed");

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = PHY_ADDRESS;
    phy_cfg.reset_gpio_num = -1;

    // ESP-IDF's ESP32-P4 defaults match JC-ESP32P4-M3:
    // MDC=31, MDIO=52, CLK_EXT_IN=50 and the module's RMII data pins.
    eth_esp32_emac_config_t emac_cfg = {};
    emac_cfg.smi_gpio.mdc_num = 31;
    emac_cfg.smi_gpio.mdio_num = 52;
    emac_cfg.interface = EMAC_DATA_INTERFACE_RMII;
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config.rmii.clock_gpio = 50;
    emac_cfg.dma_burst_len = ETH_DMA_BURST_LEN_32;
    emac_cfg.intr_priority = 0;
    emac_cfg.emac_dataif_gpio.rmii.tx_en_num = 49;
    emac_cfg.emac_dataif_gpio.rmii.txd0_num = 34;
    emac_cfg.emac_dataif_gpio.rmii.txd1_num = 35;
    emac_cfg.emac_dataif_gpio.rmii.crs_dv_num = 28;
    emac_cfg.emac_dataif_gpio.rmii.rxd0_num = 29;
    emac_cfg.emac_dataif_gpio.rmii.rxd1_num = 30;
    emac_cfg.clock_config_out_in.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config_out_in.rmii.clock_gpio = -1;
    emac_cfg.mdc_freq_hz = 0;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    ESP_RETURN_ON_FALSE(mac != nullptr, ESP_ERR_NO_MEM, TAG, "create EMAC failed");
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_cfg);
    if (phy == nullptr) {
        mac->del(mac);
        return ESP_ERR_NO_MEM;
    }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    err = esp_eth_driver_install(&eth_cfg, &m_eth_handle);
    if (err != ESP_OK) {
        phy->del(phy);
        mac->del(mac);
        return err;
    }

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    ESP_RETURN_ON_FALSE(netif != nullptr, ESP_ERR_NO_MEM, TAG, "create netif failed");
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(m_eth_handle);
    ESP_RETURN_ON_FALSE(glue != nullptr, ESP_ERR_NO_MEM, TAG, "create netif glue failed");
    ESP_RETURN_ON_ERROR(esp_netif_attach(netif, glue), TAG, "attach netif failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                   &eventHandler, this), TAG, "ETH event failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                   &eventHandler, this), TAG, "IP event failed");
    ESP_LOGI(TAG, "Starting IP101 (PHY addr=%d, power GPIO=%d)", PHY_ADDRESS, PHY_POWER_GPIO);
    return esp_eth_start(m_eth_handle);
}
