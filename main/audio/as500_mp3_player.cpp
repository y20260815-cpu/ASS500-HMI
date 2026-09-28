#include "as500_mp3_player.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <strings.h>
#include <sys/stat.h>

#include "audio_player.h"
#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_codec_dev.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"

namespace {
constexpr const char *TAG = "AS500_MP3";
constexpr const char *kMountPath = "/usb0";
esp_codec_dev_handle_t s_speaker = nullptr;

enum class EventType : uint8_t { Connected, Disconnected, Finished };
struct PlayerEvent { EventType type; uint8_t address; };

esp_err_t muteAudio(AUDIO_PLAYER_MUTE_SETTING setting)
{
    return s_speaker ? esp_codec_dev_set_out_mute(
        s_speaker, setting == AUDIO_PLAYER_MUTE) : ESP_ERR_INVALID_STATE;
}

esp_err_t writeAudio(void *data, size_t length, size_t *written, uint32_t)
{
    if (!s_speaker) return ESP_ERR_INVALID_STATE;
    const esp_err_t result = esp_codec_dev_write(s_speaker, data, length);
    if (written) *written = result == ESP_OK ? length : 0;
    return result;
}

esp_err_t configureAudio(uint32_t rate, uint32_t bits, i2s_slot_mode_t channels)
{
    if (!s_speaker) return ESP_ERR_INVALID_STATE;
    esp_codec_dev_close(s_speaker);
    esp_codec_dev_sample_info_t info = {
        .bits_per_sample = static_cast<uint8_t>(bits),
        .channel = static_cast<uint8_t>(channels),
        .sample_rate = rate,
    };
    return esp_codec_dev_open(s_speaker, &info);
}

bool hasMp3Extension(const char *name)
{
    const char *dot = strrchr(name, '.');
    return dot && strcasecmp(dot, ".mp3") == 0;
}

int compareTrackPath(const void *left, const void *right)
{
    return strcasecmp(static_cast<const char *>(left),
                      static_cast<const char *>(right));
}
} // namespace

AS500Mp3Player &AS500Mp3Player::instance()
{
    static AS500Mp3Player player;
    return player;
}

esp_err_t AS500Mp3Player::begin()
{
    if (m_queue) return ESP_OK;
    loadSettings();
    m_queue = xQueueCreate(8, sizeof(PlayerEvent));
    if (!m_queue) return ESP_ERR_NO_MEM;

    ESP_RETURN_ON_ERROR(bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true),
                        TAG, "USB host start");

    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = mscEvent,
        .callback_arg = this,
    };
    ESP_RETURN_ON_ERROR(msc_host_install(&msc_cfg), TAG, "MSC install");
    if (xTaskCreate(eventTask, "as500_mp3", 6144, this, 5, nullptr) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "USB MP3 service ready; codec remains off until playback");
    return ESP_OK;
}

void AS500Mp3Player::setEnabled(bool enabled)
{
    m_enabled.store(enabled);
    if (!enabled) {
        if (m_audio_ready.load()) audio_player_stop();
        m_playing.store(false);
    } else if (m_mounted.load() && m_track_count.load()) {
        playCurrent();
    }
}

void AS500Mp3Player::toggle() { setEnabled(!enabled()); }

void AS500Mp3Player::setVolume(uint8_t volume)
{
    volume = std::min<uint8_t>(volume, 100);
    m_volume.store(volume);
    if (s_speaker) esp_codec_dev_set_out_vol(s_speaker, volume);
    saveSettings();
}

void AS500Mp3Player::setMode(PlayMode mode)
{
    m_mode.store(mode);
    saveSettings();
}

void AS500Mp3Player::selectTrack(size_t index)
{
    if (index >= m_track_count.load()) return;
    m_current_track.store(index);
    if (enabled()) playCurrent();
}

const char *AS500Mp3Player::trackName(size_t index) const
{
    if (index >= m_track_count.load()) return "";
    const char *slash = strrchr(m_tracks[index], '/');
    return slash ? slash + 1 : m_tracks[index];
}

void AS500Mp3Player::mscEvent(const msc_host_event_t *event, void *arg)
{
    auto *self = static_cast<AS500Mp3Player *>(arg);
    if (!self || !self->m_queue) return;
    PlayerEvent message{};
    if (event->event == msc_host_event_t::MSC_DEVICE_CONNECTED) {
        message = {EventType::Connected, event->device.address};
    } else if (event->event == msc_host_event_t::MSC_DEVICE_DISCONNECTED) {
        message = {EventType::Disconnected, 0};
    } else return;
    xQueueSend(static_cast<QueueHandle_t>(self->m_queue), &message, 0);
}

void AS500Mp3Player::audioEvent(audio_player_cb_ctx_t *ctx)
{
    auto *self = static_cast<AS500Mp3Player *>(ctx->user_ctx);
    if (!self) self = &instance();
    if (ctx->audio_event == AUDIO_PLAYER_CALLBACK_EVENT_IDLE && self->enabled()) {
        const PlayerEvent message{EventType::Finished, 0};
        xQueueSend(static_cast<QueueHandle_t>(self->m_queue), &message, 0);
    }
}

void AS500Mp3Player::eventTask(void *arg)
{
    auto *self = static_cast<AS500Mp3Player *>(arg);
    PlayerEvent event{};
    for (;;) {
        if (xQueueReceive(static_cast<QueueHandle_t>(self->m_queue), &event,
                          portMAX_DELAY) != pdTRUE) continue;
        if (event.type == EventType::Connected) self->handleConnected(event.address);
        else if (event.type == EventType::Disconnected) self->handleDisconnected();
        else self->playNext();
    }
}

void AS500Mp3Player::handleConnected(uint8_t address)
{
    if (m_device) return;
    msc_host_device_handle_t device = nullptr;
    if (msc_host_install_device(address, &device) != ESP_OK) return;
    const esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false, .max_files = 4, .allocation_unit_size = 8192,
    };
    msc_host_vfs_handle_t vfs = nullptr;
    if (msc_host_vfs_register(device, kMountPath, &mount_cfg, &vfs) != ESP_OK) {
        msc_host_uninstall_device(device);
        return;
    }
    m_device = device;
    m_vfs = vfs;
    m_mounted.store(true);
    scanTracks();
    ESP_LOGI(TAG, "USB mounted, %u MP3 tracks", (unsigned)m_track_count.load());
    if (enabled() && m_track_count.load()) playCurrent();
}

void AS500Mp3Player::handleDisconnected()
{
    setEnabled(false);
    m_mounted.store(false);
    m_track_count.store(0);
    if (m_vfs) msc_host_vfs_unregister(static_cast<msc_host_vfs_handle_t>(m_vfs));
    if (m_device) msc_host_uninstall_device(static_cast<msc_host_device_handle_t>(m_device));
    m_vfs = nullptr;
    m_device = nullptr;
}

void AS500Mp3Player::scanTracks()
{
    m_track_count.store(0);
    scanDirectory(kMountPath, 0);
    qsort(m_tracks, m_track_count.load(), sizeof(m_tracks[0]), compareTrackPath);
    if (m_current_track.load() >= m_track_count.load()) m_current_track.store(0);
}

void AS500Mp3Player::scanDirectory(const char *path, unsigned depth)
{
    if (depth > 2 || m_track_count.load() >= kMaxTracks) return;
    DIR *dir = opendir(path);
    if (!dir) return;
    while (dirent *entry = readdir(dir)) {
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        char full[kMaxPath];
        if (snprintf(full, sizeof(full), "%s/%s", path, entry->d_name) >= (int)sizeof(full)) continue;
        struct stat info{};
        if (stat(full, &info) != 0) continue;
        if (S_ISDIR(info.st_mode)) scanDirectory(full, depth + 1);
        else if (hasMp3Extension(entry->d_name)) {
            const size_t index = m_track_count.load();
            if (index >= kMaxTracks) break;
            strlcpy(m_tracks[index], full, kMaxPath);
            m_track_count.store(index + 1);
        }
    }
    closedir(dir);
}

void AS500Mp3Player::playCurrent()
{
    const size_t index = m_current_track.load();
    if (!m_mounted.load() || index >= m_track_count.load()) return;
    if (ensureAudioReady() != ESP_OK) {
        ESP_LOGE(TAG, "Audio codec initialization failed");
        m_enabled.store(false);
        return;
    }
    FILE *file = fopen(m_tracks[index], "rb");
    if (!file) return;
    if (audio_player_play(file) == ESP_OK) m_playing.store(true);
    else fclose(file);
}

esp_err_t AS500Mp3Player::ensureAudioReady()
{
    if (m_audio_ready.load()) return ESP_OK;
    s_speaker = bsp_audio_codec_speaker_init();
    if (!s_speaker) return ESP_FAIL;
    ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(s_speaker, m_volume.load()),
                        TAG, "codec volume");
    audio_player_config_t audio_cfg = {
        .mute_fn = muteAudio,
        .clk_set_fn = configureAudio,
        .write_fn = writeAudio,
        .priority = 5,
        .coreID = 0,
    };
    ESP_RETURN_ON_ERROR(audio_player_new(audio_cfg), TAG, "audio player init");
    ESP_RETURN_ON_ERROR(audio_player_callback_register(audioEvent, this), TAG,
                        "audio callback");
    m_audio_ready.store(true);
    return ESP_OK;
}

void AS500Mp3Player::playNext()
{
    m_playing.store(false);
    const size_t count = m_track_count.load();
    if (!enabled() || !count) return;
    size_t next = m_current_track.load();
    if (m_mode.load() == PlayMode::Random && count > 1) {
        const size_t previous = next;
        do next = esp_random() % count; while (next == previous);
    } else next = (next + 1) % count;
    m_current_track.store(next);
    playCurrent();
}

void AS500Mp3Player::loadSettings()
{
    nvs_handle_t nvs;
    if (nvs_open("as500_mp3", NVS_READONLY, &nvs) != ESP_OK) return;
    uint8_t volume = 60, mode = 0;
    nvs_get_u8(nvs, "volume", &volume);
    nvs_get_u8(nvs, "mode", &mode);
    nvs_close(nvs);
    m_volume.store(std::min<uint8_t>(volume, 100));
    m_mode.store(mode ? PlayMode::Random : PlayMode::Sequential);
}

void AS500Mp3Player::saveSettings()
{
    nvs_handle_t nvs;
    if (nvs_open("as500_mp3", NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u8(nvs, "volume", m_volume.load());
    nvs_set_u8(nvs, "mode", static_cast<uint8_t>(m_mode.load()));
    nvs_commit(nvs);
    nvs_close(nvs);
}
