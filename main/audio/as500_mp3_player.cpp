#include "as500_mp3_player.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <strings.h>
#include <sys/stat.h>
#include <cerrno>

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
uint8_t s_codec_volume = 60;
std::atomic<esp_err_t> s_audio_error{ESP_OK};
std::atomic<bool> s_audio_written{false};

struct PlayerEvent { uint8_t address; };
constexpr EventBits_t kAudioIdle = BIT0;
constexpr uint32_t kEnabledChanged = 1u;
constexpr uint32_t kTrackChanged = 2u;
constexpr uint32_t kVolumeChanged = 4u;
constexpr uint32_t kModeChanged = 8u;

esp_err_t muteAudio(AUDIO_PLAYER_MUTE_SETTING setting)
{
    const esp_err_t err = s_speaker ? esp_codec_dev_set_out_mute(
        s_speaker, setting == AUDIO_PLAYER_MUTE) : ESP_ERR_INVALID_STATE;
    if (err != ESP_OK) s_audio_error.store(err);
    return err;
}

esp_err_t writeAudio(void *data, size_t length, size_t *written, uint32_t)
{
    if (written) *written = 0;
    if (!s_speaker) {
        s_audio_error.store(ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t volume = AS500Mp3Player::instance().volume();
    if (volume != s_codec_volume) {
        const esp_err_t err = esp_codec_dev_set_out_vol(s_speaker, volume);
        if (err != ESP_OK) {
            s_audio_error.store(err);
            return err;
        }
        s_codec_volume = volume;
    }
    const esp_err_t result = esp_codec_dev_write(s_speaker, data, length);
    if (written) *written = result == ESP_OK ? length : 0;
    if (result != ESP_OK) s_audio_error.store(result);
    else if (length) s_audio_written.store(true);
    return result;
}

esp_err_t configureAudio(uint32_t rate, uint32_t bits, i2s_slot_mode_t channels)
{
    if (!s_speaker) return ESP_ERR_INVALID_STATE;
    const esp_err_t close_result = esp_codec_dev_close(s_speaker);
    if (close_result != ESP_OK) {
        s_audio_error.store(close_result);
        return close_result;
    }
    esp_codec_dev_sample_info_t info = {
        .bits_per_sample = static_cast<uint8_t>(bits),
        .channel = static_cast<uint8_t>(channels),
        .sample_rate = rate,
    };
    esp_err_t err = esp_codec_dev_open(s_speaker, &info);
    if (err == ESP_OK) {
        s_codec_volume = AS500Mp3Player::instance().volume();
        err = esp_codec_dev_set_out_vol(s_speaker, s_codec_volume);
    }
    if (err != ESP_OK) s_audio_error.store(err);
    return err;
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
    if (m_service_ready) return ESP_OK;
    if (m_msc_installed) {
        const esp_err_t err = msc_host_uninstall();
        if (err != ESP_OK) {
            reportError("MSC startup cleanup", err);
            return err;
        }
        m_msc_installed = false;
    }
    if (m_queue) vQueueDelete(m_queue);
    if (m_tracks_mutex) vSemaphoreDelete(m_tracks_mutex);
    if (m_audio_events) vEventGroupDelete(m_audio_events);
    m_queue = nullptr;
    m_tracks_mutex = nullptr;
    m_audio_events = nullptr;
    m_last_error.store(ESP_OK);
    loadSettings();
    m_tracks_mutex = xSemaphoreCreateMutex();
    m_audio_events = xEventGroupCreate();
    QueueHandle_t queue = xQueueCreate(16, sizeof(PlayerEvent));
    esp_err_t err = ESP_OK;
    if (!m_tracks_mutex || !m_audio_events || !queue) {
        err = ESP_ERR_NO_MEM;
    } else if (!m_usb_started) {
        err = bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
        m_usb_started = err == ESP_OK;
    }
    if (err != ESP_OK) {
        if (queue) vQueueDelete(queue);
        if (m_tracks_mutex) vSemaphoreDelete(m_tracks_mutex);
        if (m_audio_events) vEventGroupDelete(m_audio_events);
        m_tracks_mutex = nullptr;
        m_audio_events = nullptr;
        reportError("USB service initialization", err);
        return err;
    }
    m_queue = queue;
    xEventGroupSetBits(m_audio_events, kAudioIdle);

    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = mscEvent,
        .callback_arg = this,
    };
    err = msc_host_install(&msc_cfg);
    m_msc_installed = err == ESP_OK;
    if (err == ESP_OK &&
        xTaskCreate(eventTask, "as500_mp3", 6144, this, 5, nullptr) != pdPASS) {
        const esp_err_t cleanup = msc_host_uninstall();
        if (cleanup != ESP_OK) {
            reportError("MSC startup cleanup", cleanup);
            return cleanup;
        }
        m_msc_installed = false;
        err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK) {
        vQueueDelete(m_queue);
        vSemaphoreDelete(m_tracks_mutex);
        vEventGroupDelete(m_audio_events);
        m_queue = nullptr;
        m_tracks_mutex = nullptr;
        m_audio_events = nullptr;
        reportError("MSC service initialization", err);
        return err;
    }
    m_service_ready = true;
    m_revision.fetch_add(1);
    ESP_LOGI(TAG, "USB MP3 service ready; codec remains off until playback");
    return ESP_OK;
}

void AS500Mp3Player::setEnabled(bool enabled)
{
    m_enabled.store(enabled);
    m_commands_pending.fetch_or(kEnabledChanged);
    m_revision.fetch_add(1);
}

void AS500Mp3Player::toggle() { setEnabled(!enabled()); }

void AS500Mp3Player::setVolume(uint8_t volume)
{
    volume = std::min<uint8_t>(volume, 100);
    m_volume.store(volume);
    m_commands_pending.fetch_or(kVolumeChanged);
    m_revision.fetch_add(1);
}

void AS500Mp3Player::setMode(PlayMode mode)
{
    if (mode != PlayMode::Sequential && mode != PlayMode::Random) {
        reportError("Invalid playback mode", ESP_ERR_INVALID_ARG);
        return;
    }
    m_mode.store(mode);
    m_commands_pending.fetch_or(kModeChanged);
    m_revision.fetch_add(1);
}

void AS500Mp3Player::selectTrack(size_t index)
{
    if (index >= m_track_count.load()) {
        reportError("Track selection", ESP_ERR_INVALID_ARG);
        return;
    }
    m_requested_track.store(index);
    m_commands_pending.fetch_or(kTrackChanged);
    m_revision.fetch_add(1);
}

bool AS500Mp3Player::copyTrackName(size_t index, char *name, size_t size) const
{
    if (!name || !size || !m_tracks_mutex) return false;
    xSemaphoreTake(m_tracks_mutex, portMAX_DELAY);
    const bool valid = index < m_track_count.load();
    if (valid) {
        const char *slash = strrchr(m_tracks[index], '/');
        strlcpy(name, slash ? slash + 1 : m_tracks[index], size);
    }
    xSemaphoreGive(m_tracks_mutex);
    return valid;
}

void AS500Mp3Player::mscEvent(const msc_host_event_t *event, void *arg)
{
    auto *self = static_cast<AS500Mp3Player *>(arg);
    if (!self || !self->m_queue) return;
    if (event->event == msc_host_event_t::MSC_DEVICE_CONNECTED) {
        const PlayerEvent message{event->device.address};
        if (xQueueSend(self->m_queue, &message, 0) != pdTRUE)
            self->reportError("USB connection queue full", ESP_ERR_NO_MEM);
    } else if (event->event == msc_host_event_t::MSC_DEVICE_DISCONNECTED) {
        // A disconnect must not be dropped or block the MSC event task.
        self->m_disconnected_device.store(event->device.handle);
    }
}

void AS500Mp3Player::audioEvent(audio_player_cb_ctx_t *ctx)
{
    auto *self = static_cast<AS500Mp3Player *>(ctx->user_ctx);
    if (!self) return;
    if (ctx->audio_event == AUDIO_PLAYER_CALLBACK_EVENT_IDLE) {
        self->m_playing.store(false);
        self->m_finished.store(true);
        xEventGroupSetBits(self->m_audio_events, kAudioIdle);
        self->m_revision.fetch_add(1);
    } else if (ctx->audio_event == AUDIO_PLAYER_CALLBACK_EVENT_PLAYING) {
        self->m_playing.store(true);
        self->m_revision.fetch_add(1);
    } else if (ctx->audio_event == AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE) {
        self->m_enabled.store(false);
        self->reportError("Unsupported audio file", ESP_ERR_NOT_SUPPORTED);
    }
}

void AS500Mp3Player::eventTask(void *arg)
{
    auto *self = static_cast<AS500Mp3Player *>(arg);
    PlayerEvent event{};
    uint8_t pending_address = 0;
    TickType_t settings_changed_at = 0;
    bool settings_dirty = false;
    for (;;) {
        if (xQueueReceive(self->m_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE)
            pending_address = event.address;
        const auto disconnected = self->m_disconnected_device.exchange(nullptr);
        if (disconnected && disconnected == self->m_device)
            self->m_disconnect_pending = true;
        if (self->m_disconnect_pending) self->handleDisconnected();
        if (pending_address && !self->m_disconnect_pending) {
            self->handleConnected(pending_address);
            pending_address = 0;
        }
        const uint32_t commands = self->m_commands_pending.exchange(0);
        const esp_err_t audio_error = s_audio_error.exchange(ESP_OK);
        if (audio_error != ESP_OK) {
            self->m_enabled.store(false);
            self->reportError("Audio output", audio_error);
            self->stopPlayback();
        }
        if (commands & (kEnabledChanged | kTrackChanged)) {
            if (commands & kTrackChanged)
                self->m_current_track.store(self->m_requested_track.load());
            if (self->stopPlayback() == ESP_OK && self->enabled())
                self->playCurrent();
        }
        if (commands & (kVolumeChanged | kModeChanged)) {
            settings_dirty = true;
            settings_changed_at = xTaskGetTickCount();
        }
        if (settings_dirty && xTaskGetTickCount() - settings_changed_at >= pdMS_TO_TICKS(750)) {
            self->saveSettings();
            settings_dirty = false;
        }
        if (self->m_finished.exchange(false) && self->enabled()) {
            if (!s_audio_written.load()) {
                self->m_enabled.store(false);
                self->reportError("MP3 contains no decodable audio", ESP_FAIL);
            } else {
                self->playNext();
            }
        }
    }
}

void AS500Mp3Player::handleConnected(uint8_t address)
{
    if (m_device) {
        ESP_LOGW(TAG, "Ignoring additional USB storage device");
        return;
    }
    msc_host_device_handle_t device = nullptr;
    esp_err_t err = msc_host_install_device(address, &device);
    if (err != ESP_OK) {
        reportError("USB device initialization", err);
        return;
    }
    m_device = device;
    const esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false, .max_files = 4, .allocation_unit_size = 8192,
    };
    msc_host_vfs_handle_t vfs = nullptr;
    err = msc_host_vfs_register(device, kMountPath, &mount_cfg, &vfs);
    if (err != ESP_OK) {
        reportError("USB FAT mount", err);
        m_disconnect_pending = true;
        handleDisconnected();
        return;
    }
    m_vfs = vfs;
    m_mounted.store(true);
    m_last_error.store(ESP_OK);
    scanTracks();
    ESP_LOGI(TAG, "USB mounted, %u MP3 tracks", (unsigned)m_track_count.load());
    m_revision.fetch_add(1);
    if (enabled() && m_track_count.load())
        m_commands_pending.fetch_or(kEnabledChanged);
}

void AS500Mp3Player::handleDisconnected()
{
    m_enabled.store(false);
    m_mounted.store(false);
    xSemaphoreTake(m_tracks_mutex, portMAX_DELAY);
    m_track_count.store(0);
    xSemaphoreGive(m_tracks_mutex);
    m_revision.fetch_add(1);
    if (stopPlayback() != ESP_OK) return;
    if (m_vfs) {
        const esp_err_t err = msc_host_vfs_unregister(m_vfs);
        if (err != ESP_OK) {
            reportError("USB unmount", err);
            return;
        }
        m_vfs = nullptr;
    }
    if (m_device) {
        const esp_err_t err = msc_host_uninstall_device(m_device);
        if (err != ESP_OK) {
            reportError("USB device removal", err);
            return;
        }
        m_device = nullptr;
    }
    m_current_track.store(0);
    m_requested_track.store(0);
    m_disconnect_pending = false;
}

void AS500Mp3Player::scanTracks()
{
    xSemaphoreTake(m_tracks_mutex, portMAX_DELAY);
    m_track_count.store(0);
    xSemaphoreGive(m_tracks_mutex);
    m_scan_count = 0;
    scanDirectory(kMountPath, 0);
    qsort(m_tracks, m_scan_count, sizeof(m_tracks[0]), compareTrackPath);
    xSemaphoreTake(m_tracks_mutex, portMAX_DELAY);
    m_track_count.store(m_scan_count);
    xSemaphoreGive(m_tracks_mutex);
    if (m_current_track.load() >= m_scan_count) m_current_track.store(0);
}

void AS500Mp3Player::scanDirectory(const char *path, unsigned depth)
{
    if (depth > 2 || m_scan_count >= kMaxTracks) return;
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Cannot scan %s: errno=%d", path, errno);
        reportError("USB directory scan", ESP_FAIL);
        return;
    }
    for (;;) {
        errno = 0;
        dirent *entry = readdir(dir);
        if (!entry) {
            if (errno) {
                ESP_LOGE(TAG, "Cannot read directory %s: errno=%d", path, errno);
                reportError("USB directory read", ESP_FAIL);
            }
            break;
        }
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
        char full[kMaxPath];
        if (snprintf(full, sizeof(full), "%s/%s", path, entry->d_name) >= (int)sizeof(full)) {
            ESP_LOGW(TAG, "Skipping path longer than %u bytes: %s",
                     static_cast<unsigned>(kMaxPath - 1), entry->d_name);
            continue;
        }
        struct stat info{};
        if (stat(full, &info) != 0) {
            ESP_LOGE(TAG, "Cannot stat %s: errno=%d", full, errno);
            reportError("USB file scan", ESP_FAIL);
            continue;
        }
        if (S_ISDIR(info.st_mode)) scanDirectory(full, depth + 1);
        else if (S_ISREG(info.st_mode) && hasMp3Extension(entry->d_name)) {
            strlcpy(m_tracks[m_scan_count++], full, kMaxPath);
        }
        if (m_scan_count >= kMaxTracks) {
            ESP_LOGW(TAG, "MP3 list limited to %u tracks", static_cast<unsigned>(kMaxTracks));
            break;
        }
    }
    closedir(dir);
}

void AS500Mp3Player::playCurrent()
{
    const size_t index = m_current_track.load();
    if (!m_mounted.load() || index >= m_track_count.load()) return;
    const esp_err_t init_result = ensureAudioReady();
    if (init_result != ESP_OK) {
        m_enabled.store(false);
        reportError("Audio initialization", init_result);
        return;
    }
    FILE *file = fopen(m_tracks[index], "rb");
    if (!file) {
        ESP_LOGE(TAG, "Cannot open %s: errno=%d", m_tracks[index], errno);
        m_enabled.store(false);
        reportError("MP3 file open", ESP_FAIL);
        return;
    }
    m_finished.store(false);
    s_audio_written.store(false);
    xEventGroupClearBits(m_audio_events, kAudioIdle);
    m_last_error.store(ESP_OK);
    const esp_err_t err = audio_player_play(file);
    if (err != ESP_OK) {
        fclose(file);
        xEventGroupSetBits(m_audio_events, kAudioIdle);
        m_enabled.store(false);
        reportError("MP3 playback request", err);
    } else {
        m_revision.fetch_add(1);
    }
}

esp_err_t AS500Mp3Player::stopPlayback()
{
    if (m_audio_ready.load() &&
        !(xEventGroupGetBits(m_audio_events) & kAudioIdle)) {
        const TickType_t started_at = xTaskGetTickCount();
        bool stop_sent = false;
        bool retry_logged = false;
        // IDLE is dispatched after the decoder has closed its owned FILE.
        while (!(xEventGroupGetBits(m_audio_events) & kAudioIdle) &&
               xTaskGetTickCount() - started_at < pdMS_TO_TICKS(5000)) {
            if (!stop_sent) {
                const esp_err_t err = audio_player_stop();
                stop_sent = err == ESP_OK;
                if (err != ESP_OK && !retry_logged) {
                    ESP_LOGW(TAG, "Audio stop not accepted yet (%s); retrying",
                             esp_err_to_name(err));
                    retry_logged = true;
                }
            }
            xEventGroupWaitBits(m_audio_events, kAudioIdle, pdFALSE, pdTRUE,
                                pdMS_TO_TICKS(20));
        }
        if (!(xEventGroupGetBits(m_audio_events) & kAudioIdle)) {
            m_enabled.store(false);
            reportError("Waiting for audio file closure", ESP_ERR_TIMEOUT);
            return ESP_ERR_TIMEOUT;
        }
    }
    m_finished.store(false);
    m_playing.store(false);
    m_revision.fetch_add(1);
    return ESP_OK;
}

esp_err_t AS500Mp3Player::ensureAudioReady()
{
    if (m_audio_ready.load()) return ESP_OK;
    if (!s_speaker) s_speaker = bsp_audio_codec_speaker_init();
    if (!s_speaker) return ESP_FAIL;
    ESP_RETURN_ON_ERROR(configureAudio(44100, 16, I2S_SLOT_MODE_STEREO),
                        TAG, "codec open");
    audio_player_config_t audio_cfg = {
        .mute_fn = muteAudio,
        .clk_set_fn = configureAudio,
        .write_fn = writeAudio,
        .priority = 5,
        .coreID = 0,
        .force_stereo = true,
    };
    if (!m_audio_created) {
        ESP_RETURN_ON_ERROR(audio_player_new(audio_cfg), TAG, "audio player init");
        m_audio_created = true;
    }
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
        next = (previous + 1 + esp_random() % (count - 1)) % count;
    } else next = (next + 1) % count;
    m_current_track.store(next);
    playCurrent();
}

void AS500Mp3Player::loadSettings()
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("as500_mp3", NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) return;
    if (err != ESP_OK) {
        reportError("MP3 settings load", err);
        return;
    }
    uint8_t volume = 60, mode = 0;
    err = nvs_get_u8(nvs, "volume", &volume);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) reportError("Volume load", err);
    err = nvs_get_u8(nvs, "mode", &mode);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) reportError("Playback mode load", err);
    nvs_close(nvs);
    if (volume > 100 || mode > 1) {
        reportError("Invalid saved MP3 settings", ESP_ERR_INVALID_ARG);
        return;
    }
    m_volume.store(volume);
    m_mode.store(static_cast<PlayMode>(mode));
}

void AS500Mp3Player::saveSettings()
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("as500_mp3", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        reportError("MP3 settings open", err);
        return;
    }
    err = nvs_set_u8(nvs, "volume", m_volume.load());
    if (err == ESP_OK) err = nvs_set_u8(nvs, "mode", static_cast<uint8_t>(m_mode.load()));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) reportError("MP3 settings save", err);
}

void AS500Mp3Player::reportError(const char *operation, esp_err_t error)
{
    ESP_LOGE(TAG, "%s: %s (0x%x)", operation, esp_err_to_name(error),
             static_cast<unsigned>(error));
    m_last_error.store(error);
    m_revision.fetch_add(1);
}
