#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "audio_player.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"

class AS500Mp3Player {
public:
    enum class PlayMode : uint8_t { Sequential = 0, Random = 1 };
    static constexpr size_t kMaxTracks = 64;
    static constexpr size_t kMaxPath = 192;

    static AS500Mp3Player &instance();

    esp_err_t begin();
    void setEnabled(bool enabled);
    void toggle();
    bool enabled() const { return m_enabled.load(); }
    bool mounted() const { return m_mounted.load(); }
    bool playing() const { return m_playing.load(); }

    void setVolume(uint8_t volume);
    uint8_t volume() const { return m_volume.load(); }
    void setMode(PlayMode mode);
    PlayMode mode() const { return m_mode.load(); }
    void selectTrack(size_t index);
    size_t trackCount() const { return m_track_count.load(); }
    size_t currentTrack() const { return m_current_track.load(); }
    bool copyTrackName(size_t index, char *name, size_t size) const;
    esp_err_t lastError() const { return m_last_error.load(); }
    uint32_t revision() const { return m_revision.load(); }

private:
    AS500Mp3Player() = default;
    static void eventTask(void *arg);
    static void mscEvent(const msc_host_event_t *event, void *arg);
    static void audioEvent(audio_player_cb_ctx_t *ctx);
    void handleConnected(uint8_t address);
    void handleDisconnected();
    void scanTracks();
    void scanDirectory(const char *path, unsigned depth);
    esp_err_t ensureAudioReady();
    void playCurrent();
    void playNext();
    esp_err_t stopPlayback();
    void reportError(const char *operation, esp_err_t error);
    void loadSettings();
    void saveSettings();

    std::atomic<bool> m_enabled{false};
    std::atomic<bool> m_mounted{false};
    std::atomic<bool> m_playing{false};
    std::atomic<bool> m_audio_ready{false};
    std::atomic<uint8_t> m_volume{60};
    std::atomic<PlayMode> m_mode{PlayMode::Sequential};
    std::atomic<size_t> m_track_count{0};
    std::atomic<size_t> m_current_track{0};
    std::atomic<size_t> m_requested_track{0};
    std::atomic<esp_err_t> m_last_error{ESP_OK};
    std::atomic<uint32_t> m_revision{0};
    std::atomic<bool> m_finished{false};
    std::atomic<uint32_t> m_commands_pending{0};
    char m_tracks[kMaxTracks][kMaxPath]{};
    size_t m_scan_count = 0;
    bool m_usb_started = false;
    bool m_service_ready = false;
    bool m_msc_installed = false;
    bool m_audio_created = false;
    std::atomic<msc_host_device_handle_t> m_disconnected_device{nullptr};
    bool m_disconnect_pending = false;
    QueueHandle_t m_queue = nullptr;
    SemaphoreHandle_t m_tracks_mutex = nullptr;
    EventGroupHandle_t m_audio_events = nullptr;
    msc_host_device_handle_t m_device = nullptr;
    msc_host_vfs_handle_t m_vfs = nullptr;
};
