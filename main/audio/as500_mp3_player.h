#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "audio_player.h"
#include "usb/msc_host.h"

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
    const char *trackName(size_t index) const;

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
    char m_tracks[kMaxTracks][kMaxPath]{};
    void *m_queue = nullptr;
    void *m_device = nullptr;
    void *m_vfs = nullptr;
};
