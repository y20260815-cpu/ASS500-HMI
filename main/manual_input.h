#pragma once

// Pure timing logic; no LVGL or hardware dependencies.
struct ManualPress {
    enum Kind { None, Tap, Hold, Stop };
    struct Action { Kind kind; int direction; };
    static constexpr unsigned long long ShortUs = 300000;
    static constexpr unsigned long long LongUs = 600000;
    int direction = 0;
    unsigned long long since = 0;
    bool active = false;
    bool held = false;
    constexpr Action begin(int value, unsigned long long now) {
        direction = value; since = now; active = true; held = true;
        return {Hold, direction};
    }
    constexpr void cancel() { active = false; held = false; }
    constexpr Action update(unsigned long long) { return {None, 0}; }
    constexpr Action release(unsigned long long, bool) {
        if (!active) return {None, 0};
        active = false; held = false;
        return {Stop, direction};
    }
};

struct ManualJog {
    static constexpr int Rpm = 750;
    static constexpr int TurnRpm = 1000;
    // STM32 limits straight taps to 30%, pivot taps to 40%, for at most 500 ms.
    // This is a command window, not a guaranteed movement or travel time.
    static constexpr unsigned AccelRaw = 10;
    static constexpr unsigned long long DurationUs = 500000;
    enum Phase { Idle, Clear, Armed, Running };
    Phase phase = Idle;
    unsigned long long deadline = 0;
    constexpr void begin() { phase = Clear; deadline = 0; }
    constexpr void cancel() { phase = Idle; deadline = 0; }
    constexpr bool active() const { return phase != Idle; }
    constexpr bool output(unsigned long long now) {
        // Always send a zero frame before a new tap, including reversals.
        if (phase == Clear) { phase = Armed; return false; }
        if (phase == Armed) { phase = Running; deadline = now + DurationUs; }
        if (phase == Running && now < deadline) return true;
        cancel(); return false;
    }
};

// Byte2=10 is reserved for the bounded legacy tap assist in STM32.
// Every hold-to-run phase uses the same STM32 S-curve, 1.1 s at full scale.
// Changing duration during an active curve can move its phase backwards.
constexpr unsigned manualDriveAccelRaw(bool tap, bool manual) {
    return tap ? ManualJog::AccelRaw : (manual ? 11u : 20u);
}

// Combined evidence only; raw driver RPM is not proof of shaft motion.
struct StartupEvidence {
    int baseline_current = 0, baseline_left = 0, baseline_right = 0;
    int direction_left = 0, direction_right = 0, peak = 0, hits = 0;
    bool valid = false;
    unsigned long long last_sample = 0, last_current = 0;
    static constexpr int magnitude(int v) { return v < 0 ? -v : v; }
    constexpr void reset(int amps, int left, int right, int dl, int dr, bool fresh) {
        baseline_current = peak = amps;
        baseline_left = magnitude(left); baseline_right = magnitude(right);
        direction_left = dl; direction_right = dr; valid = fresh;
        hits = 0; last_sample = last_current = 0;
    }
    constexpr bool sample(unsigned long long now, unsigned long long current_time,
                          unsigned long long since, int amps, int left, int right,
                          unsigned long long window = 600000) {
        if (!valid || now < since || now - since >= window ||
            current_time <= since || current_time <= last_current ||
            now < current_time || now - current_time > 150000) {
            hits = 0; return false;
        }
        if (last_sample && now - last_sample < 80000) return false;
        if (last_sample && now - last_sample > 200000) hits = 0;
        last_sample = now; last_current = current_time;
        if (amps > peak) { peak = amps; hits = 0; }
        const int drop = peak - amps;
        const bool current_fall = peak >= baseline_current + 5 && peak >= 10 &&
                                  drop >= 3 && drop * 5 >= peak;
        const bool rpm_rise = left * direction_left >= 150 &&
                              right * direction_right >= 150 &&
                              magnitude(left) >= baseline_left + 100 &&
                              magnitude(right) >= baseline_right + 100;
        if (now - since >= 200000 && current_fall && rpm_rise) ++hits;
        else hits = 0;
        return hits >= 2;
    }
};

// Bounded hold-to-run startup command floor, independent of cruise speed.
// Values are command units (2500 = full scale), not measured wheel RPM.
struct ManualStartup {
    static constexpr unsigned long long WindowUs = 600000;
    static constexpr int StraightCommand = 750;
    static constexpr int PivotCommand = 2500;
    bool armed = false;
    bool running = false;
    unsigned long long since = 0;
    unsigned long long window = WindowUs;
    StartupEvidence evidence;
    bool estimated = false;
    constexpr void begin(unsigned long long limit = WindowUs) {
        window = limit;
        armed = true; running = false; since = 0; estimated = false;
        evidence = StartupEvidence{};
    }
    constexpr bool observe(unsigned long long now, unsigned long long current_time,
                           int amps, int left, int right) {
        if (!running) return false;
        if (!evidence.sample(now, current_time, since, amps, left, right, window)) return false;
        running = false; estimated = true; return true;
    }
    constexpr void cancel() { armed = running = false; }
    constexpr bool update(unsigned long long now) {
        if (armed) { armed = false; running = true; since = now; }
        if (running && now - since >= window) running = false;
        return running;
    }
    static constexpr int command(int cruise, bool pivot) {
        if (cruise == 0) return 0;
        const int floor = pivot ? PivotCommand : StraightCommand;
        if (cruise > 0) return cruise < floor ? floor : cruise;
        return cruise > -floor ? -floor : cruise;
    }
};
