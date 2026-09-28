#pragma once
// Tagged 0x701 timing profile, preserving DLC=8 and legacy byte3 battery codes.
// byte2: acceleration in 0.1s; byte3: 0x80 | deceleration in 0.1s.
struct DriveTiming {
    static constexpr unsigned encode(float seconds) {
        return !(seconds >= 0.5f) ? 5u : seconds >= 5.0f ? 50u :
               (unsigned)(seconds * 10.0f + 0.5f);
    }
    static constexpr bool valid(unsigned accel, unsigned byte3) {
        return (byte3 & 0x80u) && accel >= 5 && accel <= 50 &&
               (byte3 & 0x7fu) >= 5 && (byte3 & 0x7fu) <= 50;
    }
    static constexpr unsigned accelIndex(bool follow) { return follow ? 5 : 6; }
    static constexpr unsigned decelIndex(bool follow) { return follow ? 4 : 7; }
    // BEMF RPM is diagnostic during pivot; do not let it skew wheel PWM.
    static constexpr bool pivotFeedbackBypass(bool can, bool tagged,
        float left, float right, float output_left, float output_right) {
        return can && tagged &&
            (left * right < 0.0f || output_left * output_right < 0.0f);
    }
    static constexpr bool innerZero(bool can, bool tagged, int command, int other) {
        return can && tagged && command == 0 && other != 0;
    }
    static constexpr float decelStep(unsigned raw, unsigned elapsed_ms) {
        return float(elapsed_ms) / float(raw * 100u);
    }
};
