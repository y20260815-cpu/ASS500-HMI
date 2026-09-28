#pragma once
#include "manual_input.h"

// Rearm only after a sustained zero request. No retries on noisy steering.
struct FollowStartup {
    ManualStartup assist;
    bool ready = false, zero_tracking = false;
    unsigned long long zero_since = 0;
    int left_sign = 0, right_sign = 0;
    int last_left = 0, last_right = 0;
    static constexpr int sign(int value) { return (value > 0) - (value < 0); }
    constexpr bool update(unsigned long long now, int left, int right, bool allowed,
                          int amps, int rpm_left, int rpm_right,
                          unsigned long long pivot_window_us = 1000000) {
        if (left == 0 && right == 0) {
            assist.cancel();
            if (!zero_tracking) { zero_tracking = true; zero_since = now; }
            if (now - zero_since >= 500000) ready = true;
            return false;
        }
        zero_tracking = false;
        if (!allowed) { assist.cancel(); ready = false; return false; }
        const bool pivot = sign(left) * sign(right) < 0;
        if (ready) {
            ready = false; assist.begin(pivot ? pivot_window_us : 600000);
            left_sign = sign(left); right_sign = sign(right);
            assist.evidence.reset(amps, rpm_left, rpm_right,
                                  left_sign, right_sign, true);
            last_left = last_right = 0;
        }
        if (sign(left) != left_sign || sign(right) != right_sign) assist.cancel();
        return assist.update(now);
    }
    // Scale both sides equally, preserving curved-follow steering and zero wheels.
    static constexpr int command(int side, int left, int right) {
        const int al = StartupEvidence::magnitude(left);
        const int ar = StartupEvidence::magnitude(right);
        const int maximum = al > ar ? al : ar;
        if (!maximum) return 0;
        const int floor = sign(left) * sign(right) < 0 ? ManualStartup::PivotCommand : ManualStartup::StraightCommand;
        return maximum >= floor ? side : side * floor / maximum;
    }
    constexpr void noteCommand(int left, int right) {
        // A current drop after changing the request is not startup evidence.
        if (StartupEvidence::magnitude(left-last_left) >= 25 ||
            StartupEvidence::magnitude(right-last_right) >= 25) {
            assist.evidence.peak = assist.evidence.baseline_current;
            assist.evidence.hits = 0;
        }
        last_left = left; last_right = right;
    }
};

// In-place tracking needs sustained drive after the startup boost. Linear
// following speed must not attenuate this separate rotation command.
constexpr int followPivotCommand(int direction, float angle_degrees, int configured) {
    if (!direction || angle_degrees <= 10.0f) return 0;
    // Maximum follow speed limits translation, not an in-place turn. Keep the
    // full command after startup assist so rotation does not fade under load.
    (void)configured;
    int magnitude = 2500;
    if (angle_degrees < 20.0f) {
        const float phase = (angle_degrees - 10.0f) / 10.0f;
        const float smooth = phase * phase * (3.0f - 2.0f * phase);
        magnitude = (int)(magnitude * smooth + 0.5f);
    }
    return direction < 0 ? -magnitude : magnitude;
}
