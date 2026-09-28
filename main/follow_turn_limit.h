#pragma once

namespace FollowTurnLimit {

constexpr int rpm(float percent, int full_scale = 2500) {
    if (percent <= 0.0f || full_scale <= 0) return 0;
    if (percent >= 100.0f) return full_scale;
    return (int)(full_scale * percent * 0.01f + 0.5f);
}

constexpr int apply(int command, float percent, int full_scale = 2500) {
    const int limit = rpm(percent, full_scale);
    return command > limit ? limit : command < -limit ? -limit : command;
}

constexpr int scale(int command, float percent) {
    if (percent <= 0.0f) return 0;
    if (percent >= 100.0f) return command;
    const float scaled = command * percent * 0.01f;
    return (int)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

constexpr bool pivot(int left, int right) {
    return (left < 0 && right > 0) || (left > 0 && right < 0);
}

} // namespace FollowTurnLimit
