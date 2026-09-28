#pragma once

namespace FollowSteering {

struct MotorPair { int left, right; };

constexpr int magnitude(int value) { return value < 0 ? -value : value; }
constexpr int sign(int value) { return (value > 0) - (value < 0); }

constexpr int scale(int command, float percent) {
    const float bounded = percent < -100.0f ? -100.0f :
                          percent > 100.0f ? 100.0f : percent;
    const float value = command * bounded * 0.01f;
    return (int)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

/* The differential controller has already selected the inner wheel by making
   its command smaller. Positive settings retain a percentage of that command.
   Negative settings reverse a percentage of the unused outer-minus-inner
   command: outer=100, inner=70 becomes -30 at setting -100. */
constexpr MotorPair applyInnerPercent(int left, int right, float percent) {
    if (left == right) return {left, right};
    const int outer = magnitude(left) > magnitude(right) ?
                      magnitude(left) : magnitude(right);
    if (magnitude(left) < magnitude(right)) {
        const int adjusted = percent >= 0.0f ? scale(left, percent) :
            scale(sign(left ? left : right) * (outer - magnitude(left)), percent);
        return {adjusted, right};
    }
    if (magnitude(right) < magnitude(left)) {
        const int adjusted = percent >= 0.0f ? scale(right, percent) :
            scale(sign(right ? right : left) * (outer - magnitude(right)), percent);
        return {left, adjusted};
    }
    return {left, right};
}

} // namespace FollowSteering
