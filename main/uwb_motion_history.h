#pragma once

// Select the front/rear intersection of two horizontally mounted anchors
// from continuity. Initial symmetric ambiguity defaults to the front branch.
namespace UwbMotion {
constexpr float abs(float x) { return x < 0 ? -x : x; }
constexpr float clamp(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
struct History {
    bool ready = false, behind_line = false, crossing_armed = false;
    float normal = 0, velocity = 0;
    long long time_us = 0;
    constexpr void flip() {
        behind_line = !behind_line;
        normal = -normal;
        velocity = -velocity;
        crossing_armed = false;
    }
    constexpr float select(float unsigned_normal_cm, long long now) {
        const float n = abs(unsigned_normal_cm);
        if (!ready) {
            ready = true; normal = behind_line ? -n : n; time_us = now;
            crossing_armed = n >= 50.0f;
            return normal;
        }
        const long long elapsed = now - time_us;
        const bool continuous = elapsed > 0 && elapsed <= 300000;
        const float dt = continuous ? elapsed * .000001f : 0;
        if (continuous) {
            const float predicted = normal + velocity * dt;
            const float front_error = abs(n - predicted);
            const float rear_error = abs(-n - predicted);
            // A completed crossing is latched. Re-arm only after the target
            // has moved well away from the antenna line; otherwise noise near
            // the line can alternate the two mirror solutions every 100 ms.
            if (!crossing_armed && n >= 50.0f) crossing_armed = true;
            if (crossing_armed && abs(normal) <= 25.0f) {
                const bool previous = behind_line;
                if (rear_error + 4 < front_error) behind_line = true;
                else if (front_error + 4 < rear_error) behind_line = false;
                if (behind_line != previous) crossing_armed = false;
            }
        } else velocity = 0;
        const float next = behind_line ? -n : n;
        if (continuous) {
            const float measured = clamp((next - normal) / dt, -400, 400);
            velocity += .5f * (measured - velocity);
        }
        normal = next; time_us = now;
        return normal;
    }
};
}
