#pragma once

#include <algorithm>
#include <cmath>

namespace UwbKalman {

constexpr float kPi = 3.14159265358979323846f;
inline float wrapRadians(float value) {
    while (value > kPi) value -= 2.0f * kPi;
    while (value < -kPi) value += 2.0f * kPi;
    return value;
}

// Two-state (position, velocity) constant-velocity Kalman filter.  The
// acceleration noise controls how readily movement is followed, while the
// measurement variance expresses the DW3000 or geometry uncertainty.
class ConstantVelocity {
public:
    void reset() { *this = {}; }
    bool ready() const { return ready_; }
    float value() const { return position_; }
    float velocity() const { return velocity_; }

    float peek(long long now_us) const {
        if (!ready_) return 0.0f;
        return position_ + velocity_ * elapsed(now_us);
    }

    float predict(long long now_us, float acceleration_std) {
        if (!ready_) return 0.0f;
        advance(now_us, acceleration_std);
        return position_;
    }

    float update(float measurement, long long now_us,
                 float measurement_std, float acceleration_std) {
        if (!ready_) {
            ready_ = true;
            position_ = measurement;
            velocity_ = 0.0f;
            p00_ = measurement_std * measurement_std;
            p01_ = p10_ = 0.0f;
            p11_ = acceleration_std * acceleration_std;
            time_us_ = now_us;
            return position_;
        }
        advance(now_us, acceleration_std);
        const float variance = std::max(0.01f, measurement_std * measurement_std);
        const float innovation = measurement - position_;
        const float residual_variance = p00_ + variance;
        const float k0 = p00_ / residual_variance;
        const float k1 = p10_ / residual_variance;
        const float old_p00 = p00_, old_p01 = p01_;
        position_ += k0 * innovation;
        velocity_ += k1 * innovation;
        p00_ -= k0 * old_p00;
        p01_ -= k0 * old_p01;
        p10_ -= k1 * old_p00;
        p11_ -= k1 * old_p01;
        return position_;
    }

protected:
    float elapsed(long long now_us) const {
        if (now_us <= time_us_) return 0.0f;
        return std::clamp((now_us - time_us_) * 1.0e-6f, 0.0f, 0.5f);
    }

    void advance(long long now_us, float acceleration_std) {
        const float dt = elapsed(now_us);
        if (dt <= 0.0f) return;
        position_ += velocity_ * dt;
        const float pp00 = p00_ + dt * (p10_ + p01_) + dt * dt * p11_;
        const float pp01 = p01_ + dt * p11_;
        const float pp10 = p10_ + dt * p11_;
        const float pp11 = p11_;
        const float q = acceleration_std * acceleration_std;
        const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
        p00_ = pp00 + q * dt4 * 0.25f;
        p01_ = pp01 + q * dt3 * 0.5f;
        p10_ = pp10 + q * dt3 * 0.5f;
        p11_ = pp11 + q * dt2;
        time_us_ = now_us;
    }

    bool ready_ = false;
    float position_ = 0.0f, velocity_ = 0.0f;
    float p00_ = 0.0f, p01_ = 0.0f, p10_ = 0.0f, p11_ = 0.0f;
    long long time_us_ = 0;
};

class Angle : public ConstantVelocity {
public:
    float peek(long long now_us) const {
        return wrapRadians(ConstantVelocity::peek(now_us));
    }
    float predict(long long now_us, float acceleration_std) {
        const float result = ConstantVelocity::predict(now_us, acceleration_std);
        position_ = wrapRadians(position_);
        return wrapRadians(result);
    }
    float update(float measurement, long long now_us,
                 float measurement_std, float acceleration_std) {
        if (!ready_) return ConstantVelocity::update(
            wrapRadians(measurement), now_us, measurement_std, acceleration_std);
        const float predicted = peek(now_us);
        const float continuous_measurement = predicted + wrapRadians(measurement - predicted);
        const float result = ConstantVelocity::update(
            continuous_measurement, now_us, measurement_std, acceleration_std);
        position_ = wrapRadians(position_);
        return wrapRadians(result);
    }
};

} // namespace UwbKalman
