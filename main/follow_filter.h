#pragma once

namespace FollowFilter {
constexpr float farWeight(float distance_cm) {
    return distance_cm <= 150 ? 0 : distance_cm >= 400 ? 1 :
        (distance_cm - 150) / 250;
}
struct DistanceRatios { float steering_percent, filter_percent; };
// Fixed 0..20 m trial profile; interpolate continuously, without menu settings.
constexpr DistanceRatios distanceRatios(float distance_cm) {
    constexpr float points[] = {100, 300, 500, 1000, 2000};
    constexpr DistanceRatios ratios[] = {{100,100}, {200,80}, {300,60}, {400,50}, {500,40}};
    if (distance_cm <= points[0]) return ratios[0];
    for (int i = 1; i < 5; ++i) {
        if (distance_cm <= points[i]) {
            const float t = (distance_cm - points[i-1]) / (points[i] - points[i-1]);
            return {ratios[i-1].steering_percent + t * (ratios[i].steering_percent - ratios[i-1].steering_percent),
                    ratios[i-1].filter_percent + t * (ratios[i].filter_percent - ratios[i-1].filter_percent)};
        }
    }
    return ratios[4];
}
constexpr float steeringEntry(float distance_cm) { return 2 * distanceRatios(distance_cm).steering_percent / 100; }
constexpr float steeringExit(float distance_cm) { return steeringEntry(distance_cm) * .5f; }
constexpr float alignmentExit(float distance_cm) { return 10 + 10 * farWeight(distance_cm); }
}
