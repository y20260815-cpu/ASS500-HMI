#pragma once

namespace RemoteDrive {
struct Command { int left, right; bool valid; };
// For a travelling turn, slow_percent is the inner-wheel output relative to
// the outer wheel: +100=same direction/speed, 0=stopped, -100=full reverse.
constexpr Command mix(unsigned keys, int rpm, int slow_percent = 50) {
    const unsigned d = keys & 15;
    if ((d & 3) == 3 || (d & 12) == 12) return {0, 0, false};
    int l = (d & 1) ? rpm : (d & 2) ? -rpm : 0;
    int r = l;
    if (l) {
        // LEFT/RIGHT always denotes the vehicle nose's yaw direction.
        const int ratio = slow_percent < -100 ? -100 :
                          slow_percent > 100 ? 100 : slow_percent;
        const int scaled = rpm * ratio;
        const int inner = (scaled + (scaled >= 0 ? 50 : -50)) / 100;
        if (d & 4) { if (l > 0) l = inner; else r = -inner; }
        if (d & 8) { if (l > 0) r = inner; else l = -inner; }
    } else if (d & 4) { l = -rpm; r = rpm; }
    else if (d & 8) { l = rpm; r = -rpm; }
    return {l, r, d != 0};
}
constexpr bool rangeGrace(long long now, long long last) {
    return last > 0 && now >= last && now - last <= 300000;
}
constexpr int assisted(int side, int left, int right, int floor) {
    const int al = left < 0 ? -left : left;
    const int ar = right < 0 ? -right : right;
    const int peak = al > ar ? al : ar;
    return peak > 0 && peak < floor ? side * floor / peak : side;
}
}
