#pragma once

namespace FollowDirection {
constexpr float wrap(float degrees) {
    while (degrees > 180) degrees -= 360;
    while (degrees < -180) degrees += 360;
    return degrees;
}

// Follow travel is forward-only. If startup geometry is in the DW3000
// front/rear branch is unresolved, probe straight ahead for at most 50 cm and request the
// opposite circle intersection only when the measured distance grows.
struct Selector {
    bool ready=false, probing=false, recovery=false, changed=false, failed=false;
    float start_distance=0, best_distance=0, probe_travel_cm=0;
    long long last_us=0;

    constexpr int update(float bearing, float distance, bool shadow,
                         float commanded_mps, long long now) {
        const float angle=wrap(bearing);
        const float magnitude=angle<0?-angle:angle;
        changed=false;failed=false;
        if (!ready) {
            ready=true; probing=shadow; recovery=!shadow&&magnitude>45.0f;
            start_distance=best_distance=distance;last_us=now;
            return 1;
        }
        const long long elapsed=now-last_us;last_us=now;
        if(probing&&elapsed>0&&elapsed<=300000)
            probe_travel_cm+=(commanded_mps<0?-commanded_mps:commanded_mps)*
                             (float)elapsed*.000001f*100.0f;
        if(distance<best_distance)best_distance=distance;
        if(probing){
            const bool clearly_closer=distance<=start_distance-10.0f;
            const bool clearly_farther=distance>=start_distance+20.0f;
            const bool limit=probe_travel_cm>=50.0f;
            if(clearly_closer){probing=false;recovery=magnitude>45.0f;}
            else if(clearly_farther||(limit&&distance>start_distance+10.0f)){
                probing=false;changed=true;recovery=true;
            }else if(limit){probing=false;failed=true;recovery=false;}
        }
        // Follow translation is forward-only. If the TAG moves behind the
        // vehicle after a previous recovery completed, re-enter rotation;
        // otherwise forward steering produces a wide orbit away from it.
        if(!probing&&magnitude>=90.0f)recovery=true;
        if(recovery&&magnitude<=45.0f)recovery=false;
        return 1;
    }
    constexpr float steeringBearing(float bearing) const { return wrap(bearing); }
};
}
