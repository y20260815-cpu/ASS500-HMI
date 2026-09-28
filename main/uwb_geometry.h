#pragma once
#include <cmath>
#include <algorithm>

namespace UwbGeometry {
// Triangle-inequality tolerance, not calibrated sensor accuracy.
// Equal-confidence projection: 30 cm near, 45 cm at 4 m, capped at 100 cm.
constexpr float kMaxExcessCm = 30.0f;
constexpr float excessLimitCm(float mean_range_cm) {
    if (mean_range_cm <= 150) return kMaxExcessCm;
    if (mean_range_cm <= 400) return 30 + (mean_range_cm - 150) * (15.0f / 250);
    if (mean_range_cm < 1000) return 45 + (mean_range_cm - 400) * (55.0f / 600);
    return 100;
}
constexpr float kLateralGapCm = 80.0f;
constexpr float kLongitudinalGapCm = 0.0f;
// Vehicle ranging/control origin is the midpoint of the rear A0-A1 line.
constexpr float kAnchorForwardCm = 0.0f;
constexpr float kAnchorBaselineCm = 80.0f;
constexpr float kBaselineCm = kLateralGapCm;
constexpr float kDw3000RangeErrorCm = 10.0f;
constexpr float kAmbiguityErrorMultiplier = 3.0f;
// True where the two circle-intersection solutions have indistinguishable
// left/right offsets within the configured DW3000 range error.
inline bool ambiguityZone(float forward_cm, float right_cm,
                          float error_cm = kDw3000RangeErrorCm,
                          float multiplier = kAmbiguityErrorMultiplier) {
    (void)right_cm;
    return std::fabs(forward_cm-kAnchorForwardCm)<=multiplier*error_cm;
}
struct Ranges {
    float left, right;
    bool valid, corrected;
};
constexpr Ranges reconcile(float left, float right, float baseline,
                           unsigned mask) {
    if ((mask & 3) != 3 || !(left > 0 && left <= 32767) ||
        !(right > 0 && right <= 32767) || !(baseline > 0 && baseline <= 32767))
        return {left, right, false, false};
    const float difference = left > right ? left - right : right - left;
    const float deficit = baseline - (left + right);
    const float excess = difference - baseline;
    const float limit = excessLimitCm((left + right) * 0.5f);
    if (deficit > limit || excess > limit)
        return {left, right, false, false};
    if (deficit > 0) {
        left += deficit * 0.5f;
        right += deficit * 0.5f;
        return {left, right, true, true};
    }
    if (excess > 0) {
        const float half = excess * 0.5f;
        if (left > right) { left -= half; right += half; }
        else { left += half; right -= half; }
        return {left, right, true, true};
    }
    return {left, right, true, false};
}
struct PoseUncertainty { float distance_cm, angle_deg; bool complete; };
// Signed separation of the two mirrored candidates' absolute lateral
// offsets. A zero crossing means the path crossed the ambiguity strip even
// when no 100 ms sample landed inside it.
inline float branchBalance(float left_cm,float right_cm,
                           float lateral_gap_cm=kLateralGapCm){
    const auto p=reconcile(left_cm,right_cm,lateral_gap_cm,3);
    if(!p.valid)return NAN;
    const float along=(p.left*p.left+lateral_gap_cm*lateral_gap_cm-p.right*p.right)/(2*lateral_gap_cm);
    const float normal=std::sqrt(std::max(0.0f,p.left*p.left-along*along));
    return normal;
}
// Propagate the +/-10 cm error of both DW3000 ranges through the same
// two-circle geometry used for control. Each error corner is matched to the
// currently selected intersection, so the tolerance follows the geometry.
inline PoseUncertainty poseUncertainty(float left_cm, float right_cm,
                                       float selected_forward_cm,
                                       float selected_right_cm,
                                       float error_cm = kDw3000RangeErrorCm,
                                       float lateral_gap_cm = kLateralGapCm) {
    const float selected_distance=std::hypot(selected_forward_cm,selected_right_cm);
    const float selected_angle=std::atan2(selected_right_cm,selected_forward_cm);
    float max_distance_error=0,max_angle_error=0; int valid_corners=0;
    for(int il=-1;il<=1;il+=2)for(int ir=-1;ir<=1;ir+=2){
        const auto p=reconcile(left_cm+il*error_cm,right_cm+ir*error_cm,lateral_gap_cm,3);
        if(!p.valid)continue;
        const float along=(p.left*p.left+lateral_gap_cm*lateral_gap_cm-p.right*p.right)/(2*lateral_gap_cm);
        const float normal=std::sqrt(std::max(0.0f,p.left*p.left-along*along));
        float bf=0,br=0,bd=1e30f;
        for(int side=-1;side<=1;side+=2){
            const float f=kAnchorForwardCm+side*normal;
            const float r=-lateral_gap_cm*.5f+along,df=f-selected_forward_cm,dr=r-selected_right_cm;
            const float d=df*df+dr*dr;if(d<bd){bd=d;bf=f;br=r;}
        }
        max_distance_error=std::max(max_distance_error,std::fabs(std::hypot(bf,br)-selected_distance));
        float da=std::atan2(br,bf)-selected_angle;
        while(da>3.14159265f) da-=6.28318531f;
        while(da< -3.14159265f) da+=6.28318531f;
        max_angle_error=std::max(max_angle_error,std::fabs(da)*57.2957795f);++valid_corners;
    }
    return {std::clamp(max_distance_error,error_cm,2.0f*error_cm),
            std::clamp(max_angle_error,2.0f,45.0f),valid_corners==4};
}
}
