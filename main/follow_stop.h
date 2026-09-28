#pragma once
namespace FollowStop {
constexpr float abs(float x){return x<0?-x:x;}
enum class Action { Drive, Align, Stop };
struct Alignment {
    bool settled=false;
    constexpr Action update(float distance_cm,float stop_cm,float heading_error,
                            float distance_error_cm=10,float angle_error_deg=3){
        const float error=abs(heading_error);
        const float enter_angle=angle_error_deg>3?angle_error_deg:3;
        const float leave_angle=enter_angle*1.5f;
        const float leave_distance=stop_cm+2*distance_error_cm;
        const bool in_stop=distance_cm<=stop_cm || (settled&&distance_cm<=leave_distance);
        if(settled&&distance_cm<=leave_distance&&error<=leave_angle)return Action::Stop;
        settled=false;
        if(in_stop&&error<=enter_angle){settled=true;return Action::Stop;}
        if(in_stop&&error>enter_angle)return Action::Align;
        return Action::Drive;
    }
};
constexpr int rpm(float error,int configured){
    const int floor=configured<750?750:configured>2500?2500:configured;
    const int proportional=(int)(abs(error)*20+.5f);
    const int magnitude=proportional<floor?floor:proportional>2500?2500:proportional;
    return error<0?-magnitude:magnitude;
}
}
