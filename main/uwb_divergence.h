#pragma once
#include <cmath>
namespace UwbMotion {
// Command-based hypothesis check, not a measurement of actual wheel travel.
struct Divergence {
    bool ready=false, side=false;
    long long last=0, start=0, cooldown=0;
    float previous[2]={}, growth=0, error[2]={};
    int samples=0, direction=0;
    bool update(float normal, float lateral, bool behind, float left_cm_s,
                float right_cm_s, bool enabled, long long now) {
        const float forward[2]={60+(behind?-normal:normal),60+(behind?normal:-normal)};
        const float distance[2]={std::hypot(forward[0],lateral),std::hypot(forward[1],lateral)};
        const float speed=(left_cm_s+right_cm_s)*.5f;
        const int sign=speed>0?1:-1;
        const long long elapsed=now-last;
        const bool usable=enabled && std::fabs(speed)>=20 &&
            std::fabs(left_cm_s-right_cm_s)<=std::fabs(speed)*.25f &&
            speed*forward[0]>0 && distance[0]>100 && now>=cooldown;
        const bool continuous=ready&&side==behind&&direction==sign&&elapsed>0&&elapsed<=300000;
        if(!usable||!continuous){start=now;growth=error[0]=error[1]=0;samples=0;}
        else {
            const float dt=elapsed*.000001f;
            const float delta=distance[0]-previous[0];
            if(std::fabs(delta)>300*dt){start=now;growth=error[0]=error[1]=0;samples=0;}
            else {
                growth+=delta;
                for(int i=0;i<2;i++)error[i]+=std::fabs(distance[i]-previous[i]+speed*forward[i]/distance[i]*dt);
                ++samples;
            }
        }
        bool correct=usable&&continuous&&now-start>=1000000&&samples>=10&&growth>=30&&error[1]+15<error[0];
        if(correct){cooldown=now+3000000;ready=false;}
        else {ready=usable;if(now-start>=2000000){start=now;growth=error[0]=error[1]=0;samples=0;}}
        last=now;side=behind;direction=sign;previous[0]=distance[0];previous[1]=distance[1];
        return correct;
    }
};
}
