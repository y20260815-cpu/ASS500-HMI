#include "../main/follow_direction.h"
using namespace FollowDirection;
constexpr bool forwardOnly() {
    Selector front, rear;
    if(front.update(0,500,false,0,0)!=1 || front.recovery) return false;
    if(rear.update(180,500,false,0,0)!=1 || !rear.recovery) return false;
    return rear.steeringBearing(180)==180;
}
constexpr bool shadowProbe() {
    Selector closer;
    closer.update(0,500,true,0,0);
    if(!closer.probing || closer.update(0,489,true,.2f,100000)!=1) return false;
    if(closer.probing || closer.changed) return false;
    Selector farther;
    farther.update(0,500,true,0,0);
    if(farther.update(0,521,true,.2f,600000)!=1) return false;
    if(farther.probing || !farther.changed) return false;
    Selector limited;
    limited.update(0,500,true,0,0);
    for(int i=1;i<=25;i++) limited.update(0,500,true,.2f,i*100000LL);
    return !limited.probing && !limited.changed && limited.failed;
}
static_assert(forwardOnly());
static_assert(shadowProbe());
constexpr bool forwardOnlyRearReentry() {
    Selector s;
    if (s.update(0,300,false,0,1) != 1 || s.recovery) return false;
    if (s.update(100,300,false,0,100001) != 1 || !s.recovery) return false;
    return s.update(40,300,false,0,200001) == 1 && !s.recovery;
}
static_assert(forwardOnlyRearReentry());
