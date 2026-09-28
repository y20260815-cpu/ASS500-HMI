#include "tap_current_start.h"
constexpr bool check(bool turning) {
 TapCurrentStart s;
 const float maximum=turning?.4f:.3f, soft=turning?10.f:6.f, hard=turning?12.f:8.f;
 if(s.step(maximum,0,0,turning)!=0)return false;
 float previous=0;
 for(unsigned t=10;t<=300;t+=10){float p=s.step(maximum,0,t,turning);if(p<previous||p>maximum)return false;previous=p;}
 if(previous<maximum-.001f)return false;
 if(s.step(maximum,soft,310,turning)>=previous)return false;
 if(s.step(maximum,hard,320,turning)!=0||!s.blocked)return false;
 if(s.step(maximum,0,330,turning)!=0)return false;
 s.step(0,0,340,turning);if(s.blocked||s.started)return false;
 s.step(.1f,0,350,turning);if(s.step(.1f,0,450,turning)>.031f)return false;
 if(s.step(.1f,0,850,turning)!=0||!s.blocked)return false;
 s.reset();if(s.step(maximum,-1,0,turning)!=0||!s.blocked)return false;
 s.reset();s.step(maximum,0,0xfffffff0u,turning);if(s.step(maximum,0,4u,turning)>.031f)return false;
 return true;
}
static_assert(check(false) && check(true), "straight and pivot: limits, timeout, reset, wrap");
