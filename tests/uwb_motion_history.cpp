#include "../main/uwb_motion_history.h"
using namespace UwbMotion;
constexpr bool crossing() {
    History h;
    h.select(100,1); h.select(80,100001); h.select(60,200001);
    h.select(40,300001); h.select(20,400001); h.select(0,500001);
    if (h.select(20,600001) >= 0 || h.select(40,700001) >= 0) return false;
    // A gap clears velocity, but does not invent a front/back change.
    return h.select(60,2000001) < 0 && h.velocity == 0;
}
constexpr bool stationary() {
    History h;
    for(int i=0;i<20;++i) if(h.select(3 + i%2,1+i*100000LL)<0) return false;
    return true;
}
static_assert(crossing());
static_assert(stationary());
constexpr bool manual_flip() {
    History h; h.select(80,1); h.select(60,100001); h.flip();
    return h.behind_line && h.normal < 0 && h.velocity > 0;
}
static_assert(manual_flip());
constexpr bool no_chatter_near_line() {
    History h;
    h.select(100,1); h.select(20,100001); h.select(0,200001);
    h.select(20,300001); // one physical crossing
    const bool selected=h.behind_line;
    for(int i=0;i<20;++i) h.select(3+(i%5),400001+i*100000LL);
    return h.behind_line==selected && !h.crossing_armed;
}
static_assert(no_chatter_near_line());
