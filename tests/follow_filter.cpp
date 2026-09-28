#include "../main/follow_filter.h"
using namespace FollowFilter;
static_assert(steeringEntry(100) == 2 && steeringEntry(400) == 5);
static_assert(steeringExit(100) == 1 && steeringExit(400) == 2.5f);
static_assert(alignmentExit(100) == 10 && alignmentExit(400) == 20);
static_assert(steeringEntry(300) > steeringEntry(200));

static_assert(steeringEntry(0) == 2 && steeringEntry(300) == 4);
static_assert(steeringEntry(500) == 6 && steeringEntry(1000) == 8);
static_assert(steeringEntry(2000) == 10 && steeringEntry(3000) == 10);
static_assert(distanceRatios(300).filter_percent == 80);
static_assert(distanceRatios(500).filter_percent == 60);
static_assert(distanceRatios(1000).filter_percent == 50);
static_assert(distanceRatios(2000).filter_percent == 40);
constexpr bool continuousRatios() {
    for (int cm=1; cm<=2100; ++cm) {
        const auto a=distanceRatios(cm-1), b=distanceRatios(cm);
        if(b.steering_percent < a.steering_percent || b.steering_percent-a.steering_percent > .51f) return false;
        if(b.filter_percent > a.filter_percent || a.filter_percent-b.filter_percent > .11f) return false;
    }
    return true;
}
static_assert(continuousRatios());
