#include "../main/follow_steering.h"
using namespace FollowSteering;

static_assert(applyInnerPercent(400,1000,100).left == 400);
static_assert(applyInnerPercent(400,1000,50).left == 200);
static_assert(applyInnerPercent(400,1000,0).left == 0);
static_assert(applyInnerPercent(700,1000,-50).left == -150);
static_assert(applyInnerPercent(700,1000,-100).left == -300);
static_assert(applyInnerPercent(1000,700,-100).right == -300);
static_assert(applyInnerPercent(70,100,100).left == 70);
static_assert(applyInnerPercent(70,100,0).left == 0);
static_assert(applyInnerPercent(70,100,-100).left == -30);
static_assert(applyInnerPercent(1000,400,-100).left == 1000);
static_assert(applyInnerPercent(700,700,-100).left == 700);
