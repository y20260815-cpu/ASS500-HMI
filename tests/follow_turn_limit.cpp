#include "../main/follow_turn_limit.h"
using namespace FollowTurnLimit;

static_assert(rpm(100)==2500);
static_assert(rpm(60)==1500);
static_assert(rpm(25)==625);
static_assert(rpm(0)==0);
static_assert(apply(2500,40)==1000);
static_assert(apply(-2500,40)==-1000);
static_assert(apply(500,40)==500);
static_assert(scale(1000,40)==400);
static_assert(scale(-1000,40)==-400);
static_assert(scale(1000,0)==0);
static_assert(scale(1000,100)==1000);
static_assert(pivot(-1,1)&&pivot(1,-1));
static_assert(!pivot(1,1)&&!pivot(0,1));
