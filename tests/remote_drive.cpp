#include "../main/remote_drive.h"
using namespace RemoteDrive;
static_assert(mix(1|4,1000).left == 500 && mix(1|4,1000).right == 1000);
static_assert(mix(1|8,1000).left == 1000 && mix(1|8,1000).right == 500);
static_assert(mix(2|4,1000).left == -1000 && mix(2|4,1000).right == -500);
static_assert(mix(2|8,1000).left == -500 && mix(2|8,1000).right == -1000);
static_assert(mix(4,1000).left == -1000 && mix(8,1000).right == -1000);
static_assert(!mix(3,1000).valid && !mix(12,1000).valid && !mix(0,1000).valid);
static_assert(mix(1,1000).left == mix(1,1000).right);
static_assert(assisted(120,120,200,750) == 450);
static_assert(assisted(-200,-200,-120,750) == -750);
static_assert(!rangeGrace(100000,0));
static_assert(rangeGrace(400000,100000));
static_assert(!rangeGrace(400001,100000));

static_assert(assisted(0,0,200,750) == 0);
static_assert(mix(1|4,1).left == 1);
static_assert(assisted(250,250,500,1000) == 500);
static_assert(mix(1|4,1000,100).left == 1000 && mix(1|4,1000,100).right == 1000);
static_assert(mix(1|4,1000,0).left == 0 && mix(1|4,1000,0).right == 1000);
static_assert(mix(1|4,1000,-100).left == -1000 && mix(1|4,1000,-100).right == 1000);
static_assert(mix(1|8,1000,-100).left == 1000 && mix(1|8,1000,-100).right == -1000);
static_assert(mix(2|4,1000,-100).left == -1000 && mix(2|4,1000,-100).right == 1000);
static_assert(mix(2|8,1000,-100).left == 1000 && mix(2|8,1000,-100).right == -1000);
