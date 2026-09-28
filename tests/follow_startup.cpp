#include "../main/follow_startup.h"
constexpr bool followStartupTest() {
    FollowStartup f;
    if(f.update(0,0,0,false,0,0,0))return false;
    if(f.update(499999,0,0,false,0,0,0)||f.ready)return false;
    f.update(500000,0,0,false,0,0,0);
    if(!f.update(600000,-100,100,true,0,0,0))return false;
    if(f.assist.window!=1000000)return false;
    f.noteCommand(-1500,1500);
    if(!f.update(1200000,-200,200,true,0,0,0))return false;
    if(!f.update(1599999,-250,250,true,0,0,0))return false;
    if(f.update(1600000,-250,250,true,0,0,0))return false;
    if(f.update(1700000,-300,300,true,0,0,0))return false;
    // A brief zero does not rearm; a sustained stop does.
    f.update(1800000,0,0,true,0,0,0);
    if(f.update(1900000,-100,100,true,0,0,0))return false;
    f.update(2000000,0,0,true,0,0,0);f.update(2500000,0,0,true,0,0,0);
    if(!f.update(2600000,100,200,true,0,0,0))return false;
    if(f.assist.window!=600000)return false;
    if(f.update(2700000,100,-200,true,0,0,0))return false;
    if(f.update(2800000,100,200,true,0,0,0))return false;
    f.update(3000000,0,0,false,0,0,0);f.update(3500000,0,0,false,0,0,0);
    // Near-target/stale-CAN/obstacle exclusion consumes the start opportunity.
    if(f.update(3600000,100,200,false,0,0,0))return false;
    if(f.update(3700000,100,200,true,0,0,0))return false;
    return FollowStartup::command(100,100,200)==375 &&
           FollowStartup::command(200,100,200)==750 &&
           FollowStartup::command(-100,-100,100)==-2500 &&
           FollowStartup::command(0,0,100)==0 &&
           FollowStartup::command(2000,2000,1500)==2000;
}
static_assert(followStartupTest(), "Follow startup rearm, timeout, cancellation and curvature");
constexpr bool latePivotEvidenceTest() {
    FollowStartup f;
    f.update(0,0,0,false,0,0,0);f.update(500000,0,0,false,0,0,0);
    f.update(600000,-100,100,true,0,0,0);f.noteCommand(-1500,1500);
    if(f.assist.observe(1200000,1200000,100,-50,50))return false;
    if(f.assist.observe(1300000,1300000,70,-200,200))return false;
    if(!f.assist.observe(1400000,1400000,70,-220,220))return false;
    if(f.update(1500000,-100,100,true,70,-220,220))return false;
    // Changing target discards peak evidence from the old command.
    f.assist.evidence.peak=100;f.assist.evidence.hits=1;
    f.noteCommand(-1200,1500);
    return f.assist.evidence.peak==0 && f.assist.evidence.hits==0;
}
static_assert(latePivotEvidenceTest(), "Pivot can confirm after 600 ms without retry");

constexpr bool sustainedPivotTest() {
    if(followPivotCommand(1,45,500)!=2500)return false;
    if(followPivotCommand(-1,90,500)!=-2500)return false;
    if(followPivotCommand(1,45,1200)!=2500)return false;
    if(followPivotCommand(1,45,5000)!=2500)return false;
    if(followPivotCommand(1,10,500)!=0 || followPivotCommand(0,90,500)!=0)return false;
    int previous=0;
    for(int angle=10;angle<=20;++angle) {
        int v=followPivotCommand(1,(float)angle,500);
        if(v<previous || v>2500 || followPivotCommand(-1,(float)angle,500)!=-v)return false;
        previous=v;
    }
    return previous==2500;
}
static_assert(sustainedPivotTest(), "Sustained pivot drive and smooth alignment taper");

static_assert(followPivotCommand(1,45,2500)==2500 &&
              followPivotCommand(-1,45,2500)==-2500, "100 percent pivot range");
static_assert(FollowStartup::command(100,-100,100)==2500 &&
              FollowStartup::command(-100,-100,100)==-2500,
              "Both pivot directions reach full-scale startup command");

constexpr bool configurablePivotWindowTest() {
    FollowStartup f;
    f.update(0,0,0,false,0,0,0,300000);
    f.update(500000,0,0,false,0,0,0,300000);
    if(!f.update(600000,-100,100,true,0,0,0,300000)) return false;
    if(f.assist.window != 300000) return false;
    if(!f.update(899999,-100,100,true,0,0,0,300000)) return false;
    return !f.update(900000,-100,100,true,0,0,0,300000);
}
static_assert(configurablePivotWindowTest(), "Pivot startup window follows setting 10");
