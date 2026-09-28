#include <initializer_list>
#include "../main/manual_input.h"
constexpr int run() {
    const unsigned long long times[] = {0, 1000, 299000, 300000, 599999, 600000, 2000000};
    for (int direction=5; direction<=8; ++direction) {
        for (auto duration : times) {
            ManualPress p;
            auto start=p.begin(direction,1000);
            if(start.kind!=ManualPress::Hold || start.direction!=direction) return 1;
            if(p.update(1000+duration).kind!=ManualPress::None) return 2;
            auto stop=p.release(1000+duration,false);
            if(stop.kind!=ManualPress::Stop || stop.direction!=direction) return 3;
            if(p.release(1000+duration,false).kind!=ManualPress::None) return 4;
            p.begin(direction,1000);
            if(p.release(1001,true).kind!=ManualPress::Stop) return 5;
            p.begin(direction,1000);p.cancel();
            if(p.update(999999).kind!=ManualPress::None) return 6;
            if(p.release(999999,false).kind!=ManualPress::None) return 7;
        }
    }
    ManualJog j;j.begin();if(j.output(100000))return 10;
    if(!j.output(200000)||!j.output(699999))return 11;
    if(j.output(700000)||j.active())return 12;
    j.begin();j.output(600000);j.output(700000);j.cancel();if(j.output(700001))return 13;
    j.begin();j.output(800000);j.output(900000);if(j.output(1500000))return 14;
    j.begin();j.output(1600000);j.output(1700000);j.begin();if(j.output(1750000))return 15;
    return 0;
}

static_assert(run() == 0, "Manual input timing/cancellation regression");

static_assert(manualDriveAccelRaw(false, false) == 20, "Non-manual default unchanged");
static_assert(manualDriveAccelRaw(false, true) == 11, "All manual phases use normal 1.1s ramp");
static_assert(manualDriveAccelRaw(true, true) == 10, "Legacy tap remains distinct");

constexpr bool startupTest() {
    ManualStartup s;
    if(s.update(0)) return false;
    s.begin(); if(!s.update(1000) || !s.update(600999)) return false;
    if(s.update(601000) || s.update(900000)) return false;
    s.begin(); if(!s.update(1000000)) return false;
    s.cancel(); if(s.update(1000001)) return false;
    s.begin();s.cancel();if(s.update(1000002)) return false;
    return ManualStartup::command(250,true)==2500 &&
           ManualStartup::command(-250,true)==-2500 &&
           ManualStartup::command(250,false)==750 &&
           ManualStartup::command(-250,false)==-750 &&
           ManualStartup::command(2000,true)==2500 &&
           ManualStartup::command(-2000,false)==-2000 &&
           ManualStartup::command(0,true)==0;
}
static_assert(startupTest(), "Independent startup command, expiry and cancellation");

constexpr bool evidenceTest() {
    for (int dl : {-1,1}) for (int dr : {-1,1}) {
        ManualStartup s;s.begin();s.evidence.reset(0,0,0,dl,dr,true);s.update(1000);
        if(s.observe(101000,101000,50,dl*50,dr*50))return false;
        if(s.observe(201000,201000,35,dl*200,dr*200))return false;
        // Re-reading the same current frame must not confirm startup.
        if(s.observe(251000,201000,35,dl*200,dr*200))return false;
        if(s.observe(301000,301000,35,dl*200,dr*200))return false;
        if(!s.observe(401000,401000,35,dl*210,dr*210))return false;
        if(s.update(402000)||!s.estimated)return false;
    }
    for(int mode=0;mode<6;++mode) {
        ManualStartup s;s.begin();s.evidence.reset(0,mode==3?2700:0,0,1,1,mode!=4);s.update(1000);
        s.observe(101000,101000,50,0,0);
        for(unsigned long long t=201000;t<601000;t+=100000) {
            int a=mode==0?50:35;
            int l=mode==1?0:mode==3?2700:200;
            int r=mode==2?-200:200;
            if(s.observe(t,mode==5?101000:t,a,l,r))return false;
        }
        if(s.update(601000)||s.estimated)return false;
    }
    ManualStartup s;s.begin();s.evidence.reset(0,0,0,1,1,true);s.update(1000);
    s.observe(101000,101000,50,0,0);s.observe(201000,201000,35,200,200);
    s.cancel();if(s.observe(301000,301000,35,200,200))return false;
    return true;
}
static_assert(evidenceTest(), "Current drop AND two-wheel RPM, freshness, polarity, release");

// A startup-to-cruise handover with an unchanged target must not move
// the STM32 smoothstep phase backwards by changing its duration.
constexpr bool handoverContinuityTest() {
    const unsigned startup_duration = manualDriveAccelRaw(false, true) * 100;
    const unsigned cruise_duration = manualDriveAccelRaw(false, true) * 100;
    for (unsigned elapsed=100; elapsed<=600; elapsed+=100) {
        const float before = float(elapsed) / startup_duration;
        const float after = float(elapsed+10) / cruise_duration;
        if(after < before) return false;
    }
    return startup_duration == cruise_duration &&
           manualDriveAccelRaw(true,true)==10 &&
           manualDriveAccelRaw(false,false)==20;
}
static_assert(handoverContinuityTest(), "Manual handover cannot rewind ramp duration");
