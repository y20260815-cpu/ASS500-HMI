#include "../main/can/drive_timing.h"
static_assert(DriveTiming::accelIndex(false)==6 && DriveTiming::decelIndex(false)==7,
              "Manual menu 15/16");
static_assert(DriveTiming::accelIndex(true)==5 && DriveTiming::decelIndex(true)==4,
              "Follow menu 06/05");
static_assert(DriveTiming::encode(0.5f)==5 && DriveTiming::encode(5.0f)==50 &&
              DriveTiming::encode(1.2f)==12, "Menu timing encoding");
static_assert(DriveTiming::valid(5,0x85) && DriveTiming::valid(50,0xb2), "Tagged timing");
static_assert(!DriveTiming::valid(10,3) && !DriveTiming::valid(4,0x85) &&
              !DriveTiming::valid(5,0x80) && !DriveTiming::valid(5,0xff), "Legacy and invalid frames");
constexpr bool decelerationTest() {
    for(unsigned raw=5;raw<=50;++raw) {
        float total=0;
        for(unsigned n=0;n<raw*10;++n)total+=DriveTiming::decelStep(raw,10);
        if(total<0.9999f||total>1.0001f)return false;
        if(DriveTiming::decelStep(raw,20)!=2*DriveTiming::decelStep(raw,10))return false;
    }
    return true;
}
static_assert(decelerationTest(), "Full-scale ramp duration and elapsed-time scaling");

static_assert(DriveTiming::pivotFeedbackBypass(true,true,1,-1,0,0), "Right pivot");
static_assert(DriveTiming::pivotFeedbackBypass(true,true,-1,1,0,0), "Left pivot");
static_assert(DriveTiming::pivotFeedbackBypass(true,true,0,0,-0.3f,0.3f), "Pivot deceleration");
static_assert(!DriveTiming::pivotFeedbackBypass(true,true,1,1,0.3f,0.3f), "Straight PID unchanged");
static_assert(!DriveTiming::pivotFeedbackBypass(true,true,-1,-1,-0.3f,-0.3f), "Reverse PID unchanged");
static_assert(!DriveTiming::pivotFeedbackBypass(false,true,-1,1,0,0), "Local profile unchanged");
static_assert(!DriveTiming::pivotFeedbackBypass(true,false,-1,1,0,0), "Legacy profile unchanged");

static_assert(DriveTiming::innerZero(true,true,0,1000));
static_assert(DriveTiming::innerZero(true,true,0,-1000));
static_assert(!DriveTiming::innerZero(true,true,0,0));
static_assert(!DriveTiming::innerZero(true,true,1000,0));
static_assert(!DriveTiming::innerZero(false,true,0,1000));
static_assert(!DriveTiming::innerZero(true,false,0,1000));
