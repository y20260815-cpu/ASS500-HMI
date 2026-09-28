#include "../main/can/current_telemetry.h"
static_assert(CurrentTelemetry::encode(6.6f)==66);
static_assert(CurrentTelemetry::encode(25.6f)==256);
static_assert(CurrentTelemetry::encode(300.1f)==3001);
static_assert(CurrentTelemetry::encode(-1.0f)==0);
static_assert(CurrentTelemetry::encode(7000.0f)==65534);
constexpr bool test() {
    CurrentTelemetry c;
    uint8_t frame[8]={0x01,0x00,1,1,0,0,0,0};
    if(c.fresh(0))return false;
    c.receive(frame,8,0);
    if(c.value!=256 || !c.fresh(500000) || c.fresh(500001))return false;
    c.receive(frame,7,10);if(c.fresh(10))return false;
    frame[2]=2;c.receive(frame,8,20);if(c.fresh(20))return false;
    frame[2]=1;frame[3]=0;c.receive(frame,8,30);if(c.fresh(30))return false;
    frame[3]=1;frame[0]=frame[1]=255;c.receive(frame,8,40);if(c.fresh(40))return false;
    frame[0]=0;frame[1]=66;c.receive(frame,8,50);
    return c.fresh(50)&&c.value==66;
}
static_assert(test(), "current telemetry units, endian, validation and freshness");
