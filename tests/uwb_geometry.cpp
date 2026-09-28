#include "../main/uwb_geometry.h"
#include <limits>
using namespace UwbGeometry;
constexpr bool near(float a,float b,float e=.001f){return a-b<e&&b-a<e;}
constexpr auto corrected=reconcile(3,103,kBaselineCm,3);
static_assert(corrected.valid&&corrected.corrected);
static_assert(near(corrected.right-corrected.left,kBaselineCm));
constexpr auto unchanged=reconcile(200,250,kBaselineCm,3);
static_assert(unchanged.valid&&!unchanged.corrected);
static_assert(!reconcile(0,100,kBaselineCm,3).valid);
static_assert(!reconcile(100,100,kBaselineCm,2).valid);
static_assert(!reconcile(std::numeric_limits<float>::quiet_NaN(),100,kBaselineCm,3).valid);
static_assert(kLateralGapCm==80&&kLongitudinalGapCm==0&&kAnchorForwardCm==0&&kBaselineCm==80);
int main(){
  // Horizontal anchors have one lateral coordinate and front/rear mirror points.
  const float f=300,r=-120,w=80;
  const float a0=std::hypot(f-kAnchorForwardCm,r+w/2);
  const float a1=std::hypot(f-kAnchorForwardCm,r-w/2);
  const auto p=reconcile(a0,a1,w,3);
  if(!p.valid)return 1;
  const float along=(p.left*p.left+w*w-p.right*p.right)/(2*w);
  const float normal=std::sqrt(std::max(0.0f,p.left*p.left-along*along));
  const float lateral=-w/2+along;
  if(!near(lateral,r,.02f)||!near(kAnchorForwardCm+normal,f,.02f))return 2;
  if(!near(kAnchorForwardCm-normal,2*kAnchorForwardCm-f,.02f))return 3;
  if(!ambiguityZone(0,400,10,3)||ambiguityZone(31,0,10,3))return 4;
  return 0;
}
