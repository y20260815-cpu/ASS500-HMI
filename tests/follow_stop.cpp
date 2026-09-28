#include "../main/follow_stop.h"
using namespace FollowStop;
constexpr bool checks(){
 Alignment a;
 if(a.update(100,20,30)!=Action::Drive)return false;
 if(a.update(60,20,30)!=Action::Drive)return false;
 if(a.update(19,20,-6)!=Action::Align)return false;
 if(a.update(19,20,3)!=Action::Stop)return false;
 if(a.update(21,20,4)!=Action::Stop)return false;
 if(a.update(19,20,9)!=Action::Align)return false;
 return a.update(100,20,0)==Action::Drive;
}
static_assert(checks());
constexpr bool uncertaintyBand(){
 Alignment a;
 if(a.update(19,20,11,10,12)!=Action::Stop)return false;
 if(a.update(35,20,-17,10,12)!=Action::Stop)return false;
 if(a.update(35,20,-19,10,12)!=Action::Align)return false;
 return true;
}
static_assert(uncertaintyBand());
static_assert(rpm(6,500)>0&&rpm(-6,500)<0&&rpm(90,500)==1800);
