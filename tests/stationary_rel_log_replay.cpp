#include <algorithm>
#include <cassert>
#include <cstdio>
#include "../simulation/experimental/stationary_rel_tracker.h"
using namespace stationary_rel;
// stdin: device seconds, calibrated Axyz and Wxyz. Sparse diagnostic records
// are linearly resampled; this does not reproduce unrecorded high-rate noise.
int main(){
  static Tracker tracker;float t,first=0,last=0;Vec3 a,w,pa,pw;
  bool begun=false,center_requested=false;int valid=0;float max_angle=0,min_angle=90;
  while(std::scanf("%f %f %f %f %f %f %f",&t,&a.x,&a.y,&a.z,&w.x,&w.y,&w.z)==7){
    if(!begun){begun=true;first=last=t;pa=a;pw=w;tracker.request(Command::Blade);continue;}
    const float gap=t-last;const int steps=std::max(1,(int)std::ceil(gap/.012f));
    for(int i=1;i<=steps;i++){
      const float f=(float)i/steps,sec=last-first+gap*f;
      if(sec>=2&&!center_requested){tracker.request(Command::Center);center_requested=true;}
      tracker.update(pa*(1-f)+a*f,pw*(1-f)+w*f,gap/steps);
      if(sec>5&&tracker.value_valid){++valid;max_angle=std::max(max_angle,tracker.angle_deg);min_angle=std::min(min_angle,tracker.angle_deg);}
    }
    last=t;pa=a;pw=w;
  }
  assert(valid>1000&&tracker.value_valid);
  // The recorded pose was not a central alignment fixture: assess drift/range,
  // not absolute accuracy or whether its apparent reference reading is zero.
  std::printf("Static replay: valid=%d range %.4f..%.4f deg beta=%.4f\n",valid,min_angle,max_angle,tracker.beta_deg);
  assert(max_angle-min_angle<.15f);
  std::puts("PASS static drift replay (not absolute angle accuracy)");
}
