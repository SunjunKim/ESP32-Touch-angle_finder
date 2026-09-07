#include <cassert>
#include <cmath>
#include <cstdio>
#include "../src/pages/accel_rel.h"
int main(){
  const float rad=1/accel_rel::kDeg;
  unsigned cases=0;
  for(int tilt=0;tilt<=60;tilt++)for(int yaw=0;yaw<360;yaw++){
    float x=sin(tilt*rad)*cos(yaw*rad),y=sin(tilt*rad)*sin(yaw*rad),z=-cos(tilt*rad);
    float actual=accel_rel::inclination(x,y,z);
    assert(fabs(actual-tilt)<.00002f);
    assert(accel_rel::relative(actual,float(tilt))<.00002f);
    assert(fabs(accel_rel::inclination(x*.98f,y*.98f,z*.98f)-tilt)<.00002f);
    cases++;
  }
  accel_rel::Filter f;
  for(int tilt=-60;tilt<=60;tilt++){
    f.reset();
    for(int i=0;i<1000;i++)assert(f.update(sin(tilt*rad),0,-cos(tilt*rad),.01f));
    assert(fabs(f.tilt-tilt)<.0001f);
  }
  assert(fabs(accel_rel::relative(accel_rel::signed_inclination(sin(15*rad),0,-cos(15*rad)),
                                accel_rel::signed_inclination(-sin(15*rad),0,-cos(15*rad)))-30)<.0001f);
  assert(accel_rel::relative(-10,-15)==5);
  // Sampling can jump across horizontal: sign comes from the current sample,
  // not observing an intermediate zero crossing.
  f.reset();f.update(-sin(15*rad),0,-cos(15*rad),.01f);float ref=f.tilt;
  for(int i=0;i<200;i++)f.update(sin(15*rad),0,-cos(15*rad),.01f);
  assert(fabs(accel_rel::relative(f.tilt,ref)-30)<.0001f);
  // Response within 0.4deg after 0.6sec for a 20deg step; no hold gate.
  f.reset();f.update(0,0,-1,.01f);
  for(int i=0;i<60;i++)f.update(sin(20*rad),0,-cos(20*rad),.01f);
  assert(fabs(f.tilt-20)<.4f);
  assert(!f.update(NAN,0,-1,.01f));
  assert(f.update(0,0,-1,.01f)&&f.tilt==0);
  // In-plane 180-degree rotation flips the X branch by design; not yaw invariant.
  assert(fabs(accel_rel::signed_inclination(-sin(20*rad),0,-cos(20*rad))+20)<.0001f);
  printf("PASS %u unsigned geometry cases + X sign, direct crossing, scale, response, invalid recovery\n",cases);
}
