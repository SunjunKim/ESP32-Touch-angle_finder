#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "../src/pages/accel_rel.h"

static void gravity(float *v,unsigned normal,float polarity,float tilt,float heading=0){
  const float r=1/accel_rel::kDeg;
  v[normal]=polarity*std::cos(tilt*r);
  v[(normal+1)%3]=std::sin(tilt*r)*std::cos(heading*r);
  v[(normal+2)%3]=std::sin(tilt*r)*std::sin(heading*r);
}
static bool sample(accel_rel::ContactRel &f,const float *v,float dt=1){
  return f.update(v[0],v[1],v[2],dt);
}
int main(){
  unsigned count=0;
  for(unsigned axis=0;axis<3;axis++)for(float polarity:{-1.f,1.f})
    for(float heading:{0.f,30.f,90.f,147.f})
      for(int beta=-15;beta<=15;beta++)for(int angle=10;angle<=25;angle++)
        for(bool reverse:{false,true}){
          float a[3],b[3];gravity(a,axis,polarity,beta,heading);
          gravity(b,axis,polarity,beta+angle,heading);
          accel_rel::ContactRel f;
          assert(sample(f,reverse?b:a));assert(f.capture());
          assert(f.angle==0&&f.normal_axis==axis);
          // Jump directly across horizontal without sampling the crossing.
          assert(sample(f,reverse?a:b));
          assert(std::fabs(f.angle-angle)<.0001f);
          assert(f.normal_axis==axis);count++;
        }
  for(unsigned axis=0;axis<3;axis++){
    float v[3];gravity(v,axis,1,0);accel_rel::ContactRel f;
    assert(sample(f,v)&&f.capture());
    for(int heading=0;heading<360;heading++){
      gravity(v,axis,1,65,heading);assert(sample(f,v));
      assert(f.normal_axis==axis&&std::fabs(f.angle-65)<.0001f);
    }
    // Missing/stale readings do not silently change contact face or lose zero.
    assert(!f.update(NAN,0,0,.01f));assert(!f.capture());
    assert(f.has_reference&&f.normal_axis==axis);
    gravity(v,axis,1,20);assert(sample(f,v));assert(std::fabs(f.angle-20)<.0001f);
    // Re-zero is the ONLY operation that selects a new contact normal.
    gravity(v,(axis+1)%3,1,0);assert(sample(f,v)&&f.capture());
    assert(f.normal_axis==(axis+1)%3&&f.angle==0);
    f.clear_reference();assert(!f.has_reference&&!f.valid);
  }
  // Full 3D sample equivalence between flat and end/side contact faces. These
  // assert mounting invariance, NOT perfect accuracy for unknown relative yaw.
  unsigned mount_cases=0;
  for(float beta:{0.f,5.f,15.f})for(int tilt=0;tilt<=80;tilt+=5)
    for(int yaw=0;yaw<360;yaw+=3){
      float expected=0;
      for(unsigned axis=0;axis<3;axis++){
        accel_rel::ContactRel mounted;float r[3],g[3];
        gravity(r,axis,1,beta,30);gravity(g,axis,1,tilt,yaw);
        assert(sample(mounted,r)&&mounted.capture());assert(sample(mounted,g));
        if(axis==0)expected=mounted.angle;
        else assert(std::fabs(mounted.angle-expected)<.0001f);
        mount_cases++;
      }
    }
  accel_rel::ContactRel f;float v[3];gravity(v,0,1,-10);
  assert(sample(f,v)&&f.capture());gravity(v,0,1,10);
  for(int i=0;i<60;i++)assert(sample(f,v,.01f));
  assert(std::fabs(f.angle-20)<.4f);
  for(int i=0;i<1000;i++)assert(sample(f,v,.01f));
  assert(std::fabs(f.angle-20)<.0002f&&!f.moving);
  for(float &a:v)a*=.98f;
  assert(sample(f,v));assert(std::fabs(f.angle-20)<.0001f);
  assert(!f.update(0,0,0,.01f));
  std::printf("PASS %u bidirectional central poses + %u 3D mounting-equivalence samples; 3 normals / 6 faces; horizontal-reference yaw; axis lock; re-zero; invalid recovery; scalar response\n",count,mount_cases);
}
