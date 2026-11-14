#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "../src/pages/accel_rel.h"
#include "../src/pages/imu_math.h"
static void feed(accel_rel::ContactRel &f,float pitch,float roll,float dt=.008f) {
  const float p=pitch/accel_rel::kDeg,r=roll/accel_rel::kDeg;
  assert(f.update(std::sin(p),std::cos(p)*std::sin(r),std::cos(p)*std::cos(r),dt));
}
int main() {
  accel_rel::ContactRel f;
  for(int i=0;i<80;++i)feed(f,15,0);
  assert(f.capture());
  assert(std::fabs(f.pitch_reference-15)<.0001f && f.pitch_angle==0);
  for(float p:{-15.f,-.1f,0.f,.1f,15.f,25.f})
    for(float r:{-80.f,-20.f,0.f,20.f,80.f}) {
      feed(f,p,r,1);
      assert(std::fabs(f.pitch_angle-std::fabs(p-15))<.0001f);
      float expected;
      imu_math::accel_vector_to_tilt_pitch_roll(std::sin(p/accel_rel::kDeg),
        std::cos(p/accel_rel::kDeg)*std::sin(r/accel_rel::kDeg),
        std::cos(p/accel_rel::kDeg)*std::cos(r/accel_rel::kDeg),&expected,nullptr,nullptr);
      assert(std::fabs(f.pitch-expected)<.0001f);
    }
  // A cross-slope can drive 2D branch jumps while 1D stays near 15 degrees.
  feed(f,-.01f,1.1f,1);float a=f.angle,pa=f.pitch_angle;
  feed(f,.01f,1.1f,1);
  assert(std::fabs(f.angle-a)>2.f && std::fabs(f.pitch_angle-pa)<.021f);
  // Scalar response and recovery keep the common zero.
  for(int i=0;i<125;++i)feed(f,0,0);
  assert(std::fabs(f.pitch_angle-15)<.001f);
  assert(!f.update(NAN,0,1,.008f) && !f.pitch_moving && f.has_reference);
  feed(f,0,0);assert(std::fabs(f.pitch_angle-15)<.0001f);
  for(int i=0;i<80;++i)feed(f,-10,0);
  assert(f.capture());feed(f,10,0,1);
  assert(std::fabs(f.pitch_angle-20)<.0001f);
  f.clear_reference();assert(!f.has_reference && !f.valid && f.pitch_reference==0);
  puts("PASS 1D pitch definition / roll independence / opposite slopes / branch noise / response / recovery / re-zero / clear");
}
