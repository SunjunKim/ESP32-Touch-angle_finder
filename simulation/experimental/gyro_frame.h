#pragma once
#include "../../src/pages/imu_math.h"

namespace imu_math {
// Historical gyro experiment. Angular velocity is an axial vector: account for
// reflections without applying accelerometer gain magnitudes to the gyro.
static inline void rotate_gyro_to_device(const float r[9],const float scale[3],float &gx,float &gy,float &gz)
{
  const float det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  const float sx=copysignf(1,scale[0]),sy=copysignf(1,scale[1]),sz=copysignf(1,scale[2]);
  const float parity=copysignf(1,det*sx*sy*sz);
  float x,y,z;
  mat3_vec3_rowmajor(r,gx*sx,gy*sy,gz*sz,&x,&y,&z);
  gx=parity*x;gy=parity*y;gz=parity*z;
}
} // namespace imu_math
