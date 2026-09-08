#include "../src/pages/imu_math.h"
#include <cassert>
#include <cstdio>
#include <limits>

int main()
{
  unsigned cases = 0;
  for (int tilt = 0; tilt <= 55; tilt += 5) {
    for (int heading = 0; heading < 360; heading += 30) {
      const float t = tilt / 57.2957795f, h = heading / 57.2957795f;
      float poses[6][3] = {{0,0,-1},{sinf(t)*cosf(h),sinf(t)*sinf(h),cosf(t)},
                           {-1,0,0},{1,0,0},{0,1,0},{0,-1,0}};
      const float bias[3] = {.025f,-.018f,.04f}, scale[3] = {1.04f,.97f,1.02f};
      float samples[6][3];
      // Tilt the sensor mount about Y by 8 degrees as well.
      const float c = cosf(.14f), sn = sinf(.14f);
      for (int i = 0; i < 6; ++i) {
        const float rect[3] = {c*poses[i][0]+sn*poses[i][2], poses[i][1],
                               -sn*poses[i][0]+c*poses[i][2]};
        for (int k = 0; k < 3; ++k) samples[i][k] = rect[k]/scale[k]+bias[k];
      }
      float b[3], g[3], r[9];
      assert(imu_math::acc_bias_scale_from_6face(samples,b,g));
      assert(imu_math::acc_mount_matrix_from_6face(samples,b,g,r));
      for (int k = 0; k < 3; ++k) {
        assert(fabsf(b[k]-bias[k]) < 1e-5f);
        assert(fabsf(g[k]-scale[k]) < 1e-5f);
      }
      for (int i = 0; i < 6; ++i) {
        float v[3];
        imu_math::mat3_vec3_rowmajor(r,g[0]*(samples[i][0]-b[0]),
          g[1]*(samples[i][1]-b[1]),g[2]*(samples[i][2]-b[2]),&v[0],&v[1],&v[2]);
        for (int k = 0; k < 3; ++k) assert(fabsf(v[k]-poses[i][k]) < 1e-5f);
        assert(fabsf(imu_math::safe_norm3(v[0],v[1],v[2])-1) < 1e-5f);
      }
      ++cases;
    }
  }
  float bad[6][3] = {}, b[3] = {7,7,7}, g[3] = {8,8,8};
  assert(!imu_math::acc_bias_scale_from_6face(bad,b,g));
  bad[0][0] = std::numeric_limits<float>::quiet_NaN();
  assert(!imu_math::acc_bias_scale_from_6face(bad,b,g));
  assert(b[0] == 7 && g[0] == 8);
  std::printf("PASS %u tilted LCD-down calibration cases; mount, bias, gain, 1g, invalid input\n",cases);
}
