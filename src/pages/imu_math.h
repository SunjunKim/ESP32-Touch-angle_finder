#pragma once

#include <math.h>

namespace imu_math {

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float rad2deg(float r) { return r * 57.29577951308232f; }

static inline float safe_norm3(float x, float y, float z)
{
  const float n2 = x * x + y * y + z * z;
  if (n2 <= 1e-12f) return 0.0f;
  return sqrtf(n2);
}

// Pitch/roll match abs_accel_to_pitch_roll; tilt is angle (deg) from nominal "flat"
// reading (0,0,-1) g in device frame — 0° when level, 90° when on edge.
static inline void accel_vector_to_tilt_pitch_roll(float ax,
                                                   float ay,
                                                   float az,
                                                   float *pitch_deg,
                                                   float *roll_deg,
                                                   float *tilt_deg)
{
  if (pitch_deg != nullptr) {
    *pitch_deg = rad2deg(atan2f(ax, sqrtf(ay * ay + az * az)));
  }
  if (roll_deg != nullptr) {
    *roll_deg = rad2deg(atan2f(ay, sqrtf(ax * ax + az * az)));
  }
  if (tilt_deg != nullptr) {
    const float n = safe_norm3(ax, ay, az);
    if (n > 1e-6f) {
      const float inv = 1.0f / n;
      const float nz = az * inv;
      const float c = clampf(-nz, -1.0f, 1.0f);
      *tilt_deg = rad2deg(acosf(c));
    } else {
      *tilt_deg = 0.0f;
    }
  }
}

// 6-face accel samples s[0..5] (raw), same order as calibration wizard; bias + diagonal
// scale from the usual opposing-pair fit. Writes row-major R with v_device = R * v_rect
// where v_rect = (sx*(ax-bx), sy*(ay-by), sz*(az-bz)).
static inline bool acc_mount_matrix_from_6face(const float s[6][3],
                                               const float bias[3],
                                               const float scale[3],
                                               float R_out[9])
{
  (void)bias;
  float dz[3], dx[3];
  for (int k = 0; k < 3; k++) {
    dz[k] = scale[k] * (s[0][k] - s[1][k]);
    dx[k] = scale[k] * (s[3][k] - s[2][k]);
  }

  const float nz = safe_norm3(dz[0], dz[1], dz[2]);
  if (nz < 0.25f) {
    return false;
  }
  float zx = dz[0] / nz;
  float zy = dz[1] / nz;
  float zz = dz[2] / nz;

  const float dpx = dx[0] - zx * (dx[0] * zx + dx[1] * zy + dx[2] * zz);
  const float dpy = dx[1] - zy * (dx[0] * zx + dx[1] * zy + dx[2] * zz);
  const float dpz = dx[2] - zz * (dx[0] * zx + dx[1] * zy + dx[2] * zz);
  const float nx = safe_norm3(dpx, dpy, dpz);
  if (nx < 0.15f) {
    return false;
  }
  float xx = dpx / nx;
  float xy = dpy / nx;
  float xz = dpz / nx;

  // y = x × z (right-handed, matches +Y reading for identity mount)
  float yx = xy * zz - xz * zy;
  float yy = xz * zx - xx * zz;
  float yz = xx * zy - xy * zx;

  // Columns: +X, +Y, +Z device axes in rectified-sensor coords (see kCalExpected).
  const float M[9] = {xx, yx, zx, xy, yy, zy, xz, yz, zz};

  // E matches identity mount: diag(1,1,-1) as 3×3 with columns (1,0,0),(0,1,0),(0,0,-1).
  const float E[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 3; k++) {
        sum += E[i * 3 + k] * M[j * 3 + k];
      }
      R_out[i * 3 + j] = sum;
    }
  }

  return true;
}

static inline void mat3_vec3_rowmajor(const float R[9], float x, float y, float z, float *ox, float *oy, float *oz)
{
  *ox = R[0] * x + R[1] * y + R[2] * z;
  *oy = R[3] * x + R[4] * y + R[5] * z;
  *oz = R[6] * x + R[7] * y + R[8] * z;
}

} // namespace imu_math
