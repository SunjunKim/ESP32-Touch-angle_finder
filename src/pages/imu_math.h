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

// Fit an axis-aligned ellipsoid from six stationary 1g samples. No opposing
// direction assumption: in particular, the LCD-down support may be tilted.
// sum(a[k]*raw[k]^2 + c[k]*raw[k]) = 1, then complete the square.
static inline bool acc_bias_scale_from_6face(const float samples[6][3],
                                             float bias[3], float scale[3])
{
  double m[6][7] = {};
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < 3; ++k) {
      const double v = samples[i][k];
      if (!isfinite(v)) return false;
      m[i][k] = v * v;
      m[i][k + 3] = v;
    }
    m[i][6] = 1.0;
  }
  for (int col = 0; col < 6; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 6; ++row)
      if (fabs(m[row][col]) > fabs(m[pivot][col])) pivot = row;
    if (fabs(m[pivot][col]) < 1e-6) return false;
    for (int k = col; k < 7; ++k) {
      const double tmp = m[col][k];
      m[col][k] = m[pivot][k];
      m[pivot][k] = tmp;
    }
    const double divisor = m[col][col];
    for (int k = col; k < 7; ++k) m[col][k] /= divisor;
    for (int row = 0; row < 6; ++row) {
      if (row == col) continue;
      const double factor = m[row][col];
      for (int k = col; k < 7; ++k) m[row][k] -= factor * m[col][k];
    }
  }
  double radius2 = 1.0;
  float b[3], gain[3];
  for (int k = 0; k < 3; ++k) {
    const double a = m[k][6], c = m[k + 3][6];
    if (!isfinite(a) || !isfinite(c) || a <= 0.0) return false;
    b[k] = -c / (2.0 * a);
    radius2 += c * c / (4.0 * a);
  }
  for (int k = 0; k < 3; ++k) {
    gain[k] = sqrt(m[k][6] / radius2);
    if (!isfinite(b[k]) || fabsf(b[k]) > 0.5f ||
        !isfinite(gain[k]) || gain[k] <= 0.5f || gain[k] >= 2.0f) return false;
  }
  for (int k = 0; k < 3; ++k) { bias[k] = b[k]; scale[k] = gain[k]; }
  return true;
}

// LCD-up alone defines level; the opposing side pair defines in-plane heading.
// v_device = R * (scale * (raw - bias)). LCD-down never defines mount direction.
static inline bool acc_mount_matrix_from_6face(const float s[6][3],
                                               const float bias[3],
                                               const float scale[3],
                                               float R_out[9])
{
  float dz[3], dx[3];
  for (int k = 0; k < 3; k++) {
    dz[k] = scale[k] * (s[0][k] - bias[k]);
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
