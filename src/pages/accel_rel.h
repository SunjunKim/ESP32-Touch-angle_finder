#pragma once
#include <cmath>

// X-signed total inclination. Sensor X orientation must stay consistent.
// No gyro integration or horizon-crossing history is used.
namespace accel_rel {
constexpr float kDeg=57.29577951308232f;
inline float inclination(float x,float y,float z){
  return std::atan2(std::hypot(x,y),std::fabs(z))*kDeg;
}
inline float signed_inclination(float x,float y,float z){
  const float angle=inclination(x,y,z);
  return x>=0?angle:-angle;
}
inline float relative(float tilt,float reference){return std::fabs(tilt-reference);}
class Filter {
 public:
  float tilt=0;
  bool valid=false,moving=false;
  void reset(){valid=false;moving=false;tilt=0;}
  bool update(float x,float y,float z,float dt){
    const float n=std::sqrt(x*x+y*y+z*z);
    if(!std::isfinite(n)||n<.1f){valid=false;return false;}
    const float next=signed_inclination(x,y,z);
    moving=std::fabs(n-1)>.05f||(valid&&std::fabs(next-tilt)>.15f);
    if(!valid||!std::isfinite(dt)||dt<=0||dt>.5f)tilt=next;
    else tilt+=(next-tilt)*(1-std::exp(-dt/.15f));
    valid=true;return true;
  }
};

// Recent stationary window used only for zero capture. Time and sample count
// both matter: a single sample following a gap cannot establish a stable zero.
class ZeroWindow {
 public:
  void reset() { count_ = next_ = 0; }
  void update(float x, float y, float z, float dt) {
    const float n = std::sqrt(x*x + y*y + z*z);
    if (!std::isfinite(n) || std::fabs(n - 1.0f) > .05f ||
        !std::isfinite(dt) || dt <= 0 || dt > .05f) {
      reset();
      return;
    }
    const float v[3] = {x/n, y/n, z/n};
    // Limit the entire window's vector spread, including gradual motion.
    for (unsigned k = 0; k < 3; ++k) {
      float lo = v[k], hi = v[k];
      for (unsigned i = 0; i < count_; ++i) {
        lo = std::fmin(lo, samples_[i][k]);
        hi = std::fmax(hi, samples_[i][k]);
      }
      if (hi - lo > .008f) { reset(); break; }
    }
    for (unsigned k = 0; k < 3; ++k) samples_[next_][k] = v[k];
    intervals_[next_] = dt;
    next_ = (next_ + 1) % kCapacity;
    if (count_ < kCapacity) ++count_;
  }
  bool mean(float out[3]) const {
    if (count_ < 32) return false;
    const unsigned oldest = count_ == kCapacity ? next_ : 0;
    float span = 0;
    for (unsigned i = 0; i < count_; ++i)
      if (i != oldest) span += intervals_[i];
    if (span < .5f) return false;
    for (unsigned k = 0; k < 3; ++k) {
      out[k] = 0;
      for (unsigned i = 0; i < count_; ++i) out[k] += samples_[i][k];
      out[k] /= count_;
    }
    return true;
  }
 private:
  static constexpr unsigned kCapacity = 80; // ~0.64 sec at the production 125 Hz.
  float samples_[kCapacity][3] = {}, intervals_[kCapacity] = {};
  unsigned count_ = 0, next_ = 0;
};

// Active REL estimator. Legacy helpers above remain for historical comparisons.
// The contact normal is chosen ONLY at zero. A signed inclination branch is
// supplied by the reference's in-plane gravity direction, not by gyro history.
// This is exact for a horizontal reference and aligned single-axis tilts; it
// cannot recover the missing heading between arbitrary inclined planes.
class ContactRel {
 public:
  float angle=0,tilt=0,reference=0;
  // 1D uses the existing device-X pitch, independently of the 2D contact normal.
  float pitch=0,pitch_angle=0,pitch_reference=0;
  bool pitch_moving=false;
  bool valid=false,moving=false,has_reference=false;
  unsigned normal_axis=2;

  void clear_reference(){
    if(has_reference){valid=false;angle=reference=0;pitch_angle=pitch_reference=0;zero_.reset();}
    has_reference=false;normal_axis=2;
  }

  bool capture(float *captured = nullptr){
    float average[3];
    if(!valid || !zero_.mean(average))return false;
    normal_axis=0;
    for(unsigned i=1;i<3;i++)
      if(std::fabs(average[i])>std::fabs(average[normal_axis]))normal_axis=i;
    for(unsigned i=0;i<3;i++)direction_[i]=i==normal_axis?0:average[i];
    reference=inclination_of(average);
    pitch_reference=pitch_of(average);
    has_reference=true;
    if (captured) for (unsigned i = 0; i < 3; ++i) captured[i] = average[i];
    tilt=reference;angle=0;
    pitch=pitch_reference;pitch_angle=0;
    return true;
  }

  bool update(float x,float y,float z,float dt){
    if (!valid) zero_.reset();
    zero_.update(x, y, z, dt);
    const float n=std::sqrt(x*x+y*y+z*z);
    if(!std::isfinite(n)||n<.1f){valid=false;moving=pitch_moving=false;return false;}
    sample_[0]=x/n;sample_[1]=y/n;sample_[2]=z/n;
    tilt=inclination_of(sample_);
    float next=tilt;
    if(has_reference){
      float alignment=0;
      for(unsigned i=0;i<3;i++)alignment+=sample_[i]*direction_[i];
      // At an exactly horizontal zero reference==0, so branch is irrelevant.
      const float signed_tilt=alignment<0?-tilt:tilt;
      next=std::fabs(signed_tilt-reference);
      // Unoriented contact planes have acute relative angles, in [0,90].
      if(next>90)next=180-next;
    }
    pitch=pitch_of(sample_);
    float next_pitch=has_reference?std::fabs(pitch-pitch_reference):std::fabs(pitch);
    pitch_moving=std::fabs(n-1)>.05f||(valid&&std::fabs(next_pitch-pitch_angle)>.15f);
    if(!valid||!std::isfinite(dt)||dt<=0||dt>.5f)pitch_angle=next_pitch;
    else pitch_angle+=(next_pitch-pitch_angle)*(1-std::exp(-dt/.15f));
    moving=std::fabs(n-1)>.05f||(valid&&std::fabs(next-angle)>.15f);
    if(!valid||!std::isfinite(dt)||dt<=0||dt>.5f)angle=next;
    else angle+=(next-angle)*(1-std::exp(-dt/.15f));
    valid=true;
    return true;
  }

 private:
  ZeroWindow zero_;
  float sample_[3]={0,0,1},direction_[3]={0,0,0};
  static float pitch_of(const float *v){
    return std::atan2(v[0],std::hypot(v[1],v[2]))*kDeg;
  }
  float inclination_of(const float *v)const{
    const unsigned a=(normal_axis+1)%3,b=(normal_axis+2)%3;
    return std::atan2(std::hypot(v[a],v[b]),std::fabs(v[normal_axis]))*kDeg;
  }
};
} // namespace accel_rel
