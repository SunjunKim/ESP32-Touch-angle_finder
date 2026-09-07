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

// Active REL estimator. Legacy helpers above remain for historical comparisons.
// The contact normal is chosen ONLY at zero. A signed inclination branch is
// supplied by the reference's in-plane gravity direction, not by gyro history.
// This is exact for a horizontal reference and aligned single-axis tilts; it
// cannot recover the missing heading between arbitrary inclined planes.
class ContactRel {
 public:
  float angle=0,tilt=0,reference=0;
  bool valid=false,moving=false,has_reference=false;
  unsigned normal_axis=2;

  void clear_reference(){
    if(has_reference){valid=false;angle=reference=0;}
    has_reference=false;normal_axis=2;
  }

  bool capture(){
    if(!valid)return false;
    normal_axis=0;
    for(unsigned i=1;i<3;i++)
      if(std::fabs(sample_[i])>std::fabs(sample_[normal_axis]))normal_axis=i;
    for(unsigned i=0;i<3;i++)direction_[i]=i==normal_axis?0:sample_[i];
    reference=inclination_of(sample_);
    has_reference=true;
    tilt=reference;angle=0;
    return true;
  }

  bool update(float x,float y,float z,float dt){
    const float n=std::sqrt(x*x+y*y+z*z);
    if(!std::isfinite(n)||n<.1f){valid=false;moving=false;return false;}
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
    moving=std::fabs(n-1)>.05f||(valid&&std::fabs(next-angle)>.15f);
    if(!valid||!std::isfinite(dt)||dt<=0||dt>.5f)angle=next;
    else angle+=(next-angle)*(1-std::exp(-dt/.15f));
    valid=true;
    return true;
  }

 private:
  float sample_[3]={0,0,1},direction_[3]={0,0,0};
  float inclination_of(const float *v)const{
    const unsigned a=(normal_axis+1)%3,b=(normal_axis+2)%3;
    return std::atan2(std::hypot(v[a],v[b]),std::fabs(v[normal_axis]))*kDeg;
  }
};
} // namespace accel_rel
