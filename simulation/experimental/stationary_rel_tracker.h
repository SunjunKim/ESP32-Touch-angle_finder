#pragma once
#include <cmath>
#include <cstddef>

// Historical gyro experiment; not used by production firmware.
// No Arduino dependencies. Input: calibrated device-frame acceleration (flat -Z),
// angular velocity in the SAME right-handed device frame, deg/s. Plane normal +Z.
namespace stationary_rel {
constexpr float kRad=0.0174532925199433f;
struct Vec3 {
  float x=0,y=0,z=0;
  Vec3 operator+(Vec3 b)const{return {x+b.x,y+b.y,z+b.z};}
  Vec3 operator-(Vec3 b)const{return {x-b.x,y-b.y,z-b.z};}
  Vec3 operator*(float s)const{return {x*s,y*s,z*s};}
  float dot(Vec3 b)const{return x*b.x+y*b.y+z*b.z;}
  Vec3 cross(Vec3 b)const{return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
  float norm()const{return std::sqrt(dot(*this));}
  Vec3 unit()const{float n=norm();return n>1e-9f?*this*(1/n):Vec3{};}
  bool finite()const{return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z);}
};
inline Vec3 transport(Vec3 v,Vec3 gyro,float dt){
  Vec3 w=gyro*(-kRad);float speed=w.norm(),a=speed*dt;
  if(speed<1e-8f)return v;
  Vec3 k=w*(1/speed);
  return (v*std::cos(a)+k.cross(v)*std::sin(a)+k*(k.dot(v)*(1-std::cos(a)))).unit();
}
enum class Status : unsigned char {NeedBlade,NeedCenter,Moving,Settling,Ready,Recenter,BadPose,Timeout,NeedCalibration};
enum class Command : unsigned char {None,Blade,Center,Zero};

class Tracker {
 public:
  static constexpr unsigned kCapacity=2048;
  static constexpr float kWindow=.8f,kMaxSegment=8.0f,kMaxBudget=.30f;
  // Entry is based on window variation, not a stale DC gyro offset.
  // Gravity-axis rotation remains weakly observable: keep a tighter yaw gate.
  static constexpr float kAccelRms=.003f,kGyroRms=.20f;
  Status status=Status::NeedBlade;
  bool blade_valid=false,anchored=false,value_valid=false,stable=false;
  float angle_deg=0,beta_deg=0,drift_budget_deg=0;
  unsigned sequence=0;
  Vec3 bias()const{return bias_;}
  Vec3 normal()const{return normal_;}
  float gravity_sd()const{return accel_sd_;}
  float gyro_sd()const{return gyro_sd_;}
  float bias_residual()const{return bias_residual_;}
  bool capturing()const{return pending_!=Command::None;}
  float display_angle()const{
    return value_valid?angle_deg:std::atan2(std::hypot(live_normal_.x,live_normal_.y),std::fabs(live_normal_.z))/kRad;
  }
  void reset(){
    // Do not materialize this 32 KB object on the small IMU task stack.
    blade_valid=anchored=value_valid=stable=bias_known_=false;
    angle_deg=beta_deg=drift_budget_deg=0;status=Status::NeedBlade;
    pending_=Command::None;pending_time_=0;clear_segment();clear_window();
    normal_=live_normal_={0,0,1};bias_={};rest_up_={};
  }
  void request(Command c){
    if(c==Command::None)return;
    pending_=c;pending_time_=0;value_valid=false;stable=false;
    if(c==Command::Blade||c==Command::Zero){blade_valid=false;anchored=false;status=Status::Settling;}
    else {anchored=false;status=blade_valid?Status::NeedCenter:Status::NeedBlade;}
    clear_segment();clear_window();
  }
  void invalidate(){
    anchored=value_valid=stable=false;pending_=Command::None;
    status=blade_valid?Status::Recenter:Status::NeedBlade;
    clear_segment();clear_window();
  }
  void update(Vec3 accel,Vec3 gyro,float dt){
    if(!accel.finite()||!gyro.finite()||!std::isfinite(dt)||dt<=0||dt>.05f||
       accel.norm()<.1f||std::fabs(gyro.x)>950||std::fabs(gyro.y)>950||std::fabs(gyro.z)>950){invalidate();return;}
    if(pending_!=Command::None){
      pending_time_+=dt;
      if(pending_time_>8){pending_=Command::None;status=Status::Timeout;}
    }
    Vec3 up=accel.unit()*-1;
    bool now_quiet=std::fabs(accel.norm()-1)<.08f;
    if(bias_known_&&pending_==Command::None){
      Vec3 residual=gyro-bias_;
      const float yaw=std::fabs(residual.dot(up));
      const float transverse=(residual-up*residual.dot(up)).norm();
      // Hysteresis: tolerate static bias creep and small contact vibration.
      now_quiet=now_quiet&&transverse<1.5f&&yaw<1.2f;
      if(stable&&(up-rest_up_).norm()>.008f)now_quiet=false;
    }
    if(!now_quiet){stable=value_valid=false;if(anchored)status=Status::Moving;}
    if(anchored){
      if(count_>=kCapacity||segment_time_+dt>kMaxSegment){invalidate();}
      else {
        samples_[count_++]={gyro,dt};segment_time_+=dt;
        // Preview only. Rest-time replay still starts from normal_, using
        // endpoint bias estimates; never integrate the preview twice.
        if(!stable)live_normal_=transport(live_normal_,gyro-bias_,dt);
      }
    }
    if(window_count_==0){first_a_=accel;first_w_=gyro;window_bad_=false;}
    Vec3 da=accel-first_a_,dw=gyro-first_w_;
    if(da.norm()>.012f||dw.norm()>1.2f||std::fabs(accel.norm()-1)>.08f)window_bad_=true;
    // Center sums near zero: E[x^2]-E[x]^2 at 1g / 5dps loses precision.
    sum_a_=sum_a_+da*dt;sum_w_=sum_w_+dw*dt;
    sum_a2_+=da.dot(da)*dt;sum_w2_+=dw.dot(dw)*dt;
    window_time_+=dt;++window_count_;
    if(window_time_<kWindow)return;
    Vec3 ma=sum_a_*(1/window_time_),mw=sum_w_*(1/window_time_);
    Vec3 a=first_a_+ma,w=first_w_+mw;
    accel_sd_=std::sqrt(fmaxf(0,sum_a2_/window_time_-ma.dot(ma)));
    gyro_sd_=std::sqrt(fmaxf(0,sum_w2_/window_time_-mw.dot(mw)));
    bool quiet=!window_bad_&&accel_sd_<kAccelRms&&gyro_sd_<kGyroRms&&da.norm()<.004f;
    bias_residual_=(w-bias_).norm();
    if(bias_known_&&pending_==Command::None){
      Vec3 mean_up=a.unit()*-1,residual=w-bias_;
      const float yaw=std::fabs(residual.dot(mean_up));
      const float transverse=(residual-mean_up*residual.dot(mean_up)).norm();
      quiet=quiet&&transverse<1.0f&&yaw<.8f;
    }
    if(quiet)finish_rest(a.unit()*-1,w);
    else {stable=value_valid=false;if(anchored)status=Status::Settling;}
    clear_window();
  }
 private:
  struct Sample {Vec3 gyro;float dt;};
  Sample samples_[kCapacity];
  unsigned count_=0,window_count_=0;
  float segment_time_=0,window_time_=0,sum_a2_=0,sum_w2_=0,accel_sd_=0,gyro_sd_=0,bias_residual_=0;
  Vec3 normal_{0,0,1},live_normal_{0,0,1},bias_{},rest_up_{},sum_a_{},sum_w_{},first_a_{},first_w_{};
  bool bias_known_=false,window_bad_=false;
  Command pending_=Command::None;
  float pending_time_=0;
  void clear_window(){window_count_=0;window_time_=sum_a2_=sum_w2_=0;sum_a_={};sum_w_={};window_bad_=false;}
  void clear_segment(){count_=0;segment_time_=0;}
  bool pose_ok(Vec3 up)const{return up.z>.5f&&std::fabs(up.y)<std::sin(.5f*kRad);}
  void apply_gravity_constraint(Vec3 up){
    float b=beta_deg*kRad;
    if(std::fabs(b)<1e-7f){normal_=up;return;}
    Vec3 h=normal_-up*normal_.dot(up);
    if(h.norm()<1e-6f){invalidate();return;}
    normal_=(up*std::cos(b)+h.unit()*std::fabs(std::sin(b))).unit();
  }
  void finish_rest(Vec3 up,Vec3 end_bias){
    // Initial bias admission requires an explicit physically-stationary user action.
    if(!bias_known_&&pending_==Command::None){stable=true;rest_up_=up;return;}
    if(pending_!=Command::None){
      Command command=pending_;pending_=Command::None;
      if(command==Command::Zero){
        // The actual contact plane is +Z at capture. No center/yaw inference
        // or single-axis alignment is needed; transport starts right here.
        beta_deg=std::atan2(std::hypot(up.x,up.y),up.z)/kRad;
        normal_={0,0,1};blade_valid=anchored=true;drift_budget_deg=0;
        bias_=end_bias;bias_known_=true;rest_up_=up;stable=true;
        clear_segment();++sequence;publish();return;
      }
      if(!pose_ok(up)){status=Status::BadPose;stable=true;return;}
      if(command==Command::Blade){
        float b=std::atan2(up.x,up.z)/kRad;
        if(b< -30||b>.5f){status=Status::BadPose;stable=true;return;}
        beta_deg=b;blade_valid=true;anchored=false;status=Status::NeedCenter;
      }else if(blade_valid){
        float b=beta_deg*kRad;
        normal_=(up*std::cos(b)-up.cross({0,-1,0}).unit()*std::sin(b)).unit();
        anchored=true;drift_budget_deg=0;status=Status::Ready;
      }else {status=Status::NeedBlade;return;}
      bias_=end_bias;bias_known_=true;rest_up_=up;stable=true;clear_segment();++sequence;
      publish();return;
    }
    if(anchored){
      // The final quiet window is excluded: no integration while confirmed at rest.
      unsigned motion_count=count_>window_count_?count_-window_count_:0;
      if(motion_count){
        float t=0;
        for(unsigned i=0;i<motion_count;++i){
          const Sample &s=samples_[i];
          float f=fminf(1,(t+s.dt*.5f+kWindow*.5f)/segment_time_);
          Vec3 interpolated=bias_*(1-f)+end_bias*f;
          normal_=transport(normal_,s.gyro-interpolated,s.dt);t+=s.dt;
        }
        // Engineering budget, NOT calibrated confidence: replay cannot observe yaw.
        drift_budget_deg+=std::fabs(std::sin(beta_deg*kRad))*(.05f*t+.01f*t*t);
        if(drift_budget_deg>kMaxBudget){invalidate();}
      }
      if(anchored)apply_gravity_constraint(up);
    }
    bias_=end_bias;bias_known_=true;rest_up_=up;stable=true;clear_segment();publish();
  }
  void publish(){
    value_valid=anchored&&stable;
    if(value_valid){
      live_normal_=normal_;
      angle_deg=std::atan2(std::hypot(normal_.x,normal_.y),std::fabs(normal_.z))/kRad;
      status=Status::Ready;
    }
  }
};
} // namespace stationary_rel
