// Reuse the independently computed knife/stone geometry and gyro generation.
#define main tracker_regression_main
#include "stationary_rel_tracker_test.cpp"
#undef main
#include <random>
float old_absolute(Vec3 up,bool has_ref){
  Vec3 a=up*-1;float tilt=atan2f(hypotf(a.x,a.y),-a.z)/kRad;
  if(!has_ref&&tilt<.15f)tilt=0;
  // HEAD 2f9d288: pitch sign is calibrated acceleration X, not up X.
  return a.x>=0?fabsf(tilt):-fabsf(tilt);
}
struct Result{float old_error,new_error;bool valid;unsigned status;};
Result evaluate(float beta,float target,int noise_mode,unsigned seed){
  static Tracker tracker;tracker.reset();std::mt19937 gen(seed);
  std::normal_distribution<float> gaussian(0,1);
  float time=0,old_ref=0;Vec3 last_up;float last_old=0;
  auto feed=[&](Vec3 up,Vec3 angular,float dt){
    time+=dt;Vec3 bias{7,2,-.15f};
    if(noise_mode)bias=bias+Vec3{-.0081f,-.000503f,.000199f}*time;
    Vec3 accel=up*-1;
    if(noise_mode==2){
      accel=accel+Vec3{.000389f*gaussian(gen),.000546f*gaussian(gen),.000368f*gaussian(gen)};
      angular=angular+Vec3{.0291f*gaussian(gen),.0343f*gaussian(gen),.0279f*gaussian(gen)};
    }
    tracker.update(accel,angular+bias,dt);last_up=accel.unit()*-1;
  };
  tracker.request(Command::Blade);
  Vec3 blade_up{sinf(beta*kRad),0,cosf(beta*kRad)};
  for(int i=0;i<160;i++)feed(blade_up,{},.01f);
  for(int i=0;i<640&&!tracker.blade_valid;i++)feed(blade_up,{},.01f);
  old_ref=old_absolute(last_up,false); // Original firmware stores one current reading.
  tracker.request(Command::Center);Pose previous=pose(.5f,beta);
  for(int i=0;i<160;i++)feed(previous.up,{},.01f);
  for(int i=0;i<640&&!tracker.anchored;i++)feed(previous.up,{},.01f);
  for(int i=1;i<=200;i++){
    Pose current=pose(.5f+(target-.5f)*i/200,beta);
    feed(current.up,gyro(previous,current,.01f),.01f);previous=current;
  }
  for(int i=0;i<300;i++)feed(previous.up,{},.01f);
  last_old=old_absolute(last_up,true)-old_ref;
  return {fabsf(last_old)-previous.truth,tracker.angle_deg-previous.truth,tracker.value_valid,(unsigned)tracker.status};
}
int main(){
  for(int mode=0;mode<=2;mode++)for(float b:{0.f,-3.f,-8.f,-15.f}){
    double oldsq=0,newsq=0;float oldmax=0,newmax=0,old_matched_max=0;unsigned valid=0,count=0,statuses[9]{};
    int positions=mode==2?21:101,repeats=mode==2?10:1;
    for(int i=0;i<positions;i++)for(int j=0;j<repeats;j++){
      Result r=evaluate(b,(float)i/(positions-1),mode,1234+i*31+j);
      oldmax=fmaxf(oldmax,fabsf(r.old_error));oldsq+=r.old_error*r.old_error;count++;
      if(!r.valid)++statuses[r.status];
      if(r.valid){valid++;old_matched_max=fmaxf(old_matched_max,fabsf(r.old_error));newmax=fmaxf(newmax,fabsf(r.new_error));newsq+=r.new_error*r.new_error;}
    }
    printf("mode=%d beta=%.0f count=%u valid=%u old_max=%.6f old_rms=%.6f new_max=%.6f new_rms=%.6f\n",
      mode,b,count,valid,oldmax,sqrt(oldsq/count),newmax,valid?sqrt(newsq/valid):NAN);
    for(unsigned i=0;i<9;i++)if(statuses[i])printf("  withheld status=%u count=%u\n",i,statuses[i]);
    if(mode==2)printf("  old_max_on_same_valid_trials=%.6f\n",old_matched_max);
  }
}
