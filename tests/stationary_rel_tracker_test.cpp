#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
#include "../simulation/experimental/stationary_rel_tracker.h"
#include "../simulation/experimental/gyro_frame.h"
using namespace stationary_rel;
struct Pose{Vec3 axes[3],up,blade;float truth;};
Vec3 rx(Vec3 v,float b){return {v.x,v.y*cosf(b)-v.z*sinf(b),v.y*sinf(b)+v.z*cosf(b)};}
Pose pose(float s,float beta){
  float b=beta*kRad,u=fmaxf(0,(s-.55f)/.45f);
  Vec3 e=Vec3{0,25,0}+rx({200*s-100,-50+50*u*u,0},b);
  Vec3 t=rx(Vec3{200,s<=.55f?0:100*u/.45f,0}.unit(),b),B=rx({0,0,1},b),U=B.cross(t);
  Vec3 v=Vec3{0,155,58.5f}-e;
  float a=atan2f(v.dot(B),v.dot(U));
  Vec3 n=B*cosf(a)-U*sinf(a),x=v.unit(),y=n.cross(x);
  return {{x,y,n},{x.z,y.z,n.z},B,atan2f(B.cross(n).norm(),fabsf(B.dot(n)))/kRad};
}
Vec3 gyro(Pose p,Pose q,float dt){
  float R[3][3];for(int i=0;i<3;i++)for(int j=0;j<3;j++)R[i][j]=q.axes[i].dot(p.axes[j]);
  Vec3 v{R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]};
  float sn=v.norm()/2,cs=(R[0][0]+R[1][1]+R[2][2]-1)/2;
  return sn<1e-7f?Vec3{}:v*(-atan2f(sn,cs)/(2*sn*dt*kRad));
}
void rest(Tracker &t,Vec3 up,Vec3 bias,int count=160){for(int i=0;i<count;i++)t.update(up*-1,bias,.01f);}
void init(Tracker &t,float b,Vec3 bias){
  t.reset();t.request(Command::Blade);rest(t,{sinf(b*kRad),0,cosf(b*kRad)},bias);
  assert(t.blade_valid&&!t.value_valid);assert(fabsf(t.beta_deg-b)<.001f);
  t.request(Command::Center);rest(t,pose(.5f,b).up,bias);
  assert(t.value_valid);assert(fabsf(t.angle_deg-pose(.5f,b).truth)<.002f);
}
int main(){
  // A polar-frame reflection requires an additional sign for angular velocity.
  for(float sign:{-1.f,1.f}){
    const float r[9]={.8660254f,-.5f,0,.5f,.8660254f,0,0,0,sign};
    const float scale[3]={1.02f,-.99f,1.04f};
    Vec3 a{.2f,.3f,.4f},b{.6f,-.7f,.1f},ca,cb;
    imu_math::mat3_vec3_rowmajor(r,a.x,-a.y,a.z,&ca.x,&ca.y,&ca.z);
    imu_math::mat3_vec3_rowmajor(r,b.x,-b.y,b.z,&cb.x,&cb.y,&cb.z);
    Vec3 g=a.cross(b),expected=ca.cross(cb);
    imu_math::rotate_gyro_to_device(r,scale,g.x,g.y,g.z);
    assert((g-expected).norm()<1e-6f);
  }
  static Tracker t;Vec3 bias{7,2,-.15f};
  // Direct plane-to-plane moves, 7..30 degrees with a 45-degree yaw
  // component, from level and tilted reference planes. Truth is transported
  // independently from the reference contact normal, not inferred from tilt.
  for(float ref:{-30.f,-.51f,0.f,.51f,30.f})for(int angle=7;angle<=30;angle++){
    t.reset();Vec3 up{sin(ref*kRad),0,cos(ref*kRad)},normal{0,0,1};
    t.request(Command::Zero);rest(t,up,bias,200);assert(t.value_valid);
    for(Vec3 rotation:{Vec3{0,float(angle),0},Vec3{0,0,45}}){
      for(int i=0;i<100;i++){
        Vec3 w=rotation*.5f;
        up=transport(up,w,.02f);normal=transport(normal,w,.02f);
        t.update(up*-1,w+bias,.02f);
        const float preview=atan2f(hypotf(normal.x,normal.y),fabsf(normal.z))/kRad;
        assert(t.anchored&&!t.value_valid&&fabsf(t.display_angle()-preview)<.025f);
      }
    }
    rest(t,up,bias,300);
    float truth=atan2f(hypotf(normal.x,normal.y),fabsf(normal.z))/kRad;
    assert(t.value_valid&&fabsf(t.angle_deg-truth)<.025f);
    assert(fabsf(t.display_angle()-t.angle_deg)<1e-6f);
  }
  // Single-action plane zero: both tilt signs, non-central yaw/roll and
  // reverse travel. No inferred central alignment or second confirmation.
  for(float beta:{-30.f,-.51f,0.f,.51f,30.f})
    for(float start:{.5f,.75f,1.f})for(float end:{.5f,.75f,1.f}){
      t.reset();Pose first=pose(start,beta),prev=first;
      t.request(Command::Zero);rest(t,first.up,bias,200);
      assert(t.value_valid&&t.angle_deg<.001f&&!t.capturing());
      for(int i=1;i<=100;i++){
        Pose next=pose(start+(end-start)*i/100,beta);
        t.update(next.up*-1,gyro(prev,next,.02f)+bias,.02f);prev=next;
      }
      rest(t,prev.up,bias,300);
      float truth=atan2f(first.axes[2].cross(prev.axes[2]).norm(),
                        fabsf(first.axes[2].dot(prev.axes[2])))/kRad;
      assert(t.value_valid&&fabsf(t.angle_deg-truth)<.025f);
      t.update(prev.up*-1,bias,.2f);assert(!t.value_valid);
      t.request(Command::Zero);rest(t,prev.up,bias,200);
      assert(t.value_valid&&t.angle_deg<.001f);
    }
  t.reset();
  // A static thermal-like offset must not latch the tracker into motion.
  for(float ref:{0.f,15.f,30.f}){
    t.reset();Vec3 up{sin(ref*kRad),0,cos(ref*kRad)};
    t.request(Command::Zero);rest(t,up,bias,200);
    float max_angle=0;
    for(int i=0;i<12000;i++){
      float seconds=i*.01f;
      Vec3 creep{.6f+1.2f*sinf(seconds*.12f),.15f*sinf(seconds*.09f),0};
      // Smooth sub-threshold contact vibration, not a static no-noise fixture.
      Vec3 vibration{.0005f*sinf(seconds*17),.0005f*cosf(seconds*13),0};
      t.update(up*-1+vibration,bias+creep,.01f);
      assert(t.anchored);
      if(i>200){assert(t.value_valid);max_angle=fmaxf(max_angle,t.angle_deg);}
    }
    assert(max_angle<.15f);
  }
  // Do not absorb clear constant yaw or a slow changing gravity direction.
  t.reset();t.request(Command::Zero);rest(t,{0,0,1},bias,200);
  for(int i=0;i<200;i++)t.update({0,0,-1},bias+Vec3{0,0,1},.01f);
  assert(!t.value_valid);
  t.reset();t.request(Command::Zero);rest(t,{0,0,1},bias,200);
  Vec3 slow_up{0,0,1};
  for(int i=0;i<300;i++){
    slow_up=transport(slow_up,{0,.5f,0},.01f);
    t.update(slow_up*-1,bias+Vec3{0,.5f,0},.01f);
  }
  assert(!t.value_valid);
  t.reset();
  rest(t,{0,0,1},bias,400);assert(!t.blade_valid&&!t.value_valid);
  for(float beta:{0.f,-3.f,-8.f,-15.f})for(float target:{0.f,.75f,1.f}){
    init(t,beta,bias);Pose previous=pose(.5f,beta);
    for(int i=1;i<=100;++i){
      Pose current=pose(.5f+(target-.5f)*i/100,beta);
      t.update(current.up*-1,gyro(previous,current,.02f)+bias,.02f);previous=current;
    }
    assert(!t.value_valid);rest(t,previous.up,bias,300);
    std::printf("beta %.0f position %.2f truth %.5f measured %.5f\n",beta,target,previous.truth,t.angle_deg);
    assert(t.value_valid);assert(fabsf(t.angle_deg-previous.truth)<.025f);
    float angle=t.angle_deg;
    for(int i=0;i<3000;i++)t.update(previous.up*-1,bias+Vec3{.008f*i*.01f,0,0},.01f);
    assert(t.value_valid);assert(fabsf(t.angle_deg-angle)<.025f);
    t.update(previous.up*-1,bias,.20f);assert(!t.anchored&&!t.value_valid&&t.blade_valid);
    t.request(Command::Center);rest(t,pose(.5f,beta).up,bias,200);assert(t.value_valid);
  }
  init(t,-8,bias);
  // Linear thermal-like drift across a short motion: endpoint replay should cancel it.
  Pose previous=pose(.5f,-8);float time=0;
  for(int i=1;i<=100;++i){
    time+=.02f;Pose current=pose(.5f+.5f*i/100,-8);
    t.update(current.up*-1,gyro(previous,current,.02f)+bias+Vec3{.008f*time,0,0},.02f);previous=current;
  }
  for(int i=0;i<300;++i){time+=.01f;t.update(previous.up*-1,bias+Vec3{.008f*time,0,0},.01f);}
  assert(t.value_valid&&fabsf(t.angle_deg-previous.truth)<.04f);
  init(t,-8,bias);
  for(int i=0;i<1000;i++)t.update(pose(.5,-8).up*-1,bias+Vec3{0,0,5},.01f);
  assert(!t.anchored&&!t.value_valid);
  init(t,-8,bias);t.update({NAN,0,0},bias,.01f);assert(!t.value_valid);
  init(t,-8,bias);t.update({0,0,-1},{999,0,0},.01f);assert(!t.value_valid);
  t.request(Command::Blade);rest(t,{0,.1f,.995f},bias,200);assert(!t.blade_valid&&t.status==Status::BadPose);
  t.request(Command::Blade);rest(t,{.2f,0,.98f},bias,200);assert(!t.blade_valid&&t.status==Status::BadPose);
  // Confirmation timeout must not reuse a pre-press stationary window.
  t.request(Command::Blade);
  for(int i=0;i<900;i++)t.update({0,0,-1},{(i%2)?10.f:0.f,0,0},.01f);
  assert(!t.blade_valid&&t.status==Status::Timeout);
  // Capacity bound at high ODR invalidates tracking, not an out-of-bounds write.
  init(t,-8,bias);
  for(int i=0;i<2200;i++)t.update({0,0,-1},bias+Vec3{0,0,5},.001f);
  assert(!t.anchored&&!t.value_valid);
  std::puts("PASS stationary REL geometry / drift / commands / discontinuity tests");
  return 0;
}
