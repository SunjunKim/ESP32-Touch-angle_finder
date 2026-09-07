// Reuses the already-validated fixed-pivot chef/tanto geometry verbatim.
#define main regression_unused
#include "stationary_rel_tracker_test.cpp"
#undef main
#include "../src/pages/accel_rel.h"
#include <random>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
struct Surface {Pose pose;Vec3 point,tangent;};
Surface shape(float s,float beta,float height,float tip,bool tanto,bool after=true){
  float x=200*s-100,y=-25,slope=0,k=tanf(tip*kRad);
  if(tanto){if(x>50||(after&&x==50)){y+=k*(x-50);slope=k;}}
  else if(x>0){y+=k*x*x/200;slope=k*x/100;}
  float b=beta*kRad;Vec3 E=Vec3{0,25,0}+rx({x,y-25,0},b);
  Vec3 T=rx(Vec3{1,slope,0}.unit(),b),B=rx({0,0,1},b),U=B.cross(T),V=Vec3{0,155,height}-E;
  float a=atan2f(V.dot(B),V.dot(U));Vec3 N=B*cosf(a)-U*sinf(a),X=V.unit(),Y=N.cross(X);
  Pose p{{X,Y,N},{X.z,Y.z,N.z},B,atan2f(B.cross(N).norm(),fabsf(B.dot(N)))/kRad};
  return {p,E,T};
}
float center_height(float beta,float angle){
  float b=beta*kRad,a=angle*kRad;
  Vec3 E=Vec3{0,25,0}+rx({0,-50,0},b),B=rx({0,0,1},b),U=rx({0,1,0},b),N=B*cosf(a)-U*sinf(a);
  return (E.dot(N)-155*N.y)/N.z;
}
// Smooth finite-duration repositioning across the tanto corner. Intermediate
// frames are a lifted stone, not asserted to be tangent to a sharp vertex.
Pose blend(Pose p,Pose q,float f){
  Vec3 v{};float trace=0;for(int i=0;i<3;i++){v=v+p.axes[i].cross(q.axes[i]);trace+=p.axes[i].dot(q.axes[i]);}
  float sn=v.norm()/2,angle=atan2f(sn,(trace-1)/2);Pose out=p;
  if(sn>1e-7f){Vec3 w=v*(-angle/(2*sn*kRad));for(int i=0;i<3;i++)out.axes[i]=transport(p.axes[i],w,f);}
  out.up={out.axes[0].z,out.axes[1].z,out.axes[2].z};return out;
}
float old_abs(Vec3 up,bool ref){float v=atan2f(hypotf(up.x,up.y),up.z)/kRad;if(!ref&&v<.15f)v=0;return up.x<=0?v:-v;}
struct Stats {
  std::vector<double> errors;double sum=0,bias=0,max=-1;
  std::string worst;
  void add(double measured,double truth,const std::string &where){
    double e=std::fabs(measured-truth);errors.push_back(e);sum+=e;bias+=measured-truth;
    if(e>max){max=e;char v[100];snprintf(v,sizeof(v)," truth=%.5f display=%.5f",truth,measured);worst=where+v;}
  }
  void print(const char *label){
    std::sort(errors.begin(),errors.end());
    printf(" %s MAE=%.5f P95=%.5f MAX=%.5f signed=%.5f\n  %s\n",label,sum/errors.size(),errors[(size_t)std::ceil(.95*errors.size())-1],max,bias/errors.size(),worst.c_str());
  }
};
struct Pair{Stats old,current;};
std::map<std::string,Pair> results;
void run(const std::string &key,Vec3 ref,Vec3 target,float truth,const std::string &where,int seed=-1){
  accel_rel::Filter filter;
  std::mt19937 rng(seed);std::normal_distribution<float> gauss(0,1);
  auto sample=[&](Vec3 up){Vec3 a=up*-1;if(seed>=0)a=a+Vec3{.000389f*gauss(rng),.000546f*gauss(rng),.000368f*gauss(rng)};return a;};
  Vec3 a;
  // Three-second hold at each surface. Main takes a single final sample;
  // current uses its deployed scalar exponential filter, tau=0.15s.
  for(int i=0;i<300;i++){a=sample(ref);filter.update(a.x,a.y,a.z,.01f);}
  float r=filter.tilt,oldref=old_abs(a.unit()*-1,false);
  for(int i=0;i<300;i++){a=sample(target);filter.update(a.x,a.y,a.z,.01f);}
  auto &s=results[key];
  s.old.add(fabsf(old_abs(a.unit()*-1,true)-oldref),truth,where);
  s.current.add(accel_rel::relative(filter.tilt,r),truth,where);
  assert(filter.valid);
}
std::string detail(float beta,float alpha,float s,float tip,bool tanto){
  char out[180];snprintf(out,sizeof(out),"beta=%.1f central=%.1f pos=%.3f tip=%.1f shape=%s",beta,alpha,s,tip,tanto?"tanto":"chef");return out;
}
Vec3 yaw(Vec3 up,float angle){float a=angle*kRad;return {up.x*cosf(a)+up.y*sinf(a),-up.x*sinf(a)+up.y*cosf(a),up.z};}
int main(){
  for(int angle=7;angle<=30;angle++)for(bool tanto:{false,true})for(float tip:{0.f,15.f,30.f,45.f})
    for(float s:{0.f,.5f,.625f,.749f,.751f,.875f,1.f})for(int group=0;group<5;group++){
      float beta=group==0?0:group==1?-angle:group==2?-7:group==3?-15:-30;
      std::string name=group==0?"blade_level":group==1?"stone_level_at_center":group==2?"beta_-7":group==3?"beta_-15":"beta_-30";
      auto end=shape(s,beta,center_height(beta,angle),tip,tanto);
      if(end.pose.truth<6.999f||end.pose.truth>30.001f)continue;
      Vec3 ref{sinf(beta*kRad),0,cosf(beta*kRad)};
      std::string where=detail(beta,angle,s,tip,tanto);
      run(name+"_ideal",ref,end.pose.up,end.pose.truth,where);
      for(int seed=0;seed<10;seed++)run(name+"_noise",ref,end.pose.up,end.pose.truth,where,seed+angle*100);
    }
  // Reverse: fixed central stone reference, measure the blade with its
  // long axis turned within the blade plane by up to 45 degrees (and 180 flip).
  for(int angle=7;angle<=30;angle++)for(int group=0;group<5;group++)
    for(float tangent:{0.f,15.f,30.f,45.f})for(bool flip:{false,true}){
      float beta=group==0?0:group==1?-angle:group==2?-7:group==3?-15:-30;
      Vec3 ref=shape(.5,beta,center_height(beta,angle),0,false).pose.up;
      Vec3 blade{sinf(beta*kRad)*cosf(tangent*kRad),-sinf(beta*kRad)*sinf(tangent*kRad),cosf(beta*kRad)};
      if(flip){ref=yaw(ref,180);blade=yaw(blade,180);}
      run(group==0?"reverse_blade_level":group==1?"reverse_stone_level":"reverse_both_tilted",ref,blade,angle,detail(beta,angle,.5,tangent,flip));
    }
  // Existing reference is held fixed while changing only sensor in-plane yaw.
  for(float tilt:{0.f,7.f,15.f,30.f})for(int y=0;y<360;y++){
    Vec3 ref{sinf(tilt*kRad),0,cosf(tilt*kRad)};
    run("same_plane_yaw",ref,yaw(ref,y),0,"tilt="+std::to_string(tilt)+" yaw="+std::to_string(y));
  }
  // Two level-reference continuous sweeps: no reference recapture required.
  for(bool stone:{false,true})for(int pass=0;pass<2;pass++)for(int i=0;i<24;i++){
    float angle=pass?30-i:7+i;
    Vec3 target=stone?shape(.5,0,center_height(0,angle),0,false).pose.up:Vec3{-sinf(angle*kRad),0,cosf(angle*kRad)};
    run(stone?"sweep_measure_stone":"sweep_measure_blade",{0,0,1},target,angle,"angle="+std::to_string(angle));
  }
  for(auto &[key,s]:results){printf("%s n=%zu\n",key.c_str(),s.old.errors.size());s.old.print("MAIN");s.current.print("ACC");}
  assert(results["blade_level_ideal"].old.errors.size()==1309);
  assert(results["stone_level_at_center_ideal"].old.errors.size()==1318);
  assert(fabs(results["beta_-30_ideal"].old.max-27.15418)<.002);
  assert(results["blade_level_ideal"].current.max<.001);
  // Current X-signed mode intentionally shares the sign-branch yaw limitation.
  assert(fabs(results["same_plane_yaw"].current.max-60)<.001);
}
