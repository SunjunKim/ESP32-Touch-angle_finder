#define main regression_unused
#include "stationary_rel_tracker_test.cpp"
#undef main
#include "../src/pages/accel_rel.h"
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
int main(){
  std::map<std::string,Stats> results;
  for(int angle=7;angle<=30;angle++)for(bool tanto:{false,true})for(float tip:{0.f,15.f,30.f,45.f})
    for(float pos:{0.f,.5f,.625f,.749f,.751f,.875f,1.f})for(int group=0;group<5;group++){
      float beta=group==0?0:group==1?-angle:group==2?-7:group==3?-15:-30;
      const char *name=group==0?"level":group==1?"stone_level_center":group==2?"beta-7":group==3?"beta-15":"beta-30";
      auto end=shape(pos,beta,center_height(beta,angle),tip,tanto);
      if(end.pose.truth<6.999f||end.pose.truth>30.001f)continue;
      Vec3 up=end.pose.up;
      const float ref=beta*kRad;
      Vec3 h=up.cross({0,-1,0}).unit();
      Vec3 n=up*cosf(ref)-h*sinf(ref);
      float normal_angle=atan2f(hypotf(n.x,n.y),fabsf(n.z))/kRad;
      float signed_tilt=up.x>=0?atan2f(hypotf(up.x,up.y),up.z)/kRad:-atan2f(hypotf(up.x,up.y),up.z)/kRad;
      float xz=atan2f(up.x,up.z)/kRad;
      float pitch=atan2f(up.x,hypotf(up.y,up.z))/kRad;
      char loc[150];snprintf(loc,sizeof(loc),"beta=%.0f central=%.0f pos=%.3f tip=%.0f %s",beta,float(angle),pos,tip,tanto?"tanto":"chef");
      results[std::string(name)+"/current"].add(fabsf(signed_tilt-beta),end.pose.truth,loc);
      results[std::string(name)+"/xz"].add(fabsf(xz-beta),end.pose.truth,loc);
      results[std::string(name)+"/pitch"].add(fabsf(pitch-beta),end.pose.truth,loc);
      results[std::string(name)+"/normal"].add(normal_angle,end.pose.truth,loc);
    }
  for(auto &[k,v]:results){printf("%s n=%zu\n",k.c_str(),v.errors.size());v.print("result");}
}
