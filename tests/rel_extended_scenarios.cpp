#define main regression_unused
#include "stationary_rel_tracker_test.cpp"
#undef main
#include <random>
#include <map>
#include <string>
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
float case_beta=0,case_alpha=0,case_s=0,case_tip=0;
struct Summary{int n=0,valid=0,bad=0,settling=0,recenter=0;float oldmax=0,newmax=0,wb=0,wa=0,ws=0,wt=0,wtruth=0;};
std::map<std::string,Summary> summaries;
void record(std::string key,Tracker &t,float old,float truth){auto &s=summaries[key];s.n++;float err=fabsf(fabsf(old)-truth);if(err>s.oldmax){s.oldmax=err;s.wb=case_beta;s.wa=case_alpha;s.ws=case_s;s.wt=case_tip;s.wtruth=truth;}if(t.value_valid){s.valid++;s.newmax=fmaxf(s.newmax,fabsf(t.angle_deg-truth));}else{if(t.status==Status::BadPose)s.bad++;if(t.status==Status::Settling)s.settling++;if(t.status==Status::Recenter)s.recenter++;}}
void forward(float beta,float alpha,float target,float tip,bool tanto,int noise,std::string key){
  case_beta=beta;case_alpha=alpha;case_s=target;case_tip=tip;
  float H=center_height(beta,alpha);Surface end=shape(target,beta,H,tip,tanto);
  if(end.pose.truth<6.999f||end.pose.truth>30.001f)return;
  static Tracker t;t.reset();std::mt19937 rng(1234+(int)(target*1000)+(int)alpha);std::normal_distribution<float> gauss(0,1);
  float time=0;Vec3 last_up;
  auto feed=[&](Vec3 up,Vec3 w,float dt){time+=dt;Vec3 a=up*-1,bias{7,2,-.15f};
    if(noise){bias=bias+Vec3{-.0081f,-.000503f,.000199f}*time;a=a+Vec3{.000389f*gauss(rng),.000546f*gauss(rng),.000368f*gauss(rng)};w=w+Vec3{.0291f*gauss(rng),.0343f*gauss(rng),.0279f*gauss(rng)};}
    t.update(a,w+bias,dt);last_up=a.unit()*-1;};
  auto rest_here=[&](Vec3 up,int n){for(int i=0;i<n;i++)feed(up,{},.01f);};
  t.request(Command::Blade);Vec3 ref{sinf(beta*kRad),0,cosf(beta*kRad)};rest_here(ref,160);
  for(int i=0;i<640&&!t.blade_valid;i++)feed(ref,{},.01f);
  float old_ref=old_abs(last_up,false);Pose previous=shape(.5,beta,H,tip,tanto).pose;
  if(!t.blade_valid){record(key,t,old_abs(end.pose.up,true)-old_ref,end.pose.truth);return;}
  t.request(Command::Center);rest_here(previous.up,160);for(int i=0;i<640&&!t.anchored;i++)feed(previous.up,{},.01f);
  for(int i=1;i<=200;i++){
    float s=.5f+(target-.5f)*i/200;Pose current=shape(s,beta,H,tip,tanto).pose;
    if(tanto&&tip>0&&target>.75f&&(.5f+(target-.5f)*(i-1)/200)<.75f&&s>=.75f){
      Pose start=previous;for(int j=1;j<=50;j++){Pose q=blend(start,current,j/50.f);feed(q.up,gyro(previous,q,.01f),.01f);previous=q;}
    }else{feed(current.up,gyro(previous,current,.01f),.01f);previous=current;}
  }
  rest_here(end.pose.up,300);record(key,t,old_abs(last_up,true)-old_ref,end.pose.truth);
}
// Literal reverse operation using the existing buttons: stone reference, then
// center confirmation on the same stone, then move the instrument to the blade.
void reverse(float angle,bool stone_horizontal,bool flip,float tangent,float override_beta=999){
  float beta=override_beta!=999?override_beta:(stone_horizontal?-angle:0);
  Pose stone=shape(.5,beta,center_height(beta,angle),0,false).pose;
  Pose blade{{rx({0,1,0},beta*kRad),{-1,0,0},rx({0,0,1},beta*kRad)},{sinf(beta*kRad),0,cosf(beta*kRad)}, {},0};
  blade.axes[0]=rx({-sinf(tangent*kRad),cosf(tangent*kRad),0},beta*kRad);
  blade.axes[1]=rx({-cosf(tangent*kRad),-sinf(tangent*kRad),0},beta*kRad);
  blade.up={blade.axes[0].z,blade.axes[1].z,blade.axes[2].z};
  if(flip){for(Pose *p:{&stone,&blade}){p->axes[0]=p->axes[0]*-1;p->axes[1]=p->axes[1]*-1;p->up={p->axes[0].z,p->axes[1].z,p->axes[2].z};}}
  static Tracker t;t.reset();Vec3 bias{7,2,-.15f};t.request(Command::Blade);rest(t,stone.up,bias,160);
  float old_ref=old_abs(stone.up,false);
  if(t.blade_valid){t.request(Command::Center);rest(t,stone.up,bias,160);Pose previous=stone;
    for(int i=1;i<=200;i++){Pose p=blend(stone,blade,i/200.f);t.update(p.up*-1,gyro(previous,p,.01f)+bias,.01f);previous=p;}rest(t,blade.up,bias,300);}
  record(std::string("reverse_")+(override_beta!=999?"tilted":(stone_horizontal?"stone_level":"blade_level"))+(flip?"_flipped":"_normal"),t,old_abs(blade.up,true)-old_ref,angle);
}
void level_sweep(bool on_stone){
  static Tracker t;t.reset();Vec3 bias{7,2,-.15f};
  Pose reference{{{0,1,0},{-1,0,0},{0,0,1}},{0,0,1},{},0};
  t.request(Command::Blade);rest(t,reference.up,bias,160);
  Pose previous=on_stone?shape(.5,0,center_height(0,7),0,false).pose:reference;
  t.request(Command::Center);rest(t,previous.up,bias,160);
  for(int pass=0;pass<2;pass++)for(int i=0;i<24;i++){
    float alpha=pass?30-i:7+i;
    Pose end;
    if(on_stone)end=shape(.5,0,center_height(0,alpha),0,false).pose;
    else{float b=-alpha*kRad;end={{{0,cosf(b),sinf(b)},{-1,0,0},{0,-sinf(b),cosf(b)}},{sinf(b),0,cosf(b)},{},alpha};}
    Pose start=previous;
    for(int j=1;j<=100;j++){Pose q=blend(start,end,j/100.f);t.update(q.up*-1,gyro(previous,q,.01f)+bias,.01f);previous=q;}
    rest(t,end.up,bias,300);
    record(on_stone?"continuous_blade_level_measure_stone":"continuous_stone_level_measure_blade",t,old_abs(end.up,true),alpha);
    assert(t.value_valid&&fabsf(t.angle_deg-alpha)<.001f);
  }
}
int main(){
  level_sweep(true);level_sweep(false);
  int impossible=0,possible=0;
  for(int alpha=7;alpha<=30;alpha++){
    for(float tangent:{0.f,15.f,30.f,45.f})for(bool flip:{false,true}){
      for(bool stone_level:{false,true})reverse(alpha,stone_level,flip,tangent);
      for(float beta:{-7.f,-15.f,-30.f})reverse(alpha,false,flip,tangent,beta);
    }
    for(int noise:{0,1})for(bool tanto:{false,true})for(float tip:{0.f,15.f,30.f,45.f})for(float target:{0.f,.5f,.625f,.749f,.751f,.875f,1.f}){
      std::string suffix=(tanto?"_tanto":"_chef")+std::string(noise?"_noise":"_ideal");
      forward(0,alpha,target,tip,tanto,noise,"blade_level"+suffix);
      forward(-alpha,alpha,target,tip,tanto,noise,"stone_level_at_center"+suffix);
      for(float beta:{-7.f,-15.f,-30.f})forward(beta,alpha,target,tip,tanto,noise,"tilted"+suffix);
    }
    for(float tangent:{0.f,15.f,30.f,45.f}){
      float vertical=sinf(-alpha*kRad)*sinf(tangent*kRad);
      if(fabsf(vertical)<1e-6)possible++;else impossible++;
    }
  }
  for(auto &[key,s]:summaries){printf("%s n=%d valid=%d badPose=%d settling=%d recenter=%d oldMax=%.5f newMax=%.5f\n",key.c_str(),s.n,s.valid,s.bad,s.settling,s.recenter,s.oldmax,s.newmax);if(key.find("ideal")!=std::string::npos){printf("  old worst beta=%.0f centralAlpha=%.0f pos=%.3f tip=%.0f truth=%.5f\n",s.wb,s.wa,s.ws,s.wt,s.wtruth);assert(s.valid==s.n&&s.newmax<.001f);}}
  printf("horizontal stone tangent constraint: feasible=%d impossible=%d\n",possible,impossible);
  // A change of knife pose after reference cannot be observed by a fixed sensor.
  static Tracker t;t.request(Command::Blade);rest(t,{sinf(-10*kRad),0,cosf(-10*kRad)},{7,2,-.15f});
  t.request(Command::Center);rest(t,{0,0,1},{7,2,-.15f});float displayed=t.angle_deg;rest(t,{0,0,1},{7,2,-.15f},300);
  assert(t.value_valid&&fabsf(displayed-t.angle_deg)<.001f);
  printf("Uninstrumented knife rotation: true angle 10->20, unchanged sensor reading %.3f; new reference required.\n",displayed);
}
