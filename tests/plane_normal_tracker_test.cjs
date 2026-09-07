const assert=require('node:assert/strict');
const M=require('../simulation/spine_axis_model.cjs');
const {PlaneNormalTracker}=require('../simulation/plane_normal_tracker.cjs');
const body=(r,v)=>r.axes.map(a=>M.dot(a,v));
// Body relative rotation logarithm: prev axes -> current axes, gyro sign reversed.
function gyro(prev,r,dt){
 const R=r.axes.map(a=>prev.axes.map(b=>M.dot(a,b)));
 const v=[R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]];
 const sine=M.norm(v)/2,cosine=(R[0][0]+R[1][1]+R[2][2]-1)/2;
 const theta=Math.atan2(sine,cosine);
 return sine<1e-12?[0,0,0]:M.mul(v,-theta/(2*sine*dt*M.rad));
}
const rows=[];
for(const beta of [0,-3,-8,-15])for(const bias of [0,.01,-.01,.05,-.05])for(const anchorEvery of [0,1000]){
 const dt=.01,tracker=new PlaneNormalTracker(beta);
 let prev=M.geometry(.5,{beta,pivotHeight:58.5});tracker.anchor(prev.g);
 let max=0,end=0;
 for(let i=1;i<=6000;i++){
  const s=.5+.5*Math.sin(2*Math.PI*i/2000),r=M.geometry(s,{beta,pivotHeight:58.5});
  // Inject hypothetical residual gyro bias around world vertical; exact static gravity.
  const w=M.add(gyro(prev,r,dt),M.mul(r.g,bias));
  const value=tracker.update(r.g,w,dt,true);end=Math.abs(value-r.truth);max=Math.max(max,end);
  if(anchorEvery&&i%anchorEvery===0)tracker.anchor(r.g);
  if(bias===0)assert(max<1e-7);
  prev=r;
 }
 tracker.anchor(prev.g);
 assert(M.norm(M.sub(tracker.n,body(prev,prev.B)))<1e-8);
 rows.push({beta,biasDps:bias,anchorEverySeconds:anchorEvery*dt,seconds:60,maxError:max,endError:end,anchorNormalError:M.norm(M.sub(tracker.n,body(prev,prev.B)))});
}
console.log(JSON.stringify({pass:true,model:'ideal gravity; synthetic gyro; 3 heel-tip cycles; known beta; no acceleration/noise',rows},null,2));
