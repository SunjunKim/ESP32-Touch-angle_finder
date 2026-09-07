'use strict';
// Static, noise-free sensor geometry. Reference captured once at s=0.5.
const M=require('./spine_axis_model.cjs');
const tilt=g=>Math.sign(g[0]||1)*Math.atan2(Math.hypot(g[0],g[1]),g[2])/M.rad;
const plane=(a,b)=>Math.atan2(M.norm(M.cross(a,b)),Math.abs(M.dot(a,b)))/M.rad;
function run(beta,pivotHeight){
 const ref=M.geometry(.5,{beta,pivotHeight}),rows=[];
 for(let i=0;i<=1000;i++){
  const s=i/1000,r=M.geometry(s,{beta,pivotHeight});
  const bladeEstimate=M.acute(tilt(r.g)-tilt(ref.g0));
  const stoneDelta=tilt(r.g)-tilt(ref.g);
  const trueDelta=r.truth-ref.truth,stonePlane=plane(ref.N,r.N);
  rows.push({s,truth:r.truth,bladeEstimate,bladeError:bladeEstimate-r.truth,
   trueDelta,stoneDelta,stoneDeltaError:stoneDelta-trueDelta,
   stonePlane,stonePlaneEstimate:M.acute(stoneDelta),
   stonePlaneError:M.acute(stoneDelta)-stonePlane,
   idealGyroAsDeltaError:stonePlane-Math.abs(trueDelta)});
 }
 const stats=k=>({max:Math.max(...rows.map(r=>Math.abs(r[k]))),rms:Math.sqrt(rows.reduce((a,r)=>a+r[k]**2,0)/rows.length)});
 return {beta,pivotHeight,center:ref.truth,blade:stats('bladeError'),stoneDelta:stats('stoneDeltaError'),stonePlane:stats('stonePlaneError'),idealGyroAsDelta:stats('idealGyroAsDeltaError'),samples:rows.filter((r,i)=>[0,500,550,750,900,1000].includes(i))};
}
if(require.main===module)console.log(JSON.stringify({assumptions:'200 mm blade, flat through 55%, quadratic tip; static ideal IMU; signed total tilt estimator; beta <= 0 only; no firmware changes',cases:[-15,-8,-3,0].map(b=>run(b,58.5)),heightCases:[20,58.5,120].map(h=>run(-8,h))},null,2));
module.exports={run};
