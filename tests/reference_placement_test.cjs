const assert=require('node:assert/strict');
const M=require('../simulation/spine_axis_model.cjs');
const {run}=require('../simulation/reference_placement.cjs');
for(const beta of [-15,-8,-3,0])for(const height of [20,58.5,120]){
 const out=run(beta,height),center=out.samples.find(r=>r.s===.5);
 assert(Math.abs(center.bladeError)<1e-9);
 assert(Math.abs(center.stoneDeltaError)<1e-9);
 assert(Math.abs(center.stonePlane)<1e-9);
 if(beta===0)assert(out.blade.max<1e-9);
 for(const row of out.samples){
  assert(Math.abs(row.bladeError-row.stoneDeltaError)<1e-9);
  const r=M.geometry(row.s,{beta,pivotHeight:height});
  assert(Math.abs(Math.acos(Math.min(1,Math.abs(M.dot(r.B,r.N))))/M.rad-row.truth)<1e-9);
 }
}
console.log('PASS: 12 configurations × 1001 positions; reference-zero and geometry checks');
