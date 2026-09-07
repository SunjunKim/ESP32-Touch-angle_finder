const assert=require('node:assert/strict');
const M=require('../simulation/spine_axis_model.cjs');
let count=0;const rows=[];
for(const beta of [-15,-8,-3,0,3,8,15])for(let i=0;i<=100;i++){
 const s=i/100,r=M.geometry(s,{beta}),zero=M.geometry(s,{beta:0});
 assert.deepEqual(r.T0,[1,0,0],'central tangent must be parallel to spine');
 assert.equal(r.centerTangentElevation,0);
 assert(M.norm(M.sub(r.P,zero.P))<1e-12,'pivot moved with beta');
 for(const x of [-100,0,100])assert(M.norm(M.sub(r.knifePoint([x,25,0]),[x,25,0]))<1e-12,'spine axis moved');
 const radius=v=>Math.hypot(v[1]-25,v[2]);
 assert(Math.abs(radius(r.E)-radius(r.base.E))<1e-12,'not rotation about spine');
 assert(Math.abs(M.dot(r.N,r.T))<1e-12);assert(Math.abs(M.dot(r.N,r.X))<1e-12);
 assert(M.norm(M.cross(r.V,r.X))<1e-9,'rod does not reach pivot');
 const e=M.estimates(r.g,r.g0);
 if(beta===0)assert(Math.abs(e.signed-r.truth)<1e-8);
 if([0,50,100].includes(i))rows.push({beta,s,truth:r.truth,approx:e.signed,referenceValid:e.referenceValid,
  edge:r.E,pivot:r.P,centerTangentElevation:r.centerTangentElevation});
 count++;
}
assert(Math.abs(M.geometry(.5,{beta:8}).truth-18)>1);
for(const pivotHeight of [20,58.5,120])for(const beta of [-15,0,15])for(let i=0;i<=100;i++){
 const r=M.geometry(i/100,{beta,pivotHeight}),base=M.geometry(i/100,{beta});
 assert.equal(r.P[2],pivotHeight);
 assert.deepEqual(r.P.slice(0,2),base.P.slice(0,2));
 assert.deepEqual(r.E,base.E,'height adjustment moved knife');
 assert(Math.abs(M.dot(r.N,r.T))<1e-12);
 assert(Math.abs(M.dot(r.N,r.X))<1e-12);
 assert(M.norm(M.cross(r.V,r.X))<1e-9);
 assert(Number.isFinite(r.truth));count++;
}
assert(M.geometry(.5,{pivotHeight:120}).truth>M.geometry(.5,{pivotHeight:20}).truth);
console.log(JSON.stringify({tests:'PASS',count,rows},null,2));
