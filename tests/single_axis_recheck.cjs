'use strict';
const assert=require('node:assert/strict');
const M=require('../simulation/single_axis_model.cjs');
const keys=['unsigned','signed','projected','vector'];
const stats=()=>Object.fromEntries(keys.map(k=>[k,{count:0,max:0,sumAbs:0,sumSq:0,worst:null}]));
function accumulate(st,r,e){for(const k of keys){const d=e[k]-r.truth,a=Math.abs(d),v=st[k];v.count++;v.sumAbs+=a;v.sumSq+=d*d;if(a>v.max){v.max=a;v.worst={s:r.s,...r.o,truth:r.truth,value:e[k]};}}}
function summarize(st){return Object.fromEntries(keys.map(k=>[k,{n:st[k].count,maxAbs:st[k].max,mae:st[k].sumAbs/st[k].count,rmse:Math.sqrt(st[k].sumSq/st[k].count),worst:st[k].worst}]))}
const total=stats(),positive=stats(),negative=stats(),nominal=stats();let count=0;
for(const power of [2,4,6])for(const distance of [140,180,240])for(const central of [12,18,25])for(const offset of [0,10])for(const beta of [-15,-8,-3,0,3,8,15]){
 for(let i=0;i<=100;i++){
  const r=M.geometry(i/100,{power,distance,central,offset,beta}),e=M.estimates(r.g,r.g0);
  assert(Math.abs(r.ref[1][2])<1e-12,'central tangent not horizontal');
  assert(Math.abs(M.dot(r.N,r.T))<1e-12,'stone plane must contain tangent');
  assert(Math.abs(M.dot(r.N,r.X))<1e-12);
  assert(M.norm(M.cross(M.sub(r.V,M.mul(r.N,offset)),r.X))<1e-9,'rod misses pivot');
  assert(r.truth>=e.lower-1e-6&&r.truth<=e.upper+1e-6);
  if(beta===0)assert(Math.abs(e.signed-r.truth)<1e-8);
  if(i===50)assert(Math.abs(e.signed-central)<1e-8,'signed central angle not exact');
  accumulate(total,r,e);if(beta>=0)accumulate(positive,r,e);else accumulate(negative,r,e);
  if(power===4&&distance===180&&central===18&&offset===0)accumulate(nominal,r,e);
  count++;
 }
}
// A straight ruler/edge keeps its stone plane fixed while the rod yaws.
for(const beta of [-15,-8,0,8,15])for(let i=0;i<=100;i++){
 const r=M.geometry(i/100,{power:0,beta}),e=M.estimates(r.g,r.g0);
 assert(Math.abs(r.truth-18)<1e-8);
 assert(Math.abs(e.signed-18)<1e-8);
}
// Pure telescope translation changes neither axes nor static accelerometer.
const r0=M.geometry(.8);
for(const displacement of [-50,-20,0,20,50]){
 const sensor=M.stationarySensorAtSlide(r0,displacement);
 assert(sensor.position.every(Number.isFinite));
 assert.deepEqual(M.estimates(sensor.gravity,r0.g0),M.estimates(r0.g,r0.g0));
}
const expected=M.noIntegration(r0.g,r0.g0,[0,0,0]).value;
for(let i=0;i<10000;i++)assert.equal(M.noIntegration(r0.g,r0.g0,[0,0,0]).value,expected);
assert.equal(M.noIntegration([0,0,1],[0,0,1],[0,0,20]).value,null);
// Central -8 degree reference demonstrates the unsigned-reference bug.
const n=M.geometry(.5,{beta:-8}),ne=M.estimates(n.g,n.g0);
assert(Math.abs(ne.unsigned-2)<1e-8);assert(Math.abs(ne.signed-18)<1e-8);
// Same central calibration and pivot geometry, different admissible knife
// profiles/contacts. Sensor observations cannot identify the true angle.
const ca=M.geometry(.9,{beta:8});
const cb=M.geometry(.8230175610276419,{beta:8,power:8,rise:34.53062768536293});
assert(M.norm(M.sub(ca.g,cb.g))<1e-10);
assert(M.norm(M.sub(ca.g0,cb.g0))<1e-12);
const centeredPivot=r=>r.transform(M.sub(r.P,r.edge(.5)));
assert(M.norm(M.sub(centeredPivot(ca),centeredPivot(cb)))<1e-10);
assert(Math.abs(ca.truth-cb.truth)>.4);
const rows=[];for(const beta of [-8,0,8,15])for(const s of [0,.5,.75,1]){
 const r=M.geometry(s,{beta}),e=M.estimates(r.g,r.g0);
 rows.push({beta,s,truth:r.truth,...Object.fromEntries(keys.map(k=>[k,e[k]])),error:e.signed-r.truth});
}
console.log(JSON.stringify({count,tests:'PASS',total:summarize(total),positive:summarize(positive),negative:summarize(negative),nominal:summarize(nominal),rows,
 counterexample:{g:ca.g,g0:ca.g0,first:{s:ca.s,options:ca.o,truth:ca.truth},second:{s:cb.s,options:cb.o,truth:cb.truth}}},null,2));
