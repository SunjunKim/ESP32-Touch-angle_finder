const fs=require('node:fs');
const vm=require('node:vm');
const assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2],'utf8');
const context={};vm.createContext(context);
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const S=context.SharpenerSim;
let count=0,maxError=0,maxLevelError=0;
for(const pitch of [-25,-15,0,10,25])for(const roll of [-25,0,15,25]){
 for(let i=0;i<=100;i++){
  const r=S.sample(i/100,pitch,roll,0,0);
  assert(Math.abs(S.dot(r.p.N,r.p.T))<1e-12,'tangent not in stone plane');
  assert(Math.abs(S.dot(r.p.N,r.p.X))<1e-12,'rod not in stone plane');
  assert(S.norm(S.cross(r.p.X,r.p.V))<1e-10,'rod misses pivot');
  assert(Math.abs(S.norm(r.g)-1)<1e-12);
  assert(r.truth>=r.acc.lower-1e-5&&r.truth<=r.acc.upper+1e-5,'bound excludes truth');
  assert(Math.abs(r.acc.value-r.result.value)<1e-12,'gyro changed static angle');
  const error=Math.abs(r.acc.value-r.truth);maxError=Math.max(maxError,error);
  if(pitch===0&&roll===0)maxLevelError=Math.max(maxLevelError,error);
  count++;
 }
}
assert(maxLevelError<1e-5);
assert(S.sample(.1).tangentError>10);
assert(S.sample(.5).tangentError<1e-6);
// No integration/state: replay order and elapsed time do not affect results.
const a=S.sample(.8,10,15,25,.03),b=S.sample(.8,10,15,25,.03);
assert(JSON.stringify(a.result)===JSON.stringify(b.result));
assert(S.gated([0,0,1],1,[0,0,1]).value===null);
assert(S.gated([0,0,1],1,[0,0,0]).value===0);
// Same gravity and reference inclination, different unknown relative heading.
const theta=25*S.D,ref=10*S.D,g=[Math.sin(theta),0,Math.cos(theta)];
const angle=heading=>Math.acos(Math.cos(theta)*Math.cos(ref)+Math.sin(theta)*Math.sin(ref)*Math.cos(heading))/S.D;
assert(Math.abs(angle(0)-15)<1e-10);assert(Math.abs(angle(Math.PI)-35)<1e-10);
const interval=S.estimate(g,Math.cos(ref));
assert(Math.abs(interval.lower-15)<1e-10&&Math.abs(interval.upper-35)<1e-10);
console.log(JSON.stringify({count,maxError,maxLevelError,indistinguishableAngles:[angle(0),angle(Math.PI)]},null,2));
for(const [pitch,roll] of [[0,0],[8,0],[10,15]]){
 const rows=[0,.25,.5,.75,1].map(s=>{const r=S.sample(s,pitch,roll,0,.03);return {
  s,truth:+r.truth.toFixed(3),acc:+r.acc.value.toFixed(3),gyro:+r.result.value.toFixed(3),
  error:+(r.acc.value-r.truth).toFixed(3),longAxisObliquity:+r.tangentError.toFixed(2)};});
 console.log(JSON.stringify({pitch,roll,rows}));
}
