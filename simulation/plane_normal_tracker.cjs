'use strict';
// Prototype only: body-frame knife normal, gyro propagation, gravity-cone correction.
const M=require('./spine_axis_model.cjs');
const {add,sub,mul,dot,cross,norm,unit,rad}=M;
function rotate(v,omega,dt){
 const speed=norm(omega),a=speed*dt;if(speed<1e-12)return v;
 const k=mul(omega,1/speed);
 return add(add(mul(v,Math.cos(a)),mul(cross(k,v),Math.sin(a))),mul(k,dot(k,v)*(1-Math.cos(a))));
}
class PlaneNormalTracker{
 constructor(beta){this.beta=beta*rad;this.n=null;}
 // User-confirmed center, stone touching blade, sensor Y opposite central tangent.
 // beta is saved from blade-face calibration, not inferred from stone zero.
 anchor(gravity){
  const g=unit(gravity),t=[0,-1,0];
  if(Math.abs(dot(g,t))>Math.sin(.5*rad))throw Error('central pose invalid');
  this.n=unit(sub(mul(g,Math.cos(this.beta)),mul(unit(cross(g,t)),Math.sin(this.beta))));
 }
 update(gravity,gyroDps,dt,gravityTrusted=true){
  if(!this.n)throw Error('anchor required');
  this.n=unit(rotate(this.n,mul(gyroDps,-rad),dt));
  if(gravityTrusted){
   const g=unit(gravity),horizontal=sub(this.n,mul(g,dot(this.n,g)));
   if(Math.abs(this.beta)<1e-9)this.n=g;
   else if(norm(horizontal)>1e-9)this.n=add(mul(g,Math.cos(this.beta)),mul(unit(horizontal),Math.abs(Math.sin(this.beta))));
  }
  return M.acute(Math.atan2(Math.hypot(this.n[0],this.n[1]),Math.abs(this.n[2]))/rad);
 }
}
module.exports={PlaneNormalTracker,rotate};
