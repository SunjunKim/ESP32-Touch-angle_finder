'use strict';
const M=require('./single_axis_model.cjs');
const {rad,add,sub,mul,dot,cross,unit,angle}=M;
const rx=(v,b)=>[v[0],v[1]*Math.cos(b)-v[2]*Math.sin(b),v[1]*Math.sin(b)+v[2]*Math.cos(b)];
// Fixed world spine line y=25,z=0, direction +X. Pivot does NOT rotate.
function geometry(s,options={}){
 const base=M.geometry(s,{power:2,...options,centerFlat:true,beta:0}),beta=options.beta??8,b=beta*rad;
 const axisPoint=[0,25,0],rotateVector=v=>rx(v,b);
 const knifePoint=v=>add(axisPoint,rotateVector(sub(v,axisPoint)));
 const E=knifePoint(base.E),T=rotateVector(base.T),B=rotateVector(base.B);
 const U=cross(B,T),P=[base.P[0],base.P[1],options.pivotHeight??base.P[2]],V=sub(P,E),h=base.o.offset;
 const a=Math.atan2(dot(V,B),dot(V,U))-Math.asin(h/Math.hypot(dot(V,B),dot(V,U)));
 const N=add(mul(B,Math.cos(a)),mul(U,-Math.sin(a)));
 const X=unit(sub(V,mul(N,h))),Y=cross(N,X);
 const T0=rotateVector(base.T0),U0=cross(B,T0),ref=[U0,mul(T0,-1),B];
 const axes=[X,Y,N],g=axes.map(v=>v[2]),g0=ref.map(v=>v[2]);
 return {s,o:{...base.o,beta,pivotHeight:P[2]},base,axisPoint,knifePoint,rotateVector,E,T,B,U,P,V,N,X,Y,T0,U0,axes,ref,g,g0,
  truth:M.acute(angle(B,N)),edge:t=>knifePoint(base.edge(t)),
  centerTangentElevation:Math.asin(T0[2])/rad};
}
module.exports={...M,geometry};
