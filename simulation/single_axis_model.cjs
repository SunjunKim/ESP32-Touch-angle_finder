'use strict';
// Geometry truth is separated from sensor-only estimators. Units: mm, degrees.
const rad=Math.PI/180;
const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]);
const mul=(a,k)=>a.map(v=>v*k),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>Math.hypot(...a),unit=a=>mul(a,1/norm(a));
const clip=x=>Math.max(-1,Math.min(1,x));
const angle=(a,b)=>Math.atan2(norm(cross(a,b)),dot(a,b))/rad;
const acute=a=>Math.acos(clip(Math.abs(Math.cos(a*rad))))/rad;
const rz=(v,a)=>[v[0]*Math.cos(a)-v[1]*Math.sin(a),v[0]*Math.sin(a)+v[1]*Math.cos(a),v[2]];
const rx=(v,a)=>[v[0],v[1]*Math.cos(a)-v[2]*Math.sin(a),v[1]*Math.sin(a)+v[2]*Math.cos(a)];
const defaults={power:4,rise:50,distance:180,central:18,offset:0,beta:8};
function geometry(s,options={}){
 const o={...defaults,...options},B=[0,0,1];
 const u=t=>o.centerFlat?Math.max(0,(t-.55)/.45):t;
 const edge=t=>[-100+200*t,-25+o.rise*(o.power===0?0:u(t)**o.power),0];
 const tangent=t=>unit([200,o.power===0||o.centerFlat&&t<=.55?0:o.rise*o.power*u(t)**(o.power-1)/(o.centerFlat?.45:1),0]);
 const T0=tangent(.5),U0=cross(B,T0),E=edge(s),T=tangent(s),U=cross(B,T);
 const height=o.distance*Math.tan(o.central*rad)+o.offset/Math.cos(o.central*rad);
 const P=add(add(edge(.5),mul(U0,o.distance)),[0,0,height]);
 const V=sub(P,E),theta=Math.atan2(V[2],dot(V,U));
 const a=theta-Math.asin(o.offset/Math.hypot(V[2],dot(V,U)));
 const N=add(mul(B,Math.cos(a)),mul(U,-Math.sin(a)));
 const X=unit(sub(V,mul(N,o.offset))),Y=cross(N,X);
 const phi=Math.atan2(T0[1],T0[0]);
 const transform=v=>rx(rz(v,-phi),o.beta*rad);
 const axes=[X,Y,N].map(transform),ref=[U0,mul(T0,-1),B].map(transform);
 const g=axes.map(v=>v[2]),g0=ref.map(v=>v[2]);
 return {s,o,E,P,T,U,N,X,Y,B,T0,U0,V,axes,ref,g,g0,transform,
  truth:angle(B,N),edge,tangent};
}
// Inputs are only measured gravity vectors in their respective body frames.
function estimates(g,g0){
 g=unit(g);g0=unit(g0);
 const theta=Math.atan2(Math.hypot(g[0],g[1]),g[2])/rad;
 const beta=Math.atan2(g0[0],g0[2])/rad;
 const refTilt=Math.atan2(Math.hypot(g0[0],g0[1]),g0[2])/rad;
 const signedTheta=(g[0]<0?-1:1)*theta;
 const lower=Math.abs(theta-refTilt),upper=theta+refTilt;
 return {
  unsigned:acute(theta-refTilt),
  signed:acute(signedTheta-beta),
  projected:acute(Math.atan2(g[0],g[2])/rad-beta),
  vector:angle(g0,g),
  beta,theta,
  branchWarning:Math.abs(g[0])<.01&&Math.hypot(g[0],g[1])>.01,
  referenceValid:Math.abs(g0[1])<Math.sin(.5*rad),
  lower:Math.min(acute(lower),acute(upper)),
  upper:lower<=90&&upper>=90?90:Math.max(acute(lower),acute(upper))
 };
}
function noIntegration(g,g0,omega,bias=[0,0,0]){
 const e=estimates(g,g0),rotating=norm(sub(omega,bias))>=.8;
 return {estimate:e.signed,value:rotating||e.branchWarning||!e.referenceValid?null:e.signed,rotating};
}
function stationarySensorAtSlide(r,slide){
 return {position:r.transform(add(add(r.E,mul(r.X,slide)),mul(r.N,12))),
  gravity:r.axes.map(axis=>axis[2])};
}
module.exports={rad,add,sub,mul,dot,cross,norm,unit,angle,acute,defaults,geometry,estimates,noIntegration,stationarySensorAtSlide};
