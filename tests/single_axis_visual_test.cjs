const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const M=require('../simulation/single_axis_model.cjs');
const ctx={};vm.createContext(ctx);const html=fs.readFileSync(process.argv[2],'utf8');
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],ctx);
let count=0;for(const beta of [-15,-8,-3,0,3,8,15])for(let i=0;i<=100;i++){
 const r=M.geometry(i/100,{beta}),e=M.estimates(r.g,r.g0),v=ctx.SharpenerSim.sample(i/100,beta,0,0);
 assert(Math.abs(v.truth-r.truth)<1e-7);
 assert(Math.abs(v.acc.value-e.signed)<1e-7);
 const expected=M.noIntegration(r.g,r.g0,[0,0,0]).value;
 if(expected===null)assert.equal(v.result.value,null);
 else assert(Math.abs(v.result.value-expected)<1e-7);count++;
}
console.log('PASS: '+count+' visual/model cross-checks');
