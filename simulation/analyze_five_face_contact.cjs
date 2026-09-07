// Read-only analysis of calibrated REL_CONTACT outputs. No calibration writes.
const fs=require('fs'),path=require('path');
const root=path.join(__dirname,'hardware_logs');
const files={bottom:'stationary-60s-20260907-235016.jsonl',end:'stationary-60s-20260907-235227.jsonl',end0:'stationary-30s-20260907-235357.jsonl',south:'stationary-60s-20260907-235525.jsonl',north:'stationary-60s-20260907-235707.jsonl',top:'stationary-60s-20260907-235847.jsonl'};
const records=Object.fromEntries(Object.entries(files).map(([k,f])=>[k,fs.readFileSync(path.join(root,f),'utf8').trim().split('\n').map(JSON.parse).flatMap(r=>{
  if(!r.line?.startsWith('REL_CONTACT '))return [];
  const m=r.line.match(/A=([-\d.]+),([-\d.]+),([-\d.]+).*valid=(\d).*moving=(\d)/);
  return m&&m[4]==='1'&&m[5]==='0'?[{t:r.elapsed_s,g:m.slice(1,4).map(Number)}]:[];
})]));
const specs=[
  {name:'bottom',normal:2,sign:-1,ranges:[['bottom',2.024,4.034],['bottom',6.055,14.105],['bottom',17.120,22.164],['bottom',25.192,59.399]]},
  {name:'end',normal:1,sign:-1,ranges:[['end0',14.994,30.068],['end',2.298,7.316],['end',9.331,15.374],['end',17.382,50.579]]},
  {name:'south',normal:0,sign:1,ranges:[['south',2.025,12.071],['south',15.096,23.134],['south',25.149,33.194],['south',36.203,59.337]]},
  {name:'north',normal:0,sign:-1,ranges:[['north',2.324,13.392],['north',15.406,24.448],['north',27.458,35.503],['north',40.533,59.646]]},
  {name:'top',normal:2,sign:1,ranges:[['top',2.046,17.168],['top',19.175,26.212],['top',29.229,38.306],['top',40.318,51.394]]}
];
const mean=a=>a.reduce((s,v)=>s+v,0)/a.length;
const meanVec=a=>[0,1,2].map(i=>mean(a.map(v=>v[i])));
const angle=(v,n)=>Math.atan2(Math.hypot(...v.filter((_,i)=>i!==n)),Math.abs(v[n]))*180/Math.PI;
const spread=a=>Math.max(...a)-Math.min(...a);
const faces=specs.map(s=>{
  const directions=s.ranges.map(([key,lo,hi])=>{const rows=records[key].filter(r=>r.t>=lo&&r.t<=hi);if(rows.length<3)throw Error('Insufficient samples');return {n:rows.length,mean:meanVec(rows.map(r=>r.g)),sd:[0,1,2].map(i=>Math.sqrt(mean(rows.map(r=>(r.g[i]-mean(rows.map(r=>r.g[i])))**2))))};});
  const inclinations=directions.map(d=>angle(d.mean,s.normal));
  return {name:s.name,normal:s.normal,sign:s.sign,directions,mean:meanVec(directions.map(d=>d.mean)),inclinations,inclinationSpread:spread(inclinations)};
});
const get=name=>faces.find(f=>f.name===name).mean;
const bx=(get('south')[0]+get('north')[0])/2,bz=(get('top')[2]+get('bottom')[2])/2;
const spanX=(get('south')[0]-get('north')[0])/2,spanZ=(get('top')[2]-get('bottom')[2])/2;
// Equal direction/face weights. Midpoints approximate residual calibrated-frame
// bias only if opposite contact normals are near antipodal. Not a full 6-face fit.
console.log(JSON.stringify({faces,pairedNormalEstimates:{biasX_g:bx,biasZ_g:bz,halfSpanX_g:spanX,halfSpanZ_g:spanZ,smallAngleEquivalentX_deg:Math.atan(Math.abs(bx))*180/Math.PI},limitations:['Calibrated output, not raw input; active stored calibration validity is not logged','Missing +Y; no complete original six-face calibration','Hand-set headings and unverified contact-face orthogonality/flatness','Paired estimates are diagnostic, not approved coefficients','Inclined-reference yaw ambiguity of REL remains even with ideal sensor calibration']},null,2));
