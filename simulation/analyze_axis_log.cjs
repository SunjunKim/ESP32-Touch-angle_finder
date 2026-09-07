const fs=require('node:fs');
const records=fs.readFileSync(process.argv[2],'utf8').trim().split('\n').map(JSON.parse);
const rows=records.filter(r=>r.line?.startsWith('AXIS ')).map(r=>{
 const fields=Object.fromEntries(r.line.split(' ').slice(1).map(s=>s.split('=')));
 const x={host:r.elapsed_s};for(const [k,v]of Object.entries(fields))x[k]=v.includes(',')?v.split(',').map(Number):Number(v);return x;
});
if(!rows.length)throw Error('No AXIS records');
const start=rows[0].t;rows.forEach(r=>r.sec=(r.t-start)/1e6);
const avg=a=>a.reduce((s,x)=>s+x,0)/a.length;
const stats=a=>({mean:avg(a),sd:Math.sqrt(avg(a.map(x=>(x-avg(a))**2))),min:Math.min(...a),max:Math.max(...a)});
const vectorStats=(rs,key)=>[0,1,2].map(k=>stats(rs.map(r=>r[key][k])));
const first=rows.filter(r=>r.sec<5),last=rows.filter(r=>r.sec>rows.at(-1).sec-5),test=rows.filter(r=>r.sec>=5);
const bias=[0,1,2].map(k=>avg(first.map(r=>r.W[k]))),integral=[0,0,0];
for(let i=1;i<test.length;i++)for(let k=0;k<3;k++)integral[k]+=((test[i].W[k]+test[i-1].W[k])/2-bias[k])*(test[i].sec-test[i-1].sec);
const gaps=rows.slice(1).map((r,i)=>(r.t-rows[i].t)/1e6);
console.log(JSON.stringify({file:process.argv[2],count:rows.length,span:rows.at(-1).sec,deviceDt:stats(rows.map(r=>r.dt)),logGap:stats(gaps),temperature:stats(rows.map(r=>r.T)),accel:vectorStats(rows,'A'),rawGyro:vectorStats(rows,'RAWW'),mappedGyro:vectorStats(rows,'W'),bias:vectorStats(rows,'B'),readyCount:rows.filter(r=>r.ready).length,stableCount:rows.filter(r=>r.stable).length,first5Mean:bias,last5Mean:[0,1,2].map(k=>avg(last.map(r=>r.W[k]))),heldOutResidual:vectorStats(test.map(r=>({...r,R:r.W.map((v,k)=>v-bias[k])})),'R'),heldOutIntegralDegrees:integral,summaryLines:records.filter(r=>r.line?.startsWith('IMU samples=')).map(r=>r.line)},null,2));
