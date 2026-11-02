// Verify the actual rendering functions, including wrap, stale data and bit order.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/assets/laser_live.html','utf8');
const code=html.slice(html.indexOf('let latestOnboard=null;'),html.indexOf('let pollInFlight=false;'));
const nodes=new Map();
function node(id){
 if(!nodes.has(id))nodes.set(id,{style:{},dataset:{},attrs:{},textContent:'',innerHTML:'',
  flags:new Set(),classList:{toggle(name,on){const f=node(id).flags;on?f.add(name):f.delete(name);}},
  setAttribute(k,v){this.attrs[k]=v;},querySelectorAll(){return this.cells||[];}});
 return nodes.get(id);
}
for(const side of ['front','rear'])node(`#gray-${side}-cells`).cells=Array.from({length:12},(_,i)=>{
 const n=node(`${side}-${i}`);n.dataset.bit=String(side==='front'?11-i:i);return n;
});
let now=50000;
const clock=class extends Date{static now(){return now;}};
const ctx={document:{querySelector:node},Date:clock,Math,Boolean,Number,String};
vm.createContext(ctx);vm.runInContext(code,ctx);
function run(js){return vm.runInContext(js,ctx);}
assert.equal(run('shortestAngle(359,1)'),2);
assert.equal(run('shortestAngle(1,359)'),-2);
const sample={track_generation:1,gray:{front:{mask:0x801,time_ms:now,source:'GRAY_LIVE'},rear:{mask:2,time_ms:now,source:'GRAY_LIVE'}},
 gyro:{yaw_deg:44,valid:true,data_age_ms:10,time_ms:now,sensor_age_ms:10},
 localization_diagnostic:{yaw_error_deg:2.5,valid:true,yaw_age_ms:10,time_ms:now,yaw_cw_sign:1}};
ctx.sample=sample;run('updateOnboard(sample)');
assert.equal(node('#gyro-yaw').innerHTML,'44.0<em>°</em>');
assert.equal(node('#gyro-error').innerHTML,'+2.5<em>°</em>');
assert.equal(node('#vehicle-rotor').style.transform,'rotate(44deg)');
assert.deepEqual(node('#gray-front-cells').cells.map(c=>c.flags.has('line-hit')),[true,false,false,false,false,false,false,false,false,false,false,true]);
assert.equal(node('#gray-rear-cells').cells.filter(c=>c.flags.has('line-hit')).length,1);
assert(node('#gray-rear-cells').cells[1].flags.has('line-hit'));
for(const yaw of [0,44,90,180,270,359,1]){
 sample.gyro.yaw_deg=yaw;run('updateOnboard(sample)');
 const positions=[1,2,3,4].map(ch=>[parseFloat(node(`#orbit-${ch}`).style.left),parseFloat(node(`#orbit-${ch}`).style.top)]);
 for(const [x,y] of positions){assert(x>=15.4&&x<=84.6);assert(y>=15.4&&y<=84.6);}
 assert(Math.abs(positions[0][0]+positions[3][0]-100)<1e-8);
 assert(Math.abs(positions[1][1]+positions[2][1]-100)<1e-8);
}
const transform=node('#vehicle-rotor').style.transform;
now+=2000;sample.gyro.data_age_ms=2010;sample.gyro.yaw_deg=180;run('updateOnboard(sample)');
assert.equal(node('#vehicle-rotor').style.transform,transform);
assert(node('#vehicle-stage').flags.has('stale'));
assert(node('#gray-front-cells').flags.has('stale'));
assert(node('#gray-front-time').textContent.includes('旧原始采样'));
sample.gray={front:null,rear:null};sample.gyro=null;run('updateOnboard(sample)');
assert(node('#gray-front-cells').cells.every(c=>!c.flags.has('line-hit')));
assert.equal(node('#gray-rear-time').textContent,'未收到数据');
const markup=html.split('<script>')[0],ids=[...markup.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
assert.equal(new Set(ids).size,ids.length,'duplicate element IDs');
assert(!markup.includes('车体感知</h2>')); // Old miniature panel removed.
console.log('Telemetry render checks passed: angle wrap, four orbit positions, 24 bits, stale freeze, no duplicate IDs.');
