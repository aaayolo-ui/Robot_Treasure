const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const root=process.argv[2]||__dirname,els=new Map(),calls=[];
let callback,fail=false;
function get(id){if(!els.has(id))els.set(id,{value:'',checked:false,disabled:false,textContent:'',hidden:false,options:[],classList:{toggle(){}},setAttribute(){},getBoundingClientRect(){return{x:0,y:0,width:800,height:500};}});return els.get(id);}
const bridge={addEventListener(_,handler){callback=handler;},postMessage(msg){calls.push(msg);if(msg.action==='end_round')queueMicrotask(()=>callback({data:{type:'result',id:msg.id,ok:!fail,error:'offline test failure'}}));}};
const ctx={console,TextEncoder,TextDecoder,Uint8Array,Promise,Map,Number,Date,Error,String,Math,atob,btoa,devicePixelRatio:1,
 document:{getElementById:get,querySelectorAll(){return[];},body:{dataset:{}}},ResizeObserver:class{observe(){}},
 window:{chrome:{webview:bridge},addEventListener(){}},setInterval(){return 1;},clearInterval(){},Option:class{}};
vm.createContext(ctx);vm.runInContext(fs.readFileSync(path.join(root,'assets/app.js'),'utf8'),ctx);
function run(s){return vm.runInContext(s,ctx);}
(async()=>{
 run('autoMonitor=true;state={running:true,state:{connected:false,serial_enabled:false,round_timer_version:3}};update()');
 await get('pause').onclick();
 assert.equal(run('paused'),true);assert.equal(calls.filter(c=>c.action==='end_round').length,1);
 assert.equal(get('pause').textContent,'恢复显示');
 await get('pause').onclick();
 assert.equal(run('paused'),false);assert.equal(calls.filter(c=>c.action==='end_round').length,1);
 assert(get('notice').textContent.includes('计时保持结束'));
 fail=true;await get('pause').onclick();
 assert.equal(run('paused'),false);assert(get('notice').textContent.includes('计时未结束'));
 run('state.state.round_timer_version=2');const before=calls.length;
 await get('pause').onclick();assert.equal(calls.length,before);assert.equal(run('paused'),false);
 assert(get('notice').textContent.includes('重开后台'));
 assert(!calls.some(c=>['send','disconnect','connect'].includes(c.action)));
 // Use actual counter selection/state code: ended time stays visible and labelled ended.
 const html=fs.readFileSync(path.join(root,'assets/laser_live.html'),'utf8');
 const timing=html.slice(html.indexOf('    const timing=data.round_timer'),html.indexOf('    if(!switchingPort){'));
 ctx.timerBox={classList:{toggle(){}}};ctx.data={round_timer_version:3,round_timer:{elapsed_ms:37000,running:false,stop_reason:'display_pause',reached_180:false}};
 ctx.document.querySelector=id=>get(id);ctx.age=0;
 vm.runInContext(timing,ctx);assert.equal(get('#drive-time').textContent,'37.000 秒');
 assert(get('#drive-time-state').textContent.includes('本轮已结束'));
 console.log(path.basename(root)+': pause ends once, resume never restarts, errors truthful, held seconds visible, no serial writes.');
})().catch(e=>{console.error(e);process.exitCode=1;});
