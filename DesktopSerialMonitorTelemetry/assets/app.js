'use strict';
const $=id=>document.getElementById(id),bridge=window.chrome.webview;
let seq=0,pending=new Map(),state={running:false,state:null},busy=false,page='serial',stream='',chunks=[],stored=0,paused=false,schedule=null,sending=false,rxBase=0,txBase=0,autoMonitor=false;
function sendHost(action,data={}){const id=String(++seq);return new Promise((resolve,reject)=>{pending.set(id,{resolve,reject});bridge.postMessage({action,id,...data});});}
function notice(text,error=false){$('notice').textContent=text;$('footDot').classList.toggle('error',error);}
async function act(action,data={},success='操作完成。'){if(busy)return;busy=true;update();try{await sendHost(action,data);notice(success);}catch(e){notice(e.message,true);if(action==='send')stopSchedule();}finally{busy=false;update();}}
function flag(name){return !!state.state?.[name];}
function enable(id,yes,tip){$(id).disabled=!yes;$(id).title=yes?'':tip;}
function update(){const run=state.running,connected=flag('connected'),enabled=flag('serial_enabled');
 $('summary').textContent=connected?`${state.state.port} 已连接`:run?'后台已开启':'全部已关闭';$('summary').classList.toggle('on',connected);
 $('serverState').textContent=run?'运行中':state.legacy?'旧后台占用':'未开启';$('serverState').classList.toggle('on',run);
 $('serialState').textContent=connected?'已连接':enabled?'连接中':'未连接';$('serialState').classList.toggle('on',connected);
 if(enabled){if([...$('port').options].some(o=>o.value===state.state.port))$('port').value=state.state.port;const cfg=state.state.serial_settings||{};$('baud').value=String(state.state.baud);$('bits').value=String(cfg.bytesize||8);$('parity').value=cfg.parity||'N';$('stopbits').value=String(cfg.stopbits||1);}
 $('step1').className='step '+(run?'done':'on');$('step2').className='step '+(connected?'done':run?'on':'');$('step3').className='step '+(connected?'on':'');
 $('next').textContent=!run?'从第一步开始':!connected?'下一步：连接串口':'数据已共享，可切换右侧监控';
 enable('start',!busy&&!run,'后台已经开启');enable('stop',!busy,'正在处理操作');
 enable('connect',!busy&&run&&!enabled,'先开启后台；更换端口前请断开');enable('disconnect',!busy&&run,'先开启后台');
 for(const id of ['port','baud','bits','parity','stopbits'])enable(id,!busy&&!enabled,'断开串口后可修改配置');
 enable('ports',!busy,'正在处理操作');enable('send',connected&&!sending&&!busy,'连接串口后才能发送');enable('periodic',connected&&!busy,'连接串口后才能启用定时发送');
 const buttons=[...document.querySelectorAll('[data-page]')];buttons.forEach(b=>{b.disabled=busy||(b.dataset.page==='local'&&!run);b.title=b.disabled?'先开启后台':'';});
 $('serialHint').textContent=!run?'先开启后台，再连接串口。':connected?'同一串口连接同时供调试区和网页使用。':enabled?(state.state.error||'正在连接，请稍候…'):'选择端口后点击连接。';
 $('receiveState').textContent=paused?(connected?'显示已暂停 · 仍在接收':'显示已暂停'):connected?'实时接收':run?'等待串口连接':'等待连接';
 $('rx').textContent=Math.max(0,(state.state?.rx_bytes||0)-rxBase).toLocaleString();$('tx').textContent=Math.max(0,(state.state?.tx_bytes||0)-txBase).toLocaleString();
 if(!connected)stopSchedule();
 if(run&&!autoMonitor){autoMonitor=true;showPage('local');}
 if((page==='local'&&!run))showPage('serial');
}
function bytesToBase64(bytes){let result='';for(let i=0;i<bytes.length;i+=4096)result+=String.fromCharCode(...bytes.subarray(i,i+4096));return btoa(result);}
function base64ToBytes(text){return Uint8Array.from(atob(text),c=>c.charCodeAt(0));}
function payload(){let text=$('sendText').value,bytes;if($('sendHex').checked){const compact=text.replace(/\s/g,'');if(!compact||compact.length%2||!/^[0-9a-f]+$/i.test(compact))throw new Error('HEX 需为成对的十六进制字节，例如：AA 55 01');bytes=Uint8Array.from(compact.match(/../g),v=>parseInt(v,16));}else bytes=new TextEncoder().encode(text);if($('newline').checked){const copy=new Uint8Array(bytes.length+2);copy.set(bytes);copy.set([13,10],bytes.length);bytes=copy;}if(!bytes.length)throw new Error('请先输入发送内容。');if(bytes.length>65536)throw new Error('每次最多发送 65536 字节。');return bytesToBase64(bytes);}
async function transmit(){if(sending||!flag('connected'))return;sending=true;update();try{await sendHost('send',{data:payload()});notice('数据已发送。');}catch(e){notice(e.message,true);stopSchedule();}finally{sending=false;update();}}
function stopSchedule(){if(schedule)clearInterval(schedule);schedule=null;$('periodic').checked=false;}
function render(){if(paused)return;const hex=$('hex').checked,times=$('timestamp').checked;let decoder=new TextDecoder('utf-8'),text='';for(const item of chunks){const content=hex?[...item.bytes].map(b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ')+'\n':decoder.decode(item.bytes,{stream:true});text+=(times?'['+item.time+'] ':'')+content;}$('log').textContent=text;$('log').scrollTop=$('log').scrollHeight;$('empty').hidden=stored>0;}
function addData(p){if(p.stream!==stream){stream=p.stream;chunks=[];stored=0;rxBase=txBase=0;}$('rx').textContent=Math.max(0,p.rx-rxBase).toLocaleString();$('tx').textContent=Math.max(0,p.tx-txBase).toLocaleString();if(!p.data)return;const bytes=base64ToBytes(p.data);chunks.push({bytes,time:new Date().toLocaleTimeString('zh-CN',{hour12:false})});stored+=bytes.length;while(stored>131072&&chunks.length>1)stored-=chunks.shift().bytes.length;if(p.dropped)notice('接收显示缓冲已跳过较旧数据，完整原始日志保存在日志文件夹。');render();}
function layout(){const r=$('stage').getBoundingClientRect();bridge.postMessage({action:'layout',x:r.x,y:r.y,w:r.width,h:r.height,scale:devicePixelRatio});}
async function showPage(target){try{layout();await sendHost('show',{page:target});page=target;$('terminal').hidden=false;$('stage').hidden=target==='serial';document.body.dataset.page=target;layout();$('reload').hidden=$('external').hidden=target==='serial';$('sharing').hidden=target!=='serial';document.querySelectorAll('[data-page]').forEach(b=>b.classList.toggle('active',b.dataset.page===target));}catch(e){notice(e.message,true);}}
bridge.addEventListener('message',event=>{const d=event.data;if(d.type==='result'){const p=pending.get(d.id);if(p){pending.delete(d.id);d.ok?p.resolve(d):p.reject(new Error(d.error));}}else if(d.type==='state'){state=d;update();}else if(d.type==='ports'){const selected=$('port').value||'COM9';$('port').replaceChildren();for(const port of d.ports){const o=new Option(port,port);$('port').add(o);}if(d.ports.includes(selected))$('port').value=selected;else if(!d.ports.length)$('port').add(new Option('未发现串口',''));}else if(d.type==='data')addData(d.packet);else if(d.type==='notice')notice(d.text,d.error);});
for(const name of ['start','stop','ports','disconnect','logs','reload','external'])$(name).onclick=()=>act(name,{},({start:'后台已开启。下一步：选择并连接串口。',stop:'后台已关闭，串口已释放。',ports:'串口列表已刷新。',disconnect:'串口已断开，可使用原串口助手。'}[name]||'操作完成。'));
 $('connect').onclick=()=>act('connect',{config:{port:$('port').value,baud:Number($('baud').value),bytesize:Number($('bits').value),parity:$('parity').value,stopbits:Number($('stopbits').value)}},'已请求连接，请查看连接状态。');
 document.querySelectorAll('[data-page]').forEach(b=>b.onclick=()=>showPage(b.dataset.page));$('send').onclick=transmit;
 $('periodic').onchange=()=>{if(!$('periodic').checked)return stopSchedule();try{payload();const ms=Number($('interval').value);if(ms<100||ms>60000||!Number.isInteger(ms))throw new Error('定时周期需在 100 至 60000 ms 之间。');schedule=setInterval(transmit,ms);notice('定时发送已手动开启。取消勾选即可停止。');}catch(e){stopSchedule();notice(e.message,true);}};
 $('clear').onclick=()=>{chunks=[];stored=0;rxBase=state.state?.rx_bytes||0;txBase=state.state?.tx_bytes||0;$('log').textContent='';$('empty').hidden=false;update();};
 function setDisplayPaused(value){paused=value;$('pause').textContent=paused?'恢复显示':'暂停显示';$('pause').classList.toggle('is-paused',paused);$('pause').setAttribute('aria-pressed',String(paused));update();if(!paused)render();}
 $('pause').onclick=async()=>{
   if(busy)return;
   if(paused){setDisplayPaused(false);notice('显示已恢复；本轮计时保持结束，下一次 KEY2 成功启动重新计时。');return;}
   if(!state.running){setDisplayPaused(true);notice('显示已暂停；后台尚未开启。');return;}
   if(state.state?.round_timer_version!==3){notice('请先重开后台再使用此功能：当前计时尚未结束。',true);return;}
   busy=true;update();
   try{await sendHost('end_round');setDisplayPaused(true);notice('显示已暂停；本轮计时已结束，秒数已保留，串口接收保持原状态。');}
   catch(error){notice('本轮计时未结束：'+error.message,true);}
   finally{busy=false;update();}
 };$('hex').onchange=$('timestamp').onchange=render;
 $('save').onclick=()=>act('save',{text:$('log').textContent},'保存操作已完成。');
 // The map and receive frame use native corner resize handles.
 // Observe both the embedded view and console so its native bounds follow.
 new ResizeObserver(layout).observe($('terminal'));
 new ResizeObserver(layout).observe($('stage'));window.addEventListener('beforeunload',stopSchedule);update();bridge.postMessage({action:'ready'});