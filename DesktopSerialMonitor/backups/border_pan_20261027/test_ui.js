const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements=new Map();
function get(id){if(!elements.has(id))elements.set(id,{value:'',checked:false,disabled:false,textContent:'',hidden:false,title:'',options:[],classList:{toggle(){}},replaceChildren(){},add(){},insertAdjacentHTML(){},setAttribute(){},getBoundingClientRect(){return{x:0,y:0,width:800,height:300};}});return elements.get(id);}
get('interval').value='1000';
const context={console,TextEncoder,TextDecoder,Uint8Array,Promise,Map,Number,Date,Error,String,Math,atob,btoa,devicePixelRatio:1,
 document:{getElementById:get,querySelectorAll(){return[];},body:{dataset:{},style:{setProperty(){}}}},ResizeObserver:class{observe(){}},
 window:{chrome:{webview:{postMessage(){},addEventListener(){}}},addEventListener(){}},setInterval(){return 1;},clearInterval(){},Option:class{},};
vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/assets/app.js','utf8'),context);
function run(code){return vm.runInContext(code,context);}
assert.equal(get('connect').disabled,true);assert.equal(get('send').disabled,true);assert.equal(get('cloud_on').disabled,true);
run('state={running:true,cloud:false,state:{connected:false,serial_enabled:false}};update()');
assert.equal(get('connect').disabled,false);assert.equal(get('send').disabled,true);
run('state.state.connected=true;state.state.serial_enabled=true;update()');
assert.equal(get('connect').disabled,true);assert.equal(get('send').disabled,false);assert.equal(get('cloud_on').disabled,false);
get('sendText').value='你好\n';assert.equal(run('payload()'),Buffer.from('你好\n').toString('base64'));
get('sendHex').checked=true;get('sendText').value='AA 55 00 ff';assert.equal(run('payload()'),'qlUA/w==');
get('newline').checked=true;assert.equal(run('payload()'),'qlUA/w0K');
for(const invalid of ['A','GG','AA,55']){get('sendText').value=invalid;assert.throws(()=>run('payload()'));}
run('schedule=1;state.state.connected=false;update()');assert.equal(run('schedule'),null);assert.equal(get('periodic').checked,false);
console.log('UI workflow, UTF-8, HEX, CRLF and periodic-stop checks passed.');
get('pane-splitter').onpointerdown({button:0,clientY:300,pointerId:1,currentTarget:{setPointerCapture(){}},preventDefault(){}});
get('pane-splitter').onpointermove({clientY:240});
assert.equal(run('monitorPercent'),45);
get('pane-splitter').onpointerup();get('pane-splitter').onpointermove({clientY:100});
assert.equal(run('monitorPercent'),45);
get('pane-splitter').onkeydown({key:'End',preventDefault(){}});assert.equal(run('monitorPercent'),80);
get('pane-splitter').onkeydown({key:'Home',preventDefault(){}});assert.equal(run('monitorPercent'),25);
get('pane-splitter').ondblclick();assert.equal(run('monitorPercent'),55);
console.log('Pane drag, release, keyboard limits and reset checks passed.');
