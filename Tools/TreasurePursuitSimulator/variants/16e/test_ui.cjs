// Offline DOM/event smoke test of our generated page. No browser or network.
// CSS, layout, fonts and real pointer handling are not tested by this stub.
const fs = require('node:fs'), path = require('node:path');
const vm = require('node:vm'), assert = require('node:assert/strict');
const root = __dirname, html = fs.readFileSync(path.join(root, 'replay.html'), 'utf8');
const report = JSON.parse(fs.readFileSync(path.join(root, 'report.json'), 'utf8'));
const elements = new Map();
class Element {
  constructor(tag) {
    this.tagName=tag;this.attrs={};this.children=[];this.events={};this.style={};
    this.value='';this.checked=false;this.hidden=false;this._text='';
    const classes=new Set();
    this.classList={toggle:(s,on)=>{on?classes.add(s):classes.delete(s);},contains:s=>classes.has(s)};
  }
  set textContent(s) {this._text=String(s);this.children=[];}
  get textContent() {return this._text+this.children.map(c=>c.textContent).join('');}
  set innerHTML(s) {assert.equal(s,'');this.children=[];this._text='';}
  setAttribute(k,v) {this.attrs[k]=String(v);if(k==='id')elements.set(String(v),this);}
  append(...c) {this.children.push(...c);}
  replaceChildren(...c) {this._text='';this.children=[...c];}
  addEventListener(name,fn) {(this.events[name]??=[]).push(fn);}
  emit(name) {for(const f of this.events[name]??[])f({key:'Enter',preventDefault(){}});}
  getBoundingClientRect() {return {width:this.width??900,height:800};}
}
for(const m of html.matchAll(/<([a-z0-9-]+)([^>]*\bid="([^"]+)"[^>]*)>/gi)) {
  const e=new Element(m[1]);e.setAttribute('id',m[3]);
  e.checked=/\bchecked\b/.test(m[2]);elements.set(m[3],e);
}
elements.get('report-data').textContent=JSON.stringify(report);
elements.get('trail').value='all';elements.get('speed').value='1';elements.get('scenario').value='0';
const context=vm.createContext({console,Blob,URL,location:{protocol:'http:'},
  document:{getElementById:id=>{assert.ok(elements.has(id),id);return elements.get(id);},
    createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag)},
  requestAnimationFrame(){return 1;},ResizeObserver:class {constructor(fn){this.fn=fn;}observe(){}}
});
let scripts=0;
for(const m of html.matchAll(/<script([^>]*)>([\s\S]*?)<\/script>/g)) {
  if(m[1].includes('application/json'))continue;
  new vm.Script(m[2],{filename:`page-script-${++scripts}`}).runInContext(context);
}
const el=id=>elements.get(id), shapes=()=>el('map').children;
const marked=(name)=>shapes().filter(e=>e.attrs.class===name);
assert.equal(el('scenario').children.length,4);
assert.equal(marked('annotation-node').length,88);
assert.ok(shapes().some(e=>e.attrs['data-label-for']==='N001' && e.textContent.startsWith('N001')));
assert.ok(el('comparison').textContent.includes('37.83'));
assert.ok(!el('comparison').textContent.includes('undefined'));
assert.ok(el('firmware').textContent.includes('N023'));
marked('annotation-node').find(e=>e.attrs['data-node-id']==='N001').emit('click');
assert.ok(el('map-info').textContent.includes('N001'));
assert.ok(el('map-info').textContent.includes('N002'));
assert.ok(el('map-info').textContent.includes('N010'));
assert.ok(el('map-info').textContent.includes('0.60'));
el('show-segments').checked=true;el('show-segments').emit('change');
assert.equal(marked('annotation-segment').length,106);
const first=report.annotations.segments[0];
marked('annotation-segment').find(e=>e.attrs['data-segment-id']===first.id).emit('click');
for(const value of [first.id,first.start,first.end,first.length_m.toFixed(2)])
  assert.ok(el('map-info').textContent.includes(value),value);
el('show-nodes').checked=false;el('show-nodes').emit('change');
assert.equal(marked('annotation-node').length,0);
assert.equal(marked('annotation-segment').length,106);
const p=shapes().find(e=>e.attrs.id==='police-trail'),b=shapes().find(e=>e.attrs.id==='thief-trail');
assert.ok(p&&b);assert.notEqual(p.attrs.stroke,b.attrs.stroke);
assert.equal(p.attrs['stroke-dasharray'],'none');assert.equal(b.attrs['stroke-dasharray'],'10 7');
el('show-segments').checked=false;el('show-segments').emit('change');
assert.equal(marked('annotation-segment').length,0);
el('show-nodes').checked=true;el('show-nodes').emit('change');
el('hide-jump').emit('click');assert.ok(Number(el('timeline').value)>0);
const hideIndex=Number(el('timeline').value);
el('decision-jump').emit('click');assert.ok(Number(el('timeline').value)>hideIndex);
assert.ok(el('firmware').textContent.includes('N039'));
el('play').emit('click');assert.notEqual(el('play').textContent,'');
el('restart').emit('click');assert.equal(Number(el('timeline').value),0);
el('scenario').value='1';el('scenario').emit('change');
assert.ok(el('parameters').textContent.includes('0.809'));
el('contact').emit('click');assert.equal(Number(el('timeline').value),report.replays[1].frames.length-1);
assert.ok(shapes().some(e=>e.attrs.id==='contact-marker'));
assert.ok(el('status').textContent.includes('police') || el('status').textContent.length>0);
assert.equal(el('download-map').href,'map_annotations.json');
assert.deepEqual(JSON.parse(fs.readFileSync(path.join(root,'map_annotations.json'),'utf8')),report.annotations);
const audit={scripts,nodes:88,segments:106,node_click:true,segment_click:true,
  toggles:true,distinct_trails:true,hide_jump:true,decision_jump:true,scenario:true,
  capture_marker:true,export_matches:true,browser:false,
  scope:'Offline DOM/event smoke test. Browser layout and hardware remain unverified.'};
fs.writeFileSync(path.join(root,'ui_smoke_result.json'),JSON.stringify(audit,null,2)+'\n');
console.log(JSON.stringify(audit));
