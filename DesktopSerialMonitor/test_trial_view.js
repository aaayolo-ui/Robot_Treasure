// Exercise the actual HTML path-poll function with a server generation rollover.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/assets/laser_live.html','utf8');
const pathCode=html.slice(html.indexOf('async function updatePath(){'),html.indexOf("document.querySelector('#show-map-nodes').addEventListener"));
const roleCode=html.slice(html.indexOf('let identifiedRole=null'),html.indexOf('async function loadMap(){'));
const elements=new Map();
function node(id){if(!elements.has(id))elements.set(id,{dataset:{},style:{},textContent:'',setAttribute(){}});return elements.get(id);}
const calls=[],packets=[{generation:2,cursor:1,points:[]},{generation:2,cursor:1,points:[{id:1,x_mm:3300,y_mm:1200}]}];
const context={document:{querySelector:node},fetch:async url=>{calls.push(url);return{ok:true,json:async()=>packets.shift()};},renderMapPose(){},clearTrail(){},endMapPan(){},showMapPosition(point){context.lastPoint=point;}};
vm.createContext(context);
vm.runInContext("let pathInFlight=false,viewEpoch=0,trackCursor=999,trackGeneration=1,trackPoints=[{id:999}],latestMapPosition=null,latestProjection=null,identityUncertain={};let mapZoom=3;const initialMapBox={x:-42,y:-42,width:744,height:624};const zoomMap={setAttribute(){}};",context);
vm.runInContext(roleCode+pathCode,context);
(async()=>{
 await vm.runInContext('updatePath()',context);
 assert.equal(vm.runInContext('trackCursor',context),0);
 assert.equal(vm.runInContext('trackPoints.length',context),0);
 await vm.runInContext('updatePath()',context);
 assert.deepEqual(calls,['/api/path?after=999','/api/path?after=0']);
 assert.equal(context.lastPoint.id,1);
 vm.runInContext("applyRole('police')",context);assert.equal(node('.map-panel').dataset.role,'police');
 vm.runInContext("applyRole('thief')",context);assert.equal(node('.map-panel').dataset.role,'thief');
 vm.runInContext("applyRole(null)",context);assert.equal(node('#role-label').textContent,'等待识别');
 assert(!html.includes('id="role-select"'));
 console.log('New-generation origin fetch, old-path clearing and automatic role view checks passed.');
})().catch(e=>{console.error(e);process.exitCode=1;});
