"""Bind build, unchanged baseline modules and offline reports to current image."""
from pathlib import Path
import json,hashlib,re,xml.etree.ElementTree as ET,difflib
r=Path(__file__).resolve().parents[1];b=r/'Reference/ROUTE2_before_trace_return'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(a)
inputs=json.loads((r/'Tests/route5_inputs.json').read_text())
assert inputs['axf_sha256']==ah
assert all(sha(r/p)==h for p,h in inputs['test_sources'].items())
assert all(sha(r/p)==h for p,h in inputs['production'].items())
suites=json.loads((r/'Tests/route5_suites.json').read_text())
assert len(suites)==14 and all(x['exit_code']==0 for x in suites)
unchanged={}
for folder in ['Control','Hardware','Core','System','Drivers']:
    ps=[p for p in (b/folder).rglob('*') if p.is_file() and p.suffix in ['.c','.h','.s']]
    assert all(sha(p)==sha(r/p.relative_to(b)) for p in ps if p.name!='RestartLineSearch.c'),folder
    unchanged[folder]=len([p for p in ps if p.name!='RestartLineSearch.c'])
for p in (b/'Algorithm').rglob('*'):
    if p.is_file() and p.suffix in ['.c','.h']:assert sha(p)==sha(r/p.relative_to(b)),p
source=(r/'User/App_ThiefStrategy.c').read_text(encoding='utf-8');old=(b/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
def fn(text,name):
    m=re.search(r'static [^;\n]+\b'+name+r'\([^;]*?\)\s*\{',text);assert m,name
    j=text.index('{',m.start());k=j+1;depth=1
    while depth:depth+=(text[k]=='{')-(text[k]=='}');k+=1
    return text[m.start():k]
frozen=['Route_ReadFront','Route_ReadRear','Route_UpdateErrors','Route_UpdateFollowMode','Route_ClassifyPosition','Route_DriveStraight','Route_DriveUnlocatedFollow','Route_DriveTurnApproach','Route_DriveLeftTurn','Route_DriveRightTurn','Route_HasReachedTurnAngle','Route_EnterTurnApproach','Route_EnterTurnStop']
assert all(fn(source,n)==fn(old,n) for n in frozen)
def pid(text):return fn(text,'Route_RunFollow').split('  Route_UpdateFollowMode(sample);')[1].split('  Route_UpdateJunctionArm(front_state);')[0]
assert pid(source)==pid(old),'PID/smoothing/PI chain changed'
def turns(text):return text[text.index('    case ROUTE_STATE_TURN_APPROACH:'):text.index('    case ROUTE_STATE_AFTER_TURN_FORWARD:')]
assert turns(source)==turns(old),'normal approach/brake/main rotation changed'
assert 'ThiefRouteTrail_Begin(&route_trail,0U,0U);' in source[source.index('void App_ThiefStrategy_Init(void)'):]
assert b'UARTCH4_16V_ROUTE5_RETRACE' in a.read_bytes()
proj=r/'MDK-ARM'/(r.name+'.uvprojx');t=ET.parse(proj);ot=ET.parse(b/'MDK-ARM'/proj.name)
paths=[x.text.lower() for x in t.findall('.//FilePath')];assert len(paths)==len(set(paths))
assert all((proj.parent/x.text.replace('\\','/')).exists() for x in t.findall('.//FilePath'))
added=t.find(".//File[FileName='ThiefRouteTrail.c']");assert added is not None
parent=next(n for n in t.iter() if added in list(n));parent.remove(added)
def tree(n):return n.tag,sorted(n.attrib.items()),(n.text or '').strip(),tuple(tree(c) for c in n)
assert tree(t.getroot())==tree(ot.getroot()),'unrelated Keil configuration changed'
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json','results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json','results_left800.json','results_live_monitor.json','west63_after.json','trace_front_after.json','results_trace_return.json','results_trace_recovery.json','results_n068_recovery.json','results_rear_retrace.json']:
    p=r/'Tests'/n;z=json.loads(p.read_text());assert not z.get('failures'),n
    assert z.get('axf_sha256',z.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=z.get('checks'),sha256=sha(p))
pre=json.loads((r/'Tests/trace_front_before.json').read_text());original=b/'MDK-ARM'/r.name/(r.name+'.axf')
assert pre['failures'] and pre['axf_sha256']==sha(original)
mp=(r/'MDK-ARM'/r.name/(r.name+'.map')).read_text(encoding='utf-8',errors='replace')
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',mp)[1]);ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',mp)[1])
htm=(r/'MDK-ARM'/r.name/(r.name+'.htm')).read_text(encoding='utf-8',errors='replace');stack=int(re.search(r'Maximum Stack Usage =\s*(\d+)',htm)[1])
assert rom<256*1024 and ram<48*1024 and stack<3072
assert '0 Error(s), 0 Warning(s)' in (r/'build_route5.log').read_text(encoding='utf-8',errors='replace')
new=json.loads((r/'Tests/results_trace_return.json').read_text());assert new['trace_bytes']==876 and len(new['lifecycles'])==16
files=['User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/ThiefRouteTrail.c','Algorithm/ThiefStrategy/ThiefRouteTrail.h','MDK-ARM/'+r.name+'.uvprojx']
patch=''.join(''.join(difflib.unified_diff(((b/n).read_text(encoding='utf-8') if (b/n).exists() else '').splitlines(True),(r/n).read_text(encoding='utf-8').splitlines(True),fromfile='ROUTE2/'+n,tofile='ROUTE5/'+n)) for n in files)
prior=r/'Reference/ROUTE4_before_rear_retrace'
allowed={'User/App_ThiefStrategy.c','Control/RestartLineSearch.c'}
manifest=json.loads((prior/'SNAPSHOT_SHA256.json').read_text())
assert all(sha(prior/n)==h for n,h in manifest.items()),'backup changed'
delta={n for n in inputs['production'] if sha(prior/n)!=sha(r/n)}
assert delta==allowed,delta
reg=json.loads((r/'Tests/rear_retrace_before.json').read_text())
assert {'rear_only','freeze_fault_pose'} <= {x['case'] for x in reg['failures']}
assert reg['axf_sha256']==sha(prior/'MDK-ARM'/r.name/(r.name+'.axf'))
patch=''.join(''.join(difflib.unified_diff((prior/n).read_text(encoding='utf-8').splitlines(True),(r/n).read_text(encoding='utf-8').splitlines(True),fromfile='ROUTE4/'+n,tofile='ROUTE5/'+n)) for n in sorted(allowed))
(r/'ROUTE5_CHANGESET.patch').write_text(patch,encoding='utf-8')
out=dict(firmware='UARTCH4_16V_ROUTE5_RETRACE',axf_sha256=ah,source_hashes=inputs['production'],reports=reports,suites=suites,changed_production=sorted(allowed),unchanged_modules=unchanged,frozen_motion_functions=frozen,normal_turns_and_pid_chain_unchanged=True,trace_bytes=876,full_return_lifecycles=16,rom_bytes=rom,ram_bytes=ram,ram_remaining_bytes=48*1024-ram,reserved_stack_bytes=3072,linker_known_stack_bytes=stack,hardware_stack_watermark_measured=False,baseline_axf_sha256=sha(original),hardware_tested=False,flashed=False,serial_opened=False)
(r/'VALIDATION_ROUTE5.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=out['firmware'],suites=len(suites),rom_bytes=rom,ram_bytes=ram,free_ram=48*1024-ram,stack=stack,axf_sha256=ah),indent=2))
