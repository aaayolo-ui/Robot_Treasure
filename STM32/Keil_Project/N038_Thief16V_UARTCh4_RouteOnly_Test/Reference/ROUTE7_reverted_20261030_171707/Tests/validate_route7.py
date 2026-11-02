"""Bind unchanged SL1 motion baseline, current ARM build and offline reports."""
from pathlib import Path
import json,hashlib,re,xml.etree.ElementTree as ET,difflib
r=Path(__file__).absolute().parents[1];b=r/'Reference/ROUTE6_SL1_before_pursuit_pass'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(a)
inputs=json.loads((r/'Tests/route7_inputs.json').read_text())
assert inputs['axf_sha256']==ah
assert all(sha(r/p)==h for p,h in inputs['production'].items())
assert all(sha(r/p)==h for p,h in inputs['test_sources'].items())
suites=json.loads((r/'Tests/route7_suites.json').read_text())
assert len(suites)==17 and all(x['exit_code']==0 for x in suites)
manifest=json.loads((b/'SNAPSHOT_SHA256.json').read_text())
assert len(manifest)==454 and all(sha(b/n)==h for n,h in manifest.items()),'SL1 snapshot changed'
allowed={'User/App_ThiefStrategy.c','MDK-ARM/'+r.name+'.uvprojx'}
added={'Algorithm/ThiefStrategy/ThiefPursuit.c','Algorithm/ThiefStrategy/ThiefPursuit.h'}
delta={n for n in inputs['production'] if n in manifest and sha(b/n)!=sha(r/n)}
assert delta==allowed,delta
assert {n for n in inputs['production'] if n not in manifest}==added
unchanged={folder:sum(1 for n in inputs['production'] if n.startswith(folder+'/'))
           for folder in ['Control','Hardware','Core','System','Drivers']}
source=(r/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
old=(b/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
def fn(text,name):
    m=re.search(r'static [^;\n]+\b'+name+r'\([^;]*?\)\s*\{',text);assert m,name
    j=text.index('{',m.start());k=j+1;depth=1
    while depth:depth+=(text[k]=='{')-(text[k]=='}');k+=1
    return text[m.start():k]
frozen=['Route_ReadFront','Route_ReadRear','Route_UpdateErrors','Route_UpdateFollowMode',
        'Route_ClassifyPosition','Route_DriveStraight','Route_DriveUnlocatedFollow',
        'Route_DriveTurnApproach','Route_DriveLeftTurn','Route_DriveRightTurn',
        'Route_HasReachedTurnAngle','Route_EnterTurnApproach','Route_EnterTurnStop']
assert all(fn(source,n)==fn(old,n) for n in frozen)
def pid(text):return fn(text,'Route_RunFollow').split('  Route_UpdateFollowMode(sample);')[1].split('  Route_UpdateJunctionArm(front_state);')[0]
def turns(text):return text[text.index('    case ROUTE_STATE_TURN_APPROACH:'):text.index('    case ROUTE_STATE_AFTER_TURN_FORWARD:')]
assert pid(source)==pid(old) and turns(source)==turns(old)
assert b'UARTCH4_16V_ROUTE7_PASS_SL1' in a.read_bytes()
proj=r/'MDK-ARM'/(r.name+'.uvprojx');t=ET.parse(proj);ot=ET.parse(b/'MDK-ARM'/proj.name)
paths=[x.text.lower() for x in t.findall('.//FilePath')]
assert len(paths)==len(set(paths))
assert all((proj.parent/x.text.replace('\\','/')).exists() for x in t.findall('.//FilePath'))
for f in list(t.findall('.//File')):
    if f.findtext('FileName') in ['ThiefPursuit.c','ThiefPursuit.h']:
        next(n for n in t.iter() if f in list(n)).remove(f)
def tree(n):return n.tag,sorted(n.attrib.items()),(n.text or '').strip(),tuple(tree(c) for c in n)
assert tree(t.getroot())==tree(ot.getroot()),'unrelated project configuration changed'
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json',
          'results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json',
          'results_left800.json','results_live_monitor.json','west63_after.json','trace_front_after.json',
          'results_trace_return.json','results_trace_recovery.json','results_n068_recovery.json',
          'results_rear_retrace.json','results_short_return.json','results_pursuit_pass.json','results_pursuit_geometry.json']:
    p=r/'Tests'/n;z=json.loads(p.read_text());assert not z.get('failures'),n
    assert z.get('axf_sha256',z.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=z.get('checks'),sha256=sha(p))
pre=json.loads((r/'Tests/pursuit_pass_before.json').read_text())
assert any(x['case']=='pursued' for x in pre['failures'])
assert pre['axf_sha256']==sha(b/'MDK-ARM'/r.name/(r.name+'.axf'))
mp=a.with_suffix('.map').read_text(encoding='utf-8',errors='replace')
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',mp)[1])
ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',mp)[1])
htm=a.with_suffix('.htm').read_text(encoding='utf-8',errors='replace')
stack=int(re.search(r'Maximum Stack Usage =\s*(\d+)',htm)[1])
assert rom<256*1024 and ram<48*1024 and stack<3072
assert '0 Error(s), 0 Warning(s)' in (r/'build_route7.log').read_text(encoding='utf-8',errors='replace')
patch=''.join(''.join(difflib.unified_diff(((b/n).read_text(encoding='utf-8') if (b/n).exists() else '').splitlines(True),
    (r/n).read_text(encoding='utf-8').splitlines(True),fromfile='ROUTE6_SL1/'+n,tofile='ROUTE7/'+n)) for n in sorted(allowed|added))
(r/'ROUTE7_CHANGESET.patch').write_text(patch,encoding='utf-8')
out=dict(firmware='UARTCH4_16V_ROUTE7_PASS_SL1',axf_sha256=ah,hex_sha256=sha(a.with_suffix('.hex')),
         source_hashes=inputs['production'],reports=reports,suites=suites,
         changed_production=sorted(allowed),added_production=sorted(added),unchanged_modules=unchanged,
         frozen_motion_functions=frozen,normal_turns_and_pid_chain_unchanged=True,
         sl1_pa3_sound_light_preserved=True,rom_bytes=rom,ram_bytes=ram,ram_remaining_bytes=48*1024-ram,
         reserved_stack_bytes=3072,linker_known_stack_bytes=stack,hardware_stack_watermark_measured=False,
         baseline_axf_sha256=pre['axf_sha256'],hardware_tested=False,flashed=False,serial_opened=False,
         physical_motors_run=False)
(r/'VALIDATION_ROUTE7.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=out['firmware'],suites=len(suites),rom_bytes=rom,ram_bytes=ram,
                     free_ram=48*1024-ram,stack=stack,axf_sha256=ah),indent=2))
