"""Current image/source/report binding; synthetic tests are not hardware proof."""
from pathlib import Path
import json,hashlib,re,difflib,xml.etree.ElementTree as ET
r=Path(__file__).resolve().parents[1];b=r/'Reference/TOP7_before_coin_speed'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:p.read_text(encoding='utf-8')
load=lambda p:json.loads(read(p))
image=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(image)
z=load(r/'Tests/speed_inputs.json');assert z['axf_sha256']==ah
assert all(sha(r/n)==h for n,h in z['production'].items())
assert all(sha(r/n)==h for n,h in z['test_sources'].items())
snapshot=load(b/'SNAPSHOT_SHA256.json');assert len(snapshot)==298 and all(sha(b/n)==h for n,h in snapshot.items())
delta={n for n in z['production'] if sha(b/n)!=sha(r/n)}
assert delta=={'User/App_ThiefStrategy.c'}
app=read(r/'User/App_ThiefStrategy.c');old=read(b/'User/App_ThiefStrategy.c')
assert app.count('UARTCH4_16V_TOP7_SPEED_SL1')==3 and b'UARTCH4_16V_TOP7_SPEED_SL1' in image.read_bytes()
normalized=app.replace('UARTCH4_16V_TOP7_SPEED_SL1','UARTCH4_16V_FIRST11_TOP7_SL1')
normalized=normalized.replace('  int32_t progress,turn,base;','  int32_t progress,turn;').replace('  base=follow_smoothing.base_rpm_x10;\n','').replace('TravelFrame_SetWheelTargets(base+turn,base-turn,base+turn,base-turn);','TravelFrame_SetWheelTargets(500L+turn,500L-turn,500L+turn,500L-turn);')
assert normalized==old,'application changed beyond bridge speed and identity'
def fn(s,name):
    m=re.search(r'^static [^;\n]+\b'+name+r'\([^;]*?\)\s*\{',s,re.M);assert m,name
    j=s.index('{',m.start());k=j+1;depth=1
    while depth:depth+=(s[k]=='{')-(s[k]=='}');k+=1
    return s[m.start():k]
frozen=['Route_ReadFront','Route_ReadRear','Route_UpdateErrors','Route_UpdateFollowMode','Route_RunFollow','Route_FinishNodeStop','Route_DriveLeftTurn','Route_DriveRightTurn','Route_EnterTurnApproach','Route_EnterTurnStop','Route_HasReachedTurnAngle']
assert all(fn(app,n)==fn(old,n) for n in frozen)
planner='Algorithm/ThiefStrategy/ThiefMapPlanner.c'
assert read(r/planner).split('uint8_t ThiefMapPlanner_PlanInitial(',1)[1]==read(b/planner).split('uint8_t ThiefMapPlanner_PlanInitial(',1)[1]
suites=load(r/'Tests/speed_suites.json');assert len(suites)==16 and all(x['exit_code']==0 for x in suites)
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json','results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json','results_left800.json','results_live_monitor.json','west63_after.json','trace_front_after.json','results_trace_return.json','results_trace_recovery.json','results_n068_recovery.json','results_rear_retrace.json','results_short_return.json','results_top7_coin.json']:
    p=r/'Tests'/n;q=load(p);assert not q.get('failures') and not q.get('failure'),n
    assert q.get('axf_sha256',q.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=q.get('checks'),sha256=sha(p))
initial=next(x for x in load(r/'Tests/results_hide_routes_stable4.json')['lifecycles'] if x['kind']=='initial')
assert initial['result']['node']==11 and initial['result']['body']==3
nodes=load(r/'Tests/results_top7_coin.json')['cases'][0]['nodes']
assert nodes==[53,44,36,35,26,17,7,6,4,3,2,11]
coins=[c for c in load(r/'Tests/results_top7_coin.json')['cases'] if c['case']=='coin']
assert {c['entry_rpm_x10'] for c in coins}=={1200,2200,2600}
assert all(l+rr==2*c['entry_rpm_x10'] for c in coins for _,l,rr in c['commands'])
proj=r/'MDK-ARM'/(r.name+'.uvprojx');paths=ET.parse(proj).findall('.//FilePath')
assert len(paths)==len({n.text.lower() for n in paths}) and all((proj.parent/n.text.replace('\\','/')).exists() for n in paths)
assert '0 Error(s), 0 Warning(s)' in read(r/'build_speed.log')
mp=read(image.with_suffix('.map'));htm=read(image.with_suffix('.htm'))
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',mp)[1]);ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',mp)[1]);stack=int(re.search(r'Maximum Stack Usage =\s*(\d+)',htm)[1])
assert (rom,ram,stack)==(68548,44896,920)
changed=[n for n,h in snapshot.items() if n.endswith(('.c','.h','.s','.md','.py','.ioc','.uvprojx','.uvoptx')) and sha(r/n)!=h]
added=['Tests/run_speed.py','Tests/validate_speed.py','Docs/SPEED_\u786c\u5e01\u77ed\u906e\u6321\u539f\u901f\u901a\u8fc7.md']
patch=''.join(''.join(difflib.unified_diff(read(b/n).splitlines(True) if (b/n).exists() else [],read(r/n).splitlines(True),fromfile='TOP7/'+n,tofile='SPEED/'+n)) for n in sorted(changed+added))
(r/'SPEED_CHANGESET.patch').write_text(patch,encoding='utf-8')
artifacts=changed+added+['SPEED_CHANGESET.patch','build_speed.log','Tests/speed_inputs.json','Tests/speed_suites.json']+['Tests/'+n for n in reports]+[p.relative_to(r).as_posix() for p in (r/'Tests').glob('speed_*.log')]+[image.with_suffix(s).relative_to(r).as_posix() for s in ('.axf','.hex','.map','.htm')]
(r/'SPEED_FILES.json').write_text(json.dumps(dict(modified=changed,added=added,hashes={n:sha(r/n) for n in sorted(set(artifacts))},generated_manifests=['SPEED_FILES.json','VALIDATION_SPEED.json']),ensure_ascii=False,indent=2),encoding='utf-8')
out=dict(firmware='UARTCH4_16V_TOP7_SPEED_SL1',project=str(r),axf_sha256=ah,hex_sha256=sha(image.with_suffix('.hex')),source_hashes=z['production'],test_source_hashes=z['test_sources'],validator_sha256=sha(Path(__file__)),production_changed=sorted(delta),frozen_functions=frozen,suites=suites,reports=reports,baseline_files_checked=298,rom_bytes=rom,ram_bytes=ram,ram_added_bytes=0,ram_remaining_bytes=48*1024-ram,reserved_stack_bytes=3072,linker_known_stack_bytes=stack,hardware_stack_watermark_measured=False,desktop_written=False,hardware_tested=False,flashed=False,serial_opened=False,motors_run=False)
(r/'VALIDATION_SPEED.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=out['firmware'],suites=len(suites),initial_hide=11,ram_bytes=ram,axf_sha256=ah,hardware_tested=False),indent=2))
