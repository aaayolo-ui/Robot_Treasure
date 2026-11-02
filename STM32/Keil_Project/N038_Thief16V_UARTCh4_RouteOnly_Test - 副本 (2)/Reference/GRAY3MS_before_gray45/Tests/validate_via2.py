"""Bind first-hide/north-route evidence to E-drive project and current AXF."""
from pathlib import Path
import json,hashlib,re,difflib,xml.etree.ElementTree as ET
r=Path(__file__).resolve().parents[1];b=r/'Reference/FIRST11_before_via2'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:p.read_text(encoding='utf-8')
load=lambda p:json.loads(read(p))
assert str(r).lower().startswith('e:')
image=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(image)
z=load(r/'Tests/via2_inputs.json');assert ah==z['axf_sha256']
assert all(sha(r/n)==h for n,h in z['production'].items())
assert all(sha(r/n)==h for n,h in z['test_sources'].items())
snapshot=load(b/'SNAPSHOT_SHA256.json');assert len(snapshot)==291 and all(sha(b/n)==h for n,h in snapshot.items())
delta={n for n in z['production'] if sha(b/n)!=sha(r/n)}
assert delta=={'User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/ThiefMapPlanner.c'}
app=read(r/'User/App_ThiefStrategy.c');old=read(b/'User/App_ThiefStrategy.c')
assert app.count('UARTCH4_16V_FIRST11_VIA2_SL1')==3
normalized=app.replace('UARTCH4_16V_FIRST11_VIA2_SL1','UARTCH4_16V_FIRST11_NORTH31_SL1')
normalized=normalized.replace('  park_target_heading=returning_home?route_trail.home_body:\n      (node==11U && map_nav.route->id==1600U && thief_plan.goal_node==11U?3U:0U);','  park_target_heading=returning_home?route_trail.home_body:0U;')
normalized=normalized.replace('      (node==11U && map_nav.route->id==1600U && thief_plan.goal_node==11U && body%2U==0U) ||\n','')
assert normalized==old,'unrelated application change'
planner='Algorithm/ThiefStrategy/ThiefMapPlanner.c';new=read(r/planner);old=read(b/planner)
def remove_initial(s):
    i=s.index('uint8_t ThiefMapPlanner_PlanInitial(');j=s.index('uint8_t ThiefMapPlanner_PlanEscape(',i)
    return s[:i]+s[j:]
assert remove_initial(new)==remove_initial(old)
assert b'UARTCH4_16V_FIRST11_VIA2_SL1' in image.read_bytes()
suites=load(r/'Tests/via2_suites.json');assert len(suites)==15 and all(s['exit_code']==0 for s in suites)
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json','results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json','results_left800.json','results_live_monitor.json','west63_after.json','trace_front_after.json','results_trace_return.json','results_trace_recovery.json','results_n068_recovery.json','results_rear_retrace.json','results_short_return.json']:
    p=r/'Tests'/n;q=load(p);assert not q.get('failures') and not q.get('failure'),n
    assert q.get('axf_sha256',q.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=q.get('checks'),sha256=sha(p))
hide=load(r/'Tests/results_hide_routes_stable4.json')
initial=next(x for x in hide['lifecycles'] if x['kind']=='initial')
assert initial['result']['node']==11 and initial['result']['body']==3
north=[x for x in hide['lifecycles'] if x.get('world_heading')==0]
assert len(north)==4 and all(x['result']['node']==63 and x['result']['body']==0 for x in north)
proj=r/'MDK-ARM'/(r.name+'.uvprojx');paths=ET.parse(proj).findall('.//FilePath')
assert len(paths)==len({n.text.lower() for n in paths}) and all((proj.parent/n.text.replace('\\','/')).exists() for n in paths)
assert '0 Error(s), 0 Warning(s)' in read(r/'build_first11.log')
mp=read(image.with_suffix('.map'));htm=read(image.with_suffix('.htm'))
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',mp)[1]);ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',mp)[1]);stack=int(re.search(r'Maximum Stack Usage =\s*(\d+)',htm)[1])
assert (rom,ram,stack)==(68232,44888,920)
changed=[n for n,h in snapshot.items() if n.endswith(('.c','.h','.s','.md','.py','.ioc','.uvprojx','.uvoptx')) and sha(r/n)!=h]
added=['Tests/run_via2.py','Tests/validate_via2.py','Docs/VIA2_\u9996\u6b21\u7ecfN002\u8fdb\u5165N011.md']
patch=''.join(''.join(difflib.unified_diff(read(b/n).splitlines(True) if (b/n).exists() else [],read(r/n).splitlines(True),fromfile='SL1/'+n,tofile='VIA2/'+n)) for n in sorted(changed+added))
(r/'VIA2_CHANGESET.patch').write_text(patch,encoding='utf-8')
artifacts=changed+added+['VIA2_CHANGESET.patch','build_first11.log','Tests/via2_inputs.json','Tests/via2_suites.json']+['Tests/'+n for n in reports]+[p.relative_to(r).as_posix() for p in (r/'Tests').glob('via2_*.log')]+[image.with_suffix(s).relative_to(r).as_posix() for s in ('.axf','.hex','.map','.htm')]
(r/'VIA2_FILES.json').write_text(json.dumps(dict(modified=changed,added=added,hashes={n:sha(r/n) for n in sorted(set(artifacts))},generated_manifests=['VIA2_FILES.json','VALIDATION_FIRST11.json']),ensure_ascii=False,indent=2),encoding='utf-8')
out=dict(firmware='UARTCH4_16V_FIRST11_VIA2_SL1',project=str(r),axf_sha256=ah,hex_sha256=sha(image.with_suffix('.hex')),production_changed=sorted(delta),app_control_unchanged_except_first_hide_parking=True,other_planner_functions_unchanged=True,source_hashes=z['production'],test_source_hashes=z['test_sources'],validator_sha256=sha(Path(__file__)),suites=suites,reports=reports,baseline_files_checked=291,rom_bytes=rom,ram_bytes=ram,ram_added_bytes=0,ram_remaining_bytes=48*1024-ram,reserved_stack_bytes=3072,linker_known_stack_bytes=stack,hardware_stack_watermark_measured=False,desktop_project_written=False,hardware_tested=False,flashed=False,serial_opened=False,motors_run=False)
(r/'VALIDATION_FIRST11.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=out['firmware'],suites=len(suites),initial_hide=11,north_goal=63,ram_bytes=ram,axf_sha256=ah,hardware_tested=False),indent=2))
