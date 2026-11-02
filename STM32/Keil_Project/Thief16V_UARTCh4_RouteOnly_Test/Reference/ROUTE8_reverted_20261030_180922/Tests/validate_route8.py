"""Bind current route-only edit, SL1 baseline and offline test evidence."""
from pathlib import Path
import hashlib,json,re,difflib
r=Path(__file__).absolute().parents[1];b=r/'Reference/ROUTE6_SL1_before_n063_near'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((b/'SNAPSHOT_SHA256.json').read_text())
assert len(manifest)==456 and all(sha(b/n)==h for n,h in manifest.items())
a=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(a)
i=json.loads((r/'Tests/route8_inputs.json').read_text())
assert i['axf_sha256']==ah
assert all(sha(r/n)==h for n,h in i['production'].items())
assert all(sha(r/n)==h for n,h in i['test_sources'].items())
allowed={'User/App_ThiefStrategy.c','Algorithm/ThiefStrategy/ThiefMapPlanner.c',
         'Algorithm/ThiefStrategy/ThiefThreatModel.c','Algorithm/ThiefStrategy/ThiefThreatModel.h'}
assert {n for n in i['production'] if n not in manifest}==set()
assert {n for n,h in i['production'].items() if h!=manifest[n]}==allowed
app=r/'User/App_ThiefStrategy.c';old=b/'User/App_ThiefStrategy.c'
assert app.read_bytes().replace(b'UARTCH4_16V_ROUTE8_N063_SL1',b'UARTCH4_16V_ROUTE6_SHORT_RETURN_SL1')==old.read_bytes()
n='Algorithm/ThiefStrategy/ThiefMapPlanner.c';source=(r/n).read_text(encoding='utf-8')
start=source.index('  if(current_node==63U && (threat_mask&1U)) {')
j=source.index('{',start);k=j+1;depth=1
while depth:depth+=(source[k]=='{')-(source[k]=='}');k+=1
assert source[:start]+source[k+1:]==(b/n).read_text(encoding='utf-8'),'old planner code changed outside new near branch'
n='Algorithm/ThiefStrategy/ThiefThreatModel.c';source=(r/n).read_text(encoding='utf-8')
added=re.search(r'uint8_t ThiefThreatModel_HeadingHit\([\s\S]+?\n\}\n\n',source);assert added
assert source[:added.start()]+source[added.end():]==(b/n).read_text(encoding='utf-8')
n='Algorithm/ThiefStrategy/ThiefThreatModel.h';source=(r/n).read_text(encoding='utf-8')
assert source.replace('/* Fresh map-gated ray hit; does not require an exact road centerline. */\nuint8_t ThiefThreatModel_HeadingHit(uint8_t heading,uint32_t now,int16_t *x,int16_t *y);\n','')==(b/n).read_text(encoding='utf-8')
assert not (r/'Algorithm/ThiefStrategy/ThiefPursuit.c').exists(),'reverted ROUTE7 must remain absent'
assert b'UARTCH4_16V_ROUTE8_N063_SL1' in a.read_bytes()
suites=json.loads((r/'Tests/route8_suites.json').read_text())
assert len(suites)==16 and all(x['exit_code']==0 for x in suites)
names=['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json',
       'results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json',
       'results_left800.json','results_live_monitor.json','west63_after.json','trace_front_after.json',
       'results_trace_return.json','results_trace_recovery.json','results_n068_recovery.json',
       'results_rear_retrace.json','results_short_return.json','results_n063_near.json']
reports={}
for n in names:
    p=r/'Tests'/n;z=json.loads(p.read_text());assert not z.get('failures'),n
    assert z.get('axf_sha256',z.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=z.get('checks'),sha256=sha(p))
pre=json.loads((r/'Tests/n063_near_before.json').read_text());now=json.loads((r/'Tests/results_n063_near.json').read_text())
assert pre['axf_sha256']==sha(b/'MDK-ARM'/r.name/(r.name+'.axf')) and len(pre['failures'])==16
assert pre['test_sha256']==now['test_sha256']==sha(r/'Tests/test_n063_near_route.py')
previous={(x['case'],x['body']):x['path'] for x in pre['cases']}
for c in now['cases']:
    if c['case'] not in ['N052','toward_N063','near_range_error','other_ray_report']:assert c['path']==previous[(c['case'],c['body'])]
assert len(now['lifecycles'])==6
mp=a.with_suffix('.map').read_text(encoding='utf-8',errors='replace')
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',mp)[1]);ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',mp)[1])
htm=a.with_suffix('.htm').read_text(encoding='utf-8',errors='replace');stack=int(re.search(r'Maximum Stack Usage =\s*(\d+)',htm)[1])
assert rom<256*1024 and ram==44888 and stack<3072
assert '0 Error(s), 0 Warning(s)' in (r/'build_route8.log').read_text(encoding='utf-8',errors='replace')
patch=''.join(''.join(difflib.unified_diff((b/n).read_text(encoding='utf-8').splitlines(True),(r/n).read_text(encoding='utf-8').splitlines(True),fromfile='ROUTE6_SL1/'+n,tofile='ROUTE8/'+n)) for n in sorted(allowed))
(r/'ROUTE8_CHANGESET.patch').write_text(patch,encoding='utf-8')
out=dict(firmware='UARTCH4_16V_ROUTE8_N063_SL1',source_hashes=i['production'],axf_sha256=ah,
         hex_sha256=sha(a.with_suffix('.hex')),baseline_axf_sha256=pre['axf_sha256'],reports=reports,
         changed_production=sorted(allowed),app_only_identification_changed=True,old_planner_branch_unchanged=True,
         all_other_production_unchanged=True,far_and_unlocated_routes_identical_to_baseline=True,
         suites=suites,rom_bytes=rom,ram_bytes=ram,ram_remaining_bytes=48*1024-ram,
         reserved_stack_bytes=3072,linker_known_stack_bytes=stack,hardware_stack_watermark_measured=False,
         hardware_tested=False,flashed=False,serial_opened=False,physical_motors_run=False)
(r/'VALIDATION_ROUTE8.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(firmware=out['firmware'],suites=len(suites),rom_bytes=rom,ram_bytes=ram,stack=stack,axf_sha256=ah),indent=2))
