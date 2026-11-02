from pathlib import Path
import json,hashlib,re,xml.etree.ElementTree as ET
r=Path(__file__).resolve().parents[1];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(a)
base=json.loads((r/'BASELINE_SHA256.json').read_text());inputs=json.loads((r/'Tests/route1_inputs.json').read_text())
assert all(sha(r/p)==h for p,h in inputs.items())
for folder in ['Control','Hardware','Core','System','Drivers']:
    assert all(sha(r/p)==h for p,h in base.items() if p.startswith(folder+'/'))
assert sha(r/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h')==base['Algorithm/ThiefStrategy/ThiefStrategyConfig.h']
s=(r/'User/App_ThiefStrategy.c').read_text(encoding='utf-8');old=(r/'Reference/Desktop_STABLE4_RouteOnly/User/App_ThiefStrategy.c').read_text(encoding='utf-8')
assert s[s.index('  switch (route_state)'):]==old[old.index('  switch (route_state)'):]
def functions(text):
    result={}
    for m in re.finditer(r'^(?:static )?(?:void|uint8_t|int32_t|uint16_t|RouteGrayState_t)\s+(\w+)\([^;]*?\)\s*\{',text,re.M):
        k=text.index('{',m.start())+1;depth=1
        while depth:depth+=(text[k]=='{')-(text[k]=='}');k+=1
        result[m[1]]=text[m.start():k]
    return result
f=functions(s);g=functions(old);changed=[n for n in f if f[n]!=g.get(n)]
assert set(changed)<=set(['Thief_OnLaserFrame','Thief_ThreatMask','Route_StopFault','Route_FinishNodeStop','App_ThiefStrategy_Init','StartupStatus_Task','App_ThiefStrategy_Task']),changed
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json','results_hide_routes_stable4.json','results_escape_completion.json','results_turn_resume.json','results_left800.json','results_live_monitor.json']:
    p=r/'Tests'/n;report=json.loads(p.read_text());assert not report.get('failures'),n
    assert report.get('axf_sha256',report.get('firmware_sha256'))==ah,(n,report.keys())
    reports[n]=dict(checks=report.get('checks'),sha256=sha(p))
proj=r/'MDK-ARM'/(r.name+'.uvprojx');t=ET.parse(proj);ps=[x.text.lower() for x in t.findall('.//FilePath')];assert len(ps)==len(set(ps))
assert all((proj.parent/x.text.replace('\\','/')).exists() for x in t.findall('.//FilePath'))
log=(r/'build_route1.log').read_text(encoding='utf-8',errors='replace');assert '0 Error(s), 0 Warning(s)' in log
out=dict(firmware='UARTCH4_16V_ROUTE1_STABLE4',axf_sha256=ah,source_hashes=inputs,preserved=['all Control','all Hardware','Core/System/Drivers','complete motion switch','laser configuration'],changed_app_functions=changed,unchanged_app_functions=len(f)-len(changed),reports=reports,flash_bytes=65216,ram_bytes=44016,ram_remaining_bytes=5136,hardware_tested=False,flashed=False,serial_opened=False)
(r/'VALIDATION_ROUTE1.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('Verified current route-only image, eight suites and preserved STABLE4 motion/laser configuration.')
