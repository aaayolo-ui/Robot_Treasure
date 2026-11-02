from pathlib import Path
import hashlib,json,re
r=Path(__file__).resolve().parents[1];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a=r/'MDK-ARM'/r.name/(r.name+'.axf');ah=sha(a)
base=json.loads((r/'BASELINE_SHA256.json').read_text());inputs=json.loads((r/'Tests/route1_inputs.json').read_text())
assert all(sha(r/p)==h for p,h in inputs.items()),'production input changed'
for folder in ['Control','Hardware','Core','System','Drivers']:
    assert all(sha(r/p)==h for p,h in base.items() if p.startswith(folder+'/'))
assert sha(r/'Algorithm/ThiefStrategy/ThiefStrategyConfig.h')==base['Algorithm/ThiefStrategy/ThiefStrategyConfig.h']
source=(r/'User/App_ThiefStrategy.c').read_text(encoding='utf-8');old=(r/'Reference/RUN8/User/App_ThiefStrategy.c').read_text(encoding='utf-8')
assert source[source.index('  switch (route_state)'):]==old[old.index('  switch (route_state)'):]
reports={}
for n in ['results_strategy_stable2.json','results_temp78_stable3.json','results_n011_first_plan_stable10.json','results_documents13.json','results_documents14.json','results_documents15.json','results_documents16.json','results_speed16t.json']:
    report=json.loads((r/'Tests'/n).read_text());assert not report.get('failures'),n
    assert report.get('axf_sha256',report.get('firmware_sha256'))==ah,n
    reports[n]=dict(checks=report.get('checks'),sha256=sha(r/'Tests'/n))
build=(r/'build_route1.log').read_text(encoding='utf-8',errors='replace');assert '0 Error(s), 0 Warning(s)' in build
out=dict(firmware='UARTCH4_16U_ROUTE1_RUN8',axf_sha256=ah,source_hashes=inputs,baseline_source='Desktop Thief16U RUN8',preserved=['Control','Hardware','Core','System','Drivers','laser configuration','complete motion switch'],changed_production=['Algorithm/ThiefStrategy/ThiefMapPlanner.c','Algorithm/ThiefStrategy/ThiefRiskSearch.c','Algorithm/ThiefStrategy/ThiefRiskSearch.h','Algorithm/ThiefStrategy/ThiefInterceptPolicy.c','User/App_ThiefStrategy.c'],reports=reports,flash_bytes=67536,ram_bytes=41952,ram_remaining_bytes=7200,linker_known_stack_bytes=1520,hardware_stack_watermark_measured=False,hardware_tested=False,flashed=False,serial_opened=False)
(r/'VALIDATION_ROUTE1.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('Validated ROUTE1 current image, 8 current suites, unchanged RUN8 motion and laser files.')
