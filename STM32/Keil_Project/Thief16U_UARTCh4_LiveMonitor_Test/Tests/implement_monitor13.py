"""One-time, explicit MONITOR12 -> MONITOR13 source integration (not a test)."""
from pathlib import Path
P=Path(__file__).resolve().parents[1]
# Mandatory repository reread before this mutation.
for f in ['Docs/修改规范与回归清单.md','README.md','AGENTS.md']:
    (P/f).read_text(encoding='utf-8')
app=P/'User/App_ThiefStrategy.c'
s=app.read_text(encoding='utf-8')
reference=(P.parent/'Thief16U_PoliceTurnCompare_Test/User/App_ThiefStrategy.c').read_text(encoding='utf-8')
def function(text,name):
    import re
    m=re.search(r'static [^;\n]+\b'+name+r'\([^;]*?\)\s*\{',text)
    assert m,name
    start=m.start();j=text.index('{',m.start());depth=1;k=j+1
    while depth:
        depth+=(text[k]=='{')-(text[k]=='}');k+=1
    return text[start:k]
for name in ['Route_HasReachedTurnAngle','Route_EnterAfterTurnForward']:
    s=s.replace(function(s,name),function(reference,name))
s=s.replace(function(s,'Route_AlignTurnAxis'),'')
s=s.replace('static uint8_t turn_axis_active;\nstatic RoadHeadingControl_t turn_axis_heading;\n','')
s=s.replace('  turn_axis_active=0U;yaw_reference_since=0U;yaw_reference_last=0U;',
            '  local_phase=0U;local_branch_raw=0U;yaw_reference_since=0U;yaw_reference_last=0U;')
s=s.replace('      if(turn_axis_active) {\n        Route_AlignTurnAxis(now,elapsed_ms);break;\n      }\n','')
s=s.replace('7500L /* Unknown gyro sign:75deg coarse turn, then measured brake/gray. */',
            '4500L /* Police flow: coarse rotation, then real line acquisition. */')
s=s.replace('UARTCH4_16U_MONITOR12','UARTCH4_16U_MONITOR13')
# Stop/restart must discard local hypotheses; ordinary new leg drops old branch.
s=s.replace('route_active = 0U;map_gray_active=0U;',
            'local_phase=0U;local_branch_raw=0U;route_active = 0U;map_gray_active=0U;')
s=s.replace('  map_leg_started_tick=now;\n  node_centered=',
            '  map_leg_started_tick=now;local_branch_raw=0U;\n  node_centered=')
# Disable the historic blind return-to-origin translation. Keep measured scans.
start=s.index('  if(turn_exit_line_phase==2U && !turn_exit_return_used &&')
end=s.index('  line.leading_valid=',start)
s=s[:start]+s[end:]
with app.open('w',encoding='utf-8',newline='\n') as f:f.write(s)
c=P/'Control/RestartLineSearch.c';s=c.read_text(encoding='utf-8')
start=s.index('  if(in->yaw_known && !in->leading_valid && !in->trailing_valid &&')
end=s.index('  RestartLineSearch_Update(s,in,out);',start)
s=s[:start]+'  (void)allow_translate; /* LINE_FIRST: no blind translation on a map axis. */\n'+s[end:]
with c.open('w',encoding='utf-8',newline='\n') as f:f.write(s)
