"""Execute real ARM code with mocked UART only; no hardware is driven."""
from pathlib import Path
import json, hashlib, sys
setup=Path(__file__).with_name('test_app_runtime.py').read_text(encoding='utf-8').split('def documents8_cases():')[0]
if '--before' in sys.argv:
    setup=setup.replace("axf=ROOT/'MDK-ARM'/ROOT.name/(ROOT.name+'.axf')", "axf=ROOT/'Tests/before_live_monitor/original.axf'")
exec(compile(setup,str(Path(__file__).with_name('test_app_runtime.py')),'exec'))
output=[]
def capture():
    a=u.reg_read(UC_ARM_REG_R0)
    output.append(bytes(u.mem_read(a,256)).split(b'\0')[0].decode())
    return 1
hook('UartDebug_TrySendString',capture)
hook('HAL_GetTick',lambda:clock)
ret('UartCh4_Ready',1)
rx=[]
hook('UartCh4_SetReceiving',lambda:rx.append(u.reg_read(UC_ARM_REG_R0)) or 1)
if 'LaserRx_SetEnabled' in hooks:u.hook_del(hooks.pop('LaserRx_SetEnabled'))
def inject(channel,distance=365,confidence=100,corrupt=False):
    frame=[170,5,104,0,distance&255,distance>>8,confidence]
    frame.append((sum(frame)+(1 if corrupt else 0))&255)
    for byte in [ord('1')+channel,ord('#'),*frame]:call('Byte',byte,clock)
def report():
    data=dict(checks=checks,failures=0,axf=str(axf),axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),hardware_tested=False)
    path=ROOT/'Tests'/('results_live_monitor_before.json' if '--before' in sys.argv else 'results_live_monitor.json')
    path.write_text(json.dumps(data,indent=2),encoding='utf-8')
    print(json.dumps(data,indent=2))
if '--before' in sys.argv:
    call('LaserRx_SetEnabled',0)
    inject(0);call('Thief_LaserLog',clock)
    failed=not rx or rx[-1]!=1 or not output or 'D=365 C=100' not in output[-1]
    data=dict(expected_failure=failed,reason='RUN21 disables receiver and hides raw ranges outside strategy gate',axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),observed=output[-1:] )
    (ROOT/'Tests/results_live_monitor_before.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    print(json.dumps(data,indent=2));assert failed;sys.exit(0)
call('AI1811Mux_Init',symbols['mux'])
check(call('LaserRx_SetEnabled',0)==1 and rx[-1]==1,'monitor RX stays on while strategy remains off')
check(call('LaserRx_IsEnabled')==0,'monitoring does not enable strategy')
for channel,distance in enumerate([410,365,901,1250]):inject(channel,distance)
for _ in range(4):
    clock+=25;call('Thief_LaserLog',clock)
for direction,distance in zip(['FRONT','LEFT','RIGHT','REAR'],[410,365,901,1250]):
    check(any('DIR='+direction+' D='+str(distance)+' C=100 STATUS=MONITOR_ONLY' in line for line in output),'raw range visible before treasure: '+direction)
check(var('intercept_pending')==0 and var('thief_escape_requested')==0,'raw monitoring cannot request escape')
output.clear();clock+=300
for _ in range(4):clock+=25;call('Thief_LaserLog',clock)
check(len(output)==4 and all('STATUS=STALE' in line for line in output),'stale frames are marked instead of refreshed as live')
inject(1,600);inject(1,700,corrupt=True)
snapshot=0x2000c500
check(call('LaserRx_GetSnapshot',1,snapshot)==1,'snapshot is available')
check(get(snapshot,2)==600,'bad checksum cannot replace the last real distance')
inject(0,0,0);output.clear();clock+=25;call('Thief_LaserLog',clock)
check('D=0 C=0 STATUS=BAD_DATA' in output[-1],'invalid measurement remains invalid')
var('thief_strategy_state',4);inject(1,500);output.clear()
for _ in range(4):clock+=25;call('Thief_LaserLog',clock)
check(any('DIR=LEFT D=500 C=100 STATUS=MONITOR_ONLY' in line and 'STATE=FAULT' in line for line in output),'fault preserves real range monitoring')
check(call('LaserRx_SetEnabled',1)==1 and call('LaserRx_IsEnabled')==1,'normal strategy enabling still works')
check(call('LaserRx_SetEnabled',0)==1 and call('LaserRx_IsEnabled')==0,'strategy can be disabled independently of monitoring')
polled=[]
ret('UartCh4_TakeLost',0)
hook('UartCh4_Poll',lambda:polled.append(get(symbols['application_handler'])) or 0)
call('LaserRx_Poll',symbols['Thief_OnLaserFrame'])
check(polled[-1]==0,'real RX poll cannot dispatch to disabled strategy')
call('LaserRx_SetEnabled',1);call('LaserRx_Poll',symbols['Thief_OnLaserFrame'])
check(polled[-1]==symbols['Thief_OnLaserFrame'],'enabled RX poll dispatches only the normal strategy handler')
call('LaserRx_SetEnabled',0)
check('MapTimingLog_Task' in symbols and 'MapTimingLog_Event' in symbols,'existing map telemetry remains linked')
# Emit real linked MT3 records and feed them, together with raw laser lines,
# through the desktop receiver without opening any serial device.
output.clear();var('thief_strategy_state',0)
for channel,distance in enumerate([410,365,901,1250]):inject(channel,distance)
for _ in range(4):clock+=25;call('Thief_LaserLog',clock)
if 'MapTimingLog_Task' in hooks:u.hook_del(hooks.pop('MapTimingLog_Task'))
ret('JY61P_GetData',0);ret('Encoder_GetTotal',0)
call('MapTimingLog_Init');call('MapTimingLog_SetRoute',1600,0,54,53)
call('MapTimingLog_Begin',clock);call('MapTimingLog_Task')
clock+=500;call('MapTimingLog_Event',15,1,clock,4032);call('MapTimingLog_Task')
desktop=ROOT.parents[3]/'DesktopSerialMonitor'
sys.path.insert(0,str(desktop/'assets'))
from console_server import SharedFeed
feed=SharedFeed()
for line in output:feed.record(line.encode());feed.parse_line(line.strip())
view=feed.snapshot()
check([view['channels'][str(i)]['mm'] for i in range(1,5)]==[410,365,901,1250],'all four raw ranges reach the shared desktop feed')
check(view['map_match'] is True and view['position']['source']=='MT3','real linked route records produce a map position')
check(view['position']['id']>=2 and view['position']['x_mm']==3000 and view['position']['y_mm']==1500,'real N053 arrival advances displayed path to its map coordinates')
check(feed.console(0)['rx']>0,'same input remains available to serial console')
report()
