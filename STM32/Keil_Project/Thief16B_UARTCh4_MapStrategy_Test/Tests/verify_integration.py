"""Read-only source/project isolation check; no serial or target connection."""
from pathlib import Path
import hashlib,json,re,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
reference=json.loads((ROOT/'COPY_REFERENCE.json').read_text(encoding='utf-8'))
parent=ROOT.parent/'Thief16B_DIN2_PB15_Compare_Test'
checks=0
def check(ok,message):
    global checks
    checks+=1
    assert ok,message
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for rel,value in reference['parent_sha256_before'].items():
    check(sha(parent/rel)==value,'Baseline changed: '+rel)
for dirname in ['Control','Algorithm']:
    for old in (parent/dirname).rglob('*'):
        if old.is_file():
            rel=old.relative_to(parent)
            if rel.as_posix()=='Algorithm/ThiefStrategy/ThiefStrategyConfig.h':
                # The documented 650 -> 800 mm range change is the sole
                # permitted difference; retain every other inherited setting.
                original=old.read_text(encoding='utf-8')
                check('#define THIEF_LASER_MAX_MM 650U' in original,
                      'Unexpected reference range')
                check((ROOT/rel).read_text(encoding='utf-8')==original.replace(
                    '#define THIEF_LASER_MAX_MM 650U','#define THIEF_LASER_MAX_MM 800U'),
                    'Unexpected strategy configuration change')
            else:
                check(sha(old)==sha(ROOT/rel),'Verified control/algorithm changed: '+str(old))
for rel in ['Hardware/MotorDriver.c','Hardware/Encoder.c','Hardware/Key.c','Hardware/JY61P.c',
            'Core/Inc/main.h','Core/Src/stm32f1xx_it.c','System/UartDebug.c']:
    check(sha(ROOT/rel)==sha(parent/rel),'Unrelated hardware code changed: '+rel)
def body(source,name):
    m=re.search(r'(?:static\s+)?void\s+'+name+r'\(void\)\s*\{',source);assert m
    i=m.end();depth=1
    while depth:depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[m.start():i]
main=(ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
old_main=(parent/'Core/Src/main.c').read_text(encoding='utf-8')
for name in ['SystemClock_Config','MX_GPIO_Init','MX_TIM1_Init','MX_USART1_UART_Init']:
    check(body(main,name)==body(old_main,name),'Protected initialization changed: '+name)
check('MX_USART2_UART_Init' not in main and 'MX_UART4_Init' not in main,'Unused UART initialized')
for rel in ['Hardware/UartCh4.c','Hardware/LaserRx.c','Hardware/UartRxDispatcher.c']:
    check('USART2' not in (ROOT/rel).read_text(encoding='utf-8'),'USART2 referenced by new runtime RX')
app=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
check('!LaserRx_LinkReady() || !LaserRx_SetEnabled(0U)' in app,'Startup must require verified receive or ACK configuration')
check('UARTCH4_CONFIG_FAILED' in app,'Configuration failure is not reported')
check('BOOT FW=UARTCH4_RUN4' in app and 'BOOT_STATUS FW=UARTCH4_RUN4' in app,
      'Pre-configuration and repeated stopped status missing')
check('START_REJECT STATE=' in app,'Rejected manual start must include its reason')
transport=(ROOT/'Hardware/UartCh4.c').read_text(encoding='utf-8')
check('BootService();boot_service=0;' in transport,'Boot service remains enabled after initialization')
check('Exchange(4U,e,120U)' in transport and 'main_ack?7U:2U' in transport,
      'Main-baud-first configuration or ACK guard missing')
check('if(uart->ErrorCode!=HAL_UART_ERROR_NONE) return;' in transport,
      'An erroneous RX completion must leave error handling to HAL ErrorCallback')
check('if(!route_active && (thief_strategy_state==THIEF_STRATEGY_READY ||' in app and
      'thief_strategy_state==THIEF_STRATEGY_FAULT)) LaserRx_BootLogTask();' in app,
      'Raw boot trace formatting must be parked-only')
facade=(ROOT/'Hardware/LaserRx.c').read_text(encoding='utf-8')
check('UartCh4_Init(BootService)' in facade and 'UartDebug_Task();' in facade,
      'Blocking startup configuration starves the independent debug output')
check('UartCh4_OpenPassive()' in facade and 'proof_count[i]<3U' in facade and
      '(uint32_t)(proof_last[i]-proof_first[i])<20U' in facade,
      'Passive readiness must require repeated, time-separated data on all four channels')
check('AI1811Mux_Byte(&mux,byte,tick,ProofFrame,0)' in facade and
      'COMMAND_ACK=UNVERIFIED' in facade,'Passive data proof must use the real decoder and retain command-channel uncertainty')
check('LaserRx_SetEnabled(1U)' in app and 'TreasureLaserStart_Crossed' in app,'Treasure gate missing')
ioc=(ROOT/(ROOT.name+'.ioc')).read_text(encoding='utf-8')
check('USART2' not in ioc and 'UART4' not in ioc,'CubeMX old RX resources remain')
check('PB6.Signal=USART1_TX' in ioc and 'PB7.Signal=USART1_RX' in ioc,'Wrong expansion UART pins')
for key,number in [('IP','IPNb'),('Pin','PinsNb')]:
    values=re.findall(r'^Mcu\.'+key+r'\d+=(.+)$',ioc,re.M)
    count=int(re.search(r'^Mcu\.'+number+r'=(\d+)$',ioc,re.M)[1])
    check(len(values)==count and len(values)==len(set(values)),'CubeMX inventory differs: '+key)
tree=ET.parse(ROOT/'MDK-ARM'/(ROOT.name+'.uvprojx'))
old_tree=ET.parse(parent/'MDK-ARM'/(parent.name+'.uvprojx'))
paths=[]
for el in tree.findall('.//FilePath'):
    p=(ROOT/'MDK-ARM'/el.text.replace('\\','/')).resolve()
    check(p.is_relative_to(ROOT) and p.exists(),'External/missing input: '+str(p));paths.append(str(p).lower())
check(len(paths)==len(set(paths)),'Duplicate Keil file paths')
for el in tree.findall('.//IncludePath'):
    for entry in (el.text or '').split(';'):
        if entry:check((ROOT/'MDK-ARM'/entry.replace('\\','/')).is_dir(),'Missing include: '+entry)
for query in ['.//pCCUsed','.//uAC6','.//Device','.//Cpu','.//Define','.//Utilities','.//OnChipMemories']:
    check(ET.tostring(tree.find(query))==ET.tostring(old_tree.find(query)),'Toolchain/target/Flash config changed: '+query)
result={'checks':checks,'failures':0,'baseline_files_unchanged':len(reference['parent_sha256_before']),
        'keil_inputs':len(paths),'scope':'Source and project configuration only; no hardware or serial access'}
(ROOT/'Tests/build').mkdir(exist_ok=True)
(ROOT/'Tests/build/integration_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
