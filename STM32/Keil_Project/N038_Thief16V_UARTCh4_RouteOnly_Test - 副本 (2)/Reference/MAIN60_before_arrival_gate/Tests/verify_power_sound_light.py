"""Check linked ARM GPIO initialization and unchanged ROUTE6 motion sources.

MMIO is simulated memory: validates configuration writes, not physical output.
No flashing, serial port, or physical motor execution.
"""
from pathlib import Path
import json, hashlib, re, xml.etree.ElementTree as ET

ROOT=Path(__file__).absolute().parents[1]
setup=(ROOT/'Tests/test_app_runtime.py').read_text(encoding='utf-8').split('def documents8_cases():')[0]
setup=setup.replace('Path(__file__).resolve()', 'Path(__file__).absolute()')
exec(compile(setup,str(ROOT/'Tests/test_app_runtime.py'),'exec'))

gpioa=0x40010800
rcc=0x40021018
put(gpioa,4,0x44444444);put(rcc,4,0x100)
call('PowerSoundLight_Init')
check(get(rcc)==0x104,'enable GPIOA clock while preserving other clock enables')
check(get(gpioa)==0x44442444,'PA3 only: push-pull output, low speed; preserve adjacent pin configuration')
check(get(gpioa+0x10)==(8<<16),'real HAL writes PA3 reset command to BSRR (active low)')

baseline=ROOT/'Reference/ROUTE6_before_power_sound_light'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((baseline/'SNAPSHOT_SHA256.json').read_text())
check(all(sha(baseline/n)==h for n,h in manifest.items()),'ROUTE6 backup is intact')
main=(ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
old=(baseline/'Core/Src/main.c').read_text(encoding='utf-8')
check(main.replace('#include "PowerSoundLight.h"\n','').replace('  PowerSoundLight_Init();\n','')==old,
      'main changes only the include and one startup call')
block=main.split('/* USER CODE BEGIN 2 */')[1].split('/* USER CODE END 2 */')[0]
check(block.index('PowerSoundLight_Init();')<block.index('App_ThiefStrategy_Init();'),
      'sound/light starts before strategy initialization, without waiting for KEY2')
app=(ROOT/'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
check(app.replace('UARTCH4_16V_ROUTE6_SHORT_RETURN_SL1','UARTCH4_16V_ROUTE6_SHORT_RETURN')==
      (baseline/'User/App_ThiefStrategy.c').read_text(encoding='utf-8'),'strategy changes only firmware identification')
check('MX_USART2_UART_Init();' not in main,'PA3 is not reconfigured for USART2 at startup')
allowed={'Core/Src/main.c','User/App_ThiefStrategy.c','MDK-ARM/'+ROOT.name+'.uvprojx'}
changed=[]
for n,h in manifest.items():
    if Path(n).suffix.lower() in ['.c','.h','.s','.uvprojx']:
        if sha(ROOT/n)!=h:changed.append(n)
check(set(changed)==allowed,'existing production changes are limited to startup, identification and project reference')
proj=ROOT/'MDK-ARM'/(ROOT.name+'.uvprojx');t=ET.parse(proj)
paths=[x.text.lower() for x in t.findall('.//FilePath')]
check(len(paths)==len(set(paths)),'no duplicate Keil source references')
check(all((proj.parent/x.text.replace('\\','/')).exists() for x in t.findall('.//FilePath')),'all Keil file references exist')
check(paths.count('../hardware/powersoundlight.c')==1,'one sound/light implementation in Keil')
oldtree=ET.parse(baseline/'MDK-ARM'/proj.name)
for file in list(t.findall('.//File')):
    if file.findtext('FileName') in ['PowerSoundLight.c','PowerSoundLight.h']:
        next(n for n in t.iter() if file in list(n)).remove(file)
def tree(n):return n.tag,sorted(n.attrib.items()),(n.text or '').strip(),tuple(tree(c) for c in n)
check(tree(t.getroot())==tree(oldtree.getroot()),'toolchain, pins and existing project settings unchanged')
check(b'UARTCH4_16V_ROUTE6_SHORT_RETURN_SL1' in axf.read_bytes(),'linked image has SL1 identification')
check('0 Error(s), 0 Warning(s)' in (ROOT/'build_power_sound_light.log').read_text(encoding='utf-8',errors='replace'),
      'ARMCC5 build is clean')
maptext=axf.with_suffix('.map').read_text(encoding='utf-8',errors='replace')
rom=int(re.search(r'Total ROM Size[^\n]+?\)\s+(\d+)',maptext)[1])
ram=int(re.search(r'Total RW\s+Size[^\n]+?\)\s+(\d+)',maptext)[1])
check(rom<256*1024 and ram<48*1024,'image fits STM32F103RC memory')
out=dict(firmware='UARTCH4_16V_ROUTE6_SHORT_RETURN_SL1',checks=checks,failures=[],
         axf_sha256=sha(axf),hex_sha256=sha(axf.with_suffix('.hex')),rom_bytes=rom,ram_bytes=ram,
         modified_production=sorted(changed),added_production=['Hardware/PowerSoundLight.c','Hardware/PowerSoundLight.h'],
         source_hashes={n:sha(ROOT/n) for n in sorted(allowed|{'Hardware/PowerSoundLight.c','Hardware/PowerSoundLight.h'})},
         test_sha256=sha(Path(__file__)),
         hardware_tested=False,flashed=False,serial_opened=False,physical_motors_run=False)
(ROOT/'VALIDATION_POWER_SOUND_LIGHT.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
