"""Read-only structural checks for the continuous Keil experiment (no hardware)."""
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
PROJECT=ROOT/'STM32/Keil_Project/Gray12_MapFullRoute_Calibration_Test'


def verify():
    manifest=json.loads((PROJECT/'BASELINE_SOURCE_SHA256.json').read_text(encoding='utf-8'))
    source=ROOT/'STM32/Keil_Project/Gray12_DesktopBase_ReverseReturn_Test'
    for name,digest in manifest['files'].items():
        assert hashlib.sha256((source/name).read_bytes()).hexdigest()==digest, f'Source changed since copy: {name}'
        if name.split('/')[0] in ('Core','Drivers','Control','Hardware'):
            assert (PROJECT/name).read_bytes()==(source/name).read_bytes(),f'Baseline module changed: {name}'
    project_file=PROJECT/'MDK-ARM'/f'{PROJECT.name}.uvprojx'
    tree=ET.parse(project_file)
    paths=[(project_file.parent/entry.text.replace('\\','/')).resolve() for entry in tree.findall('.//FilePath')]
    normalized=[str(p).lower() for p in paths]
    assert len(normalized)==len(set(normalized)), 'Duplicate Keil source entry'
    for p in paths:
        assert p.is_relative_to(PROJECT),f'External project dependency: {p}'
        assert p.exists(),f'Missing source: {p}'
    for entry in tree.findall('.//IncludePath'):
        for path in (entry.text or '').split(';'):
            if path:
                p=(project_file.parent/path.replace('\\','/')).resolve()
                assert p.is_relative_to(PROJECT) and p.is_dir(),f'Invalid include: {p}'
    source_tree=ET.parse(source/'MDK-ARM'/f'{source.name}.uvprojx')
    for tag in ('Device','Vendor','Cpu','uAC6','FlashDriverDll','DriverSelection'):
        assert [e.text for e in tree.findall('.//'+tag)]==[e.text for e in source_tree.findall('.//'+tag)],f'Changed toolchain/device/flash: {tag}'
    assert (PROJECT/f'{PROJECT.name}.ioc').read_text(encoding='utf-8-sig')==(source/f'{source.name}.ioc').read_text(encoding='utf-8-sig').replace(source.name,PROJECT.name),'Unexpected IOC hardware change'
    assert (PROJECT/'MDK-ARM/startup_stm32f103xe.s').read_bytes()==(source/'MDK-ARM/startup_stm32f103xe.s').read_bytes()
    print(f'PASS: {len(manifest["files"])} baseline files unchanged; {len(paths)} unique local Keil file entries; Core/Drivers/Control/Hardware identical; IOC/device/startup preserved.')


if __name__=='__main__':verify()
