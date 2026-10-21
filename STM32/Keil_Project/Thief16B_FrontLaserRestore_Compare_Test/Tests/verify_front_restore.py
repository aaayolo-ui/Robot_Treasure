"""Read-only integration audit for restoring front RX on the verified PB15 base.

This checks source and project configuration, not the physical UART/PWM mux.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = json.loads((ROOT / 'COPY_REFERENCE_FRONT.json').read_text(encoding='utf-8'))
PARENT = ROOT.with_name(REFERENCE['source_project'])
OLD = PARENT.name.encode('ascii')
NEW = ROOT.name.encode('ascii')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    unchanged, adjusted = [], []
    for source, expected_hash in REFERENCE['source_sha256'].items():
        original = PARENT / source
        assert digest(original) == expected_hash, 'Parent changed: ' + source
        destination = REFERENCE['copied_paths'][source]
        actual = (ROOT / destination).read_bytes()
        expected = original.read_bytes().replace(OLD, NEW)
        if source == 'User/App_ThiefStrategy.c':
            expected = expected.replace(
                b'DIAG=DIN2_PB15 FRONT_UART2=OFF OTHER_THREE_ON',
                b'DIAG=FRONT_RESTORE DIN2=PB15 FRONT_UART2=ON_AFTER_TREASURE FOUR_RX')
            expected = expected.replace(b'LASER_RX=LEFT_RIGHT_REAR FRONT_UART2=OFF',
                                        b'LASER_RX=ALL_ON FRONT_UART2=ON')
        if source == 'Hardware/LaserRx.c':
            assert digest(ROOT / destination) == REFERENCE['restored_receiver_sha256']
        elif source not in ('Tests/test_uart.c', 'Tests/run_algorithms.py'):
            assert actual == expected, 'Unexpected change: ' + destination
        (unchanged if actual == original.read_bytes() else adjusted).append(destination)

    project = ROOT / 'MDK-ARM' / (ROOT.name + '.uvprojx')
    tree = ET.parse(project)
    paths = [item.text.replace('\\', '/') for item in tree.findall('.//FilePath')]
    assert len(paths) == len(set(path.lower() for path in paths))
    for path in paths:
        assert (project.parent / path).is_file(), path
    for path in tree.find('.//Cads/VariousControls/IncludePath').text.split(';'):
        assert (project.parent / path.replace('\\', '/')).is_dir(), path
    assert tree.find('.//Device').text == 'STM32F103RC'
    assert tree.find('.//uAC6').text == '0'

    app = (ROOT / 'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
    driver = (ROOT / 'Hardware/LaserRx.c').read_text(encoding='utf-8')
    main_source = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
    assert 'i==LASER_FRONT' not in driver and 'i!=LASER_FRONT' not in driver
    assert 'if(value && enabled) return 1U;' in driver
    assert 'budget=LASER_RX_POLL_BYTES_PER_CHANNEL' in driver
    assert 'LaserRx_SetEnabled(thief_treasure_passed)' in app
    assert 'Thief_EnableAfterTreasure(MapRouteNavigator_Current(&map_nav)' in app
    assert 'TREASURE_N005 LASER_RX=ALL_ON FRONT_UART2=ON RANGE_MM=650' in app
    assert 'MapLaserMask_FrameAllowed' in app
    assert 'huart2.Init.Mode = UART_MODE_RX;' in main_source

    result = dict(parent_inputs_checked=len(REFERENCE['source_sha256']),
                  unchanged_copied_files=len(unchanged), adjusted_copied_files=adjusted,
                  unique_keil_source_paths=len(paths), duplicate_sources=0,
                  parent_unchanged=True, front_receiver_restored=True,
                  runtime_behavior_change='Front UART2 RX restored after treasure; DIN2 stays PB15',
                  not_physical_mux_proof=True)
    output = ROOT / 'Tests/build/front_restore_integration.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
