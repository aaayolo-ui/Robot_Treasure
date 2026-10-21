"""Read-only project/source audit for the in-place 16B upgrade."""
from pathlib import Path
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).absolute().parents[1]
REPO = ROOT.parents[2]
J = ROOT.with_name('Thief16J_RefugeViability_FourLaser_Test')
PROJECT = ROOT / 'MDK-ARM' / (ROOT.name + '.uvprojx')
BASE = 'c15d11b9ed7382862fe2c03c15bd97bbcb2b17ed'


def main():
    tree = ET.parse(PROJECT)
    paths = [n.text.replace('\\', '/') for n in tree.findall('.//FilePath')]
    assert len(paths) == len(set(p.lower() for p in paths))
    for p in paths:
        assert (PROJECT.parent / p).is_file(), p
    assert tree.find('.//Device').text == 'STM32F103RC'
    include = tree.find('.//Cads/VariousControls/IncludePath').text
    assert '../Algorithm/MapLocalization' in include
    strategy = []
    adjusted_strategy = {'FrontLaserGate.c', 'FrontLaserGate.h',
                         'ThiefStrategyConfig.h', 'ThiefRollout.c'}
    for file in sorted((J / 'Algorithm/ThiefStrategy').iterdir()):
        if file.suffix not in ('.c', '.h'):
            continue
        ours = ROOT / 'Algorithm/ThiefStrategy' / file.name
        if file.name not in adjusted_strategy:
            assert ours.read_bytes() == file.read_bytes(), file.name
            strategy.append(file.name)
    protected = []
    adjusted_hardware = {'Hardware/LaserRx.c', 'Hardware/LaserRx.h'}
    for folder in ('Core', 'Hardware', 'Control', 'Drivers', 'System'):
        for file in sorted((ROOT / folder).rglob('*')):
            if not file.is_file():
                continue
            path = file.relative_to(REPO).as_posix()
            if file.relative_to(ROOT).as_posix() in adjusted_hardware:
                continue
            original = subprocess.run(['git', 'show', BASE + ':' + path],
                cwd=str(REPO), check=True, capture_output=True).stdout
            assert original == file.read_bytes(), path
            protected.append(path)
    old = ET.fromstring(subprocess.run(['git', 'show', BASE + ':' + PROJECT.relative_to(REPO).as_posix()],
        cwd=str(REPO), check=True, capture_output=True).stdout)
    # Only source registrations and the new include folder may change.
    current = tree.getroot()
    for t in (old, current):
        for parent in t.iter():
            for child in list(parent):
                if child.tag in ('Groups', 'IncludePath'):
                    parent.remove(child)
            if parent.text is not None and not parent.text.strip():
                parent.text = None
            if parent.tail is not None and not parent.tail.strip():
                parent.tail = None
    assert ET.tostring(old) == ET.tostring(current), 'Target/debug/flash configuration changed'
    app = (ROOT / 'User/App_ThiefStrategy.c').read_text(encoding='utf-8')
    driver = (ROOT / 'Hardware/LaserRx.c').read_text(encoding='utf-8')
    init = driver.split('void LaserRx_Init(void)', 1)[1].split('uint8_t LaserRx_IsEnabled', 1)[0]
    assert 'LaserRx_SetEnabled(1U)' not in init and 'enabled=0U' in init
    assert 'if(value && enabled) return 1U;' in driver
    assert 'budget=LASER_RX_POLL_BYTES_PER_CHANNEL' in driver
    assert 'while(c->tail!=c->head && budget>0U)' in driver
    callback = app.split('static void Thief_OnLaserFrame', 1)[1].split('static uint8_t Thief_ThreatMask', 1)[0]
    assert 'Thief_UpdatePose();' not in callback
    assert 'THIEF_LASER_MOVING_FRAME_MAX_MS' in callback and 'ThiefThreatModel_Record' in callback
    assert '(int32_t)(tick-thief_laser_pose_since)<0L' in callback
    assert 'thief_laser_pose_since=now;' in app
    assert 'LaserRx_SetEnabled(thief_treasure_passed)' in app
    assert 'TREASURE_N005 LASER_RX=ALL_ON RANGE_MM=650' in app
    assert 'Thief_EnableAfterTreasure(MapRouteNavigator_Current(&map_nav)' in app
    assert app.count('LaserRx_Poll(Thief_OnLaserFrame)') == 1
    assert 'TreasureLaserStart.c' in [Path(p).name for p in paths]
    assert 'MapLaserMask.c' in [Path(p).name for p in paths]
    assert callback.index('MapLaserMask_FrameAllowed') < callback.index('ThiefRollout_Evidence')
    assert callback.index('MapLaserMask_FrameAllowed') < callback.index('FrontLaserGate_Observe')
    mask_update = app.split('static void Thief_UpdateLaserMask', 1)[1].split('static void Thief_OnLaserFrame', 1)[0]
    assert 'MapLaserMask_DetectionMask(&thief_pose)' in mask_update
    assert 'FrontLaserGate_ResetEvidence' in mask_update and 'ThiefRollout_ResetEvidence' in mask_update
    assert 'thief_laser_seen[i]=0U' in mask_update and 'thief_laser_allowed_since[i]=now' in mask_update
    threat = app.split('static uint8_t Thief_ThreatMask', 1)[1].split('static void Thief_LaserLog', 1)[0]
    assert 'if(!(thief_laser_active_mask&(1U<<i))) continue;' in threat
    task_end = app.split('task_end:', 1)[1]
    assert task_end.index('Thief_UpdateLaserMask(') < task_end.index('LaserRx_Poll(')
    geom = (ROOT / 'Algorithm/MapLocalization/MapLaserMask.c').read_text(encoding='utf-8')
    assert 'travel_heading' not in geom and 'HAL_' not in geom
    assert '(int32_t)(frame_tick-enabled_since)>0L' in geom
    gray_origin = ROOT.with_name('Gray12_MapFullRoute_TreasureMask_Test') / 'Algorithm/MapRoute'
    for name in ('TreasureGrayMask.c', 'TreasureGrayMask.h'):
        assert (ROOT / 'Algorithm/MapRoute' / name).read_bytes() == (gray_origin / name).read_bytes()
    assert 'TreasureGrayMask.c' in [Path(p).name for p in paths]
    front = app.split('static RouteGrayState_t Route_ReadFront', 1)[1].split('static RouteGrayState_t Route_ReadRear', 1)[0]
    rear = app.split('static RouteGrayState_t Route_ReadRear', 1)[1].split('static RouteGrayState_t Route_ReadLeadingOnce', 1)[0]
    assert 'Route_FilterTreasureGray(NCHD12_Extract12(raw16),1U)' in front
    assert 'Route_FilterTreasureGray(NCHD12_Extract12(raw16),0U)' in rear
    for block in (front, rear):
        assert block.index('Route_FilterTreasureGray(') < block.index('GrayPosition_Calculate(') < block.index('Route_ClassifyPosition(')
    gray_gate = app.split('static uint16_t Route_FilterTreasureGray', 1)[1].split('static RouteGrayState_t Route_ReadFront', 1)[0]
    assert '!map_nav.arrived' in gray_gate and 'step->action==MAP_ACTION_STRAIGHT' in gray_gate
    assert 'front_resume_state==ROUTE_STATE_MAP_PASS' in gray_gate
    assert 'TURN_APPROACH' not in gray_gate and 'TURN_LEFT' not in gray_gate and 'AFTER_TURN_FORWARD' not in gray_gate
    assert 'TREASURE_GRAY=MASK1 N004_N006=NORTH_OUTER_TWO TRANSLATION_ONLY' in app
    result = {'baseline_commit': BASE, 'strategy_files_identical_to_16j': strategy,
              'strategy_files_adjusted_for_laser65': sorted(adjusted_strategy),
              'hardware_files_adjusted_for_laser65': sorted(adjusted_hardware),
              'protected_files_unchanged': len(protected), 'project_entries': len(paths),
              'duplicate_sources': False, 'target_debug_flash_unchanged': True,
              'laser_before_treasure_off': True, 'bounded_uart_poll': True,
              'replan_mode': 'settled_hide_only',
              'boundary_mask_before_all_evidence': True,
              'boundary_unmask_requires_fresh_frames': True,
              'treasure_gray_module_identical_to_prior_full_route': True,
              'treasure_gray_filters_before_position_and_branch': True,
              'treasure_gray_post_turn_alignment_unmasked': True,
              'hardware': False}
    print(json.dumps(result, indent=2))
    output = ROOT / 'Tests/build'
    output.mkdir(exist_ok=True)
    (output / 'integration_result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
