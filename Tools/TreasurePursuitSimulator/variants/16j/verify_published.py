"""Check a downloaded publication without private workspace/build dependencies."""
from pathlib import Path
import hashlib
import json
import re
import sys

HERE = Path(__file__).absolute().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE.with_name('16f')))
from body_geometry import Body, Pose, separation


def main():
    report = json.loads((HERE / 'report.json').read_text(encoding='utf-8'))
    html = (HERE / 'replay.html').read_text(encoding='utf-8')
    embedded = re.search(r'<script id="report-data" type="application/json">(.*?)</script>', html, re.S)
    assert embedded is not None, 'Embedded replay data missing'
    assert json.loads(embedded.group(1)) == report, 'HTML/report mismatch'
    hashes = dict(report['source_hashes'])
    for group in report['comparison']['source_hashes'].values():
        for name, digest in group.items():
            assert name not in hashes or hashes[name] == digest, name
            hashes[name] = digest
    for name, digest in hashes.items():
        assert hashlib.sha256((REPO / name).read_bytes()).hexdigest() == digest, name
    annotations = json.loads((HERE / 'map_annotations.json').read_text(encoding='utf-8'))
    assert annotations == report['annotations']
    nodes = {n['id'] for n in annotations['nodes']}
    assert len(nodes) == 88 and 'N001' in nodes
    assert len(annotations['segments']) == 106
    for edge in annotations['segments']:
        assert edge['start'] in nodes and edge['end'] in nodes
        assert edge['length_m'] > 0
    bodies = {role: Body(**size) for role, size in report['body_model']['bodies'].items()}
    spacing = report['map']['grid_spacing_m']
    tolerance = report['body_model']['collision']['contact_tolerance_m']
    frame_count = 0
    for replay in report['replays']:
        frames = replay['frames']
        for i, frame in enumerate(frames):
            assert len(frame) == 38
            if i:
                assert frame[0] > frames[i - 1][0]
            police = Pose(frame[2] * spacing, frame[3] * spacing, frame[33])
            thief = Pose(frame[9] * spacing, frame[10] * spacing, frame[34])
            gap = separation(bodies['P'], police, bodies['B'], thief)
            assert abs(gap - frame[37]) < 1e-8
            if i < len(frames) - 1 or replay['collision'] is None:
                assert gap > tolerance - 1e-8
            frame_count += 1
        if replay['collision']:
            assert replay['winner'] == 'police'
            assert frames[-1][37] <= tolerance + 1e-8
            assert abs(replay['duration_s'] - replay['collision']['time_s']) < 1e-8
        assert not any(event['kind'] == 'fault' for event in replay['planner_events'])
    source = REPO / 'Tools/MapTimingLogger/datasets/20261006-210343-671.log'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    assert digest == '138576fbe7416f90fad63818e5d3d552f5463ef1cb8ddebf997868bcf0b09d12'
    print(json.dumps({'source_files': len(hashes), 'nodes': len(nodes),
                      'segments': 106, 'replays': len(report['replays']),
                      'frames': frame_count, 'embedded_report_matches': True,
                      'original_log_sha256': digest, 'hardware_test': False}, indent=2))


if __name__ == '__main__':
    main()
