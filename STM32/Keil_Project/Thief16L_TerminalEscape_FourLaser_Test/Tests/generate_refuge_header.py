"""Deterministically emit the small Flash metadata table from the map audit."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def emit():
    data = json.loads((ROOT/'Tests/refuge_metadata.json').read_text(encoding='utf-8'))
    rows = {row['node']:row for row in data['candidates']}
    out = ['/* Public map: Class0 weak, Class1 normal, Class2 conditional role; not ordinal. */',
           '#ifndef THIEF_REFUGE_METADATA_H', '#define THIEF_REFUGE_METADATA_H', '#include <stdint.h>']
    tables = [('uint8_t','class','tier'),('uint8_t','fast_exits','fast_exit_count'),
              ('uint16_t','patrol_detection_ms','patrol_search_ms'),
              ('uint8_t','conditional','conditional')]
    for kind,name,key in tables:
        values = [int(rows.get(n,{}).get(key,0)) for n in range(1,89)]
        out.append(f'static const {kind} thief_refuge_{name}[88] = {{')
        for a in range(0,88,16):out.append('  '+', '.join(f'{v}U' for v in values[a:a+16])+',')
        out.append('};')
    values = [int(n in data['dominant_corridor']['nodes']) for n in range(1,89)]
    out.append('static const uint8_t thief_refuge_patrol_corridor[88] = {')
    for a in range(0,88,16):out.append('  '+', '.join(f'{v}U' for v in values[a:a+16])+',')
    out.extend(['};','#endif',''])
    return '\n'.join(out)

if __name__=='__main__':
    (ROOT/'Algorithm/ThiefStrategy/ThiefRefugeMetadata.h').write_text(emit(),encoding='utf-8')
