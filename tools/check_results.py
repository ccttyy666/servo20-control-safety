#!/usr/bin/env python3
"""Regression checks against the C++ simulation outputs."""
import csv
import math
from pathlib import Path

folder=Path('results')
expect={
    'nominal':('RUNNING','none',20),
    'jam':('STOPPED','persistent_following_error joint=8',0),
    'controller_heartbeat':('STOPPED','controller_heartbeat_timeout',0),
    'drive_heartbeat':('STOPPED','drive_heartbeat_timeout joint=13',0),
}
for name,(state,reason,arrived) in expect.items():
    summary=dict(line.strip().split('=',1) for line in (folder/f'{name}_summary.txt').read_text().splitlines() if '=' in line)
    assert summary['final_state']==state,(name,summary['final_state'])
    assert summary['stop_reason']==reason,(name,summary['stop_reason'])
    assert int(summary['arrived_count'].split('/')[0])==arrived,(name,summary['arrived_count'])
    if name=='nominal':
        assert 0<=float(summary['arrival_spread_ms'])<250
    else:
        assert 1.45<=float(summary['safety_trigger_s'])<2.2
        assert float(summary['stopped_s'])>float(summary['safety_trigger_s'])
    with open(folder/f'{name}.csv',newline='') as f:
        rows=list(csv.DictReader(f))
    assert len(rows)==4501,(name,len(rows))
    assert all(math.isfinite(float(r['q_8'])) for r in rows)
    print(f'PASS {name}: {state}; {reason}')
print('All four scenario checks passed.')
