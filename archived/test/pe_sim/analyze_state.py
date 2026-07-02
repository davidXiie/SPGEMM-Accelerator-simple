"""Analyze VCD state signal to compute cycle distribution."""
import re

import os
vcd_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_build', 'pe_dump.vcd')
if not os.path.exists(vcd_path):
    # Try relative to pe_sim
    vcd_path = 'sim_build/pe_dump.vcd'
with open(vcd_path, 'r') as f:
    lines = f.readlines()

codes = {}
for line in lines:
    m = re.match(r'\$var\s+\w+\s+(\d+)\s+(\S+)\s+(.*?)\s+\$end', line)
    if m: codes[m.group(2)] = m.group(3)

# Find state signal code
state_code = None
for c, n in codes.items():
    if n.startswith('state') and 'acc_state' not in n and 'state_' not in n:
        state_code = c
        break

if state_code is None:
    print("ERROR: state signal not found")
    exit()

time_ps = 0
events = []
for line in lines:
    line = line.strip()
    if not line: continue
    if line.startswith('#'):
        time_ps = int(line[1:])
    elif line[0] == 'b':
        s = line.find(' '); val = line[1:s] if s > 0 else ''; code = line[s+1:] if s > 0 else ''
        if code == state_code: events.append((time_ps, val))
    elif line[0] in '01xz':
        val, code = line[0], line[1:]
        if code == state_code: events.append((time_ps, val))

state_names = ['IDLE', 'LD_DESC', 'CLEAR', 'LD_A', 'LD_B',
               'STREAM', 'FLUSH', 'WT_TASK', 'WT_PROD', 'WRITE', 'NEXT', 'DONE']

# Time in each state (ps)
dist = {}
prev_state = None
prev_time = 0
for t, v in events:
    if prev_state is not None:
        dt = t - prev_time
        dist[prev_state] = dist.get(prev_state, 0) + dt
    prev_state = v
    prev_time = t

# Add final state to end
if prev_state is not None:
    dist[prev_state] = dist.get(prev_state, 0) + (events[-1][0] - prev_time)

total = sum(dist.values())
total_cycles = total / 100000  # ps → cycles (10ns = 100000ps)

print(f"Total simulated time: {total_cycles:.0f} cycles")
print()
print(f"{'State':<12} {'Cycles':>8} {'%':>7}")
print("-" * 28)

# Merge similar states
merged = {
    'CLEAR_ACC':  dist.get('0010', 0),   # state=2
    'LOAD+STREAM': dist.get('0011', 0) + dist.get('0100', 0) + dist.get('0101', 0),  # 3,4,5
    'FLUSH+DRAIN': dist.get('0110', 0) + dist.get('0111', 0) + dist.get('1000', 0),  # 6,7,8
    'WRITE_ROW':   dist.get('1001', 0),   # 9
    'NEXT+DONE':   dist.get('1010', 0) + dist.get('1011', 0),  # 10,11
    'IDLE':        dist.get('0000', 0),   # 0
    'LD_ROW_DESC': dist.get('0001', 0),   # 1
}

for name, ps in sorted(merged.items(), key=lambda x: -x[1]):
    cyc = ps / 100000
    pct = 100 * ps / total
    print(f"{name:<12} {cyc:>8.0f} {pct:>6.1f}%")

print()
print("--- Detailed ---")
for v, ps in sorted(dist.items(), key=lambda x: -x[1]):
    cyc = ps / 100000
    pct = 100 * ps / total
    idx = int(v, 2)
    sn = state_names[idx] if idx < 12 else '?'
    print(f"  {v} ({sn:<7})  {cyc:>8.0f}  {pct:>5.1f}%")
