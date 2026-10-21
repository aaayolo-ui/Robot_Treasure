"""Sample SP at basic-block entries while executing the actual ARMCC 16L planner.

This is offline ARM CPU evidence for the specified adversarial inputs. It does
not measure board wall-clock time, IRQ nesting, the application's caller stack,
or prove a production stack high-water mark. It never flashes or drives motors.
"""
from pathlib import Path
import json
import re
import time
import argparse

from arm_adapter import FIRMWARE, HERE, Planner, build
from unicorn import UC_HOOK_BLOCK
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_LR, UC_ARM_REG_R0


class ProfiledPlanner(Planner):
    def __init__(self, elf):
        super().__init__(elf)
        self.base_sp = 0x2000bff0
        self.minimum_sp = self.base_sp
        self.minimum_pc = 0
        self.blocks = 0
        self.samples = []
        self.u.hook_add(UC_HOOK_BLOCK, self.sample)

    def sample(self, uc, address, size, data):
        sp = uc.reg_read(UC_ARM_REG_SP)
        self.blocks += 1
        if sp < self.minimum_sp:
            self.minimum_sp, self.minimum_pc = sp, address

    def execute(self, command):
        self.minimum_sp, self.minimum_pc, self.blocks = self.base_sp, 0, 0
        started = time.perf_counter()
        super().execute(command)
        self.samples.append({
            'command': command, 'peak_sampled_bytes': self.base_sp-self.minimum_sp,
            'minimum_sp': hex(self.minimum_sp), 'minimum_pc': hex(self.minimum_pc),
            'return_pc': hex(self.u.reg_read(UC_ARM_REG_PC)),
            'block_samples': self.blocks,
            'host_emulation_seconds': round(time.perf_counter()-started, 3),
        })

    def work_bytes(self):
        self.u.reg_write(UC_ARM_REG_SP,self.base_sp)
        self.u.reg_write(UC_ARM_REG_LR,0x080ff001)
        self.u.emu_start(self.sym['ThiefTerminalForecast_WorkBytes']|1,0x080ff000,count=1000)
        assert self.u.reg_read(UC_ARM_REG_PC)==0x080ff000
        return self.u.reg_read(UC_ARM_REG_R0)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--optimization',choices=('-O1','-O3'),default='-O1')
    parser.add_argument('--output',default='stack_profile.json')
    parser.add_argument('--label',default='stack_profile')
    args=parser.parse_args()
    stack_source = FIRMWARE/'MDK-ARM/startup_stm32f103xe.s'
    stack_bytes = int(re.search(r'Stack_Size\s+EQU\s+(0x[0-9A-Fa-f]+|\d+)',
                              stack_source.read_text()).group(1), 0)
    elf, hashes = build(label=args.label,optimization=args.optimization)
    planner = ProfiledPlanner(elf)
    work_bytes = planner.work_bytes()
    initial = planner.initial()
    cases = [{'name': 'initial', 'plan': initial, 'samples': planner.samples[-1:]}]
    print(json.dumps({'case':'initial','peak':planner.samples[-1]['peak_sampled_bytes']}), flush=True)
    # Geographic compass headings rotate into front/left/right/rear channels.
    # 300 mm clears the immediate overlap envelope; confidence 20 is accepted.
    # Inputs include two simultaneous rays to stress four-laser evidence handling.
    for node in (38, 26, 62):
        body = 0 if node==26 else 1  # Actual admitted N/S vs E/W parking axis.
        for name, headings, distance in (
                ('far_north', (0,), 600), ('far_south', (2,), 600),
                ('far_east', (1,), 600), ('far_west', (3,), 600),
                ('north', (0,), 300), ('south', (2,), 300),
                ('east', (1,), 300), ('west', (3,), 300),
                ('north_south', (0,2), 300), ('east_west', (1,3), 300),
                ('close_fore_aft', (0,2), 65)):
            planner.initial()  # Identical fresh planner state for each fixture.
            planner.clear_gate()
            before = len(planner.samples)
            offsets = (0,3,1,2)
            distances = [distance if (body+h)%4 in headings else 5000 for h in offsets]
            # Four recent accepted-quality out-of-range frames reproduce the
            # finite endpoint-entry evidence observed in default replay game1.
            if distance==600:
                for frame in range(4):
                    planner.observe(node,body,1000+20*frame,[5000]*4,[20]*4,15,0)
            observation = None
            for frame in range(5):
                observation = planner.observe(node, body, 1080+20*frame,
                                               distances, [20]*4, 15, 0)
                if observation['ok']:
                    break
            item = {'name': f'N{node:03d}_{name}', 'node': node, 'body_heading':body,
                    'distance_mm':distances, 'confidence':[20]*4, 'fresh_mask':15,
                    'preceding_clear_frames':4 if distance==600 else 0,
                    'observation':observation, 'samples':planner.samples[before:]}
            cases.append(item)
            peak = max(s['peak_sampled_bytes'] for s in item['samples'])
            print(json.dumps({'case':item['name'],'peak':peak,
                              'planned':observation['ok'],
                              'terminal_candidates':observation['plan']['terminal_candidates']
                                  if observation['ok'] else 0}), flush=True)
    peak = max(s['peak_sampled_bytes'] for c in cases for s in c['samples'])
    result = {'scope':'Actual ARMCC5 Cortex-M3 pure algorithm bridge; offline CPU execution only',
              'startup_stack_bytes':stack_bytes, 'peak_sampled_bytes':peak,
              'terminal_workspace_bytes_compiled_accessor':work_bytes,
              'remaining_bytes_against_startup':stack_bytes-peak,
              'requires_review':peak>=900, 'sample_method':'Unicorn UC_HOOK_BLOCK entry SP',
              'limitations':['A block entry can miss an intra-block transient SP decrement.',
                             'Application caller stack and IRQ nesting are absent.',
                             'Host emulation elapsed seconds do not measure board or IRQ timing.',
                             'Only listed initial/adversarial observations were executed.'],
              'build_label':args.label, 'optimization':args.optimization, 'defines':[], 'source_sha256':hashes,
              'hardware_run':False, 'board_timing_run':False, 'cases':cases}
    (HERE/args.output).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'peak_sampled_bytes':peak,'startup_stack_bytes':stack_bytes,
                      'requires_review':result['requires_review']}),flush=True)


if __name__=='__main__':
    main()
