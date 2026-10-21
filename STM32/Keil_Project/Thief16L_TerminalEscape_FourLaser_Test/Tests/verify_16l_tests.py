"""Run final ARMCC5 / Cortex-M3 test modes sequentially; no hardware access."""
from pathlib import Path
import hashlib,json,subprocess,sys

ROOT=Path(__file__).resolve().parents[1]
MODES=[('main',()),('policy_on',('--refuge-policy',)),
       ('policy_off',('--refuge-policy','--define=THIEF_REFUGE_POLICY=0')),
       ('terminal',('--terminal',)),('conditional_on',('--conditional',)),
       ('conditional_off',('--conditional','--define=THIEF_TERMINAL_ESCAPE_ENABLED=0')),
       ('localization',('--localization',)),('treasure_gray',('--treasure-gray',)),
       ('edge_mask',('--edge-mask',)),('treasure_laser',('--treasure-laser',)),('uart',('--uart',))]

def main():
    results={}
    for name,args in MODES:
        proc=subprocess.run([sys.executable,str(ROOT/'Tests/run_algorithms.py'),*args],
                            capture_output=True,text=True)
        data=next((json.loads(line) for line in reversed(proc.stdout.splitlines())
                   if line.startswith('{') and 'test_checks' in line),None)
        diagnostics=proc.stderr+proc.stdout
        warnings=[line for line in diagnostics.splitlines() if 'warning:' in line.lower()]
        errors=[line for line in diagnostics.splitlines() if 'error:' in line.lower()]
        if data is None:raise RuntimeError(f'{name}: missing test result; {diagnostics[-1500:]}')
        data.update(exit_code=proc.returncode,compiler_warnings=warnings,compiler_errors=errors)
        results[name]=data
        print(json.dumps({'mode':name,'checks':data['test_checks'],
                          'failures':data['test_failures'],'exit_code':proc.returncode}),flush=True)
        if proc.returncode:break
    passed=len(results)==len(MODES) and all(not row['test_failures'] and row['test_done']==1
               and row['exit_code']==0 and not row['compiler_warnings'] and not row['compiler_errors']
               for row in results.values())
    report={'passed':passed,'scope':'Actual ARMCC5 Cortex-M3 offline CPU tests; no vehicle/IRQ/timing proof',
            'mode_count':len(results),'total_checks_including_ablation_repeats':sum(r['test_checks'] for r in results.values()),
            'test_failures':sum(r['test_failures'] for r in results.values()),'modes':results,
            'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'hardware':False,'status':'仅编译通过，尚未完成硬件验证',
            'conditional_limit':'No actual N026 admission demonstrated. Tests exercise partial/unknown rejection and emergency fallback, plus standalone clear/collision forecasts.'}
    (ROOT/'VALIDATION_16L_TESTS.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not passed:sys.exit(1)

if __name__=='__main__':main()
