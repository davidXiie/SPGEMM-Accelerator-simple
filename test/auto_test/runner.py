#!/usr/bin/env python3
"""
Automated SPGEMM test runner.

Flow per test case:
  1. Generate A/B matrix files via matrix_gen
  2. Call vvp (pre-compiled cocotb sim) with env vars pointing to generated files
  3. Parse results from cocotb log and write summary CSV.
"""
import sys, os, json, subprocess, glob, csv, time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CASES_JSON = os.path.join(SCRIPT_DIR, 'cases.json')
GEN_DIR    = os.path.join(SCRIPT_DIR, 'generated_cases')
SIM_BUILD  = os.path.join(SCRIPT_DIR, 'sim_build')
VVP_BIN    = os.path.join(SIM_BUILD, 'sim_mmap.vvp')
RESULTS_CSV= os.path.join(SCRIPT_DIR, 'results.csv')

# cocotb env — same as run.bat
COCOTB_LIB_DIR = r'C:/Users/Administrator/.conda/envs/gcnenv/Lib/site-packages/cocotb/libs'
COCOTB_VPI     = 'cocotbvpi_icarus'
PYTHON_BIN     = r'C:/Users/Administrator/.conda/envs/gcnenv/python.exe'
PYTHON_DIR     = r'C:/Users/Administrator/.conda/envs/gcnenv'
IVERILOG_DIR   = r'C:/iverilog/bin'


def compile_sim():
    """Compile the Verilog once (reuse for all test cases)."""
    os.makedirs(SIM_BUILD, exist_ok=True)
    rtl_dir = os.path.join(SCRIPT_DIR, '..', '..', 'rtl', 'core')
    inf_dir = os.path.join(SCRIPT_DIR, '..', '..', 'rtl', 'infrastructure')
    inc_dir = os.path.join(SCRIPT_DIR, '..', '..', 'rtl', 'include')
    tb_file = os.path.join(SCRIPT_DIR, 'tb_mmap.v')

    # If tb_mmap.v doesn't exist locally, use the one from mmap_sim
    if not os.path.exists(tb_file):
        tb_file = os.path.join(SCRIPT_DIR, '..', 'mmap_sim', 'tb_mmap.v')

    sources = [
        tb_file,
        os.path.join(rtl_dir, 'accelerator_axi_top.v'),
        os.path.join(rtl_dir, 'axi_loader.v'),
        os.path.join(rtl_dir, 'axi_c_drain.v'),
        os.path.join(rtl_dir, 'pe_cluster.v'),
        os.path.join(rtl_dir, 'pe_top.v'),
        os.path.join(rtl_dir, 'pe_mul_array.v'),
        os.path.join(rtl_dir, 'fp16_mul.v'),
        os.path.join(rtl_dir, 'fp16_add.v'),
        os.path.join(rtl_dir, 'accum_bank.v'),
        os.path.join(rtl_dir, 'accum_bank_16.v'),
        os.path.join(rtl_dir, 'row_accumulator_16bank.v'),
        os.path.join(inf_dir, 'scratchpad.v'),
    ]

    cmd = ['iverilog', '-g2012', '-DCOCOTB_SIM=1', f'-I{inc_dir}',
           '-s', 'tb_mmap', '-o', VVP_BIN] + sources
    print(f"[COMPILE] {' '.join(cmd)}")
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=SCRIPT_DIR)
    if proc.returncode != 0:
        print("[COMPILE] FAILED")
        print(proc.stderr)
        return False
    print("[COMPILE] OK")
    return True


def run_case(case):
    """Run a single test case via vvp. Returns dict with results."""
    name    = case['name']
    M, K, N = case['M'], case['K'], case['N']
    density = case['density']
    seed    = case.get('seed', 42)

    print(f"\n{'='*60}")
    print(f"[RUN] {name}: A({M},{K}) × B({K},{N}) @ {density*100:.0f}%")
    print(f"{'='*60}")

    # Generate matrix files
    import matrix_gen
    gen_dir = os.path.join(GEN_DIR, name)
    matrix_gen.generate_matrix_files(name, M, K, N, density, seed, out_dir=gen_dir)

    # Build env for vvp
    env = os.environ.copy()
    env['PATH'] = f'{IVERILOG_DIR};{PYTHON_DIR};{PYTHON_DIR}/Scripts;{PYTHON_DIR}/Library/bin;{env.get("PATH","")}'
    env['COCOTB_LIB'] = COCOTB_LIB_DIR
    env['COCOTB_VPI_MODULE'] = COCOTB_VPI
    env['LIBPYTHON_DIR'] = PYTHON_DIR
    env['PYGPI_PYTHON_BIN'] = PYTHON_BIN
    env['COCOTB_TEST_MODULES'] = 'test'
    env['COCOTB_TESTCASE'] = 'test_mmap'
    env['COCOTB_TOPLEVEL'] = 'tb_mmap'
    env['COCOTB_LOG_LEVEL'] = 'WARNING'
    env['COCOTB_SIM'] = '1'
    env['PYTHONIOENCODING'] = 'utf-8'
    env['PYTHONPATH'] = SCRIPT_DIR
    env['A_INDEX']  = 'A_Index.txt'
    env['A_MATRIX'] = 'A_Matrix.txt'
    env['B_INDEX']  = 'B_Index.txt'
    env['B_MATRIX'] = 'B_Matrix.txt'
    env['MATRIX_SUBDIR'] = gen_dir
    env['N_PE'] = '3'

    log_file = os.path.join(SIM_BUILD, f'log_{name}.txt')
    t0 = time.time()
    with open(log_file, 'w') as f:
        proc = subprocess.run(['vvp', '-M', COCOTB_LIB_DIR, '-m', COCOTB_VPI, VVP_BIN],
                              env=env, stdout=f, stderr=subprocess.STDOUT, cwd=SCRIPT_DIR,
                              timeout=1800)
    elapsed = time.time() - t0
    rc = proc.returncode

    # Parse result from log
    result = {
        'name': name, 'M': M, 'K': K, 'N': N, 'density': density,
        'elapsed_s': round(elapsed, 1), 'rc': rc,
        'ddr_pass': False, 'pe_pass': False, 'mismatches_ddr': '', 'mismatches_pe': '',
        'cycles': '', 'mac': '', 'ops_cyc': ''
    }
    with open(log_file, encoding='utf-8', errors='replace') as f:
        for line in f:
            if 'RESULT: PASS' in line:
                result['ddr_pass'] = True
                result['pe_pass']  = True
                parts = line.split()
                try:
                    result['cycles'] = int(parts[parts.index('cycles=')+1])
                    result['mac']    = int(parts[parts.index('MAC=')+1])
                    result['ops_cyc']= parts[parts.index('ops/cyc=')+1]
                except (ValueError, IndexError):
                    pass
            elif 'RESULT: DDR FAILED' in line:
                parts = line.split()
                try:
                    result['mismatches_ddr'] = parts[-1].strip('()')
                except IndexError:
                    pass
            elif 'RESULT: PE  FAILED' in line:
                parts = line.split()
                try:
                    result['mismatches_pe'] = parts[-1].strip('()')
                except IndexError:
                    pass

    status = "PASS" if (result['ddr_pass'] and result['pe_pass']) else "FAIL"
    print(f"[DONE] {name}: {status}  ({elapsed:.0f}s)")
    if not result['ddr_pass']:
        print(f"       DDR mismatches: {result['mismatches_ddr']}")
    if not result['pe_pass']:
        print(f"       PE  mismatches: {result['mismatches_pe']}")
    return result


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--compile-only':
        compile_sim()
        return

    with open(CASES_JSON, encoding='utf-8') as f:
        cases = json.load(f)

    # Compile once
    if not os.path.exists(VVP_BIN):
        if not compile_sim():
            sys.exit(1)

    # Run all cases
    results = []
    for case in cases:
        r = run_case(case)
        results.append(r)

    # Write summary CSV
    fields = ['name','M','K','N','density','ddr_pass','pe_pass',
              'mismatches_ddr','mismatches_pe','cycles','mac','ops_cyc','elapsed_s']
    with open(RESULTS_CSV, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(results)

    passed = sum(1 for r in results if r['ddr_pass'] and r['pe_pass'])
    print(f"\n{'='*60}")
    print(f"SUMMARY: {passed}/{len(results)} PASSED")
    print(f"Results → {RESULTS_CSV}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
