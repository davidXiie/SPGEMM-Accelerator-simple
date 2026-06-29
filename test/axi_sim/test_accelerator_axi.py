#!/usr/bin/env python3
"""
AXI DDR-direct accelerator test — load-balanced partition.

  DDR stores A pre-partitioned (partition_a) and B broadcast.
  Hardware axi_loader reads header → loads each PE's section → starts compute.

  Same dataset and partition method as test_comp.py's test_comp_case1_cluster.
"""
import os, struct, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N = 512; C_ROW_STRIDE = MAX_N

# =========================================================================
# FP16 + descriptor helpers
# =========================================================================
def int_to_fp16_bits(v):
    return int.from_bytes(struct.pack('<e', float(v)), 'little')

def fp16_from_bits(bits):
    return struct.unpack('<e', struct.pack('<H', bits & 0xFFFF))[0]

def a_desc(off, nnz, crow):
    return (int(off) << 19) | (int(nnz) << 9) | int(crow)

def a_desc_crow(d): return int(d) & 0x1FF
def a_desc_nnz(d):  return (int(d) >> 9) & 0x3FF
def a_desc_off(d):  return (int(d) >> 19) & 0x3FFF

def b_desc(off, nnz):
    return (int(off) << 10) | int(nnz)

def b_desc_nnz(d): return int(d) & 0x3FF
def b_desc_off(d): return (int(d) >> 10) & 0x1FFFF


# =========================================================================
# Matrix loading
# =========================================================================
def load_comp_matrix(index_file, matrix_file, is_B=False, subdir='TC1_RAW'):
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'test_case_for_reference', subdir)
    with open(os.path.join(base, matrix_file)) as f:
        mat_lines = [list(map(int, l.split())) for l in f if l.strip()]
    with open(os.path.join(base, index_file)) as f:
        idx_lines_raw = [list(map(int, l.split())) for l in f]
    if not is_B:
        rows = len(mat_lines); K = mat_lines[0][1]
        idx_lines = idx_lines_raw[:rows]
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        for r in range(rows):
            nnz = mat_lines[r][0]; cols = idx_lines[r] if nnz > 0 else []
            assert len(cols) == nnz
            for c in cols:
                v = (r * 37 + c * 13 + 1) % 7 + 1
                col_arr.append(c); val_arr.append(int_to_fp16_bits(v))
            row_desc.append(a_desc(offset, nnz, r))
            offset += nnz
        return row_desc, col_arr, val_arr, offset, rows, K
    else:
        B_cols = len(mat_lines); B_rows = mat_lines[0][1]
        idx_lines = idx_lines_raw[:B_cols]
        coo = [(row, col, (row * 37 + col * 13 + 1) % 7 + 1)
               for col in range(B_cols) for row in idx_lines[col]]
        coo.sort()
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        cur_row = 0; row_nnz = 0
        for (r, c, v) in coo:
            vv = int_to_fp16_bits(v)
            while cur_row < r:
                row_desc.append(b_desc(offset, row_nnz))
                offset += row_nnz; cur_row += 1; row_nnz = 0
            col_arr.append(c); val_arr.append(vv); row_nnz += 1
        while cur_row < B_rows:
            row_desc.append(b_desc(offset, row_nnz))
            offset += row_nnz; cur_row += 1; row_nnz = 0
        return row_desc, col_arr, val_arr, offset, B_rows, B_cols


# =========================================================================
# Load-balanced A partition (same as test_comp.py partition_a)
# =========================================================================
def partition_a(Ad, Ac, Av, M, n_pe, Bd=None):
    row_tasks = []
    for ri in range(M):
        nnza = a_desc_nnz(Ad[ri]); st = a_desc_off(Ad[ri])
        if Bd is not None:
            t = sum((b_desc_nnz(Bd[Ac[st + ti] & 0xFFFF]) + 15) // 16 for ti in range(nnza))
        else:
            t = nnza
        row_tasks.append(t)
    total = sum(row_tasks); avg = total / n_pe if n_pe > 0 else 0
    pe_tasks = [0] * n_pe; assignment = [0] * M; remaining = []; cur_pe = 0
    for ri in range(M):
        if cur_pe < n_pe and pe_tasks[cur_pe] + row_tasks[ri] > avg: cur_pe += 1
        if cur_pe < n_pe:
            assignment[ri] = cur_pe; pe_tasks[cur_pe] += row_tasks[ri]
        else:
            remaining.append(ri)
    for ri in remaining:
        pid = min(range(n_pe), key=lambda p: pe_tasks[p])
        assignment[ri] = pid; pe_tasks[pid] += row_tasks[ri]
    pe_desc = [[] for _ in range(n_pe)]
    pe_val  = [[] for _ in range(n_pe)]
    pe_col  = [[] for _ in range(n_pe)]
    for ri in range(M):
        pid = assignment[ri]
        gr = a_desc_crow(Ad[ri]); nnza = a_desc_nnz(Ad[ri]); gs = a_desc_off(Ad[ri])
        ls = len(pe_val[pid])
        for t in range(nnza):
            pe_val[pid].append(Av[gs + t]); pe_col[pid].append(Ac[gs + t] & 0xFFFF)
        pe_desc[pid].append(a_desc(ls, nnza, gr))
    return pe_desc, pe_val, pe_col


# =========================================================================
# Golden C
# =========================================================================
def compute_golden_c(A_desc, A_col, A_val, B_desc, B_col, B_val, M, N, K):
    def fp16b(b): return struct.unpack('<e', struct.pack('<H', int(b) & 0xFFFF))[0]
    def tof(v):  return struct.unpack('<e', struct.pack('<e', float(v)))[0]
    C = [[0.0] * MAX_N for _ in range(MAX_N)]
    for ri in range(M):
        gid = a_desc_crow(A_desc[ri]); nnza = a_desc_nnz(A_desc[ri]); st = a_desc_off(A_desc[ri])
        for t in range(nnza):
            k = A_col[st + t] & 0xFFFF; a = fp16b(A_val[st + t])
            bn = b_desc_nnz(B_desc[k]); bs = b_desc_off(B_desc[k])
            for u in range(bn):
                j = B_col[bs + u] & 0xFFFF; b = fp16b(B_val[bs + u])
                C[gid][j] = tof(C[gid][j] + tof(a * b))
    gf = [[0.0] * N for _ in range(M)]; gv = {}
    for ri in range(M):
        gid = a_desc_crow(A_desc[ri])
        for j in range(N):
            v = tof(C[gid][j])
            if v != 0.0: gv[gid * C_ROW_STRIDE + j] = v
            gf[gid][j] = v
    return gv, gf


def count_total_macs(Ad, Ac, Bd, M):
    total = 0
    for ri in range(M):
        nnza = a_desc_nnz(Ad[ri]); st = a_desc_off(Ad[ri])
        for t in range(nnza): total += b_desc_nnz(Bd[Ac[st + t] & 0xFFFF])
    return total


# =========================================================================
# DDR host write
# =========================================================================
async def ddr_write(dut, addr, data16):
    dut.host_wr_addr.value = addr; dut.host_wr_data.value = data16
    dut.host_wr_en.value = 1
    await RisingEdge(dut.aclk); dut.host_wr_en.value = 0


# =========================================================================
# Write partitioned A to DDR
#
# DDR layout (16-bit word addresses):
#   0x0000000: header[0..5]  = {row_counts[0:2], nnz_counts[0:2]}  (6 words)
#   0x0000100: A_desc_flat   = PE0's descs + PE1's descs + PE2's descs
#   0x0002000: A_col_flat    = PE0's cols + PE1's cols + PE2's cols
#   0x0018000: A_val_flat    = PE0's vals + PE1's vals + PE2's vals
#   0x0200000: B_desc
#   0x0210000: B_col / B_val
# =========================================================================
async def load_a_to_ddr(dut, pe_desc, pe_val, pe_col, row_counts):
    n_pe = len(pe_desc)
    nnz_counts = [len(pe_val[p]) for p in range(n_pe)]

    dut._log.info("  Loading partitioned A to DDR: rows=%s nnz=%s", row_counts, nnz_counts)

    # --- Header ---
    for p in range(n_pe):
        await ddr_write(dut, p, row_counts[p] & 0xFFFF)
        await ddr_write(dut, n_pe + p, nnz_counts[p] & 0xFFFF)

    # --- A_desc ---
    off = 0x100
    for p in range(n_pe):
        for d in pe_desc[p]:
            for w in range(4):
                await ddr_write(dut, off, (d >> (w * 16)) & 0xFFFF)
                off += 1

    # --- A_col ---
    off = 0x2000
    for p in range(n_pe):
        for v in pe_col[p]:
            await ddr_write(dut, off, v & 0xFFFF); off += 1

    # --- A_val ---
    off = 0x18000
    for p in range(n_pe):
        for v in pe_val[p]:
            await ddr_write(dut, off, v & 0xFFFF); off += 1


async def load_b_to_ddr(dut, Bd, Bc, Bv, K):
    dut._log.info("  Loading B to DDR: %d rows, %d nnz", K, len(Bc))
    for k in range(K):
        d = Bd[k]
        await ddr_write(dut, 0x0200000 + k * 2, (d) & 0xFFFF)
        await ddr_write(dut, 0x0200000 + k * 2 + 1, (d >> 16) & 0xFFFF)
    for i, v in enumerate(Bc):
        await ddr_write(dut, 0x0210000 + i, v & 0xFFFF)


# =========================================================================
# Read PE C banks (same logic as test_comp.py cluster test)
# =========================================================================
async def read_c_pe_bank(dut, n_pe, row_counts, N):
    C_RD_ADDR_W = 7 + 5; MAX_DIM_BITS = 10
    cp = {}; ngroups = (N + 15) // 16
    for pid in range(n_pe):
        for local_row in range(row_counts[pid]):
            for g in range(ngroups):
                addr = (local_row << 5) | g
                dut.c_rd_addr.value = addr << (pid * C_RD_ADDR_W)
                await RisingEdge(dut.aclk); await RisingEdge(dut.aclk)
                row_raw = int(dut.c_rd_row.value)
                r = (row_raw >> (pid * MAX_DIM_BITS)) & ((1 << MAX_DIM_BITS) - 1)
                vals = int(dut.c_rd_data.value)
                for b in range(16):
                    fp16 = (vals >> (pid * 16 * 16 + b * 16)) & 0xFFFF
                    if fp16 == 0: continue
                    j = g * 16 + b
                    if j < N: cp[r * C_ROW_STRIDE + j] = fp16_from_bits(fp16)
    return cp


# =========================================================================
# Verify (±4 ULP)
# =========================================================================
def fp16_ulp_diff(a, b):
    def bits(v):
        bx = struct.unpack('<H', struct.pack('<e', float(v)))[0]
        return bx ^ 0x8000 if bx & 0x8000 else bx
    return abs(bits(a) - bits(b))

def verify(dut, M, N, Ad, gf, cp):
    ULP_TOL = 4; e = 0; nz_ok = 0; z_ok = 0
    for ri in range(M):
        gid = a_desc_crow(Ad[ri]); b = gid * C_ROW_STRIDE
        for j in range(N):
            exp = float(gf[gid][j]); act = float(cp.get(b + j, 0.0))
            if act != exp:
                ulp = fp16_ulp_diff(act, exp)
                if ulp <= ULP_TOL:
                    if exp != 0.0: nz_ok += 1
                    else: z_ok += 1
                else:
                    if e < 5:
                        dut._log.error("C[%d][%d]: got %g, exp %g (ULP=%d)", gid, j, act, exp, ulp); e += 1
                    elif e == 5: dut._log.error("... (further errors suppressed)"); e += 1
            else:
                if exp != 0.0: nz_ok += 1
                else: z_ok += 1
    return e, nz_ok, z_ok


# =========================================================================
# Reset
# =========================================================================
async def reset(dut):
    dut.aresetn.value = 0; dut.start.value = 0
    dut.M.value = 0; dut.K.value = 0; dut.N.value = 0
    dut.op_mode.value = 0; dut.op_sub.value = 0
    dut.c_rd_addr.value = 0
    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 10); dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)


# =========================================================================
# Main test
# =========================================================================
@cocotb.test()
async def test_axi_case1(dut):
    """AXI DDR-direct + load-balanced A partition: A(251,257) × B(257,121)"""
    # ---- 1. Load ----
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    n_pe = 3

    # ---- 2. Partition A (load-balanced) ----
    pe_desc, pe_val, pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]

    # ---- 3. Golden ----
    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    dut._log.info("=" * 70)
    dut._log.info("AXI ACCELERATOR TEST (load-balanced): A(%d,%d)×B(%d,%d)→C(%d,%d)",
                  M, K, K2, N, M, N)
    dut._log.info("A: %d rows, %d nnz", M, An)
    dut._log.info("B: %d rows, %d nnz", K2, Bn)
    dut._log.info("Golden C: %d non-zero entries", len(gv))
    dut._log.info("Load-balanced row distribution: %s",
                  "  ".join(f"PE{p}={row_counts[p]}" for p in range(n_pe)))

    # ---- 4. Reset ----
    await reset(dut)

    # ---- 5. Write partitioned A + B to DDR ----
    await load_a_to_ddr(dut, pe_desc, pe_val, pe_col, row_counts)
    await load_b_to_ddr(dut, Bd, Bc, Bv, K)

    # ---- 6. Launch ----
    dut.M.value = M; dut.K.value = K; dut.N.value = N
    dut.op_mode.value = 0; dut.op_sub.value = 0
    await RisingEdge(dut.aclk)
    dut.start.value = 1; await RisingEdge(dut.aclk); dut.start.value = 0
    dut._log.info("Start pulse issued, waiting for done...")

    # ---- 7. Wait for done ----
    cyc = 0
    while True:
        await RisingEdge(dut.aclk); cyc += 1
        if cyc % 100000 == 0:
            try: dut._log.info("  [cyc %d] FSM state=%d", cyc, int(dut.u_accel.state.value))
            except Exception: pass
        if int(dut.done.value): dut._log.info("  [cyc %d] DONE!", cyc); break
        if cyc > 20000000: dut._log.error("TIMEOUT at %d", cyc); break

    # ---- 8. Read C ----
    dut._log.info("Reading C from PE banks...")
    await RisingEdge(dut.aclk)
    cp = await read_c_pe_bank(dut, n_pe, row_counts, N)
    dut._log.info("C entries: %d", len(cp))

    # ---- 9. Verify ----
    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("VERIFY: PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("VERIFY: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches"

    # ---- 10. Stats ----
    total_macs = count_total_macs(Ad, Ac, Bd, M)
    dut._log.info("=" * 70)
    dut._log.info("STATS: N_PE=%d  cycles=%d  MAC ops=%d  ops/cyc=%.2f",
                  n_pe, cyc, total_macs, total_macs / cyc if cyc else 0)
    dut._log.info("AXI ACCELERATOR TEST PASSED")
