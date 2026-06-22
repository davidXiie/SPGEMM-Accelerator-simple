#!/usr/bin/env python3
"""
Competition test case: A(32,317) × B(317,6), 30% sparsity.
Loads TC1_RAW pattern 1, converts to compact row-desc, runs PE, collects stats.
"""
import os, random, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N=512; C_ROW_STRIDE=MAX_N

#-------------------------------------------------------------------------
# Load competition matrix files
#-------------------------------------------------------------------------
def load_comp_matrix(index_file, matrix_file, is_B=False):
    """Load competition format matrices, return compact row-desc.

    A (CSR): index_file rows = row indices, matrix_file = (row_weight, cols)
    B (CSC): index_file rows = col entries, matrix_file = (col_weight, rows)

    For B, we need to transpose from CSC to CSR (row-major) for the PE.
    """
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'test_case_for_reference', 'TC1_RAW')
    idx_path = os.path.join(base, index_file)
    mat_path = os.path.join(base, matrix_file)

    with open(mat_path) as f:
        mat_lines = [list(map(int, l.split())) for l in f if l.strip()]
    with open(idx_path) as f:
        # Keep ALL lines (empty B-cols are truly empty; empty A-rows use "0" sentinel).
        # Slice to mat_lines count so trailing newlines don't inflate row count.
        idx_lines_raw = [list(map(int, l.split())) for l in f]

    if not is_B:
        # A: CSR format → compact row-desc
        rows = len(mat_lines)  # authoritative count from matrix file
        idx_lines = idx_lines_raw[:rows]
        K = mat_lines[0][1]  # total columns
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        for r in range(rows):
            nnz = mat_lines[r][0]
            cols = idx_lines[r] if nnz > 0 else []  # A_0 uses "0" sentinel for empty rows
            assert len(cols) == nnz, f"Row {r}: expected {nnz} cols, got {len(cols)}"
            for c in cols:
                v = (r * 37 + c * 13 + 1) % 7 + 1  # integer [1..7]
                col_arr.append(c); val_arr.append(v)
            desc = (offset << 32) | (nnz << 16) | r
            row_desc.append(desc); offset += nnz
        return row_desc, col_arr, val_arr, offset, rows, K

    else:
        # B: CSC format → transposed to CSR for PE
        B_cols = len(mat_lines)  # authoritative count from matrix file
        idx_lines = idx_lines_raw[:B_cols]  # slice; empty lines preserved as []
        B_rows = mat_lines[0][1]  # number of B rows (K)
        # Build COO then convert to CSR
        coo = []
        for col in range(B_cols):
            rows_in_col = idx_lines[col]
            for row in rows_in_col:
                v = (row * 37 + col * 13 + 1) % 7 + 1
                coo.append((row, col, v))
        coo.sort()
        # Build CSR from COO
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        cur_row = 0; row_nnz = 0; row_cols = []
        for (r, c, v) in coo:
            while cur_row < r:
                desc = (offset << 32) | (0 << 16) | row_nnz  # B descriptor
                row_desc.append(desc); offset += row_nnz
                cur_row += 1; row_nnz = 0
            col_arr.append(c); val_arr.append(v); row_nnz += 1
        while cur_row < B_rows:
            desc = (offset << 32) | (0 << 16) | row_nnz
            row_desc.append(desc); offset += row_nnz
            cur_row += 1; row_nnz = 0
        return row_desc, col_arr, val_arr, offset, B_rows, B_cols

def align_b_4wide(Bd, Bc, Bv):
    """Pad each B row to ceil(nnz/4)*4 elements so b_ptr is always 4-aligned.
    Padding elements use col=0, val=0 (harmless: products are 0 and lane_valid
    masks them out).  Must be called before compute_golden_c and hardware load.
    """
    new_Bc = []; new_Bv = []; new_Bd = []; new_off = 0
    for d in Bd:
        start  = (d >> 32) & 0xFFFFFFFF
        nnz    = d & 0xFFFF
        new_Bd.append((new_off << 32) | nnz)
        for t in range(nnz):
            new_Bc.append(Bc[start + t])
            new_Bv.append(Bv[start + t])
        padded = ((nnz + 3) // 4) * 4
        for _ in range(padded - nnz):
            new_Bc.append(0); new_Bv.append(0)
        new_off += padded
    return new_Bd, new_Bc, new_Bv

def count_total_macs(Ad, Ac, Bd, M):
    """Exact count of MAC operations: for each A[i,k] nonzero, add B row k's nnz."""
    total = 0
    for ri in range(M):
        nnza = (Ad[ri] >> 16) & 0xFFFF
        st   = (Ad[ri] >> 32) & 0xFFFFFFFF
        for t in range(nnza):
            k = Ac[st + t] & 0xFFFF
            total += Bd[k] & 0xFFFF
    return total

def compute_golden_c(A_desc, A_col, A_val, B_desc, B_col, B_val, M, N, K):
    """Integer golden C = A × B."""
    C = [[0] * MAX_N for _ in range(MAX_N)]
    for ri in range(M):
        gid = A_desc[ri] & 0xFFFF
        nnza = (A_desc[ri] >> 16) & 0xFFFF
        st = (A_desc[ri] >> 32) & 0xFFFFFFFF
        for t in range(nnza):
            k = A_col[st + t] & 0xFFFF
            a = A_val[st + t]
            bn = B_desc[k] & 0xFFFF
            bs = (B_desc[k] >> 32) & 0xFFFFFFFF
            for u in range(bn):
                j = B_col[bs + u] & 0xFFFF
                b = B_val[bs + u]
                C[gid][j] += a * b
    golden_f32 = [[0.0] * N for _ in range(M)]
    golden = {}
    for ri in range(M):
        gid = A_desc[ri] & 0xFFFF
        for j in range(N):
            addr = gid * C_ROW_STRIDE + j
            v = C[gid][j]
            if v != 0: golden[addr] = v
            golden_f32[gid][j] = float(v)
    return golden, golden_f32

#-------------------------------------------------------------------------
# PE helpers
#-------------------------------------------------------------------------
async def LAd(dut, Ad, Ac, Av):
    for i,d in enumerate(Ad): dut.a_desc_we.value=1;dut.a_desc_waddr.value=i;dut.a_desc_wdata.value=d;await RisingEdge(dut.aclk)
    dut.a_desc_we.value=0
    for i,v in enumerate(Ac): dut.a_col_we.value=1;dut.a_col_waddr.value=i;dut.a_col_wdata.value=v;await RisingEdge(dut.aclk)
    dut.a_col_we.value=0
    for i,v in enumerate(Av): dut.a_val_we.value=1;dut.a_val_waddr.value=i;dut.a_val_wdata.value=v;await RisingEdge(dut.aclk)
    dut.a_val_we.value=0

async def LBd(dut, Bd, Bc, Bv):
    for i,d in enumerate(Bd): dut.b_desc_we.value=1;dut.b_desc_waddr.value=i;dut.b_desc_wdata.value=d;await RisingEdge(dut.aclk)
    dut.b_desc_we.value=0
    for i,v in enumerate(Bc): dut.b_col_we.value=1;dut.b_col_waddr.value=i;dut.b_col_wdata.value=v;await RisingEdge(dut.aclk)
    dut.b_col_we.value=0
    for i,v in enumerate(Bv): dut.b_val_we.value=1;dut.b_val_waddr.value=i;dut.b_val_wdata.value=v;await RisingEdge(dut.aclk)
    dut.b_val_we.value=0

async def rst(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,unit='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value=0;dut.row_count.value=0;dut.cbuf_wr_ready.value=0
    dut.a_desc_we.value=0;dut.a_col_we.value=0;dut.a_val_we.value=0
    dut.b_desc_we.value=0;dut.b_col_we.value=0;dut.b_val_we.value=0

async def run_pe(dut, rc, to=10000000):
    dut.row_count.value=rc;dut.cbuf_wr_ready.value=1;dut.start.value=1;await RisingEdge(dut.aclk);dut.start.value=0
    cp={};dc=0;lane_busy=[0,0,0,0];rmw_busy=[0,0,0,0]
    mac_sig  = dut.u_pe.mac_lane_valid
    # ping-pong: combine bank stats from both accumulators (at most one draining at a time)
    rmw_sigs_0 = [dut.u_pe.u_row_acc_0.rmw_b0, dut.u_pe.u_row_acc_0.rmw_b1,
                  dut.u_pe.u_row_acc_0.rmw_b2, dut.u_pe.u_row_acc_0.rmw_b3]
    rmw_sigs_1 = [dut.u_pe.u_row_acc_1.rmw_b0, dut.u_pe.u_row_acc_1.rmw_b1,
                  dut.u_pe.u_row_acc_1.rmw_b2, dut.u_pe.u_row_acc_1.rmw_b3]
    for cy in range(to):
        await RisingEdge(dut.aclk)
        if dut.cbuf_wr_valid.value and dut.cbuf_wr_ready.value: cp[int(dut.cbuf_wr_addr.value)]=int(dut.cbuf_wr_data.value)
        mlv = int(mac_sig.value)
        for i in range(4):
            if (mlv >> i) & 1: lane_busy[i] += 1
            if int(rmw_sigs_0[i].value) or int(rmw_sigs_1[i].value): rmw_busy[i] += 1
        if int(dut.done.value): dc=cy;break
    else: assert False,f"timeout {to}"
    await ClockCycles(dut.aclk,50)
    for _ in range(100):
        await RisingEdge(dut.aclk)
        if dut.cbuf_wr_valid.value and dut.cbuf_wr_ready.value: cp[int(dut.cbuf_wr_addr.value)]=int(dut.cbuf_wr_data.value)
    return cp,dc,lane_busy,rmw_busy

def verify(dut, M, N, Ad, gf, cp):
    e=0;nz_ok=0;z_ok=0
    for ri in range(M):
        gid=Ad[ri]&0xFFFF;b=gid*C_ROW_STRIDE
        for j in range(N):
            exp=int(gf[gid][j]);act=cp.get(b+j,0)
            if act!=exp:
                if e<5:dut._log.error("C[%d][%d]: got %d, exp %d",gid,j,act,exp);e+=1
            else:
                if exp!=0:nz_ok+=1
                else:z_ok+=1
    return e,nz_ok,z_ok

#=========================================================================
@cocotb.test()
async def test_comp_case1_p0(dut):
    """Competition Case1 Pattern0: A(250,257) × B(257,121)"""
    # Load data
    Ad, Ac, Av, An, M, K = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2, f"K mismatch: {K} vs {K2}"
    Bd, Bc, Bv = align_b_4wide(Bd, Bc, Bv)

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST: A(%d,%d) × B(%d,%d) → C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("A: %d rows, %d nnz (%.1f%% density)", M, An, 100*An/(M*K))
    dut._log.info("B: %d rows, %d nnz (%.1f%% density)", K2, Bn, 100*Bn/(K2*N))
    dut._log.info("Golden C: %d non-zero entries", len(gv))
    for ri in range(min(3, M)):
        gid = Ad[ri] & 0xFFFF
        dut._log.info("  C[%d] samples: %s", gid,
                      [int(gf[gid][j]) for j in range(min(10, N))])

    # Run
    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LAd(dut, Ad, Ac, Av)
    await LBd(dut, Bd, Bc, Bv)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, to=50000000)

    dut._log.info("PE done at cycle %d, captured %d writes", cyc, len(cp))

    # Verify
    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    total = M * N
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d", total, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

    # Stats
    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / cyc * 100 if cyc > 0 else 0.0 for lb in lane_busy]
    rmw_utils  = [rb / cyc * 100 if cyc > 0 else 0.0 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  Total cycles:      %d", cyc)
    dut._log.info("  Total MAC ops:     %d  (exact)", total_macs)
    dut._log.info("  Per-lane MAC utilization (mac_lane_valid):")
    for i in range(4):
        dut._log.info("    Lane %d: %6d busy cycles  →  %.2f%%", i, lane_busy[i], lane_utils[i])
    dut._log.info("  Average MAC util:  %.2f%%", sum(lane_utils) / 4)
    dut._log.info("  Per-bank accumulator RMW utilization (rmw_busy):")
    for i in range(4):
        dut._log.info("    Bank %d: %6d RMW cycles   →  %.2f%%", i, rmw_busy[i], rmw_utils[i])
    dut._log.info("  Average RMW util:  %.2f%%", sum(rmw_utils) / 4)
    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST PASSED")

#=========================================================================
@cocotb.test()
async def test_comp_case1_p1(dut):
    """Competition Case1 Pattern1: A(32,317) × B(317,6)"""
    # Load data
    Ad, Ac, Av, An, M, K = load_comp_matrix('A_1_Index.txt', 'A_1_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_1_Index.txt', 'B_1_Matrix.txt', True)
    assert K == K2, f"K mismatch: {K} vs {K2}"
    Bd, Bc, Bv = align_b_4wide(Bd, Bc, Bv)

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST: A(%d,%d) × B(%d,%d) → C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("A: %d rows, %d nnz (%.1f%% sparsity)", M, An, 100*An/(M*K))
    dut._log.info("B: %d rows, %d nnz (%.1f%% sparsity)", K2, Bn, 100*Bn/(K2*N))
    dut._log.info("Golden C: %d non-zero entries", len(gv))
    for ri in range(min(3, M)):
        gid = Ad[ri] & 0xFFFF
        dut._log.info("  C[%d] samples: %s", gid,
                      [int(gf[gid][j]) for j in range(min(6, N))])

    # Run
    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LAd(dut, Ad, Ac, Av)
    await LBd(dut, Bd, Bc, Bv)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, to=10000000)

    dut._log.info("PE done at cycle %d, captured %d writes", cyc, len(cp))

    # Verify
    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    total = M * N
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d", total, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

    # Stats
    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / cyc * 100 if cyc > 0 else 0.0 for lb in lane_busy]
    rmw_utils  = [rb / cyc * 100 if cyc > 0 else 0.0 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  Total cycles:      %d", cyc)
    dut._log.info("  Total MAC ops:     %d  (exact)", total_macs)
    dut._log.info("  Per-lane MAC utilization (mac_lane_valid):")
    for i in range(4):
        dut._log.info("    Lane %d: %6d busy cycles  →  %.2f%%", i, lane_busy[i], lane_utils[i])
    dut._log.info("  Average MAC util:  %.2f%%", sum(lane_utils) / 4)
    dut._log.info("  Per-bank accumulator RMW utilization (rmw_busy):")
    for i in range(4):
        dut._log.info("    Bank %d: %6d RMW cycles   →  %.2f%%", i, rmw_busy[i], rmw_utils[i])
    dut._log.info("  Average RMW util:  %.2f%%", sum(rmw_utils) / 4)
    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST PASSED")


#=========================================================================
# Cluster helpers — packed-bus interface, N_PE-parametric
#
# All per-PE signals use packed buses: field for PE i is at bus[i*W +: W].
# n_pe is read from dut.n_pe_sig so changing N_PE in defines.vh + recompile
# is all that's needed — no Python constant to update.
#
# Width constants must stay in sync with defines.vh:
_A_ROW_ADDR_W = 8    # A_ROW_ADDR_BITS
_A_NNZ_ADDR_W = 14   # A_NNZ_ADDR_BITS
_DATA_W       = 16   # DATA_WIDTH
_B_ROW_ADDR_W = 9    # B_ROW_ADDR_BITS
_B_NNZ_ADDR_W = 17   # B_NNZ_ADDR_BITS
_C_DEPTH_LOG  = 18   # C_DENSE_DEPTH_LOG
#=========================================================================

def partition_a(Ad, Ac, Av, M, n_pe, Bd=None):
    """Distribute A rows with greedy sequential + min-task fallback.

    Phase 1 (sequential): walk rows in order; advance to the next PE whenever
    adding the current row would push the PE's task count past the per-PE
    average.  Rows that exhaust all n_pe slots go to Phase 2.
    Phase 2 (greedy): each leftover row is assigned to the PE with the current
    minimum task count.

    Task count = exact MAC ops when Bd is supplied, else A-row nnz.
    Descriptors keep the original global row ID in [15:0] so each PE's
    cbuf_wr_addr computation produces correct global addresses.
    """
    # Per-row task count
    row_tasks = []
    for ri in range(M):
        nnza = (Ad[ri] >> 16) & 0xFFFF
        st   = (Ad[ri] >> 32) & 0xFFFFFFFF
        if Bd is not None:
            t = sum((Bd[Ac[st + ti] & 0xFFFF]) & 0xFFFF for ti in range(nnza))
        else:
            t = nnza
        row_tasks.append(t)

    total = sum(row_tasks)
    avg   = total / n_pe if n_pe > 0 else 0

    pe_tasks   = [0] * n_pe
    assignment = [0]  * M
    remaining  = []
    cur_pe     = 0

    # Phase 1: sequential greedy
    for ri in range(M):
        # Advance to next PE if this row would push current PE past average
        if cur_pe < n_pe and pe_tasks[cur_pe] + row_tasks[ri] > avg:
            cur_pe += 1
        if cur_pe < n_pe:
            assignment[ri] = cur_pe
            pe_tasks[cur_pe] += row_tasks[ri]
        else:
            remaining.append(ri)

    # Phase 2: each leftover row → PE with current minimum task count
    for ri in remaining:
        pid = min(range(n_pe), key=lambda p: pe_tasks[p])
        assignment[ri] = pid
        pe_tasks[pid] += row_tasks[ri]

    # Build output arrays
    pe_desc = [[] for _ in range(n_pe)]
    pe_col  = [[] for _ in range(n_pe)]
    pe_val  = [[] for _ in range(n_pe)]
    for ri in range(M):
        pid          = assignment[ri]
        global_row   = Ad[ri] & 0xFFFF
        nnz          = (Ad[ri] >> 16) & 0xFFFF
        global_start = (Ad[ri] >> 32) & 0xFFFFFFFF
        local_off    = len(pe_col[pid])
        for t in range(nnz):
            pe_col[pid].append(Ac[global_start + t])
            pe_val[pid].append(Av[global_start + t])
        pe_desc[pid].append((local_off << 32) | (nnz << 16) | global_row)

    return pe_desc, pe_col, pe_val

async def LAd_pe(dut, pid, Ad, Ac, Av):
    """Load A into PE pid via packed buses."""
    for i, d in enumerate(Ad):
        dut.a_desc_we.value    = 1 << pid
        dut.a_desc_waddr.value = i << (pid * _A_ROW_ADDR_W)
        dut.a_desc_wdata.value = d << (pid * 64)
        await RisingEdge(dut.aclk)
    dut.a_desc_we.value = 0; dut.a_desc_waddr.value = 0; dut.a_desc_wdata.value = 0
    for i, v in enumerate(Ac):
        dut.a_col_we.value    = 1 << pid
        dut.a_col_waddr.value = i << (pid * _A_NNZ_ADDR_W)
        dut.a_col_wdata.value = v << (pid * _DATA_W)
        await RisingEdge(dut.aclk)
    dut.a_col_we.value = 0; dut.a_col_waddr.value = 0; dut.a_col_wdata.value = 0
    for i, v in enumerate(Av):
        dut.a_val_we.value    = 1 << pid
        dut.a_val_waddr.value = i << (pid * _A_NNZ_ADDR_W)
        dut.a_val_wdata.value = v << (pid * _DATA_W)
        await RisingEdge(dut.aclk)
    dut.a_val_we.value = 0; dut.a_val_waddr.value = 0; dut.a_val_wdata.value = 0

async def LBd_cluster(dut, Bd, Bc, Bv):
    """Load B broadcast (single set of ports drives all PEs simultaneously)."""
    for i, d in enumerate(Bd): dut.b_desc_we.value=1; dut.b_desc_waddr.value=i; dut.b_desc_wdata.value=d; await RisingEdge(dut.aclk)
    dut.b_desc_we.value = 0
    for i, v in enumerate(Bc): dut.b_col_we.value=1; dut.b_col_waddr.value=i; dut.b_col_wdata.value=v; await RisingEdge(dut.aclk)
    dut.b_col_we.value = 0
    for i, v in enumerate(Bv): dut.b_val_we.value=1; dut.b_val_waddr.value=i; dut.b_val_wdata.value=v; await RisingEdge(dut.aclk)
    dut.b_val_we.value = 0

async def rst_cluster(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,units='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value       = 0
    dut.row_count.value   = 0
    dut.cbuf_wr_ready.value = 0
    dut.a_desc_we.value   = 0; dut.a_desc_waddr.value = 0; dut.a_desc_wdata.value = 0
    dut.a_col_we.value    = 0; dut.a_col_waddr.value  = 0; dut.a_col_wdata.value  = 0
    dut.a_val_we.value    = 0; dut.a_val_waddr.value  = 0; dut.a_val_wdata.value  = 0
    dut.b_desc_we.value   = 0; dut.b_col_we.value     = 0; dut.b_val_we.value     = 0

async def run_cluster(dut, row_counts, n_pe, to=50000000):
    """Start all n_pe PEs, collect C from packed cbuf buses, return (cp, cycles)."""
    rc_packed = sum(row_counts[p] << (p * 16) for p in range(n_pe))
    dut.row_count.value    = rc_packed
    dut.cbuf_wr_ready.value = (1 << n_pe) - 1
    dut.start.value=1; await RisingEdge(dut.aclk); dut.start.value=0
    cp = {}; dc = 0
    addr_mask = (1 << _C_DEPTH_LOG) - 1
    data_mask = (1 << _DATA_W) - 1
    for cy in range(to):
        await RisingEdge(dut.aclk)
        vb = int(dut.cbuf_wr_valid.value)
        rb = int(dut.cbuf_wr_ready.value)
        ab = int(dut.cbuf_wr_addr.value)
        db = int(dut.cbuf_wr_data.value)
        for pid in range(n_pe):
            if (vb >> pid) & 1 and (rb >> pid) & 1:
                cp[(ab >> (pid * _C_DEPTH_LOG)) & addr_mask] = (db >> (pid * _DATA_W)) & data_mask
        if int(dut.done.value): dc=cy; break
    else:
        assert False, f"cluster timeout at {to} cycles"
    await ClockCycles(dut.aclk, 50)
    for _ in range(100):
        await RisingEdge(dut.aclk)
        vb = int(dut.cbuf_wr_valid.value); rb = int(dut.cbuf_wr_ready.value)
        ab = int(dut.cbuf_wr_addr.value);  db = int(dut.cbuf_wr_data.value)
        for pid in range(n_pe):
            if (vb >> pid) & 1 and (rb >> pid) & 1:
                cp[(ab >> (pid * _C_DEPTH_LOG)) & addr_mask] = (db >> (pid * _DATA_W)) & data_mask
    return cp, dc

#=========================================================================
@cocotb.test()
async def test_comp_case1_cluster(dut):
    """N_PE-wide cluster: A(251,257) x B(257,121), rows distributed round-robin."""
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    Bd, Bc, Bv = align_b_4wide(Bd, Bc, Bv)

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    # rst_cluster starts the clock; read n_pe_sig only after clock is running
    # so the initial block in tb_pe_cluster has had time to fire.
    await rst_cluster(dut)
    n_pe = int(dut.n_pe_sig.value)

    dut._log.info("=" * 70)
    dut._log.info("%d-PE CLUSTER TEST: A(%d,%d) x B(%d,%d) -> C(%d,%d)",
                  n_pe, M, K, K2, N, M, N)

    pe_desc, pe_col, pe_val = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]
    dut._log.info("Row distribution: %s", "  ".join(f"PE{p}={row_counts[p]}" for p in range(n_pe)))

    dut.M.value=M; dut.K.value=K; dut.N.value=N

    for pid in range(n_pe):
        await LAd_pe(dut, pid, pe_desc[pid], pe_col[pid], pe_val[pid])
    await LBd_cluster(dut, Bd, Bc, Bv)

    dut._log.info("Starting %d-PE cluster...", n_pe)
    cp, cyc = await run_cluster(dut, row_counts, n_pe)
    dut._log.info("Cluster done at cycle %d, captured %d writes", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d",
                  M*N, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    single_pe_cycles = 209513
    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  N_PE:                    %d", n_pe)
    dut._log.info("  Cluster wall-time cycles: %d", cyc)
    dut._log.info("  Total MAC ops:            %d", total_macs)
    dut._log.info("  Speedup vs single PE:     %.2fx  (single=%d cycles)", single_pe_cycles/cyc, single_pe_cycles)
    dut._log.info("=" * 70)
    dut._log.info("%d-PE CLUSTER TEST PASSED", n_pe)
