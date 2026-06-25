#!/usr/bin/env python3
"""
Competition test case: A(32,317) × B(317,6), 30% sparsity.
Loads TC1_RAW pattern 1, converts to compact row-desc, runs PE, collects stats.

Hardware online generation: PE reads A_col_buf + B_desc_buf on-chip and emits
4-wide groups each cycle.  No pre-computed instruction buffer required.
"""
import os, random, struct, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N=512; C_ROW_STRIDE=MAX_N

def int_to_fp16_bits(v):
    """Convert integer v to its FP16 bit pattern (uint16 value). Uses struct '<e' (Python 3.6+)."""
    return int.from_bytes(struct.pack('<e', float(v)), 'little')

def fp32_from_bits(bits):
    """Interpret a 32-bit integer as an IEEE 754 FP32 float."""
    return struct.unpack('<f', struct.pack('<I', bits & 0xFFFFFFFF))[0]

def fp16_from_bits(bits):
    """Interpret a 16-bit integer as an IEEE 754 FP16 float."""
    return struct.unpack('<e', struct.pack('<H', bits & 0xFFFF))[0]

#-------------------------------------------------------------------------
# Load competition matrix files
#-------------------------------------------------------------------------
def load_comp_matrix(index_file, matrix_file, is_B=False):
    """Load competition format matrices, return compact row-desc.

    A (CSR): index_file rows = row indices, matrix_file = (row_weight, cols)
    B (CSC): index_file rows = col entries, matrix_file = (col_weight, rows)

    For B, we need to transpose from CSC to CSR (row-major) for the PE.

    A row descriptor (64-bit):
      [63:32] a_off   — start index into A_col/A_val buffers
      [31:16] a_nnz   — number of A nonzeros in this row
      [15: 0] c_row   — global C output row id (host readback only)
    """
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'test_case_for_reference', 'TC1_RAW')
    idx_path = os.path.join(base, index_file)
    mat_path = os.path.join(base, matrix_file)

    with open(mat_path) as f:
        mat_lines = [list(map(int, l.split())) for l in f if l.strip()]
    with open(idx_path) as f:
        idx_lines_raw = [list(map(int, l.split())) for l in f]

    if not is_B:
        # A: CSR format → {a_off, a_nnz, c_row}
        rows = len(mat_lines)
        idx_lines = idx_lines_raw[:rows]
        K = mat_lines[0][1]
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        for r in range(rows):
            nnz = mat_lines[r][0]
            cols = idx_lines[r] if nnz > 0 else []
            assert len(cols) == nnz, f"Row {r}: expected {nnz} cols, got {len(cols)}"
            for c in cols:
                v = (r * 37 + c * 13 + 1) % 7 + 1
                col_arr.append(c); val_arr.append(int_to_fp16_bits(v))
            row_desc.append((offset << 32) | (nnz << 16) | r)
            offset += nnz
        return row_desc, col_arr, val_arr, offset, rows, K

    else:
        # B: CSC format → transposed to CSR
        B_cols = len(mat_lines)
        idx_lines = idx_lines_raw[:B_cols]
        B_rows = mat_lines[0][1]
        coo = []
        for col in range(B_cols):
            rows_in_col = idx_lines[col]
            for row in rows_in_col:
                v = (row * 37 + col * 13 + 1) % 7 + 1
                coo.append((row, col, int_to_fp16_bits(v)))
        coo.sort()
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        cur_row = 0; row_nnz = 0
        for (r, c, v) in coo:
            while cur_row < r:
                row_desc.append((offset << 32) | row_nnz)
                offset += row_nnz; cur_row += 1; row_nnz = 0
            col_arr.append(c); val_arr.append(v); row_nnz += 1
        while cur_row < B_rows:
            row_desc.append((offset << 32) | row_nnz)
            offset += row_nnz; cur_row += 1; row_nnz = 0
        return row_desc, col_arr, val_arr, offset, B_rows, B_cols

def align_b_4wide(Bd, Bc, Bv):
    """Lay out B elements in 4-bank storage with per-row rotation.

    Row r starts at absolute position b_off where b_off % 4 == r % 4.
    This distributes the tail element of partial rows evenly across 4 lanes.
    The hardware generator uses lane = (b_off + u) % 4 (general form).
    """
    new_Bc = []; new_Bv = []; new_Bd = []; new_off = 0
    for r, d in enumerate(Bd):
        start  = (d >> 32) & 0xFFFFFFFF
        nnz    = d & 0xFFFF
        target_mod = r % 4
        gap = (target_mod - new_off % 4) % 4
        for _ in range(gap):
            new_Bc.append(0); new_Bv.append(0)
        new_off += gap
        new_Bd.append((new_off << 32) | nnz)
        for t in range(nnz):
            new_Bc.append(Bc[start + t])
            new_Bv.append(Bv[start + t])
        new_off += nnz
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
    """FP16 × FP16 → FP16 accumulate golden C.

    A_val and B_val contain FP16 bit patterns (uint16).
    Products and accumulation are performed in FP16 (matching hardware).
    Returns (golden_dict, golden_f16_2d) where values are Python floats
    derived from FP16 bit patterns.
    """
    import struct as _struct

    def fp16b(bits):
        return _struct.unpack('<e', _struct.pack('<H', int(bits) & 0xFFFF))[0]

    def to_fp16(v):
        """Round float v to the nearest FP16 value."""
        return _struct.unpack('<e', _struct.pack('<e', float(v)))[0]

    # Accumulate using FP16 arithmetic (simulate hardware behaviour)
    C = [[0.0] * MAX_N for _ in range(MAX_N)]
    for ri in range(M):
        gid  = A_desc[ri] & 0xFFFF
        nnza = (A_desc[ri] >> 16) & 0xFFFF
        st   = (A_desc[ri] >> 32) & 0xFFFFFFFF
        for t in range(nnza):
            k  = A_col[st + t] & 0xFFFF
            a  = fp16b(A_val[st + t])
            bn = B_desc[k] & 0xFFFF
            bs = (B_desc[k] >> 32) & 0xFFFFFFFF
            for u in range(bn):
                j = B_col[bs + u] & 0xFFFF
                b = fp16b(B_val[bs + u])
                prod = to_fp16(a * b)          # FP16 multiply
                C[gid][j] = to_fp16(C[gid][j] + prod)  # FP16 accumulate

    golden_f16 = [[0.0] * N for _ in range(M)]
    golden = {}
    for ri in range(M):
        gid = A_desc[ri] & 0xFFFF
        for j in range(N):
            addr = gid * C_ROW_STRIDE + j
            v = to_fp16(C[gid][j])
            if v != 0.0:
                golden[addr] = v
            golden_f16[gid][j] = v
    return golden, golden_f16

#-------------------------------------------------------------------------
# PE load helpers
#-------------------------------------------------------------------------
async def LRD(dut, Ad):
    """Load A row descriptors {a_off, a_nnz, c_row} directly from Ad."""
    for i, d in enumerate(Ad):
        dut.a_desc_we.value=1; dut.a_desc_waddr.value=i; dut.a_desc_wdata.value=d
        await RisingEdge(dut.aclk)
    dut.a_desc_we.value=0

async def LA_val(dut, Av):
    for i, v in enumerate(Av):
        dut.a_val_we.value=1; dut.a_val_waddr.value=i; dut.a_val_wdata.value=v
        await RisingEdge(dut.aclk)
    dut.a_val_we.value=0

async def LAcol(dut, Ac):
    """Load A column index buffer (k_idx per A nonzero)."""
    for i, v in enumerate(Ac):
        dut.a_col_we.value=1; dut.a_col_waddr.value=i; dut.a_col_wdata.value=v & 0xFFFF
        await RisingEdge(dut.aclk)
    dut.a_col_we.value=0

async def LBdata(dut, Bc, Bv):
    for i, v in enumerate(Bc):
        dut.b_col_we.value=1; dut.b_col_waddr.value=i; dut.b_col_wdata.value=v
        await RisingEdge(dut.aclk)
    dut.b_col_we.value=0
    for i, v in enumerate(Bv):
        dut.b_val_we.value=1; dut.b_val_waddr.value=i; dut.b_val_wdata.value=v
        await RisingEdge(dut.aclk)
    dut.b_val_we.value=0

async def LBdesc(dut, Bd):
    """Load B row descriptors {b_off[31:0], b_nnz[15:0]} into B_desc_buf."""
    for k, d in enumerate(Bd):
        dut.b_desc_we.value=1; dut.b_desc_waddr.value=k; dut.b_desc_wdata.value=d
        await RisingEdge(dut.aclk)
    dut.b_desc_we.value=0

async def rst(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,unit='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value=0; dut.row_count.value=0
    dut.c_rd_en.value=0; dut.c_rd_addr.value=0
    dut.a_desc_we.value=0; dut.a_val_we.value=0; dut.a_col_we.value=0
    dut.b_col_we.value=0; dut.b_val_we.value=0; dut.b_desc_we.value=0

async def read_c_buffer(dut, Ad, N):
    """Read internal per-PE C buffer after done. Returns {global_flat_addr: fp32_float}.

    c_rd_addr = {local_row_idx[7:0], col[8:0]}  (17-bit).
    Registered read: 1-cycle latency.
    """
    cp = {}
    dut.c_rd_en.value = 1
    prev_base = 0; prev_col = -1; first = True
    for local_idx, rd in enumerate(Ad):
        c_row = rd & 0xFFFF
        base  = c_row * C_ROW_STRIDE
        for col in range(N):
            dut.c_rd_addr.value = (local_idx << _COL_W) | col
            await RisingEdge(dut.aclk)
            if not first:
                try:
                    val = fp16_from_bits(int(dut.c_rd_data.value))
                except ValueError:
                    val = 0.0
                if val != 0.0:
                    cp[prev_base + prev_col] = val
            first = False
            prev_base = base; prev_col = col
    await RisingEdge(dut.aclk)
    try:
        val = fp16_from_bits(int(dut.c_rd_data.value))
    except ValueError:
        val = 0.0
    if val != 0.0:
        cp[prev_base + prev_col] = val
    dut.c_rd_en.value = 0
    return cp

async def run_pe(dut, rc, Ad, N, to=10000000):
    """Run PE for rc rows, collect stats, then read C buffer."""
    dut.row_count.value = rc
    dut.start.value=1; await RisingEdge(dut.aclk); dut.start.value=0
    dc=0; lane_busy=[0,0,0,0]; rmw_busy=[0,0,0,0]
    mac_sig    = dut.u_pe.mac_lane_valid
    rmw_sigs_0 = [dut.u_pe.u_row_acc_0.u_bank0.rmw_busy,
                  dut.u_pe.u_row_acc_0.u_bank1.rmw_busy,
                  dut.u_pe.u_row_acc_0.u_bank2.rmw_busy,
                  dut.u_pe.u_row_acc_0.u_bank3.rmw_busy]
    rmw_sigs_1 = [dut.u_pe.u_row_acc_1.u_bank0.rmw_busy,
                  dut.u_pe.u_row_acc_1.u_bank1.rmw_busy,
                  dut.u_pe.u_row_acc_1.u_bank2.rmw_busy,
                  dut.u_pe.u_row_acc_1.u_bank3.rmw_busy]
    for cy in range(to):
        await RisingEdge(dut.aclk)
        mlv = int(mac_sig.value)
        for i in range(4):
            if (mlv >> i) & 1: lane_busy[i] += 1
            if int(rmw_sigs_0[i].value) or int(rmw_sigs_1[i].value): rmw_busy[i] += 1
        if int(dut.done.value): dc=cy; break
    else: assert False, f"timeout {to}"
    await ClockCycles(dut.aclk, 10)
    cp = await read_c_buffer(dut, Ad, N)
    return cp, dc, lane_busy, rmw_busy

def fp16_ulp_diff(a, b):
    """Return the absolute ULP difference between two FP16 values."""
    import struct as _s
    def bits(v):
        b = _s.unpack('<H', _s.pack('<e', float(v)))[0]
        return b ^ 0x8000 if b & 0x8000 else b  # signed-magnitude → biased
    return abs(bits(a) - bits(b))

def verify(dut, M, N, Ad, gf, cp):
    """Compare FP16 C buffer against golden, tolerating ±4 ULP rounding differences.

    FP16 accumulation is non-associative; carry-buffer task packing changes the
    product ordering vs. the sequential golden, producing small (≤4 ULP) FP16
    rounding differences in a small fraction of entries.
    """
    ULP_TOL = 4
    e=0;nz_ok=0;z_ok=0
    for ri in range(M):
        gid=Ad[ri]&0xFFFF;b=gid*C_ROW_STRIDE
        for j in range(N):
            exp=float(gf[gid][j]);act=float(cp.get(b+j,0.0))
            if act != exp:
                ulp = fp16_ulp_diff(act, exp)
                if ulp <= ULP_TOL:
                    # Acceptable rounding difference — count as correct
                    if exp != 0.0: nz_ok += 1
                    else:          z_ok  += 1
                else:
                    if e<5:dut._log.error("C[%d][%d]: got %g, exp %g (diff=%g, ULP=%d)",gid,j,act,exp,exp-act,ulp);e+=1
                    elif e==5:dut._log.error("... (further errors suppressed)");e+=1
            else:
                if exp!=0.0:nz_ok+=1
                else:z_ok+=1
    return e,nz_ok,z_ok

#=========================================================================
@cocotb.test()
async def test_comp_case1_p0(dut):
    """Competition Case1 Pattern0: A(250,257) × B(257,121)"""
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

    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LRD(dut, Ad)
    await LA_val(dut, Av)
    await LAcol(dut, Ac)
    await LBdata(dut, Bc, Bv)
    await LBdesc(dut, Bd)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, Ad, N, to=50000000)

    dut._log.info("PE done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    total = M * N
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d", total, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

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

    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LRD(dut, Ad)
    await LA_val(dut, Av)
    await LAcol(dut, Ac)
    await LBdata(dut, Bc, Bv)
    await LBdesc(dut, Bd)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, Ad, N, to=10000000)

    dut._log.info("PE done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    total = M * N
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d", total, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

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
# Width constants must stay in sync with defines.vh:
_A_ROW_ADDR_W  = 8    # A_ROW_ADDR_BITS
_A_NNZ_ADDR_W  = 14   # A_NNZ_ADDR_BITS
_DATA_W        = 16   # DATA_WIDTH (FP16 input)
_C_DATA_W      = 16   # C buffer output width (FP16)
_B_NNZ_ADDR_W  = 17   # B_NNZ_ADDR_BITS
_COL_W         = 9    # log2(MAX_N=512), matches ACC_COL_W in pe_top.v
#=========================================================================

def partition_a(Ad, Ac, Av, M, n_pe, Bd=None):
    """Distribute A rows to PEs.  Returns (pe_desc, pe_val, pe_col).

      pe_desc[pid]  = list of 64-bit row descriptors {a_off_local, a_nnz, c_row}
      pe_val[pid]   = flat FP16 A_val array (local indexing)
      pe_col[pid]   = flat k_idx (column index) array (local indexing)

    Bd is used for load-balance estimation only (group count per A nnz).
    pe_desc format matches hardware A_row_desc_buf: [63:32]=a_off, [31:16]=a_nnz, [15:0]=c_row.
    """
    row_tasks = []
    for ri in range(M):
        nnza = (Ad[ri] >> 16) & 0xFFFF
        st   = (Ad[ri] >> 32) & 0xFFFFFFFF
        if Bd is not None:
            t = sum(((Bd[Ac[st + ti] & 0xFFFF] & 0xFFFF) + 3) // 4 for ti in range(nnza))
        else:
            t = nnza
        row_tasks.append(t)

    total = sum(row_tasks)
    avg   = total / n_pe if n_pe > 0 else 0
    pe_tasks   = [0] * n_pe
    assignment = [0] * M
    remaining  = []
    cur_pe     = 0

    for ri in range(M):
        if cur_pe < n_pe and pe_tasks[cur_pe] + row_tasks[ri] > avg:
            cur_pe += 1
        if cur_pe < n_pe:
            assignment[ri] = cur_pe
            pe_tasks[cur_pe] += row_tasks[ri]
        else:
            remaining.append(ri)
    for ri in remaining:
        pid = min(range(n_pe), key=lambda p: pe_tasks[p])
        assignment[ri] = pid
        pe_tasks[pid] += row_tasks[ri]

    pe_desc = [[] for _ in range(n_pe)]
    pe_val  = [[] for _ in range(n_pe)]
    pe_col  = [[] for _ in range(n_pe)]

    for ri in range(M):
        pid          = assignment[ri]
        global_row   = Ad[ri] & 0xFFFF
        nnza         = (Ad[ri] >> 16) & 0xFFFF
        global_start = (Ad[ri] >> 32) & 0xFFFFFFFF
        local_start  = len(pe_val[pid])

        for t in range(nnza):
            pe_val[pid].append(Av[global_start + t])
            pe_col[pid].append(Ac[global_start + t] & 0xFFFF)

        pe_desc[pid].append((local_start << 32) | (nnza << 16) | global_row)

    return pe_desc, pe_val, pe_col

async def LRD_pe(dut, pid, row_descs):
    """Load row descriptors into PE pid."""
    for i, d in enumerate(row_descs):
        dut.a_desc_we.value    = 1 << pid
        dut.a_desc_waddr.value = i << (pid * _A_ROW_ADDR_W)
        dut.a_desc_wdata.value = d << (pid * 64)
        await RisingEdge(dut.aclk)
    dut.a_desc_we.value = 0; dut.a_desc_waddr.value = 0; dut.a_desc_wdata.value = 0

async def LA_val_pe(dut, pid, Av):
    """Load A_val into PE pid."""
    for i, v in enumerate(Av):
        dut.a_val_we.value    = 1 << pid
        dut.a_val_waddr.value = i << (pid * _A_NNZ_ADDR_W)
        dut.a_val_wdata.value = v << (pid * _DATA_W)
        await RisingEdge(dut.aclk)
    dut.a_val_we.value = 0; dut.a_val_waddr.value = 0; dut.a_val_wdata.value = 0

async def LAcol_pe(dut, pid, Ac_local):
    """Load A_col (k_idx) buffer into PE pid."""
    for i, v in enumerate(Ac_local):
        dut.a_col_we.value    = 1 << pid
        dut.a_col_waddr.value = i << (pid * _A_NNZ_ADDR_W)
        dut.a_col_wdata.value = (v & 0xFFFF) << (pid * _DATA_W)
        await RisingEdge(dut.aclk)
    dut.a_col_we.value = 0; dut.a_col_waddr.value = 0; dut.a_col_wdata.value = 0

async def LBdata_cluster(dut, Bc, Bv):
    """Broadcast B col/val to all PEs."""
    for i, v in enumerate(Bc):
        dut.b_col_we.value = 1; dut.b_col_waddr.value = i; dut.b_col_wdata.value = v
        await RisingEdge(dut.aclk)
    dut.b_col_we.value = 0
    for i, v in enumerate(Bv):
        dut.b_val_we.value = 1; dut.b_val_waddr.value = i; dut.b_val_wdata.value = v
        await RisingEdge(dut.aclk)
    dut.b_val_we.value = 0

async def LBdesc_cluster(dut, Bd):
    """Broadcast B row descriptors to all PEs."""
    for k, d in enumerate(Bd):
        dut.b_desc_we.value = 1; dut.b_desc_waddr.value = k; dut.b_desc_wdata.value = d
        await RisingEdge(dut.aclk)
    dut.b_desc_we.value = 0

async def rst_cluster(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,units='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value     = 0
    dut.row_count.value = 0
    dut.c_rd_en.value   = 0; dut.c_rd_addr.value = 0
    dut.a_desc_we.value = 0; dut.a_desc_waddr.value = 0; dut.a_desc_wdata.value = 0
    dut.a_val_we.value  = 0; dut.a_val_waddr.value  = 0; dut.a_val_wdata.value  = 0
    dut.a_col_we.value  = 0; dut.a_col_waddr.value  = 0; dut.a_col_wdata.value  = 0
    dut.b_col_we.value  = 0; dut.b_val_we.value     = 0
    dut.b_desc_we.value = 0

async def read_c_buffer_pe(dut, pid, row_descs_pid, N):
    """Read internal C buffer of cluster PE pid after done.

    c_rd_addr packed bus: PE pid's field is at bits [pid*17 +: 17].
    Returns {global_flat_addr: value} for this PE's assigned rows.
    """
    cp = {}
    fp32_mask = (1 << _C_DATA_W) - 1
    prev_base = 0; prev_col = -1; first = True
    for local_idx, rd in enumerate(row_descs_pid):
        c_row = rd & 0xFFFF
        base  = c_row * C_ROW_STRIDE
        for col in range(N):
            addr = (local_idx << _COL_W) | col
            dut.c_rd_en.value   = 1 << pid
            dut.c_rd_addr.value = addr << (pid * 17)
            await RisingEdge(dut.aclk)
            if not first:
                try:
                    bits = (int(dut.c_rd_data.value) >> (pid * _C_DATA_W)) & fp32_mask
                    val  = fp16_from_bits(bits)
                except ValueError:
                    val = 0.0
                if val != 0.0:
                    cp[prev_base + prev_col] = val
            first = False
            prev_base = base; prev_col = col
    await RisingEdge(dut.aclk)
    try:
        bits = (int(dut.c_rd_data.value) >> (pid * _C_DATA_W)) & fp32_mask
        val  = fp16_from_bits(bits)
    except ValueError:
        val = 0.0
    if val != 0.0:
        cp[prev_base + prev_col] = val
    dut.c_rd_en.value   = 0
    dut.c_rd_addr.value = 0
    return cp

async def run_cluster(dut, row_counts, n_pe, pe_desc, N, to=50000000):
    """Start all n_pe PEs, wait for done, collect stats, read C buffers."""
    rc_packed = sum(row_counts[p] << (p * 16) for p in range(n_pe))
    dut.row_count.value = rc_packed
    dut.start.value=1; await RisingEdge(dut.aclk); dut.start.value=0
    dc = 0

    mac_sigs = [dut.u_cluster.gen_pe[pid].u_pe.mac_lane_valid for pid in range(n_pe)]
    rmw_acc0 = [[dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.u_bank0.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.u_bank1.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.u_bank2.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.u_bank3.rmw_busy]
                for pid in range(n_pe)]
    rmw_acc1 = [[dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.u_bank0.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.u_bank1.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.u_bank2.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.u_bank3.rmw_busy]
                for pid in range(n_pe)]

    lane_busy = [0] * 4
    rmw_busy  = [0] * 4

    for cy in range(to):
        await RisingEdge(dut.aclk)
        for pid in range(n_pe):
            mlv = int(mac_sigs[pid].value)
            for i in range(4):
                if (mlv >> i) & 1: lane_busy[i] += 1
            for i in range(4):
                if int(rmw_acc0[pid][i].value) or int(rmw_acc1[pid][i].value):
                    rmw_busy[i] += 1
        if int(dut.done.value): dc=cy; break
    else:
        assert False, f"cluster timeout at {to} cycles"

    await ClockCycles(dut.aclk, 10)
    cp = {}
    for pid in range(n_pe):
        pe_cp = await read_c_buffer_pe(dut, pid, pe_desc[pid], N)
        cp.update(pe_cp)
    return cp, dc, lane_busy, rmw_busy

#=========================================================================
@cocotb.test()
async def test_comp_case1_cluster(dut):
    """N_PE-wide cluster: A(251,257) x B(257,121), rows distributed round-robin."""
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    Bd, Bc, Bv = align_b_4wide(Bd, Bc, Bv)

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    await rst_cluster(dut)
    n_pe = int(dut.n_pe_sig.value)

    dut._log.info("=" * 70)
    dut._log.info("%d-PE CLUSTER TEST: A(%d,%d) x B(%d,%d) -> C(%d,%d)",
                  n_pe, M, K, K2, N, M, N)

    pe_desc, pe_val, pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]
    dut._log.info("Row distribution: %s", "  ".join(f"PE{p}={row_counts[p]}" for p in range(n_pe)))

    dut.M.value=M; dut.K.value=K; dut.N.value=N

    for pid in range(n_pe):
        await LRD_pe(dut, pid, pe_desc[pid])
        await LA_val_pe(dut, pid, pe_val[pid])
        await LAcol_pe(dut, pid, pe_col[pid])
    await LBdata_cluster(dut, Bc, Bv)
    await LBdesc_cluster(dut, Bd)

    dut._log.info("Starting %d-PE cluster...", n_pe)
    cp, cyc, lane_busy, rmw_busy = await run_cluster(dut, row_counts, n_pe, pe_desc, N)
    dut._log.info("Cluster done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d",
                  M*N, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / (n_pe * cyc) * 100 for lb in lane_busy]
    rmw_utils  = [rb / (n_pe * cyc) * 100 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  N_PE:                     %d", n_pe)
    dut._log.info("  Cluster wall-time cycles:  %d", cyc)
    dut._log.info("  Total MAC ops:             %d", total_macs)
    dut._log.info("  Per-lane MAC utilization (summed across %d PEs):", n_pe)
    for i in range(4):
        dut._log.info("    Lane %d: %7d PE-cycles  →  %.2f%%", i, lane_busy[i], lane_utils[i])
    dut._log.info("  Average MAC util:          %.2f%%", sum(lane_utils) / 4)
    dut._log.info("  Per-bank RMW utilization (summed across %d PEs):", n_pe)
    for i in range(4):
        dut._log.info("    Bank %d: %7d PE-cycles  →  %.2f%%", i, rmw_busy[i], rmw_utils[i])
    dut._log.info("  Average RMW util:          %.2f%%", sum(rmw_utils) / 4)
    dut._log.info("=" * 70)
    dut._log.info("%d-PE CLUSTER TEST PASSED", n_pe)
