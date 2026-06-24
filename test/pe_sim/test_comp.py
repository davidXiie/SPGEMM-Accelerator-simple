#!/usr/bin/env python3
"""
Competition test case: A(32,317) × B(317,6), 30% sparsity.
Loads TC1_RAW pattern 1, converts to compact row-desc, runs PE, collects stats.
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
                v = (r * 37 + c * 13 + 1) % 7 + 1  # integer [1..7] → stored as FP16 bits
                col_arr.append(c); val_arr.append(int_to_fp16_bits(v))
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
                v = (row * 37 + col * 13 + 1) % 7 + 1  # stored as FP16 bits
                coo.append((row, col, int_to_fp16_bits(v)))
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
    """Lay out B elements in 4-bank storage with per-row rotation.

    Row r starts at absolute position b_off where b_off % 4 == r % 4.
    This distributes the "extra" tail element of partial rows evenly across
    all 4 lanes rather than always burdening Lane 0.

    The instruction generator uses lane = (b_off + u) % 4 (general form) so
    no hardware change is required — element at abs_pos goes to bank abs_pos%4.
    """
    new_Bc = []; new_Bv = []; new_Bd = []; new_off = 0
    for r, d in enumerate(Bd):
        start  = (d >> 32) & 0xFFFFFFFF
        nnz    = d & 0xFFFF
        # Advance new_off until its residue == r % 4
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
    """FP16 × FP16 → FP32 accumulate golden C.

    A_val and B_val contain FP16 bit patterns (uint16).
    Products are computed as FP32 (FP16×FP16 widening) and accumulated in FP32.
    Returns (golden_dict, golden_f32_2d).
    """
    def fp16_bits_to_float(bits):
        """FP16 bit pattern → Python float via struct '<e' (half-float, Python 3.6+)."""
        return struct.unpack('<e', struct.pack('<H', int(bits) & 0xFFFF))[0]

    def to_fp32(v):
        """Round Python float to FP32 precision via struct round-trip."""
        return struct.unpack('<f', struct.pack('<f', v))[0]

    # Accumulate in FP64; all values 1-7 have exact FP16/FP32 representations,
    # products 1-49 are exact in FP32, and sums for test matrices are < 2^23.
    C = [[0.0] * MAX_N for _ in range(MAX_N)]
    for ri in range(M):
        gid  = A_desc[ri] & 0xFFFF
        nnza = (A_desc[ri] >> 16) & 0xFFFF
        st   = (A_desc[ri] >> 32) & 0xFFFFFFFF
        for t in range(nnza):
            k  = A_col[st + t] & 0xFFFF
            a  = fp16_bits_to_float(A_val[st + t])
            bn = B_desc[k] & 0xFFFF
            bs = (B_desc[k] >> 32) & 0xFFFFFFFF
            for u in range(bn):
                j = B_col[bs + u] & 0xFFFF
                b = fp16_bits_to_float(B_val[bs + u])
                # FP16 × FP16 widened to FP32, then accumulated; round product to FP32
                C[gid][j] += to_fp32(a * b)

    golden_f32 = [[0.0] * N for _ in range(M)]
    golden = {}
    for ri in range(M):
        gid = A_desc[ri] & 0xFFFF
        for j in range(N):
            addr = gid * C_ROW_STRIDE + j
            v = to_fp32(C[gid][j])   # round accumulated sum to FP32
            if v != 0.0:
                golden[addr] = v
            golden_f32[gid][j] = v
    return golden, golden_f32

#-------------------------------------------------------------------------
# Instruction schedule builder
#-------------------------------------------------------------------------
def build_instructions(Ad, Ac, Av, Bd, M):
    """Generate per-lane instruction groups (128-bit each).

    Instruction format — 4 × 32-bit per-lane words, lane k at bits [k*32+31:k*32]:
      [31:16] a_val_fp16  — FP16 A-value embedded directly
      [15: 1] b_group     — B bank address = abs_B_pos // 4; lane k reads bank k
      [    0] valid

    Lane k can ONLY carry B elements at abs positions where abs_pos % 4 == k
    (forced by the 4-bank B storage layout).  Elements from different A non-zeros
    (different B rows) are mixed greedily into one instruction to keep all 4 MACs
    busy, eliminating idle lanes at partial B-row boundaries.

    Row descriptor (64-bit):
      [63:32] instr_start  — absolute start index into instr_buf
      [31:16] instr_count  — number of instruction groups for this row
      [15: 0] c_row        — C output row id

    Bd must already be 4-align padded (align_b_4wide called first) so that
    b_off is always a multiple of 4 and lane == u % 4 holds exactly.
    """
    from collections import deque
    row_descs = []
    instrs    = []
    for ri in range(M):
        c_row       = Ad[ri] & 0xFFFF
        a_nnz       = (Ad[ri] >> 16) & 0xFFFF
        a_off       = (Ad[ri] >> 32) & 0xFFFFFFFF
        instr_start = len(instrs)

        # Per-lane queues: items = (a_val_fp16, b_group).
        # B element u of B row k_idx is at absolute position abs_pos = b_off + u.
        # lane = abs_pos % 4 (general; b_off % 4 == r % 4 after align_b_4wide rotation).
        lane_q = [deque() for _ in range(4)]
        for t in range(a_nnz):
            a_val_ptr = a_off + t
            a_fp16    = Av[a_val_ptr]
            k_idx     = Ac[a_val_ptr] & 0xFFFF
            b_nnz     = Bd[k_idx] & 0xFFFF
            b_off     = (Bd[k_idx] >> 32) & 0xFFFFFFFF
            for u in range(b_nnz):
                abs_pos = b_off + u
                lane    = abs_pos % 4
                b_grp   = abs_pos // 4
                lane_q[lane].append((a_fp16, b_grp))

        # Emit one 128-bit instruction per cycle until all lane queues are empty.
        while any(lane_q):
            word = 0
            for k in range(4):
                if lane_q[k]:
                    a_fp16, b_grp = lane_q[k].popleft()
                    word |= (a_fp16 & 0xFFFF) << (k * 32 + 16)
                    word |= (b_grp  & 0x7FFF) << (k * 32 +  1)
                    word |= 1                  << (k * 32)
            instrs.append(word)

        instr_count = len(instrs) - instr_start
        row_descs.append((instr_start << 32) | (instr_count << 16) | c_row)
    return row_descs, instrs

#-------------------------------------------------------------------------
# PE load helpers
#-------------------------------------------------------------------------
async def LRD(dut, row_descs):
    for i,d in enumerate(row_descs):
        dut.a_desc_we.value=1; dut.a_desc_waddr.value=i; dut.a_desc_wdata.value=d
        await RisingEdge(dut.aclk)
    dut.a_desc_we.value=0

async def LA_val(dut, Av):
    for i,v in enumerate(Av):
        dut.a_val_we.value=1; dut.a_val_waddr.value=i; dut.a_val_wdata.value=v
        await RisingEdge(dut.aclk)
    dut.a_val_we.value=0

async def LBdata(dut, Bc, Bv):
    for i,v in enumerate(Bc):
        dut.b_col_we.value=1; dut.b_col_waddr.value=i; dut.b_col_wdata.value=v
        await RisingEdge(dut.aclk)
    dut.b_col_we.value=0
    for i,v in enumerate(Bv):
        dut.b_val_we.value=1; dut.b_val_waddr.value=i; dut.b_val_wdata.value=v
        await RisingEdge(dut.aclk)
    dut.b_val_we.value=0

async def LInstr(dut, instrs):
    for i,instr in enumerate(instrs):
        dut.instr_we.value=1; dut.instr_waddr.value=i; dut.instr_wdata.value=instr
        await RisingEdge(dut.aclk)
    dut.instr_we.value=0

async def rst(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,unit='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value=0; dut.row_count.value=0
    dut.c_rd_en.value=0; dut.c_rd_addr.value=0
    dut.a_desc_we.value=0; dut.a_val_we.value=0
    dut.b_col_we.value=0; dut.b_val_we.value=0
    dut.instr_we.value=0

async def read_c_buffer(dut, row_descs, N):
    """Read internal per-PE C buffer after done. Returns {global_flat_addr: fp32_float}.

    c_rd_addr = {local_row_idx[7:0], col[8:0]}  (17-bit, _COL_W=9 col bits).
    Registered read: 1-cycle latency.  Pipeline: issue addr at posedge T,
    read data (from T-1) at posedge T+1.  One extra posedge drains the last entry.
    C buffer stores FP32 (32-bit); values are interpreted as IEEE 754 floats.
    """
    cp = {}
    dut.c_rd_en.value = 1
    prev_base = 0; prev_col = -1; first = True
    for local_idx, rd in enumerate(row_descs):
        c_row = rd & 0xFFFF
        base  = c_row * C_ROW_STRIDE
        for col in range(N):
            dut.c_rd_addr.value = (local_idx << _COL_W) | col
            await RisingEdge(dut.aclk)
            if not first:
                try:
                    val = fp32_from_bits(int(dut.c_rd_data.value))
                except ValueError:
                    val = 0.0
                if val != 0.0:
                    cp[prev_base + prev_col] = val
            first = False
            prev_base = base; prev_col = col
    await RisingEdge(dut.aclk)
    try:
        val = fp32_from_bits(int(dut.c_rd_data.value))
    except ValueError:
        val = 0.0
    if val != 0.0:
        cp[prev_base + prev_col] = val
    dut.c_rd_en.value = 0
    return cp

async def run_pe(dut, rc, row_descs, N, to=10000000):
    """Run PE for rc rows, collect stats, then read C buffer. Returns (cp, dc, lane_busy, rmw_busy)."""
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
    cp = await read_c_buffer(dut, row_descs, N)
    return cp, dc, lane_busy, rmw_busy

def verify(dut, M, N, Ad, gf, cp):
    """Compare FP32 C buffer against golden.

    Values 1-7 in FP16, products exact in FP32, sums < 2^23 (< 3.1M): exact match expected.
    """
    e=0;nz_ok=0;z_ok=0
    for ri in range(M):
        gid=Ad[ri]&0xFFFF;b=gid*C_ROW_STRIDE
        for j in range(N):
            exp=float(gf[gid][j]);act=float(cp.get(b+j,0.0))
            if act!=exp:
                if e<5:dut._log.error("C[%d][%d]: got %g, exp %g (diff=%g)",gid,j,act,exp,exp-act);e+=1
                elif e==5:dut._log.error("... (further errors suppressed)");e+=1
            else:
                if exp!=0.0:nz_ok+=1
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

    # Build instruction schedule
    row_descs, instrs = build_instructions(Ad, Ac, Av, Bd, M)
    dut._log.info("Instructions: %d groups total", len(instrs))

    # Load and run
    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LRD(dut, row_descs)
    await LA_val(dut, Av)
    await LBdata(dut, Bc, Bv)
    await LInstr(dut, instrs)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, row_descs, N, to=50000000)

    dut._log.info("PE done at cycle %d, C buffer entries=%d", cyc, len(cp))

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

    # Build instruction schedule
    row_descs, instrs = build_instructions(Ad, Ac, Av, Bd, M)
    dut._log.info("Instructions: %d groups total", len(instrs))

    # Load and run
    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LRD(dut, row_descs)
    await LA_val(dut, Av)
    await LBdata(dut, Bc, Bv)
    await LInstr(dut, instrs)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, row_descs, N, to=10000000)

    dut._log.info("PE done at cycle %d, C buffer entries=%d", cyc, len(cp))

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
_A_ROW_ADDR_W  = 8    # A_ROW_ADDR_BITS
_A_NNZ_ADDR_W  = 14   # A_NNZ_ADDR_BITS
_DATA_W        = 16   # DATA_WIDTH (FP16 input)
_C_DATA_W      = 32   # C buffer output width (FP32)
_B_NNZ_ADDR_W  = 17   # B_NNZ_ADDR_BITS
_C_DEPTH_LOG   = 18   # C_DENSE_DEPTH_LOG
_INSTR_ADDR_W  = 16   # INSTR_ADDR_BITS
_COL_W         = 9    # log2(MAX_N=512), matches ACC_COL_W in pe_top.v
#=========================================================================

def partition_a(Ad, Ac, Av, M, n_pe, Bd=None):
    """Distribute A rows to PEs and build per-PE instruction schedules.

    Returns (pe_desc, pe_val, pe_instrs):
      pe_desc[pid]   = list of 64-bit row descriptors {instr_start, instr_count, c_row}
      pe_val[pid]    = flat A_val array (kept for A_val_buf load, unused during exec)
      pe_instrs[pid] = flat 128-bit instruction array (per-lane format)

    Instruction format: 4 × 32-bit per-lane words (see build_instructions).
    Bd must already be 4-align padded (align_b_4wide called first).
    """
    from collections import deque

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

    pe_desc   = [[] for _ in range(n_pe)]
    pe_val    = [[] for _ in range(n_pe)]
    pe_instrs = [[] for _ in range(n_pe)]

    for ri in range(M):
        pid          = assignment[ri]
        global_row   = Ad[ri] & 0xFFFF
        nnza         = (Ad[ri] >> 16) & 0xFFFF
        global_start = (Ad[ri] >> 32) & 0xFFFFFFFF
        instr_start  = len(pe_instrs[pid])

        for t in range(nnza):
            pe_val[pid].append(Av[global_start + t])

        if Bd is not None:
            # Per-lane queues: items = (a_val_fp16, b_group).
            # abs_pos = b_off + u; lane = abs_pos % 4 (general form, works with rotation).
            lane_q = [deque() for _ in range(4)]
            for t in range(nnza):
                a_fp16 = Av[global_start + t]
                k      = Ac[global_start + t] & 0xFFFF
                b_nnz  = Bd[k] & 0xFFFF
                b_off  = (Bd[k] >> 32) & 0xFFFFFFFF
                for u in range(b_nnz):
                    abs_pos = b_off + u
                    lane    = abs_pos % 4
                    b_grp   = abs_pos // 4
                    lane_q[lane].append((a_fp16, b_grp))

            while any(lane_q):
                word = 0
                for k in range(4):
                    if lane_q[k]:
                        a_fp16, b_grp = lane_q[k].popleft()
                        word |= (a_fp16 & 0xFFFF) << (k * 32 + 16)
                        word |= (b_grp  & 0x7FFF) << (k * 32 +  1)
                        word |= 1                  << (k * 32)
                pe_instrs[pid].append(word)

        instr_count = len(pe_instrs[pid]) - instr_start
        pe_desc[pid].append((instr_start << 32) | (instr_count << 16) | global_row)

    return pe_desc, pe_val, pe_instrs

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

async def LInstr_pe(dut, pid, instrs):
    """Load instruction buffer into PE pid."""
    for i, instr in enumerate(instrs):
        dut.instr_we.value    = 1 << pid
        dut.instr_waddr.value = i << (pid * _INSTR_ADDR_W)
        dut.instr_wdata.value = instr << (pid * 128)
        await RisingEdge(dut.aclk)
    dut.instr_we.value = 0; dut.instr_waddr.value = 0; dut.instr_wdata.value = 0

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

async def rst_cluster(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,units='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value     = 0
    dut.row_count.value = 0
    dut.c_rd_en.value   = 0; dut.c_rd_addr.value = 0
    dut.a_desc_we.value = 0; dut.a_desc_waddr.value = 0; dut.a_desc_wdata.value = 0
    dut.a_val_we.value  = 0; dut.a_val_waddr.value  = 0; dut.a_val_wdata.value  = 0
    dut.instr_we.value  = 0; dut.instr_waddr.value  = 0; dut.instr_wdata.value  = 0
    dut.b_col_we.value  = 0; dut.b_val_we.value     = 0

async def read_c_buffer_pe(dut, pid, row_descs_pid, N):
    """Read internal C buffer of cluster PE pid after done.

    c_rd_addr packed bus: PE pid's field is at bits [pid*17 +: 17].
    Returns {global_flat_addr: value} for this PE's assigned rows.
    """
    cp = {}
    fp32_mask = (1 << _C_DATA_W) - 1  # 0xFFFFFFFF for 32-bit FP32
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
                    val  = fp32_from_bits(bits)
                except ValueError:
                    val = 0.0
                if val != 0.0:
                    cp[prev_base + prev_col] = val
            first = False
            prev_base = base; prev_col = col
    # drain pipeline
    await RisingEdge(dut.aclk)
    try:
        bits = (int(dut.c_rd_data.value) >> (pid * _C_DATA_W)) & fp32_mask
        val  = fp32_from_bits(bits)
    except ValueError:
        val = 0.0
    if val != 0.0:
        cp[prev_base + prev_col] = val
    dut.c_rd_en.value   = 0
    dut.c_rd_addr.value = 0
    return cp

async def run_cluster(dut, row_counts, n_pe, pe_desc, N, to=50000000):
    """Start all n_pe PEs, wait for done, collect stats, read C buffers.

    Returns (cp, cycles, lane_busy, rmw_busy) where lane_busy[i] and
    rmw_busy[i] are totals summed across all n_pe PEs for lane/bank i.
    """
    rc_packed = sum(row_counts[p] << (p * 16) for p in range(n_pe))
    dut.row_count.value = rc_packed
    dut.start.value=1; await RisingEdge(dut.aclk); dut.start.value=0
    dc = 0

    # Pre-build per-PE signal handles (accessed once outside the loop)
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

    lane_busy = [0] * 4  # summed across all PEs
    rmw_busy  = [0] * 4  # summed across all PEs

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

    # rst_cluster starts the clock; read n_pe_sig only after clock is running
    # so the initial block in tb_pe_cluster has had time to fire.
    await rst_cluster(dut)
    n_pe = int(dut.n_pe_sig.value)

    dut._log.info("=" * 70)
    dut._log.info("%d-PE CLUSTER TEST: A(%d,%d) x B(%d,%d) -> C(%d,%d)",
                  n_pe, M, K, K2, N, M, N)

    pe_desc, pe_val, pe_instrs = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]
    dut._log.info("Row distribution: %s", "  ".join(f"PE{p}={row_counts[p]}" for p in range(n_pe)))

    dut.M.value=M; dut.K.value=K; dut.N.value=N

    for pid in range(n_pe):
        await LRD_pe(dut, pid, pe_desc[pid])
        await LA_val_pe(dut, pid, pe_val[pid])
        await LInstr_pe(dut, pid, pe_instrs[pid])
    await LBdata_cluster(dut, Bc, Bv)

    dut._log.info("Starting %d-PE cluster...", n_pe)
    cp, cyc, lane_busy, rmw_busy = await run_cluster(dut, row_counts, n_pe, pe_desc, N)
    dut._log.info("Cluster done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    dut._log.info("Verification: total=%d, nz_ok=%d, z_ok=%d, errors=%d",
                  M*N, nz_ok, z_ok, e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    single_pe_cycles = 49085  # updated from banked-drain measurement
    lane_utils = [lb / (n_pe * cyc) * 100 for lb in lane_busy]
    rmw_utils  = [rb / (n_pe * cyc) * 100 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  N_PE:                     %d", n_pe)
    dut._log.info("  Cluster wall-time cycles:  %d", cyc)
    dut._log.info("  Total MAC ops:             %d", total_macs)
    dut._log.info("  Speedup vs single PE:      %.2fx  (single=%d cycles)", single_pe_cycles/cyc, single_pe_cycles)
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
