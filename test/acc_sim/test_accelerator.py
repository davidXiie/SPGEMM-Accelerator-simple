#!/usr/bin/env python3
"""
Cocotb test for the full accelerator (accelerator_top).

Test flow:
  1. Load test_comp_case1 matrices
  2. Write A/B to global buffers via host ports
  3. Set M/K/N, trigger start, run 3-phase pipeline
  4. Wait for done, read C from global buffer
  5. Compare against golden C

Also provides debug probing of PE internals after load phase.
"""
import os, struct, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N = 512
C_ROW_STRIDE = MAX_N

# =========================================================================
# A desc: {3'b0, a_off[13:0], a_nnz[9:0], c_row[8:0]}  (36-bit)
# B desc: {5'b0, b_off[16:0], b_nnz[9:0]}               (32-bit)
# =========================================================================
def a_desc(off, nnz, crow):
    return (int(off) << 19) | (int(nnz) << 9) | int(crow)

def a_desc_crow(d, nbits=9):
    return int(d) & ((1 << nbits) - 1)

def a_desc_nnz(d):
    return (int(d) >> 9) & 0x3FF

def a_desc_off(d):
    return (int(d) >> 19) & 0x3FFF

def b_desc(off, nnz):
    return (int(off) << 10) | int(nnz)

def b_desc_nnz(d):
    return int(d) & 0x3FF

def b_desc_off(d):
    return (int(d) >> 10) & 0x1FFFF

# =========================================================================
# FP16 helpers
# =========================================================================
def int_to_fp16_bits(v):
    return int.from_bytes(struct.pack('<e', float(v)), 'little')

def fp16_from_bits(bits):
    return struct.unpack('<e', struct.pack('<H', bits & 0xFFFF))[0]

# =========================================================================
# Matrix loading (reuses test_comp.py logic)
# =========================================================================
def load_comp_matrix(index_file, matrix_file, is_B=False, subdir='TC1_RAW'):
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'test_case_for_reference', subdir)
    with open(os.path.join(base, matrix_file)) as f:
        mat_lines = [list(map(int, l.split())) for l in f if l.strip()]
    with open(os.path.join(base, index_file)) as f:
        idx_lines_raw = [list(map(int, l.split())) for l in f]

    if not is_B:
        rows = len(mat_lines)
        idx_lines = idx_lines_raw[:rows]
        K = mat_lines[0][1]
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        for r in range(rows):
            nnz = mat_lines[r][0]
            cols = idx_lines[r] if nnz > 0 else []
            assert len(cols) == nnz
            for c in cols:
                v = (r * 37 + c * 13 + 1) % 7 + 1
                col_arr.append(c); val_arr.append(int_to_fp16_bits(v))
            row_desc.append(a_desc(offset, nnz, r))
            offset += nnz
        return row_desc, col_arr, val_arr, offset, rows, K
    else:
        B_cols = len(mat_lines)
        idx_lines = idx_lines_raw[:B_cols]
        B_rows = mat_lines[0][1]
        coo = []
        for col in range(B_cols):
            for row in idx_lines[col]:
                v = (row * 37 + col * 13 + 1) % 7 + 1
                coo.append((row, col, int_to_fp16_bits(v)))
        coo.sort()
        row_desc = []; col_arr = []; val_arr = []; offset = 0
        cur_row = 0; row_nnz = 0
        for (r, c, v) in coo:
            while cur_row < r:
                row_desc.append(b_desc(offset, row_nnz))
                offset += row_nnz; cur_row += 1; row_nnz = 0
            col_arr.append(c); val_arr.append(v); row_nnz += 1
        while cur_row < B_rows:
            row_desc.append(b_desc(offset, row_nnz))
            offset += row_nnz; cur_row += 1; row_nnz = 0
        return row_desc, col_arr, val_arr, offset, B_rows, B_cols


def compute_golden_c(A_desc, A_col, A_val, B_desc, B_col, B_val, M, N, K):
    """FP16 × FP16 → FP16 golden reference."""
    def fp16b(bits):
        return struct.unpack('<e', struct.pack('<H', int(bits) & 0xFFFF))[0]
    def to_fp16(v):
        return struct.unpack('<e', struct.pack('<e', float(v)))[0]

    C = [[0.0] * MAX_N for _ in range(MAX_N)]
    for ri in range(M):
        gid  = a_desc_crow(A_desc[ri])
        nnza = a_desc_nnz(A_desc[ri])
        st   = a_desc_off(A_desc[ri])
        for t in range(nnza):
            k  = A_col[st + t] & 0xFFFF
            a  = fp16b(A_val[st + t])
            bn = b_desc_nnz(B_desc[k])
            bs = b_desc_off(B_desc[k])
            for u in range(bn):
                j = B_col[bs + u] & 0xFFFF
                b = fp16b(B_val[bs + u])
                prod = to_fp16(a * b)
                C[gid][j] = to_fp16(C[gid][j] + prod)

    golden = {}
    for ri in range(M):
        gid = a_desc_crow(A_desc[ri])
        for j in range(N):
            v = to_fp16(C[gid][j])
            if v != 0.0:
                golden[gid * C_ROW_STRIDE + j] = v
    return golden


def fp16_ulp_diff(a, b):
    def bits(v):
        bx = struct.unpack('<H', struct.pack('<e', float(v)))[0]
        return bx ^ 0x8000 if bx & 0x8000 else bx
    return abs(bits(a) - bits(b))


# =========================================================================
# Host interface helpers
# =========================================================================

async def write_a_desc(dut, row, data64):
    """Write one A descriptor to global buffer."""
    dut.a_host_desc_wr_addr.value = row
    dut.a_host_desc_wr_data.value = data64
    dut.a_host_desc_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.a_host_desc_wr_en.value   = 0

async def write_a_col(dut, addr, data16):
    dut.a_host_col_wr_addr.value = addr
    dut.a_host_col_wr_data.value = data16
    dut.a_host_col_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.a_host_col_wr_en.value   = 0

async def write_a_val(dut, addr, data16):
    dut.a_host_val_wr_addr.value = addr
    dut.a_host_val_wr_data.value = data16
    dut.a_host_val_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.a_host_val_wr_en.value   = 0

async def write_b_desc(dut, row, data32):
    dut.b_host_desc_wr_addr.value = row
    dut.b_host_desc_wr_data.value = data32
    dut.b_host_desc_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.b_host_desc_wr_en.value   = 0

async def write_b_col(dut, addr, data16):
    dut.b_host_col_wr_addr.value = addr
    dut.b_host_col_wr_data.value = data16
    dut.b_host_col_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.b_host_col_wr_en.value   = 0

async def write_b_val(dut, addr, data16):
    dut.b_host_val_wr_addr.value = addr
    dut.b_host_val_wr_data.value = data16
    dut.b_host_val_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.b_host_val_wr_en.value   = 0

async def read_c_global(dut, addr):
    """Read one FP16 element from C global buffer (registered read)"""
    dut.c_host_rd_addr.value = addr
    await RisingEdge(dut.aclk)       # latch address
    await RisingEdge(dut.aclk)       # data valid
    return int(dut.c_host_rd_data.value) & 0xFFFF


# =========================================================================
# Reset
# =========================================================================
async def reset(dut):
    """Reset the accelerator and start clock."""
    dut.aresetn.value = 0
    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Initialize all control/host signals after reset
    dut.start.value   = 0
    dut.op_mode.value = 0
    dut.op_sub.value  = 0
    dut.M.value = 0; dut.K.value = 0; dut.N.value = 0

    dut.a_host_desc_wr_en.value = 0
    dut.a_host_col_wr_en.value  = 0
    dut.a_host_val_wr_en.value  = 0
    dut.b_host_desc_wr_en.value = 0
    dut.b_host_col_wr_en.value  = 0
    dut.b_host_val_wr_en.value  = 0
    dut.c_host_rd_addr.value = 0
    await RisingEdge(dut.aclk)


async def load_a_global(dut, Ad, Ac, Av, M):
    """Write full A matrix to a_global_buffer."""
    dut._log.info("  Writing A descriptors (%d rows)...", M)
    for r in range(M):
        await write_a_desc(dut, r, Ad[r])
    dut._log.info("  Writing A columns (%d nnz)...", len(Ac))
    for i, v in enumerate(Ac):
        await write_a_col(dut, i, v & 0xFFFF)
    dut._log.info("  Writing A values (%d nnz)...", len(Av))
    for i, v in enumerate(Av):
        await write_a_val(dut, i, v & 0xFFFF)
    dut._log.info("  A global buffer loaded: %d rows, %d nnz", M, len(Ac))


async def load_b_global(dut, Bd, Bc, Bv, K):
    """Write full B matrix to b_global_buffer."""
    dut._log.info("  Writing B descriptors (%d rows)...", K)
    for k in range(K):
        await write_b_desc(dut, k, Bd[k])
    dut._log.info("  Writing B columns (%d nnz)...", len(Bc))
    for i, v in enumerate(Bc):
        await write_b_col(dut, i, v & 0xFFFF)
    dut._log.info("  Writing B values (%d nnz)...", len(Bv))
    for i, v in enumerate(Bv):
        await write_b_val(dut, i, v & 0xFFFF)
    dut._log.info("  B global buffer loaded: %d rows, %d nnz", K, len(Bc))


async def drain_c_global(dut, M, N):
    """Read all C entries from c_global_buffer."""
    cp = {}
    for r in range(M):
        for c in range(N):
            addr = r * MAX_N + c
            v = await read_c_global(dut, addr)
            if v != 0:
                cp[r * C_ROW_STRIDE + c] = fp16_from_bits(v)
    return cp


# =========================================================================
# PE internal probing (load verification)
# =========================================================================
async def probe_pe_load_state(dut, n_pe):
    """Probe each PE's internal state after load phase to verify correctness."""
    dut._log.info("--- PE Load Probe ---")
    for pid in range(n_pe):
        try:
            pe = dut.u_accel.u_cluster.gen_pe[pid].u_pe
            # Check key registers
            pe_state_val = int(pe.state.value) if hasattr(pe, 'state') else -1
            row_idx_val  = int(pe.row_idx.value) if hasattr(pe, 'row_idx') else -1
            a_desc_valid = int(pe.a_desc_valid.value) if hasattr(pe, 'a_desc_valid') else -1
            dut._log.info("  PE%d: state=%d row_idx=%d a_desc_valid=%d",
                          pid, pe_state_val, row_idx_val, a_desc_valid)
        except Exception as e:
            dut._log.info("  PE%d: probe failed - %s", pid, e)


# =========================================================================
# Main test
# =========================================================================
@cocotb.test()
async def test_accel_case1(dut):
    """Full accelerator test: A(251,257) × B(257,121)"""
    # ---- 1. Load matrices ----
    dut._log.info("=" * 70)
    dut._log.info("ACCELERATOR TEST: Loading test_comp_case1")
    dut._log.info("=" * 70)

    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2, f"K mismatch: {K} vs {K2}"

    dut._log.info("A: %d rows, %d nnz (%.1f%% density)", M, An, 100*An/(M*K))
    dut._log.info("B: %d rows, %d nnz (%.1f%% density)", K2, Bn, 100*Bn/(K2*N))
    dut._log.info("Golden C computing...")
    gv = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)
    dut._log.info("Golden C: %d non-zero entries", len(gv))

    # ---- 2. Reset ----
    await reset(dut)

    # ---- 3. Load A to global buffer ----
    dut._log.info("--- Phase: Load A to global buffer ---")
    await load_a_global(dut, Ad, Ac, Av, M)

    # ---- 4. Load B to global buffer ----
    dut._log.info("--- Phase: Load B to global buffer ---")
    await load_b_global(dut, Bd, Bc, Bv, K)

    # ---- 5. Launch accelerator ----
    dut._log.info("--- Phase: Launch accelerator ---")
    dut.M.value = M; dut.K.value = K; dut.N.value = N
    dut.op_mode.value = 0; dut.op_sub.value = 0
    await RisingEdge(dut.aclk)

    dut.start.value = 1
    await RisingEdge(dut.aclk)
    dut.start.value = 0
    dut._log.info("Start pulse issued, waiting for done...")

    # ---- 6. Wait for done (full pipeline) ----
    cyc = 0
    load_a_done_seen = False
    load_b_done_seen = False
    while True:
        await RisingEdge(dut.aclk)
        cyc += 1

        if not load_a_done_seen:
            try:
                if int(dut.u_accel.u_load.a_done.value):
                    dut._log.info("  [cyc %d] LOAD_A done", cyc)
                    await probe_pe_load_state(dut, 3)
                    load_a_done_seen = True
            except Exception:
                pass

        if not load_b_done_seen and load_a_done_seen:
            try:
                if int(dut.u_accel.u_load.b_done.value):
                    dut._log.info("  [cyc %d] LOAD_B done", cyc)
                    load_b_done_seen = True
            except Exception:
                pass

        # Check top-level FSM state
        if cyc % 1000 == 0:
            try:
                fsm = int(dut.u_accel.state.value)
                dut._log.info("  [cyc %d] FSM state=%d", cyc, fsm)
            except Exception:
                pass

        if int(dut.done.value):
            dut._log.info("  [cyc %d] DONE!", cyc)
            break

        if cyc > 5000000:
            dut._log.error("TIMEOUT at cycle %d", cyc)
            break

    # ---- 7. Read C from global buffer ----
    dut._log.info("--- Phase: Read C from global buffer ---")
    await RisingEdge(dut.aclk)
    cp = await drain_c_global(dut, M, N)
    dut._log.info("C global buffer: %d non-zero entries", len(cp))

    # ---- 8. Verify ----
    dut._log.info("--- Phase: Verify ---")
    ULP_TOL = 4
    e = 0; nz_ok = 0; z_ok = 0
    for ri in range(M):
        gid = a_desc_crow(Ad[ri])
        for j in range(N):
            exp = float(gv.get(gid * C_ROW_STRIDE + j, 0.0))
            act = float(cp.get(gid * C_ROW_STRIDE + j, 0.0))
            if act != exp:
                ulp = fp16_ulp_diff(act, exp)
                if ulp <= ULP_TOL:
                    if exp != 0.0: nz_ok += 1
                    else:          z_ok  += 1
                else:
                    if e < 5:
                        dut._log.error("C[%d][%d]: got %g, exp %g (ULP=%d)", gid, j, act, exp, ulp)
                        e += 1
                    elif e == 5:
                        dut._log.error("... (further errors suppressed)")
                        e += 1
            else:
                if exp != 0.0: nz_ok += 1
                else:          z_ok  += 1

    if e == 0:
        dut._log.info("VERIFICATION: PASSED (%d nz correct, %d z correct)", nz_ok, z_ok)
    else:
        dut._log.error("VERIFICATION: FAILED (%d mismatches)", e)

    # ---- 9. Summary ----
    dut._log.info("=" * 70)
    dut._log.info("SUMMARY:")
    dut._log.info("  Total cycles:     %d", cyc)
    dut._log.info("  Matrix:           A(%d,%d) × B(%d,%d) → C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("  A nnz:            %d", An)
    dut._log.info("  B nnz:            %d", Bn)
    dut._log.info("  C non-zero (act): %d", len(cp))
    dut._log.info("  C non-zero (exp): %d", len(gv))
    dut._log.info("=" * 70)

    assert e == 0, f"{e} mismatches in C"
    dut._log.info("ACCELERATOR TEST PASSED")
