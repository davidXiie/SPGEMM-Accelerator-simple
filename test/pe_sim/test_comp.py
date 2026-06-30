#!/usr/bin/env python3
"""
Competition test case: A(32,317) × B(317,6), 30% sparsity.
Loads TC1_RAW pattern 1, converts to compact row-desc, runs PE, collects stats.

Hardware online generation: PE reads A_col_buf + B_desc_buf on-chip and emits
4-wide groups each cycle.  No pre-computed instruction buffer required.
"""
import os, random, struct, heapq, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N=512; C_ROW_STRIDE=MAX_N

# ---------------------------------------------------------------------------
# Descriptor packing/unpacking helpers (must match pe_top.v bit layout)
#
# A desc streamed to HW (36-bit): {1'b0, a_off[15:0], a_nnz[9:0], c_row[8:0]} — HW
# reads a_off at bits [34:19] (16-bit, covers A_NNZ_SLOT_PER_PE=40960 at N_PE=2).  The
# test's GLOBAL Ad reuses a_desc with offsets up to total-A-nnz (~17-bit); those live
# only in Python (golden / partition / per-PE copy), never streamed, so a_desc_off 17b.
def a_desc(off, nnz, crow): return (int(off) << 19) | (int(nnz) << 9) | int(crow)
def a_desc_crow(d): return int(d) & 0x1FF
def a_desc_nnz(d):  return (int(d) >> 9) & 0x3FF
def a_desc_off(d):  return (int(d) >> 19) & 0x1FFFF  # 17-bit: global A offset can exceed 15b

# B desc (32-bit): {5'b0, b_off[16:0], b_nnz[9:0]}
def b_desc(off, nnz): return (int(off) << 10) | int(nnz)
def b_desc_nnz(d):  return int(d) & 0x3FF
def b_desc_off(d):  return (int(d) >> 10) & 0x1FFFF
# ---------------------------------------------------------------------------

def int_to_fp16_bits(v):
    """Convert integer v to its FP16 bit pattern (uint16 value). Uses struct '<e' (Python 3.6+)."""
    return int.from_bytes(struct.pack('<e', float(v)), 'little')

def fp32_from_bits(bits):
    """Interpret a 32-bit integer as an IEEE 754 FP32 float."""
    return struct.unpack('<f', struct.pack('<I', bits & 0xFFFFFFFF))[0]

def fp16_from_bits(bits):
    """Interpret a 16-bit integer as an IEEE 754 FP16 float."""
    return struct.unpack('<e', struct.pack('<H', bits & 0xFFFF))[0]

def slice_bits(logic_value, lo, width):
    """Extract a width-bit field starting at bit `lo` from a cocotb value.

    Reads the binary string (MSB-first) directly so unrelated X/Z bits elsewhere
    in the bus don't break int() conversion; x/z within the field resolve to 0.
    """
    s = logic_value.binstr
    n = len(s)
    out = 0
    for k in range(width):
        if s[n - 1 - (lo + k)] == '1':
            out |= (1 << k)
    return out

#-------------------------------------------------------------------------
# Load competition matrix files
#-------------------------------------------------------------------------
def load_comp_matrix(index_file, matrix_file, is_B=False, subdir='TC1_RAW'):
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
                        '..', 'test_case_for_reference', subdir)
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
            row_desc.append(a_desc(offset, nnz, r))
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
                row_desc.append(b_desc(offset, row_nnz))
                offset += row_nnz; cur_row += 1; row_nnz = 0
            col_arr.append(c); val_arr.append(v); row_nnz += 1
        while cur_row < B_rows:
            row_desc.append(b_desc(offset, row_nnz))
            offset += row_nnz; cur_row += 1; row_nnz = 0
        return row_desc, col_arr, val_arr, offset, B_rows, B_cols

def align_b_16wide(Bd, Bc, Bv):
    """Lay out B elements in NB-bank storage with per-row rotation.

    Row r starts at absolute position b_off where b_off % NB == r % NB.
    This distributes the tail element of partial rows evenly across NB lanes.
    The hardware generator uses lane = (b_off + u) % NB (general form).
    Alignment is load-balance only; correctness holds for any b_off.
    """
    new_Bc = []; new_Bv = []; new_Bd = []; new_off = 0
    for r, d in enumerate(Bd):
        start  = b_desc_off(d)
        nnz    = b_desc_nnz(d)
        target_mod = r % _NB
        gap = (target_mod - new_off % _NB) % _NB
        for _ in range(gap):
            new_Bc.append(0); new_Bv.append(0)
        new_off += gap
        new_Bd.append(b_desc(new_off, nnz))
        for t in range(nnz):
            new_Bc.append(Bc[start + t])
            new_Bv.append(Bv[start + t])
        new_off += nnz
    return new_Bd, new_Bc, new_Bv

def slice_b_columns(Bd, Bc, Bv, K, N, t, T):
    """Column-tile of B for output-column tiling.

    Tile t covers GLOBAL columns [t*tw, t*tw+width).  Returns a CSR of B
    restricted to that column range (Bd_t, Bc_t, Bv_t) with TILE-LOCAL column
    indices (col - lo), plus (lo, width).  Each B row keeps only its nonzeros
    whose column falls in the tile.  Because C's columns are independent, running
    the PE once per tile (with N=width) and re-basing the output columns by lo
    reconstructs the full C while only 1/T of B need be resident at a time.
    """
    tw  = (N + T - 1) // T
    lo  = t * tw
    hi  = min(lo + tw, N)
    width = hi - lo
    Bd_t = []; Bc_t = []; Bv_t = []; off = 0
    for k in range(K):
        d = Bd[k]
        start = b_desc_off(d); nnz = b_desc_nnz(d)
        cnt = 0
        for u in range(nnz):
            c = int(Bc[start + u]) & 0xFFFF
            if lo <= c < hi:
                Bc_t.append(c - lo); Bv_t.append(Bv[start + u]); cnt += 1
        Bd_t.append(b_desc(off, cnt)); off += cnt
    return Bd_t, Bc_t, Bv_t, lo, width


def compute_col_perm(Bc_raw, N):
    """Greedy column→bank assignment to balance RMW load.

    Sorts B columns by nnz descending, then assigns each to the bank with
    the least accumulated nnz so far (min-heap). Returns perm[original_col]
    = new_col_id where new_col_id % 8 == assigned_bank.
    """
    col_nnz = [0] * N
    for c in Bc_raw:
        col_nnz[int(c) & 0xFFFF] += 1
    sorted_cols = sorted(range(N), key=lambda c: -col_nnz[c])
    heap = [(0, 0, b) for b in range(16)]   # (total_nnz, col_count, bank_id)
    heapq.heapify(heap)
    perm = [0] * N
    for j in sorted_cols:
        total, cnt, b = heapq.heappop(heap)
        perm[j] = b + 16 * cnt             # bank b, slot cnt within bank
        heapq.heappush(heap, (total + col_nnz[j], cnt + 1, b))
    return perm

def compute_col_perm_online(Bc, N):
    """Online column→bank assignment during B loading (hardware-simulated).

    Processes B column indices in CSR storage order (row by row).
    On first occurrence of a column, assigns it to the bank with the fewest
    columns assigned so far (argmin over 16 counters = combinational tree in HW).
    Hardware cost: 512-bit col_assigned bitmap + 16 bank_cnt counters + 512×9-bit perm SRAM.
    Extra latency vs plain B loading: 0 cycles.
    """
    col_assigned = [False] * N
    bank_cnt = [0] * 16      # columns assigned per bank (used for both argmin and perm slot)
    perm = [0] * N
    for c in Bc:
        col_id = int(c) & 0xFFFF
        if col_id >= N or col_assigned[col_id]:
            continue
        b = bank_cnt.index(min(bank_cnt))    # argmin → 16-way comparator tree in HW
        perm[col_id] = b + 16 * bank_cnt[b]
        bank_cnt[b] += 1
        col_assigned[col_id] = True
    # Zero-NNZ columns: assign to remaining slots in round-robin order
    for col_id in range(N):
        if not col_assigned[col_id]:
            b = bank_cnt.index(min(bank_cnt))
            perm[col_id] = b + 16 * bank_cnt[b]
            bank_cnt[b] += 1
    return perm

def count_total_macs(Ad, Ac, Bd, M):
    """Exact count of MAC operations: for each A[i,k] nonzero, add B row k's nnz."""
    total = 0
    for ri in range(M):
        nnza = a_desc_nnz(Ad[ri])
        st   = a_desc_off(Ad[ri])
        for t in range(nnza):
            k = Ac[st + t] & 0xFFFF
            total += b_desc_nnz(Bd[k])
    return total

#-------------------------------------------------------------------------
# Elementwise (C = A +/- B) helpers — both inputs are M x N, same shape.
#-------------------------------------------------------------------------
def gen_sparse_rows(M, N, density, seed):
    """Random sparse M x N matrix as a list of rows; each row is [(col, val), ...]
    with distinct, ascending columns and small positive integer values."""
    rng = random.Random(seed)
    k = min(N, max(1, int(round(N * density))))
    rows = []
    for r in range(M):
        cols = sorted(rng.sample(range(N), k))
        rows.append([(c, rng.randint(1, 7)) for c in cols])
    return rows

def pack_csr(rows, is_B):
    """Pack rows into (row_desc, col[], val[]). A uses a_desc{off,nnz,crow},
    B uses b_desc{off,nnz}; col/val are flat in row order (B banks by index%16)."""
    desc = []; col = []; val = []; off = 0
    for r, row in enumerate(rows):
        for (c, v) in row:
            col.append(c); val.append(int_to_fp16_bits(v))
        desc.append(a_desc(off, len(row), r) if not is_B else b_desc(off, len(row)))
        off += len(row)
    return desc, col, val

def golden_addsub(A_rows, B_rows, M, N, sub):
    """FP16 golden for C = A + B (sub=0) or C = A - B (sub=1)."""
    gf = [[0.0] * N for _ in range(M)]
    for r in range(M):
        for (c, v) in A_rows[r]:
            gf[r][c] = gf[r][c] + v
        for (c, v) in B_rows[r]:
            gf[r][c] = gf[r][c] + (-v if sub else v)
    for r in range(M):
        for j in range(N):
            gf[r][j] = fp16_from_bits(int_to_fp16_bits(gf[r][j]))
    return gf

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
                prod = to_fp16(a * b)          # FP16 multiply
                C[gid][j] = to_fp16(C[gid][j] + prod)  # FP16 accumulate

    golden_f16 = [[0.0] * N for _ in range(M)]
    golden = {}
    for ri in range(M):
        gid = a_desc_crow(A_desc[ri])
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
async def stream_a_desc(dut, Ad):
    """Stream A row descriptors to PE using eager pre-assertion.

    Keeps a_desc_valid=1 whenever a descriptor is available, so the PE
    captures it in 1 cycle instead of 2 (reduces LOAD_ROW_DESC by 1/row).
    """
    if not Ad:
        dut.a_desc_valid.value = 0
        return
    # Pre-load the first descriptor before PE starts requesting
    dut.a_desc_data.value = Ad[0]
    dut.a_desc_valid.value = 1
    for i in range(len(Ad)):
        # Wait until PE consumes this descriptor (a_desc_ready=1 on rising edge)
        while True:
            await RisingEdge(dut.aclk)
            if int(dut.a_desc_ready.value):
                break
        # Immediately load the next descriptor (or de-assert if last)
        if i + 1 < len(Ad):
            dut.a_desc_data.value = Ad[i + 1]
            dut.a_desc_valid.value = 1
        else:
            dut.a_desc_valid.value = 0


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
    dut.op_mode.value=0; dut.op_sub.value=0
    dut.a_desc_valid.value=0; dut.a_desc_data.value=0
    dut.a_val_we.value=0; dut.a_col_we.value=0
    dut.b_col_we.value=0; dut.b_val_we.value=0; dut.b_desc_we.value=0

async def reset_pulse(dut):
    """Light reset (single PE): pulse aresetn to clear the FSMs/accumulator tags
    between column-tile passes WITHOUT restarting the clock or reloading the A
    buffers (BRAM survives reset)."""
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 40)
    dut.start.value = 0


async def read_c_buffer(dut, Ad, N):
    """Read internal per-PE C buffer — DISABLED (c_bank removed). Returns empty dict."""
    return {}

async def run_pe(dut, rc, Ad, N, to=10000000):
    """Run PE for rc rows, collect stats, then read C buffer."""
    cocotb.start_soon(stream_a_desc(dut, Ad))
    dut.row_count.value = rc
    dut.c_rd_en.value = 0; dut.c_rd_addr.value = 0
    dut.start.value=1; await RisingEdge(dut.aclk); dut.start.value=0
    dc=0; lane_busy=[0]*16; rmw_busy=[0]*16
    # PE state names (indices match pe_top.v localparams)
    PE_STATE_NAMES = ['IDLE','LOAD_ROW_DESC','CLEAR_ACC','STREAM_INSTRS',
                      'WAIT_TASK_DRAIN','WAIT_PRODUCT_DRAIN','NEXT_ROW','DONE']
    GEN_STATE_NAMES = ['IDLE','FETCH','EMIT','ROW_DONE','FLUSH']
    state_cycles    = [0] * 8   # cycles spent in each PE state
    gen_state_cyc   = [0] * 5   # cycles in each generator state
    mac_idle_in_stream = 0      # MAC idle cycles within PE_STREAM_INSTRS
    issue_rdy_stall = 0         # cycles issue_ready=0 in any acc (product FIFO backpressure)
    mac_sig    = dut.u_pe.mac_lane_valid
    pe_state   = dut.u_pe.state
    gen_state  = dut.u_pe.gen_state
    iry0       = dut.u_pe.acc_issue_ready_0
    iry1       = dut.u_pe.acc_issue_ready_1
    task_cnt        = dut.u_pe.u_task_fifo.count
    tg_wr           = dut.u_pe.task_group_wr_en
    prd_cnt0        = dut.u_pe.product_fifo_cnt_0
    prd_cnt1        = dut.u_pe.product_fifo_cnt_1
    c_sel           = dut.u_pe.comp_sel
    exec_state_sig  = dut.u_pe.exec_state
    exec_safe_sig   = dut.u_pe.exec_prod_safe
    ptr_empty_sig   = dut.u_pe.ptr_fifo_empty
    rmw_sigs_0 = [dut.u_pe.u_row_acc_0.g_bank[0].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[1].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[2].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[3].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[4].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[5].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[6].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[7].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[8].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[9].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[10].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[11].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[12].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[13].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[14].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_0.g_bank[15].u_bank.rmw_busy]
    rmw_sigs_1 = [dut.u_pe.u_row_acc_1.g_bank[0].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[1].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[2].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[3].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[4].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[5].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[6].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[7].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[8].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[9].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[10].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[11].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[12].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[13].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[14].u_bank.rmw_busy,
                  dut.u_pe.u_row_acc_1.g_bank[15].u_bank.rmw_busy]
    cp = {}
    prev_gs = -1; prev_s = -1
    task_cnt_at_row_done = []; wait_task_drain_cyc = []
    wt_start = -1
    # Additional counters
    gen_emit_no_write    = 0   # GEN_EMIT cycles where no task written (accumulate case)
    task_empty_in_stream = 0   # STREAM_INSTRS + MAC idle: both FIFOs empty (gen2 catching up)
    prod_full_task_stall = 0   # STREAM_INSTRS + MAC idle: task FIFO has data, prod full
    ptr_exec_prod_stall  = 0   # STREAM_INSTRS + MAC idle: exec_state=PTR, prod full
    for cy in range(to):
        await RisingEdge(dut.aclk)
        mlv = int(mac_sig.value)
        s   = int(pe_state.value)
        gs  = int(gen_state.value)
        state_cycles[s] += 1
        if gs < 5: gen_state_cyc[gs] += 1
        for i in range(16):
            if (mlv >> i) & 1: lane_busy[i] += 1
            if int(rmw_sigs_0[i].value) or int(rmw_sigs_1[i].value): rmw_busy[i] += 1
        # MAC idle within STREAM_INSTRS
        if s == 3 and mlv == 0:
            mac_idle_in_stream += 1
            es  = int(exec_state_sig.value)
            eps = int(exec_safe_sig.value)
            pe  = int(ptr_empty_sig.value)
            tc  = int(task_cnt.value)
            if es == 1 and not eps:
                ptr_exec_prod_stall += 1   # ptr executor has work, prod FIFO full
            elif es == 0 and not pe:
                pass                        # IDLE→PTR transition bubble (1 cycle)
            elif tc > 0 and not eps:
                prod_full_task_stall += 1  # task FIFO has data, prod FIFO full
            else:
                task_empty_in_stream += 1  # truly nothing to do (gen2 catching up)
        # issue_ready stalls on ACTIVE acc only
        cs = int(c_sel.value)
        if cs == 0 and not int(iry0.value): issue_rdy_stall += 1
        if cs == 1 and not int(iry1.value): issue_rdy_stall += 1
        # GEN_EMIT with no task written (accumulate case)
        if gs == 2 and not int(tg_wr.value): gen_emit_no_write += 1
        # Detect GEN_ROW_DONE transition
        if gs == 3 and prev_gs != 3:
            task_cnt_at_row_done.append(int(task_cnt.value))
        # Track WAIT_TASK_DRAIN duration
        if s == 4 and prev_s != 4: wt_start = cy
        if s != 4 and prev_s == 4 and wt_start >= 0:
            wait_task_drain_cyc.append(cy - wt_start)
            wt_start = -1
        prev_gs = gs; prev_s = s
        if int(dut.done.value): dc=cy; break
    else: assert False, f"timeout {to}"
    # Print state breakdown
    dut._log.info("--- PE State Cycle Breakdown ---")
    for i,n in enumerate(PE_STATE_NAMES):
        dut._log.info("  %s: %d cycles", n, state_cycles[i])
    dut._log.info("--- Generator State Breakdown ---")
    for i,n in enumerate(GEN_STATE_NAMES):
        dut._log.info("  GEN_%s: %d cycles", n, gen_state_cyc[i])
    dut._log.info("  GEN_EMIT no-write (accumulate): %d cycles", gen_emit_no_write)
    dut._log.info("  MAC idle in STREAM_INSTRS: %d cycles", mac_idle_in_stream)
    dut._log.info("    ptr executor prod_full:  %d", ptr_exec_prod_stall)
    dut._log.info("    task path prod_full:     %d", prod_full_task_stall)
    dut._log.info("    gen2 catching up (idle): %d", task_empty_in_stream)
    dut._log.info("  issue_ready stall (active acc): %d cycles", issue_rdy_stall)
    if task_cnt_at_row_done:
        avg_tc = sum(task_cnt_at_row_done)/len(task_cnt_at_row_done)
        dut._log.info("  task_fifo count at GEN_ROW_DONE: avg=%.1f max=%d min=%d",
                      avg_tc, max(task_cnt_at_row_done), min(task_cnt_at_row_done))
    if wait_task_drain_cyc:
        avg_wt = sum(wait_task_drain_cyc)/len(wait_task_drain_cyc)
        dut._log.info("  PE_WAIT_TASK_DRAIN cycles: avg=%.1f max=%d min=%d",
                      avg_wt, max(wait_task_drain_cyc), min(wait_task_drain_cyc))
    # Read C back from the independent on-chip C bank (synchronous 1-cycle read).
    # Proves C physically landed in SRAM rather than being snooped off drain wires.
    cp = await read_c_bank(dut.c_rd_en, dut.c_rd_addr, dut.c_rd_data, dut.c_rd_row,
                           dut.aclk, rc, N)
    await ClockCycles(dut.aclk, 10)
    return cp, dc, lane_busy, rmw_busy

async def read_c_bank(c_rd_en, c_rd_addr, c_rd_data, c_rd_row, clk, rc, N):
    """Read every computed C row out of the on-chip, local-row-indexed C bank.

    Address = {local_row[C_ROW_ADDR_BITS-1:0], gaddr[4:0]}; each read returns 16
    FP16 lanes (column j = gaddr*16 + lane) plus the slot's global C row from
    C_row_map (c_rd_row).  Registered read → valid one cycle after the address.
    Non-zero results go into cp; explicit zeros are dropped so verify() treats
    them as 0.  The host learns row placement from c_rd_row, not the partition.
    """
    cp = {}
    ngroups = (N + _NB - 1) // _NB
    c_rd_en.value = 1
    for local in range(rc):
        for g in range(ngroups):
            c_rd_addr.value = (local << _GBITS) | g
            await RisingEdge(clk)          # latch address
            await RisingEdge(clk)          # registered data now valid
            r    = int(c_rd_row.value)     # global C row for this local slot
            vals = int(c_rd_data.value)
            for b in range(_NB):
                fp16_bits = (vals >> (b * 16)) & 0xFFFF
                if fp16_bits == 0:
                    continue
                j = g * _NB + b
                if j < N:
                    cp[r * C_ROW_STRIDE + j] = fp16_from_bits(fp16_bits)
    c_rd_en.value = 0
    return cp

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
        gid=a_desc_crow(Ad[ri]);b=gid*C_ROW_STRIDE
        for j in range(N):
            exp=float(gf[gid][j]);act=float(cp.get(b+j,0.0))
            if act != exp:
                ulp = fp16_ulp_diff(act, exp)
                if ulp <= ULP_TOL:
                    # Acceptable rounding difference — count as correct
                    if exp != 0.0: nz_ok += 1
                    else:          z_ok  += 1
                else:
                    if e<5:dut._log.error("C[%d][%d]: got %g, exp %g (diff=%g, ULP=%d)",gid,j,act,exp,exp-act,ulp)
                    elif e==5:dut._log.error("... (further errors suppressed)")
                    e+=1   # always count (was capped at 6 -> under-reported)
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

    # No permutation: feed raw column IDs; hardware banks by col_id % 16.
    Bc_hw = [int(c) & 0xFFFF for c in Bc]

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc_hw, Bv, M, N, K)

    # Bank nnz distribution (raw col_id % 16, no padding)
    orig_bank_nnz = [0] * 16
    for c in Bc:
        orig_bank_nnz[int(c) & 0xF] += 1

    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST: A(%d,%d) × B(%d,%d) → C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("A: %d rows, %d nnz (%.1f%% density)", M, An, 100*An/(M*K))
    dut._log.info("B: %d rows, %d nnz (%.1f%% density)", K2, Bn, 100*Bn/(K2*N))
    dut._log.info("Bank nnz (raw col%%16): %s  range=%d",
                  orig_bank_nnz, max(orig_bank_nnz) - min(orig_bank_nnz))
    dut._log.info("Golden C: %d non-zero entries", len(gv))

    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LA_val(dut, Av)
    await LAcol(dut, Ac)
    await LBdata(dut, Bc_hw, Bv)
    await LBdesc(dut, Bd)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, Ad, N, to=50000000)

    dut._log.info("PE done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("Verification: PASSED (%d nz correct, %d z correct)", nz_ok, z_ok)
    else:
        dut._log.error("Verification: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in C"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / cyc * 100 if cyc > 0 else 0.0 for lb in lane_busy]
    rmw_utils  = [rb / cyc * 100 if cyc > 0 else 0.0 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  Total cycles:      %d", cyc)
    dut._log.info("  Total MAC ops:     %d  (exact)", total_macs)
    dut._log.info("  Per-lane MAC utilization (mac_lane_valid):")
    for i in range(16):
        dut._log.info("    Lane %d: %6d busy cycles  →  %.2f%%", i, lane_busy[i], lane_utils[i])
    dut._log.info("  Average MAC util:  %.2f%%", sum(lane_utils) / 16)
    dut._log.info("  Per-bank accumulator RMW utilization (rmw_busy):")
    for i in range(16):
        dut._log.info("    Bank %d: %6d RMW cycles   →  %.2f%%", i, rmw_busy[i], rmw_utils[i])
    dut._log.info("  Average RMW util:  %.2f%%", sum(rmw_utils) / 16)
    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST PASSED")

#=========================================================================
@cocotb.test()
async def test_comp_tiled_p0(dut):
    """Single-PE OUTPUT-COLUMN TILING: split B into T column tiles, run the PE
    once per tile (N=tile width, tile-local columns), and reassemble the full C.
    Proves only 1/T of B needs to be resident at a time — the basis for cutting
    the per-PE B BRAM in the cluster."""
    T = 2
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    Bc_hw = [int(c) & 0xFFFF for c in Bc]
    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc_hw, Bv, M, N, K)

    dut._log.info("=" * 70)
    dut._log.info("TILED TEST: A(%d,%d) x B(%d,%d) -> C(%d,%d), T=%d column tiles",
                  M, K, K2, N, M, N, T)

    await rst(dut); dut.M.value = M; dut.K.value = K
    await LA_val(dut, Av); await LAcol(dut, Ac)   # A loaded ONCE, reused every pass

    cp_full = {}
    for t in range(T):
        Bd_t, Bc_t, Bv_t, lo, width = slice_b_columns(Bd, Bc_hw, Bv, K2, N, t, T)
        await reset_pulse(dut)
        dut.N.value = width
        await LBdata(dut, Bc_t, Bv_t)
        await LBdesc(dut, Bd_t)
        cp_t, cyc, _, _ = await run_pe(dut, M, Ad, width, to=50000000)
        for key, val in cp_t.items():
            r  = key // C_ROW_STRIDE
            lc = key %  C_ROW_STRIDE
            cp_full[r * C_ROW_STRIDE + (lo + lc)] = val
        dut._log.info("  tile %d: cols[%d:%d] width=%d  cyc=%d  nz=%d",
                      t, lo, lo + width, width, cyc, len(cp_t))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp_full)
    if e == 0:
        dut._log.info("TILED VERIFICATION: PASSED (%d nz correct, %d z correct)", nz_ok, z_ok)
    else:
        dut._log.error("TILED VERIFICATION: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in tiled C"
    dut._log.info("TILED TEST PASSED")

#=========================================================================
@cocotb.test()
async def test_comp_case1_p1(dut):
    """Competition Case1 Pattern1: A(32,317) × B(317,6)"""
    Ad, Ac, Av, An, M, K = load_comp_matrix('A_1_Index.txt', 'A_1_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_1_Index.txt', 'B_1_Matrix.txt', True)
    assert K == K2, f"K mismatch: {K} vs {K2}"
    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST: A(%d,%d) × B(%d,%d) → C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("A: %d rows, %d nnz (%.1f%% sparsity)", M, An, 100*An/(M*K))
    dut._log.info("B: %d rows, %d nnz (%.1f%% sparsity)", K2, Bn, 100*Bn/(K2*N))
    dut._log.info("Golden C: %d non-zero entries", len(gv))
    for ri in range(min(3, M)):
        gid = a_desc_crow(Ad[ri])
        dut._log.info("  C[%d] samples: %s", gid,
                      [int(gf[gid][j]) for j in range(min(6, N))])

    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LA_val(dut, Av)
    await LAcol(dut, Ac)
    await LBdata(dut, Bc, Bv)
    await LBdesc(dut, Bd)

    dut._log.info("Starting PE, row_count=%d...", M)
    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, Ad, N, to=10000000)

    dut._log.info("PE done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("Verification: PASSED (%d nz correct, %d z correct)", nz_ok, z_ok)
    else:
        dut._log.error("Verification: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in C"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / cyc * 100 if cyc > 0 else 0.0 for lb in lane_busy]
    rmw_utils  = [rb / cyc * 100 if cyc > 0 else 0.0 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  Total cycles:      %d", cyc)
    dut._log.info("  Total MAC ops:     %d  (exact)", total_macs)
    dut._log.info("  Per-lane MAC utilization (mac_lane_valid):")
    for i in range(16):
        dut._log.info("    Lane %d: %6d busy cycles  →  %.2f%%", i, lane_busy[i], lane_utils[i])
    dut._log.info("  Average MAC util:  %.2f%%", sum(lane_utils) / 16)
    dut._log.info("  Per-bank accumulator RMW utilization (rmw_busy):")
    for i in range(16):
        dut._log.info("    Bank %d: %6d RMW cycles   →  %.2f%%", i, rmw_busy[i], rmw_utils[i])
    dut._log.info("  Average RMW util:  %.2f%%", sum(rmw_utils) / 16)
    dut._log.info("=" * 70)
    dut._log.info("COMPETITION TEST PASSED")


#=========================================================================
# Cluster helpers — packed-bus interface, N_PE-parametric
#
# Width constants must stay in sync with defines.vh:
_A_NNZ_ADDR_W  = 17   # A_NNZ_ADDR_BITS (per-PE stride in the packed a_*_waddr bus)
_DATA_W        = 16   # DATA_WIDTH (FP16 input)
_COL_W         = 9    # log2(MAX_N=512), matches ACC_COL_W in pe_top.v
_NB            = 32   # N_MAC: banks/MAC lanes per PE (must match defines.vh)
_GBITS         = 9 - (_NB.bit_length() - 1)   # C gaddr width = 9 - log2(NB)
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
        nnza = a_desc_nnz(Ad[ri])
        st   = a_desc_off(Ad[ri])
        if Bd is not None:
            t = sum((b_desc_nnz(Bd[Ac[st + ti] & 0xFFFF]) + 15) // 16 for ti in range(nnza))
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
        global_row   = a_desc_crow(Ad[ri])
        nnza         = a_desc_nnz(Ad[ri])
        global_start = a_desc_off(Ad[ri])
        local_start  = len(pe_val[pid])

        for t in range(nnza):
            pe_val[pid].append(Av[global_start + t])
            pe_col[pid].append(Ac[global_start + t] & 0xFFFF)

        pe_desc[pid].append(a_desc(local_start, nnza, global_row))

    return pe_desc, pe_val, pe_col

async def stream_a_desc_pe(dut, pid, row_descs):
    """Stream A row descriptors into cluster PE pid via valid/ready handshake."""
    a_valid = getattr(dut, f"a_desc_valid_{pid}")
    a_ready = getattr(dut, f"a_desc_ready_{pid}")
    a_data  = getattr(dut, f"a_desc_data_{pid}")
    a_valid.value = 0
    if not row_descs:
        return
    # Pre-load first descriptor
    a_data.value = row_descs[0]
    a_valid.value = 1
    for i in range(len(row_descs)):
        while True:
            await RisingEdge(dut.aclk)
            if int(a_ready.value):
                break
        if i + 1 < len(row_descs):
            a_data.value = row_descs[i + 1]
            a_valid.value = 1
        else:
            a_valid.value = 0

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
    await ClockCycles(dut.aclk,3)
    n_pe = int(dut.n_pe_sig.value)   # read after clock is running
    await ClockCycles(dut.aclk,7); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value     = 0
    dut.row_count.value = 0
    dut.op_mode.value   = 0
    dut.op_sub.value    = 0
    for pid in range(n_pe):
        getattr(dut, f"a_desc_valid_{pid}").value = 0
        getattr(dut, f"a_desc_data_{pid}").value  = 0
    dut.a_val_we.value  = 0; dut.a_val_waddr.value  = 0; dut.a_val_wdata.value  = 0
    dut.a_col_we.value  = 0; dut.a_col_waddr.value  = 0; dut.a_col_wdata.value  = 0
    dut.b_col_we.value  = 0; dut.b_val_we.value     = 0
    dut.b_desc_we.value = 0

async def read_c_buffer_pe(dut, pid, row_descs_pid, N):
    """Read internal C buffer of cluster PE — DISABLED (c_bank removed). Returns empty dict."""
    return {}

async def run_cluster(dut, row_counts, n_pe, pe_desc, N, to=50000000):
    """Start all n_pe PEs, wait for done, collect stats, read C buffers."""
    rc_packed = sum(row_counts[p] << (p * 16) for p in range(n_pe))
    dut.row_count.value = rc_packed
    # Launch per-PE descriptor streaming; they'll handshake with the PEs
    # after start fires and a_desc_ready goes high.
    for pid in range(n_pe):
        cocotb.start_soon(stream_a_desc_pe(dut, pid, pe_desc[pid]))
    dut.start.value=1; await RisingEdge(dut.aclk); dut.start.value=0
    dc = 0

    mac_sigs = [dut.u_cluster.gen_pe[pid].u_pe.mac_lane_valid for pid in range(n_pe)]
    rmw_acc0 = [[dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[0].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[1].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[2].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[3].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[4].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[5].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[6].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[7].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[8].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[9].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[10].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[11].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[12].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[13].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[14].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_0.g_bank[15].u_bank.rmw_busy]
                for pid in range(n_pe)]
    rmw_acc1 = [[dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[0].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[1].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[2].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[3].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[4].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[5].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[6].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[7].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[8].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[9].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[10].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[11].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[12].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[13].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[14].u_bank.rmw_busy,
                 dut.u_cluster.gen_pe[pid].u_pe.u_row_acc_1.g_bank[15].u_bank.rmw_busy]
                for pid in range(n_pe)]

    drain_sigs = []
    for pid in range(n_pe):
        pe = dut.u_cluster.gen_pe[pid].u_pe
        drain_sigs.append((
            pe.drain_valid_0, pe.drain_gaddr_0, pe.drain_row_id_0, pe.drain_values_0,
            pe.drain_valid_1, pe.drain_gaddr_1, pe.drain_row_id_1, pe.drain_values_1,
        ))
    cp = {}
    lane_busy = [0] * 16
    rmw_busy  = [0] * 16

    for cy in range(to):
        await RisingEdge(dut.aclk)
        for pid in range(n_pe):
            mlv = int(mac_sigs[pid].value)
            for i in range(16):
                if (mlv >> i) & 1: lane_busy[i] += 1
            for i in range(16):
                if int(rmw_acc0[pid][i].value) or int(rmw_acc1[pid][i].value):
                    rmw_busy[i] += 1
        if int(dut.done.value): dc=cy; break
    else:
        assert False, f"cluster timeout at {to} cycles"

    # Read each PE's independent local-row-indexed C bank through the packed
    # read port.  Local slot i -> global row via c_rd_row (C_row_map).
    # Derive the per-PE field widths from the actual bus widths so this stays
    # correct regardless of the build's C_ROW_ADDR_BITS override.
    C_RD_ADDR_W  = len(dut.c_rd_addr) // n_pe
    MAX_DIM_BITS = len(dut.c_rd_row)  // n_pe
    ngroups = (N + _NB - 1) // _NB
    for pid in range(n_pe):
        for local in range(row_counts[pid]):
            for g in range(ngroups):
                dut.c_rd_en.value   = 1 << pid
                dut.c_rd_addr.value = ((local << _GBITS) | g) << (pid * C_RD_ADDR_W)
                await RisingEdge(dut.aclk)   # latch address
                await RisingEdge(dut.aclk)   # registered data valid
                # Only PE pid's slice is driven; others may be X → x/z resolve to 0.
                r    = slice_bits(dut.c_rd_row.value,  pid * MAX_DIM_BITS, MAX_DIM_BITS)
                vals = slice_bits(dut.c_rd_data.value, pid * _NB * 16, _NB * 16)
                for b in range(_NB):
                    fp16_bits = (vals >> (b * 16)) & 0xFFFF
                    if fp16_bits == 0:
                        continue
                    j = g * _NB + b
                    if j < N:
                        cp[r * C_ROW_STRIDE + j] = fp16_from_bits(fp16_bits)
    dut.c_rd_en.value = 0

    await ClockCycles(dut.aclk, 10)
    return cp, dc, lane_busy, rmw_busy

#=========================================================================
@cocotb.test()
async def test_comp_case1_cluster(dut):
    """N_PE-wide cluster: A(251,257) x B(257,121), rows distributed round-robin."""
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
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
        await LA_val_pe(dut, pid, pe_val[pid])
        await LAcol_pe(dut, pid, pe_col[pid])
    await LBdata_cluster(dut, Bc, Bv)
    await LBdesc_cluster(dut, Bd)

    dut._log.info("Starting %d-PE cluster...", n_pe)
    cp, cyc, lane_busy, rmw_busy = await run_cluster(dut, row_counts, n_pe, pe_desc, N)
    dut._log.info("Cluster done at cycle %d, C buffer entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("Verification: PASSED (%d nz correct, %d z correct)", nz_ok, z_ok)
    else:
        dut._log.error("Verification: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in C"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / (n_pe * cyc) * 100 for lb in lane_busy]
    rmw_utils  = [rb / (n_pe * cyc) * 100 for rb in rmw_busy]

    dut._log.info("=" * 70)
    dut._log.info("STATISTICS:")
    dut._log.info("  N_PE:                     %d", n_pe)
    dut._log.info("  Cluster wall-time cycles:  %d", cyc)
    dut._log.info("  Total MAC ops:             %d", total_macs)
    dut._log.info("  Per-lane MAC utilization (summed across %d PEs):", n_pe)
    for i in range(16):
        dut._log.info("    Lane %d: %7d PE-cycles  →  %.2f%%", i, lane_busy[i], lane_utils[i])
    dut._log.info("  Average MAC util:          %.2f%%", sum(lane_utils) / 16)
    dut._log.info("  Per-bank RMW utilization (summed across %d PEs):", n_pe)
    for i in range(16):
        dut._log.info("    Bank %d: %7d PE-cycles  →  %.2f%%", i, rmw_busy[i], rmw_utils[i])
    dut._log.info("  Average RMW util:          %.2f%%", sum(rmw_utils) / 16)
    dut._log.info("=" * 70)
    dut._log.info("%d-PE CLUSTER TEST PASSED", n_pe)


#=========================================================================
@cocotb.test()
async def test_comp_tiled_cluster(dut):
    """N-PE cluster + OUTPUT-COLUMN TILING (T): A row-partitioned (round-robin),
    B column-tiled and broadcast ONE tile at a time so each PE only ever holds
    1/T of B.  This is the configuration that lets more PEs fit under the BRAM
    cap (B is replicated per PE)."""
    T = 2
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    Bc_hw = [int(c) & 0xFFFF for c in Bc]
    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc_hw, Bv, M, N, K)

    await rst_cluster(dut)
    n_pe = int(dut.n_pe_sig.value)

    # Round-robin A row partition (structural, no matrix-feature analysis).
    pe_desc = [[] for _ in range(n_pe)]
    pe_val  = [[] for _ in range(n_pe)]
    pe_col  = [[] for _ in range(n_pe)]
    for ri in range(M):
        pid  = ri % n_pe
        gr   = a_desc_crow(Ad[ri]); nnza = a_desc_nnz(Ad[ri]); gs = a_desc_off(Ad[ri])
        ls   = len(pe_val[pid])
        for u in range(nnza):
            pe_val[pid].append(Av[gs + u]); pe_col[pid].append(Ac[gs + u] & 0xFFFF)
        pe_desc[pid].append(a_desc(ls, nnza, gr))
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]

    dut.M.value = M; dut.K.value = K
    for pid in range(n_pe):                      # A loaded ONCE, reused every pass
        await LA_val_pe(dut, pid, pe_val[pid])
        await LAcol_pe(dut, pid, pe_col[pid])

    dut._log.info("=" * 70)
    dut._log.info("%d-PE TILED CLUSTER: A(%d,%d)xB(%d,%d) -> C(%d,%d), T=%d, rows=%s",
                  n_pe, M, K, K2, N, M, N, T, row_counts)

    cp_full = {}
    tot_cyc = 0
    lane_busy_sum = [0] * 16
    rmw_busy_sum  = [0] * 16
    for t in range(T):
        Bd_t, Bc_t, Bv_t, lo, width = slice_b_columns(Bd, Bc_hw, Bv, K2, N, t, T)
        await reset_pulse_cluster(dut)
        dut.N.value = width
        await LBdata_cluster(dut, Bc_t, Bv_t)
        await LBdesc_cluster(dut, Bd_t)
        cp_t, cyc, lane_busy, rmw_busy = await run_cluster(dut, row_counts, n_pe, pe_desc, width)
        tot_cyc += cyc
        for i in range(16):
            lane_busy_sum[i] += lane_busy[i]
            rmw_busy_sum[i]  += rmw_busy[i]
        for key, val in cp_t.items():
            r  = key // C_ROW_STRIDE
            lc = key %  C_ROW_STRIDE
            cp_full[r * C_ROW_STRIDE + (lo + lc)] = val
        dut._log.info("  tile %d: cols[%d:%d] width=%d  cyc=%d  nz=%d",
                      t, lo, lo + width, width, cyc, len(cp_t))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp_full)
    if e == 0:
        dut._log.info("TILED CLUSTER VERIFICATION: PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("TILED CLUSTER VERIFICATION: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in tiled cluster C"

    # Performance: cycles summed over the T passes; util over n_pe lanes x tot_cyc.
    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / (n_pe * tot_cyc) * 100 if tot_cyc > 0 else 0.0 for lb in lane_busy_sum]
    rmw_utils  = [rb / (n_pe * tot_cyc) * 100 if tot_cyc > 0 else 0.0 for rb in rmw_busy_sum]
    dut._log.info("=" * 70)
    dut._log.info("TILED CLUSTER STATISTICS (T=%d, %d PEs):", T, n_pe)
    dut._log.info("  Total wall-time cycles (sum of %d tiles):  %d", T, tot_cyc)
    dut._log.info("  Total MAC ops:                             %d", total_macs)
    dut._log.info("  Per-lane MAC utilization (summed across %d PEs x %d tiles):", n_pe, T)
    for i in range(16):
        dut._log.info("    Lane %d: %8d PE-cycles  →  %.2f%%", i, lane_busy_sum[i], lane_utils[i])
    dut._log.info("  Average MAC util:                          %.2f%%", sum(lane_utils) / 16)
    dut._log.info("  Per-bank RMW utilization (summed across %d PEs x %d tiles):", n_pe, T)
    for i in range(16):
        dut._log.info("    Bank %d: %8d PE-cycles  →  %.2f%%", i, rmw_busy_sum[i], rmw_utils[i])
    dut._log.info("  Average RMW util:                          %.2f%%", sum(rmw_utils) / 16)
    dut._log.info("=" * 70)
    dut._log.info("%d-PE TILED CLUSTER TEST PASSED", n_pe)


@cocotb.test()
async def test_comp_peak_p0(dut):
    """PEAK worst-case on a SINGLE 32-MAC PE: A(512,512) x B(512,512) at 30%
    density (A max row-weight, B max col-weight).  One PE owns the whole problem:
    full A (~78643 <= A_NNZ_SLOT 81920), full B (~78336 <= B_NNZ_SLOT 81920), all
    512 C rows (needs C_ROW_ADDR_BITS=9).  Dataset TC2_PEAK.
        COCOTB_TESTCASE=test_comp_peak_p0 bash run_comp.sh
    """
    sub = os.environ.get('PEAK_SUBDIR', 'TC2_PEAK')
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False, subdir=sub)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True,  subdir=sub)
    assert K == K2
    Bc_hw = [int(c) & 0xFFFF for c in Bc]
    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc_hw, Bv, M, N, K)

    dut._log.info("=" * 70)
    dut._log.info("PEAK SINGLE-PE: A(%d,%d)xB(%d,%d) -> C(%d,%d); A nnz=%d B nnz=%d",
                  M, K, K2, N, M, N, An, Bn)

    await rst(dut); dut.M.value=M; dut.K.value=K; dut.N.value=N
    await LA_val(dut, Av)
    await LAcol(dut, Ac)
    await LBdata(dut, Bc_hw, Bv)
    await LBdesc(dut, Bd)

    cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, Ad, N, to=50000000)
    dut._log.info("PE done at cycle %d, C entries=%d", cyc, len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("PEAK SINGLE-PE VERIFICATION: PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("PEAK SINGLE-PE VERIFICATION: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in peak single-PE C"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    dut._log.info("PEAK SINGLE-PE: %d cyc, %d MAC ops, %.2f%% MAC util",
                  cyc, total_macs, total_macs / (cyc * _NB) * 100 if cyc else 0.0)
    dut._log.info("PEAK SINGLE-PE TEST PASSED")


@cocotb.test()
async def test_comp_peak_cluster(dut):
    """PEAK worst-case demand: A(512,512) x B(512,512) at 30% density, where A has
    max ROW weight (153 nnz/row) and B has max COLUMN weight (153 nnz/col) — the
    structural worst case for the per-PE A/B buffers.  Dataset: TC2_PEAK.

    SINGLE PASS (no tiling, default PEAK_T=1): each PE holds the FULL B (78336 nnz
    <= B_NNZ_SLOT=81920) and 1/2 of A (~39168 <= A_NNZ_SLOT_PER_PE=40960).  At N_PE=2,
    M=512 -> 256 rows/PE so C needs C_ROW_ADDR_BITS=8 (the new default).  Run with:
        COCOTB_TESTCASE=test_comp_peak_cluster bash run_cluster.sh
    Set PEAK_T=2 to fall back to output-column tiling (smaller resident B).
    """
    T = int(os.environ.get('PEAK_T', '1'))
    sub = os.environ.get('PEAK_SUBDIR', 'TC2_PEAK')   # override for smaller dense repros
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False, subdir=sub)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True,  subdir=sub)
    assert K == K2
    Bc_hw = [int(c) & 0xFFFF for c in Bc]
    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc_hw, Bv, M, N, K)

    await rst_cluster(dut)
    n_pe = int(dut.n_pe_sig.value)

    # Round-robin A row partition (structural, no matrix-feature analysis).
    pe_desc = [[] for _ in range(n_pe)]
    pe_val  = [[] for _ in range(n_pe)]
    pe_col  = [[] for _ in range(n_pe)]
    for ri in range(M):
        pid  = ri % n_pe
        gr   = a_desc_crow(Ad[ri]); nnza = a_desc_nnz(Ad[ri]); gs = a_desc_off(Ad[ri])
        ls   = len(pe_val[pid])
        for u in range(nnza):
            pe_val[pid].append(Av[gs + u]); pe_col[pid].append(Ac[gs + u] & 0xFFFF)
        pe_desc[pid].append(a_desc(ls, nnza, gr))
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]
    a_nnz_pe   = [len(pe_val[p])  for p in range(n_pe)]

    dut.M.value = M; dut.K.value = K
    for pid in range(n_pe):                      # A loaded ONCE, reused every pass
        await LA_val_pe(dut, pid, pe_val[pid])
        await LAcol_pe(dut, pid, pe_col[pid])

    dut._log.info("=" * 70)
    dut._log.info("%d-PE PEAK CLUSTER: A(%d,%d)xB(%d,%d) -> C(%d,%d), T=%d", n_pe, M, K, K2, N, M, N, T)
    dut._log.info("  A rows/PE=%s  A nnz/PE=%s (slot %d)  B nnz=%d (tile~%d, slot %d)",
                  row_counts, a_nnz_pe, 40960, Bn, Bn // T, 81920)

    cp_full = {}
    tot_cyc = 0
    lane_busy_sum = [0] * 16
    rmw_busy_sum  = [0] * 16
    for t in range(T):
        Bd_t, Bc_t, Bv_t, lo, width = slice_b_columns(Bd, Bc_hw, Bv, K2, N, t, T)
        await reset_pulse_cluster(dut)
        dut.N.value = width
        await LBdata_cluster(dut, Bc_t, Bv_t)
        await LBdesc_cluster(dut, Bd_t)
        cp_t, cyc, lane_busy, rmw_busy = await run_cluster(dut, row_counts, n_pe, pe_desc, width)
        tot_cyc += cyc
        for i in range(16):
            lane_busy_sum[i] += lane_busy[i]
            rmw_busy_sum[i]  += rmw_busy[i]
        for key, val in cp_t.items():
            r  = key // C_ROW_STRIDE
            lc = key %  C_ROW_STRIDE
            cp_full[r * C_ROW_STRIDE + (lo + lc)] = val
        dut._log.info("  tile %d: cols[%d:%d] width=%d  cyc=%d  nz=%d",
                      t, lo, lo + width, width, cyc, len(cp_t))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp_full)
    if e == 0:
        dut._log.info("PEAK CLUSTER VERIFICATION: PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("PEAK CLUSTER VERIFICATION: FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches in peak cluster C"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    lane_utils = [lb / (n_pe * tot_cyc) * 100 if tot_cyc > 0 else 0.0 for lb in lane_busy_sum]
    rmw_utils  = [rb / (n_pe * tot_cyc) * 100 if tot_cyc > 0 else 0.0 for rb in rmw_busy_sum]
    dut._log.info("=" * 70)
    dut._log.info("PEAK CLUSTER STATISTICS (T=%d, %d PEs):", T, n_pe)
    dut._log.info("  Total wall-time cycles (sum of %d tiles):  %d", T, tot_cyc)
    dut._log.info("  Total MAC ops:                             %d", total_macs)
    dut._log.info("  Average MAC util:                          %.2f%%", sum(lane_utils) / 16)
    dut._log.info("  Average RMW util:                          %.2f%%", sum(rmw_utils) / 16)
    dut._log.info("=" * 70)
    dut._log.info("%d-PE PEAK CLUSTER TEST PASSED", n_pe)


#=========================================================================
# Elementwise (C = A +/- B) — single PE
#=========================================================================
@cocotb.test()
async def test_elementwise_p0(dut):
    """Single-PE elementwise: C = A + B and C = A - B, both M x N sparse."""
    M, N = 24, 48
    A_rows = gen_sparse_rows(M, N, 0.20, seed=1)
    B_rows = gen_sparse_rows(M, N, 0.20, seed=2)
    Ad, Ac, Av = pack_csr(A_rows, is_B=False)
    Bd, Bc, Bv = pack_csr(B_rows, is_B=True)

    A_nnz = sum(len(A_rows[r]) for r in range(M))
    B_nnz = sum(len(B_rows[r]) for r in range(M))
    total_elem_ops = A_nnz + B_nnz

    for sub in (0, 1):
        gf = golden_addsub(A_rows, B_rows, M, N, sub)
        await rst(dut)
        dut.M.value = M; dut.K.value = N; dut.N.value = N
        dut.op_mode.value = 1; dut.op_sub.value = sub
        await LA_val(dut, Av); await LAcol(dut, Ac)
        await LBdata(dut, Bc, Bv)
        await LBdesc(dut, Bd)

        op_name = "SUB" if sub else "ADD"
        dut._log.info("=" * 70)
        dut._log.info("ELEMENTWISE %s: C(%d,%d) = A %s B", op_name,
                      M, N, "-" if sub else "+")
        cp, cyc, lane_busy, rmw_busy = await run_pe(dut, M, Ad, N, to=2000000)
        e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
        if e == 0:
            dut._log.info("Verification: PASSED (%d nz correct, %d z correct)", nz_ok, z_ok)
        else:
            dut._log.error("Verification: FAILED (%d mismatches)", e)
        assert e == 0, f"{e} mismatches (sub={sub})"

        lane_utils = [lb / cyc * 100 if cyc > 0 else 0.0 for lb in lane_busy]
        rmw_utils  = [rb / cyc * 100 if cyc > 0 else 0.0 for rb in rmw_busy]
        dut._log.info("--- ELEMENTWISE %s STATISTICS ---", op_name)
        dut._log.info("  Total cycles:        %d", cyc)
        dut._log.info("  Total elem ops:      %d  (A_nnz=%d + B_nnz=%d)",
                      total_elem_ops, A_nnz, B_nnz)
        dut._log.info("  Throughput:          %.2f ops/cycle", total_elem_ops / cyc if cyc else 0)
        dut._log.info("  Avg MAC utilization: %.2f%%  (1-lane/op, theoretical max=6.25%%)",
                      sum(lane_utils) / 16)
        dut._log.info("  Avg RMW utilization: %.2f%%", sum(rmw_utils) / 16)

    dut._log.info("ELEMENTWISE P0 TEST PASSED")


async def reset_pulse_cluster(dut):
    """Light reset: pulse aresetn (clears tags + FSMs) without restarting the
    clock or the loaded A/B buffers. Used between back-to-back cluster ops."""
    dut.aresetn.value = 0
    await ClockCycles(dut.aclk, 5)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 3)


@cocotb.test()
async def test_elementwise_cluster(dut):
    """N-PE cluster elementwise: C = A + B and C = A - B.
    A is row-partitioned (round-robin); B is broadcast (full, global-row indexed)."""
    M, N = 40, 64
    A_rows = gen_sparse_rows(M, N, 0.20, seed=3)
    B_rows = gen_sparse_rows(M, N, 0.20, seed=4)
    Bd, Bc, Bv = pack_csr(B_rows, is_B=True)          # broadcast B
    Ad_full = [a_desc(0, len(A_rows[r]), r) for r in range(M)]  # for verify (crow=row)

    await rst_cluster(dut)
    n_pe = int(dut.n_pe_sig.value)

    # Round-robin partition of A rows across PEs (no matrix-feature analysis).
    pe_desc = [[] for _ in range(n_pe)]
    pe_val  = [[] for _ in range(n_pe)]
    pe_col  = [[] for _ in range(n_pe)]
    for r in range(M):
        pid = r % n_pe
        off = len(pe_val[pid])
        for (c, v) in A_rows[r]:
            pe_val[pid].append(int_to_fp16_bits(v)); pe_col[pid].append(c)
        pe_desc[pid].append(a_desc(off, len(A_rows[r]), r))
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]

    # Total elem ops = A nnz + B nnz (same for ADD and SUB)
    cl_A_nnz = sum(len(A_rows[r]) for r in range(M))
    cl_B_nnz = sum(len(B_rows[r]) for r in range(M))
    cl_total_elem_ops = cl_A_nnz + cl_B_nnz

    dut.M.value = M; dut.K.value = N; dut.N.value = N
    for pid in range(n_pe):
        await LA_val_pe(dut, pid, pe_val[pid])
        await LAcol_pe(dut, pid, pe_col[pid])
    await LBdata_cluster(dut, Bc, Bv)
    await LBdesc_cluster(dut, Bd)

    for sub in (0, 1):
        await reset_pulse_cluster(dut)          # clear tags; A/B buffers persist
        dut.op_mode.value = 1; dut.op_sub.value = sub
        gf = golden_addsub(A_rows, B_rows, M, N, sub)
        op_name = "SUB" if sub else "ADD"
        dut._log.info("=" * 70)
        dut._log.info("CLUSTER ELEMENTWISE %s: C(%d,%d), %d PEs, rows=%s",
                      op_name, M, N, n_pe, row_counts)
        cp, cyc, lane_busy, rmw_busy = await run_cluster(dut, row_counts, n_pe, pe_desc, N)
        e, nz_ok, z_ok = verify(dut, M, N, Ad_full, gf, cp)
        if e == 0:
            dut._log.info("Verification: PASSED (%d nz, %d z)", nz_ok, z_ok)
        else:
            dut._log.error("Verification: FAILED (%d mismatches)", e)
        assert e == 0, f"{e} mismatches (sub={sub})"

        lane_utils = [lb / (n_pe * cyc) * 100 if cyc > 0 else 0.0 for lb in lane_busy]
        rmw_utils  = [rb / (n_pe * cyc) * 100 if cyc > 0 else 0.0 for rb in rmw_busy]
        dut._log.info("--- CLUSTER ELEMENTWISE %s STATISTICS ---", op_name)
        dut._log.info("  N_PE:                 %d", n_pe)
        dut._log.info("  Wall-time cycles:     %d", cyc)
        dut._log.info("  Total elem ops:       %d  (A_nnz=%d + B_nnz=%d)",
                      cl_total_elem_ops, cl_A_nnz, cl_B_nnz)
        dut._log.info("  Throughput:           %.2f ops/cycle  (%.2f ops/PE-cycle)",
                      cl_total_elem_ops / cyc if cyc else 0,
                      cl_total_elem_ops / (n_pe * cyc) if cyc else 0)
        dut._log.info("  Avg MAC utilization:  %.2f%%  (1-lane/op, theoretical max=6.25%%)",
                      sum(lane_utils) / 16)
        dut._log.info("  Avg RMW utilization:  %.2f%%", sum(rmw_utils) / 16)

    dut._log.info("CLUSTER ELEMENTWISE TEST PASSED")
