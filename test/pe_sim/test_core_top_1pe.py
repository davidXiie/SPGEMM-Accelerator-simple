#!/usr/bin/env python3
"""
Cocotb test for core_top_1pe with DDR model.
Loads A(16,16) × B(16,4) small matrix for fast simulation.
"""
import struct, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# Byte address → ddr_mem index (64-byte aligned)
MEM_SHIFT = 6

def _mem_idx(addr):
    return addr >> MEM_SHIFT

# DDR address map (from defines.vh)
DDR_A_GROUPS_BASE = 0x0000_0000
DDR_B_BASE        = 0x0020_0000
DDR_C_DENSE_BASE  = 0x0030_0000
DDR_DESC_BASE     = 0x0040_0000

# Offsets within A/B group
A_ROW_DESC_OFF = 0x0000
A_COL_OFF      = 0x1000
A_VAL_OFF      = 0x9000
B_ROW_DESC_OFF = 0x0000
B_COL_OFF      = 0x1000
B_VAL_OFF      = 0x0000  # B val offset (different from A)

AXI_DATA_W = 512
N_ELEM_PER_BEAT = 32  # 512/16


def fp16_bits(v):
    """int → IEEE 754 FP16 bit pattern."""
    return int.from_bytes(struct.pack('<e', float(v)), 'little')


def fp16_to_float(bits):
    """FP16 bit pattern → float."""
    return struct.unpack('<e', struct.pack('<H', int(bits) & 0xFFFF))[0]


def write_axi_beat(dut, byte_addr, data_512):
    """Write one 512-bit AXI beat directly into ddr_mem."""
    idx = _mem_idx(byte_addr)
    dut.ddr_mem[idx].value = data_512


def pack_elems_512(elems):
    """Pack a list of 16-bit elements into N 512-bit words."""
    out = []
    beat = 0
    for i, e in enumerate(elems):
        beat |= (e & 0xFFFF) << (16 * (i % N_ELEM_PER_BEAT))
        if (i + 1) % N_ELEM_PER_BEAT == 0:
            out.append(beat)
            beat = 0
    if beat != 0:
        out.append(beat)
    return out


def build_sparse_test():
    """Build A(16,16) × B(16,4) with ~30% density. Returns golden C."""
    import random
    random.seed(42)
    M, K, N = 16, 16, 4

    # A: 16×16, ~30% non-zero
    A_nnz = 0
    A_rows = []
    for r in range(M):
        cols = sorted(random.sample(range(K), random.randint(3, 6)))
        vals = [float((r * 7 + c * 3 + 1) % 5 + 1) for c in cols]
        A_rows.append((cols, vals))
        A_nnz += len(cols)

    # B: 16×4, ~30% non-zero
    B_nnz = 0
    B_rows = []
    for r in range(K):
        cols = sorted(random.sample(range(N), random.randint(1, 3)))
        vals = [float((r * 11 + c * 5 + 1) % 5 + 1) for c in cols]
        B_rows.append((cols, vals))
        B_nnz += len(cols)

    # Golden C
    C = [[0.0] * N for _ in range(M)]
    for ri in range(M):
        for ck, av in zip(A_rows[ri][0], A_rows[ri][1]):
            for cj, bv in zip(B_rows[ck][0], B_rows[ck][1]):
                C[ri][cj] += av * bv

    return M, K, N, A_rows, B_rows, C, A_nnz, B_nnz


def ddr_write_A(dut, M, A_rows):
    """Write A compact row-desc into DDR at DDR_A_GROUPS_BASE."""
    # Build A row_desc (4×16-bit per row = 64-bit)
    # Each row: [a_off[31:0] in 2 words, a_nnz[15:0], c_row[15:0]]
    # Stored as 4 consecutive 16-bit values: off_hi, off_lo, nnz, c_row
    desc_elems = []
    col_elems = []
    val_elems = []
    offset = 0
    for ri in range(M):
        cols, vals = A_rows[ri]
        nnz = len(cols)
        off_lo = offset & 0xFFFF
        off_hi = (offset >> 16) & 0xFFFF
        desc_elems.extend([off_hi, off_lo, nnz, ri])  # 4×16-bit per row
        for c in cols:
            col_elems.append(c)
        for v in vals:
            val_elems.append(fp16_bits(v))
        offset += nnz

    # Write phase 0: row_desc
    for i, beat in enumerate(pack_elems_512(desc_elems)):
        write_axi_beat(dut, DDR_A_GROUPS_BASE + A_ROW_DESC_OFF + i * (AXI_DATA_W // 8), beat)
    # Write phase 1: col
    for i, beat in enumerate(pack_elems_512(col_elems)):
        write_axi_beat(dut, DDR_A_GROUPS_BASE + A_COL_OFF + i * (AXI_DATA_W // 8), beat)
    # Write phase 2: val
    for i, beat in enumerate(pack_elems_512(val_elems)):
        write_axi_beat(dut, DDR_A_GROUPS_BASE + A_VAL_OFF + i * (AXI_DATA_W // 8), beat)

    return len(col_elems)


def ddr_write_B(dut, K, B_rows):
    """Write B compact row-desc into DDR at DDR_B_BASE."""
    desc_elems = []
    col_elems = []
    val_elems = []
    offset = 0
    for ri in range(K):
        cols, vals = B_rows[ri]
        nnz = len(cols)
        off_lo = offset & 0xFFFF
        off_hi = (offset >> 16) & 0xFFFF
        desc_elems.extend([off_hi, off_lo, nnz, 0])  # 4×16-bit per row
        for c in cols:
            col_elems.append(c)
        for v in vals:
            val_elems.append(fp16_bits(v))
        offset += nnz

    for i, beat in enumerate(pack_elems_512(desc_elems)):
        write_axi_beat(dut, DDR_B_BASE + B_ROW_DESC_OFF + i * (AXI_DATA_W // 8), beat)
    for i, beat in enumerate(pack_elems_512(col_elems)):
        write_axi_beat(dut, DDR_B_BASE + B_COL_OFF + i * (AXI_DATA_W // 8), beat)
    for i, beat in enumerate(pack_elems_512(val_elems)):
        write_axi_beat(dut, DDR_B_BASE + B_VAL_OFF + i * (AXI_DATA_W // 8), beat)

    return len(col_elems)


def ddr_write_desc(dut, M, K, N, A_nnz, B_nnz):
    """Write descriptor at DDR_DESC_BASE (12 × 16-bit elements)."""
    desc = [
        M, K, N,         # M, K, N
        A_nnz, B_nnz,    # A/B nnz counts
        1, 0, 0, 0, 0, 0 # padding
    ]
    assert len(desc) == 12
    for i, beat in enumerate(pack_elems_512(desc)):
        write_axi_beat(dut, DDR_DESC_BASE + i * (AXI_DATA_W // 8), beat)


def ddr_read_C(dut, M, N):
    """Read C back from DDR at DDR_C_DENSE_BASE."""
    C = [[0.0] * N for _ in range(M)]
    for ri in range(M):
        for cj in range(N):
            addr = DDR_C_DENSE_BASE + (ri * N + cj) * 2  # ×2 bytes per FP16
            idx = _mem_idx(addr)
            shift = ((addr >> 1) % N_ELEM_PER_BEAT) * 16  # 16-bit word offset
            try:
                val = int(dut.ddr_mem[idx].value)
                bits = ((val >> shift) & 0xFFFF)
                C[ri][cj] = fp16_to_float(bits)
            except Exception:
                C[ri][cj] = 0.0
    return C


@cocotb.test()
async def test_core_top_1pe_small(dut):
    """Small matrix: A(16,16)×B(16,4), verifies full DDR→PE→DDR flow."""
    M, K, N, A_rows, B_rows, C_golden, A_nnz, B_nnz = build_sparse_test()

    # Reset
    dut.aresetn.value = 0
    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 10)
    dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Pre-load DDR
    dut._log.info("=" * 60)
    dut._log.info("TEST: A(%d,%d)×B(%d,%d), A_nnz=%d B_nnz=%d", M, K, K, N, A_nnz, B_nnz)
    ddr_write_desc(dut, M, K, N, A_nnz, B_nnz)
    ddr_write_A(dut, M, A_rows)
    ddr_write_B(dut, K, B_rows)
    dut._log.info("DDR pre-loaded.")

    # Start accelerator
    dut.M.value = M; dut.K.value = K; dut.N.value = N
    dut.cr_start.value = 1
    await RisingEdge(dut.aclk)
    dut.cr_start.value = 0

    # Wait for finish
    to = 500000
    for cy in range(to):
        await RisingEdge(dut.aclk)
        if int(dut.cr_finish.value):
            break
    else:
        dut._log.error("Timeout at %d cycles", to)
        assert False, "Timeout"

    cyc = int(dut.cycle_counter.value)
    dut._log.info("Accelerator done at cycle %d", cyc)

    # Read C back from DDR
    C_hw = ddr_read_C(dut, M, N)

    # Verify
    errors = 0
    for ri in range(M):
        for cj in range(N):
            exp = C_golden[ri][cj]
            act = C_hw[ri][cj]
            if abs(exp - act) > 0.1:
                if errors < 5:
                    dut._log.error("C[%d][%d]: exp=%g act=%g", ri, cj, exp, act)
                errors += 1

    dut._log.info("Verification: errors=%d, cycles=%d", errors, cyc)
    assert errors == 0, f"{errors} mismatches"
    dut._log.info("TEST PASSED")
