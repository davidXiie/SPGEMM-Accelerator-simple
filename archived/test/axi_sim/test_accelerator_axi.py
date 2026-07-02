#!/usr/bin/env python3
"""
AXI-DDR simulation: cocotb pre-loads partitioned A/B into ddr_model via host_wr_*
ports, then axi_loader reads via AXI bus into PE cluster.

C is drained from PE C banks (same as test_comp.py cluster drain).
"""
import cocotb, sys, os, struct
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pe_sim'))
from test_comp import (
    load_comp_matrix, partition_a, compute_golden_c, count_total_macs,
    fp16_ulp_diff, verify, slice_bits, fp16_from_bits
)
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N = 512
C_ROW_STRIDE = MAX_N

# DDR address map (16-bit word addresses, matches axi_loader.v layout)
DDR_HEADER_BASE   = 0x000000
DDR_A_DESC_BASE   = 0x000100   # 256 × 4 words = 1024 words
DDR_A_COL_BASE    = 0x002000   # column indices
DDR_A_VAL_BASE    = 0x018000   # FP16 values
DDR_B_DESC_BASE   = 0x200000   # 512 × 2 words
DDR_B_COL_BASE    = 0x210000   # B columns
DDR_B_VAL_BASE    = 0x290000   # B values


def a_desc_pack(off, nnz, crow):
    """36-bit A descriptor: {3'b0, off[13:0], nnz[9:0], crow[8:0]}"""
    return ((off & 0x3FFF) << 19) | ((nnz & 0x3FF) << 9) | (crow & 0x1FF)


def b_desc_pack(off, nnz):
    """32-bit B descriptor: {off[16:0], 5'b0, nnz[9:0]}"""
    return ((off & 0x1FFFF) << 10) | (nnz & 0x3FF)


async def ddr_write(dut, addr, data):
    """Single-cycle write to ddr_model via host port."""
    dut.host_wr_addr.value = addr & ((1 << 22) - 1)
    dut.host_wr_data.value = data & 0xFFFF
    dut.host_wr_en.value   = 1
    await RisingEdge(dut.aclk)
    dut.host_wr_en.value   = 0


async def load_to_ddr(dut, pe_desc, pe_col, pe_val, Bd, Bc, Bv, n_pe, K):
    """
    Pack partitioned A + full B into DDR layout.
    Header (6 words per PE): row_count[15:0], a_nnz_total[15:0]
    A data per PE: desc, col, val in separate regions.
    B data: broadcast to all PEs (same data for every PE).
    """
    # -- Header --
    row_counts = []
    nnz_counts = []
    a_desc_off = DDR_A_DESC_BASE
    a_col_off  = DDR_A_COL_BASE
    a_val_off  = DDR_A_VAL_BASE

    for pid in range(n_pe):
        rc = len(pe_desc[pid])
        nc = len(pe_col[pid])
        row_counts.append(rc)
        nnz_counts.append(nc)

        # Header: words 0-5 per PE (currently only 2 used)
        await ddr_write(dut, DDR_HEADER_BASE + pid * 6 + 0, rc & 0xFFFF)
        await ddr_write(dut, DDR_HEADER_BASE + pid * 6 + 1, nc & 0xFFFF)

        # A descriptors (each is 36-bit → transport as 3×16-bit words, or 4×16-bit padded)
        for ri, d in enumerate(pe_desc[pid]):
            addr = a_desc_off + ri * 4
            # Little-endian 16-bit words
            await ddr_write(dut, addr + 0, (d >>  0) & 0xFFFF)
            await ddr_write(dut, addr + 1, (d >> 16) & 0xFFFF)
            await ddr_write(dut, addr + 2, (d >> 32) & 0xFFFF)

        a_desc_off += 0x100  # next PE offset
        a_col_off  += 0x4000
        a_val_off  += 0x4000

    # -- A columns & values (continuous region across all PEs) --
    a_col_addr = DDR_A_COL_BASE
    a_val_addr = DDR_A_VAL_BASE
    for pid in range(n_pe):
        for v in pe_col[pid]:
            await ddr_write(dut, a_col_addr, v & 0xFFFF)
            a_col_addr += 1
        for v in pe_val[pid]:
            await ddr_write(dut, a_val_addr, v & 0xFFFF)
            a_val_addr += 1

    # -- B descriptors --
    for k in range(K):
        addr = DDR_B_DESC_BASE + k * 2
        d = Bd[k]
        await ddr_write(dut, addr + 0, (d >>  0) & 0xFFFF)
        await ddr_write(dut, addr + 1, (d >> 16) & 0xFFFF)

    # -- B columns & values --
    for i, v in enumerate(Bc):
        await ddr_write(dut, DDR_B_COL_BASE + i, v & 0xFFFF)
    for i, v in enumerate(Bv):
        await ddr_write(dut, DDR_B_VAL_BASE + i, v & 0xFFFF)

    return row_counts, nnz_counts


async def drain_c_from_pe(dut, n_pe, row_counts, N):
    """Read C from PE C banks (same as test_comp.py cluster drain)."""
    C_RD_ADDR_W = len(dut.c_rd_addr) // n_pe
    MAX_DIM_BITS = len(dut.c_rd_row) // n_pe
    cp = {}
    ngroups = (N + 15) // 16
    for pid in range(n_pe):
        for local in range(row_counts[pid]):
            for g in range(ngroups):
                dut.c_rd_en.value   = 1 << pid
                dut.c_rd_addr.value = ((local << 5) | g) << (pid * C_RD_ADDR_W)
                await RisingEdge(dut.aclk)
                await RisingEdge(dut.aclk)
                r    = slice_bits(dut.c_rd_row.value,  pid * MAX_DIM_BITS, MAX_DIM_BITS)
                vals = slice_bits(dut.c_rd_data.value, pid * 16 * 16, 16 * 16)
                for b in range(16):
                    fp16 = (vals >> (b * 16)) & 0xFFFF
                    if fp16 == 0:
                        continue
                    j = g * 16 + b
                    if j < N:
                        cp[r * C_ROW_STRIDE + j] = fp16_from_bits(fp16)
    dut.c_rd_en.value = 0
    return cp


@cocotb.test()
async def test_axi_case1(dut):
    """Full AXI-DDR pipeline: load DDR → axi_loader → PE compute → drain C."""
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    n_pe = 3

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)
    pe_desc, pe_col, pe_val = partition_a(Ad, Ac, Av, M, n_pe, Bd)

    dut._log.info("=" * 70)
    dut._log.info("AXI-DDR TEST: A(%d,%d)×B(%d,%d)→C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("A nnz=%d  B nnz=%d  Golden nz=%d", An, Bn, len(gv))

    # Reset
    dut.aresetn.value = 0; dut.start.value = 0
    dut.host_wr_en.value = 0; dut.c_rd_en.value = 0
    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 10); dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Load data into DDR model
    dut._log.info("Loading partitioned data into ddr_model...")
    row_counts, nnz_counts = await load_to_ddr(dut, pe_desc, pe_col, pe_val, Bd, Bc, Bv, n_pe, K)
    dut._log.info("Row counts: %s  NNZ: %s",
                  " ".join(str(row_counts[p]) for p in range(n_pe)),
                  " ".join(str(nnz_counts[p]) for p in range(n_pe)))

    # Launch accelerator
    dut.M.value = M; dut.K.value = K; dut.N.value = N
    dut.op_mode.value = 0; dut.op_sub.value = 0
    await RisingEdge(dut.aclk)
    dut.start.value = 1; await RisingEdge(dut.aclk); dut.start.value = 0

    dut._log.info("Launched, waiting for done...")
    cyc = 0
    while True:
        await RisingEdge(dut.aclk); cyc += 1
        if cyc % 100000 == 0:
            dut._log.info("  [cyc %d] waiting...", cyc)
        if int(dut.done.value):
            dut._log.info("  [cyc %d] DONE!", cyc)
            break
        if cyc > 20000000:
            dut._log.error("TIMEOUT")
            break

    # Drain C
    dut._log.info("Draining C from PE banks...")
    cp = await drain_c_from_pe(dut, n_pe, row_counts, N)
    dut._log.info("C entries: %d", len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    dut._log.info("STATS: cycles=%d MAC=%d ops/cyc=%.2f", cyc, total_macs, total_macs / cyc if cyc else 0)
    dut._log.info("AXI-DDR TEST PASSED")
