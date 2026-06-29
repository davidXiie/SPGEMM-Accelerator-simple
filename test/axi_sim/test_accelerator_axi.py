#!/usr/bin/env python3
"""
AXI test — imports everything from proven test_comp.py.
Uses exact same readback logic as test_comp's run_cluster.
"""
import cocotb, sys, os, struct
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pe_sim'))
from test_comp import (
    load_comp_matrix, partition_a, compute_golden_c, count_total_macs,
    LA_val_pe, LAcol_pe, write_a_desc_pe, LBdata_cluster, LBdesc_cluster,
    fp16_ulp_diff, verify, a_desc_crow, a_desc_nnz, a_desc_off,
    b_desc_nnz, b_desc_off, slice_bits, fp16_from_bits
)
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N = 512; C_ROW_STRIDE = MAX_N


async def rst_cluster(dut):
    dut.aresetn.value = 0
    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 3)
    n_pe = int(dut.n_pe_sig.value)
    await ClockCycles(dut.aclk, 7); dut.aresetn.value = 1; await ClockCycles(dut.aclk, 5)
    dut.start.value = 0; dut.row_count.value = 0
    dut.op_mode.value = 0; dut.op_sub.value = 0
    for pid in range(n_pe):
        getattr(dut, f"a_desc_we_{pid}").value    = 0
        getattr(dut, f"a_desc_waddr_{pid}").value = 0
        getattr(dut, f"a_desc_wdata_{pid}").value = 0
    dut.a_val_we.value  = 0; dut.a_val_waddr.value  = 0; dut.a_val_wdata.value  = 0
    dut.a_col_we.value  = 0; dut.a_col_waddr.value  = 0; dut.a_col_wdata.value  = 0
    dut.b_col_we.value  = 0; dut.b_val_we.value     = 0
    dut.b_desc_we.value = 0
    return n_pe


# EXACT same C drain as test_comp.py's run_cluster
async def drain_c(dut, n_pe, row_counts, N):
    C_RD_ADDR_W = len(dut.c_rd_addr) // n_pe
    MAX_DIM_BITS = len(dut.c_rd_row) // n_pe
    cp = {}; ngroups = (N + 15) // 16
    for pid in range(n_pe):
        for local in range(row_counts[pid]):
            for g in range(ngroups):
                dut.c_rd_en.value   = 1 << pid
                dut.c_rd_addr.value = ((local << 5) | g) << (pid * C_RD_ADDR_W)
                await RisingEdge(dut.aclk); await RisingEdge(dut.aclk)
                r    = slice_bits(dut.c_rd_row.value,  pid * MAX_DIM_BITS, MAX_DIM_BITS)
                vals = slice_bits(dut.c_rd_data.value, pid * 16 * 16, 16 * 16)
                for b in range(16):
                    fp16 = (vals >> (b * 16)) & 0xFFFF
                    if fp16 == 0: continue
                    j = g * 16 + b
                    if j < N:
                        cp[r * C_ROW_STRIDE + j] = fp16_from_bits(fp16)
    dut.c_rd_en.value = 0
    return cp


@cocotb.test()
async def test_axi_case1(dut):
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)

    n_pe = await rst_cluster(dut)
    pe_desc, pe_val, pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]

    dut._log.info("=" * 70)
    dut._log.info("AXI TEST: A(%d,%d)×B(%d,%d)→C(%d,%d)", M,K,K2,N,M,N)
    dut._log.info("Rows: %s  Golden: %d nz", " ".join(f"PE{p}={row_counts[p]}" for p in range(n_pe)), len(gv))

    dut.M.value = M; dut.K.value = K; dut.N.value = N

    for pid in range(n_pe):
        await LA_val_pe(dut, pid, pe_val[pid])
        await LAcol_pe(dut, pid, pe_col[pid])
        await write_a_desc_pe(dut, pid, pe_desc[pid])
    await LBdata_cluster(dut, Bc, Bv)
    await LBdesc_cluster(dut, Bd)

    rc_packed = sum(row_counts[p] << (p * 16) for p in range(n_pe))
    dut.row_count.value = rc_packed
    dut.c_rd_en.value = 0; dut.c_rd_addr.value = 0
    dut.start.value = 1; await RisingEdge(dut.aclk); dut.start.value = 0
    dut._log.info("Started...")

    cyc = 0
    while True:
        await RisingEdge(dut.aclk); cyc += 1
        if int(dut.done.value): dut._log.info("[cyc %d] DONE!", cyc); break
        if cyc > 5000000: dut._log.error("TIMEOUT"); break

    cp = await drain_c(dut, n_pe, row_counts, N)
    dut._log.info("C entries: %d", len(cp))

    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0: dut._log.info("PASSED (%d nz, %d z)", nz_ok, z_ok)
    else: dut._log.error("FAILED (%d)", e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    dut._log.info("STATS: cycles=%d MAC=%d ops/cyc=%.2f", cyc, total_macs, total_macs/cyc if cyc else 0)
    dut._log.info("PASSED")
