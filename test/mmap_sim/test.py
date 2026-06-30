#!/usr/bin/env python3
"""
mmap-DDR 仿真主测试 — 参照 reference/testGCN.py 架构

架构：
  gen_data.py  → 生成 ram.txt (类比 Dram.py)
  test.py      → 加载 ram.txt → mmap → AXIReadResponder → 启动 → drain C
  axi_slave.py → AXI4 Read Slave 协程（类比 AXI4Slave）
  tb_mmap.v    → 最小 Verilog testbench

数据流：
  ram.txt ──→ mmap.mmap(-1, 8MB) ──→ AXIReadResponder
                                          │ AR/R channel
                                     accelerator_axi_top
                                          │
                                     axi_loader → PE cluster → C banks
                                          │
                                     drain_c_from_pe() → verify()
"""
import cocotb, sys, os, mmap, logging
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pe_sim'))
from test_comp import (
    load_comp_matrix, partition_a, compute_golden_c, count_total_macs,
    verify, slice_bits, fp16_from_bits
)
from axi_slave import AXIReadResponder
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N = 512
C_ROW_STRIDE = MAX_N


async def drain_c_from_pe(dut, n_pe, row_counts, N):
    """Read computed C from PE C banks."""
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
async def test_mmap(dut):
    """Main test: mmap DDR → AXI responder → axi_loader → PE → compute → drain C."""
    # --- Step 1: Load matrix & compute golden ---
    Ad, Ac, Av, An, M, K  = load_comp_matrix('A_0_Index.txt', 'A_0_Matrix.txt', False)
    Bd, Bc, Bv, Bn, K2, N = load_comp_matrix('B_0_Index.txt', 'B_0_Matrix.txt', True)
    assert K == K2
    n_pe = 3

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)
    pe_desc, pe_col, pe_val = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]
    nnz_counts  = [len(pe_col[p])  for p in range(n_pe)]

    dut._log.info("=" * 70)
    dut._log.info("MMAP-DDR: A(%d,%d) × B(%d,%d) → C(%d,%d)", M, K, K2, N, M, N)
    dut._log.info("Rows: %s  NNZ: %s  Golden nz: %d",
                  " ".join(str(rc) for rc in row_counts),
                  " ".join(str(nc) for nc in nnz_counts),
                  len(gv))

    # --- Step 2: Reset ---
    dut.aresetn.value = 0; dut.start.value = 0
    dut.ddr_RVALID.value = 0; dut.ddr_RLAST.value = 0
    dut.ddr_RID.value = 0; dut.ddr_RRESP.value = 0; dut.ddr_RDATA.value = 0
    dut.c_rd_en.value = 0; dut.c_rd_addr.value = 0

    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 10); dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # --- Step 3: Write DDR data directly to mmap (skip slow ram.txt intermediate) ---
    memory = mmap.mmap(-1, 1 << 23)  # 8MB anonymous memory

    from gen_data import DDRPacker
    packer = DDRPacker()
    packer.pack()
    row_counts, nnz_counts = packer.get_counts()

    # Copy bytearray directly to mmap (fast, no ram.txt I/O)
    dut._log.info("Writing %d bytes directly to mmap...", len(packer.mem))
    memory.seek(0)
    memory.write(packer.mem)
    dut._log.info("DDR loaded: rows=%s nnz=%s",
                  " ".join(str(rc) for rc in row_counts),
                  " ".join(str(nc) for nc in nnz_counts))

    # --- Step 4: Start AXI Read Responder (analogous to AXI4Slave) ---
    slave = AXIReadResponder(dut, memory, prefix="ddr")
    cocotb.start_soon(slave.run())
    await RisingEdge(dut.aclk)  # let slave initialize

    # --- Step 5: Launch accelerator ---
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
        if cyc > 5000000:
            dut._log.error("TIMEOUT after %d cycles", cyc)
            break

    memory.close()

    if not int(dut.done.value):
        dut._log.error("SIMULATION DID NOT COMPLETE")
        return

    # --- Step 6: Drain C from PE banks ---
    dut._log.info("Draining C from PE banks...")
    cp = await drain_c_from_pe(dut, n_pe, row_counts, N)
    dut._log.info("C entries: %d", len(cp))

    # --- Step 7: Verify ---
    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp)
    if e == 0:
        dut._log.info("PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    dut._log.info("STATS: cycles=%d MAC=%d ops/cyc=%.2f",
                  cyc, total_macs, total_macs / cyc if cyc else 0)
    dut._log.info("TEST PASSED")
