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
    verify, slice_bits, fp16_from_bits, a_desc_crow
)
from axi_slave import AXIReadResponder, AXIWriteResponder
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N = 512
C_ROW_STRIDE = MAX_N


async def monitor_top_fsm(dut):
    """Lightweight monitor: log top-level FSM phase transitions."""
    prev_top = -1
    while True:
        await RisingEdge(dut.aclk)
        try:
            top = int(dut.u_accel.state.value)  # top-level FSM: 0=IDLE,1=LOAD,2=COMPUTE,3=DONE
        except:
            continue
        if top != prev_top:
            names = {0:"IDLE", 1:"LOAD", 2:"COMPUTE", 3:"WAIT_DRAIN", 4:"DRAIN", 5:"DONE"}
            dut._log.warning("[TOP] → %s", names.get(top, "?"))
            prev_top = top


async def read_c_from_mmap(memory, M, N, Ad, C_BYTE_BASE=0x0060_0000):
    """Read C matrix from mmap memory (written by hardware AXI drain).
    Returns a dict keyed by (global_row * C_ROW_STRIDE + col).
    Uses ngroups*32 as effective row stride to match hardware address formula."""
    cp = {}
    ngroups = (N + 31) // 32  # ceil(N/32), matches axi_c_drain ngroups
    row_stride = ngroups * 32
    for r in range(M):
        gid = a_desc_crow(Ad[r])
        for j in range(N):
            byte_addr = C_BYTE_BASE + (gid * row_stride + j) * 2
            memory.seek(byte_addr)
            b = memory.read(2)
            if len(b) < 2:
                val = 0
            else:
                val = (b[1] << 8) | b[0]  # little-endian
            if val != 0:
                cp[gid * C_ROW_STRIDE + j] = fp16_from_bits(val)
    return cp


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
    pe_desc, pe_val, pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
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
    dut.ddr_BVALID.value = 0; dut.ddr_BID.value = 0; dut.ddr_BRESP.value = 0
    dut.ddr_AWREADY.value = 0; dut.ddr_WREADY.value = 0
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

    # Copy bytearray directly to mmap + dump binary file for debug
    dut._log.info("Writing %d bytes directly to mmap...", len(packer.mem))
    memory.seek(0)
    memory.write(packer.mem)
    # Also write to sim_build/ddr_dump.bin for inspection
    import os as _os
    _dump = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'sim_build', 'ddr_dump.bin')
    with open(_dump, 'wb') as f:
        f.write(bytes(packer.mem))
    dut._log.info("DDR dumped: %d bytes → %s", len(packer.mem), _dump)
    dut._log.info("DDR loaded: rows=%s nnz=%s",
                  " ".join(str(rc) for rc in row_counts),
                  " ".join(str(nc) for nc in nnz_counts))

    # --- Step 4: Start AXI Read + Write Responders ---
    slave_rd = AXIReadResponder(dut, memory, prefix="ddr")
    slave_wr = AXIWriteResponder(dut, memory, prefix="ddr")
    cocotb.start_soon(slave_rd.run())
    cocotb.start_soon(slave_wr.run())
    cocotb.start_soon(monitor_top_fsm(dut))
    await RisingEdge(dut.aclk)

    # --- Step 5: Launch accelerator ---
    dut.M.value = M; dut.K.value = K; dut.N.value = N
    dut.op_mode.value = 0; dut.op_sub.value = 0
    await RisingEdge(dut.aclk)
    dut.start.value = 1; await RisingEdge(dut.aclk); dut.start.value = 0

    dut._log.info("Launched, waiting for done...")
    cyc = 0
    TOP_NAMES = {0:"IDLE", 1:"LOAD", 2:"COMPUTE", 3:"WAIT_DRAIN", 4:"DRAIN", 5:"DONE"}
    while True:
        await RisingEdge(dut.aclk); cyc += 1

        if cyc % 50000 == 0:
            try:
                top = int(dut.u_accel.state.value)
            except:
                top = -1
            dut._log.warning("  [cyc %d] top=%s", cyc, TOP_NAMES.get(top, "?"))

        if int(dut.done.value):
            dut._log.warning("  [cyc %d] DONE!", cyc)
            break
        if cyc > 5000000:
            dut._log.error("TIMEOUT after %d cycles", cyc)
            break

    if not int(dut.done.value):
        dut._log.error("SIMULATION DID NOT COMPLETE")
        return

    # --- Step 6: Debug — peek at raw C bank entries ---
    C_RD_ADDR_W = len(dut.c_rd_addr) // n_pe
    MAX_DIM_BITS_local = len(dut.c_rd_row) // n_pe
    dut._log.warning("=== DEBUG: Raw C bank dump (first 3 rows of each PE) ===")
    for pid in range(n_pe):
        nrows = row_counts[pid]
        ngrps = (N + 15) // 16
        for local in range(min(3, nrows)):
            # Read first group (grp=0) to get row id and first 16 values
            dut.c_rd_en.value   = 1 << pid
            dut.c_rd_addr.value = ((local << 5) | 0) << (pid * C_RD_ADDR_W)
            await RisingEdge(dut.aclk)
            await RisingEdge(dut.aclk)
            r = slice_bits(dut.c_rd_row.value, pid * MAX_DIM_BITS_local, MAX_DIM_BITS_local)
            vals = slice_bits(dut.c_rd_data.value, pid * 16 * 16, 16 * 16)
            nonzero = [(b, vals >> (b*16) & 0xFFFF) for b in range(16) if (vals >> (b*16) & 0xFFFF) != 0]
            dut._log.warning("  PE%d local=%d global_row=%d nonzero=%s",
                             pid, local, r,
                             " ".join(f"col{j}={v:#06x}" for j, v in nonzero[:5]))
    dut.c_rd_en.value = 0
    dut._log.warning("=== End raw dump ===")

    # --- Step 7: Read C from mmap (written by hardware AXI drain) ---
    dut._log.info("Reading C from mmap (hardware AXI drain)...")
    cp_mmap = await read_c_from_mmap(memory, M, N, Ad)
    dut._log.info("C entries (from mmap): %d", len(cp_mmap))

    # --- Cross-check: also read directly from PE C banks ---
    dut._log.info("Reading C from PE banks (Python drain, for cross-check)...")
    cp_pe = await drain_c_from_pe(dut, n_pe, row_counts, N)
    dut._log.info("C entries (from PE): %d", len(cp_pe))

    # --- Verify hardware drain results ---
    e, nz_ok, z_ok = verify(dut, M, N, Ad, gf, cp_mmap)

    # --- Dump C results to text files in sim_build/ ---
    out_dir = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), 'sim_build')
    _os.makedirs(out_dir, exist_ok=True)
    fgold     = _os.path.join(out_dir, 'results_golden.txt')
    fhw_ddr   = _os.path.join(out_dir, 'results_hardware_ddr.txt')
    fhw_pe    = _os.path.join(out_dir, 'results_hardware_pe.txt')

    def _golden(gid, j): return float(gf[gid][j])
    def _hw_ddr(gid, j): return float(cp_mmap.get(gid * 512 + j, 0.0))
    def _hw_pe(gid, j):  return float(cp_pe.get(gid * 512 + j, 0.0))

    for fname, getter, label in [(fgold, _golden, "Golden"),
                                  (fhw_ddr, _hw_ddr, "Hardware-DDR"),
                                  (fhw_pe, _hw_pe, "Hardware-PE")]:
        with open(fname, 'w') as f:
            f.write(f"# {label} C: {M}×{N}\n")
            for ri in range(M):
                gid = a_desc_crow(Ad[ri])
                for j in range(N):
                    v = getter(gid, j)
                    if v != 0.0:
                        f.write(f"{gid:4d} {j:4d} {v:12.6f}\n")
        dut._log.info("Dumped %s → %s", label, fname)

    if e == 0:
        dut._log.warning("PASSED (%d nz, %d z)", nz_ok, z_ok)
    else:
        dut._log.error("FAILED (%d mismatches)", e)
    assert e == 0, f"{e} mismatches"

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    dut._log.info("STATS: cycles=%d MAC=%d ops/cyc=%.2f",
                  cyc, total_macs, total_macs / cyc if cyc else 0)
    memory.close()
    dut._log.info("TEST PASSED")
