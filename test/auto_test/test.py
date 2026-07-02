#!/usr/bin/env python3
"""
Parameterized cocotb test for automated SPGEMM verification.
Reads matrix file paths from environment variables:
  A_INDEX, A_MATRIX, B_INDEX, B_MATRIX, N_PE (default 3)
"""
import cocotb, sys, os, mmap, logging

_this_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _this_dir)  # auto_test first (gen_data with from_arrays)
sys.path.insert(0, os.path.join(_this_dir, '..', 'pe_sim'))
sys.path.insert(0, os.path.join(_this_dir, '..', 'mmap_sim'))
from test_comp import (
    load_comp_matrix, partition_a, compute_golden_c, count_total_macs,
    verify, slice_bits, fp16_from_bits, a_desc_crow
)
from axi_slave import AXIReadResponder, AXIWriteResponder
from ddr_packer import DDRPacker
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

C_ROW_STRIDE = 512


def _load_matrix_direct(idx_path, mat_path, is_B):
    """Load matrices from direct paths (no test_case_for_reference prefix)."""
    from test_comp import a_desc, b_desc, int_to_fp16_bits
    with open(mat_path) as f:
        mat_lines = [list(map(int, l.split())) for l in f if l.strip()]
    with open(idx_path) as f:
        idx_lines_raw = [list(map(int, l.split())) for l in f]

    if not is_B:
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
        B_cols = len(mat_lines)
        idx_lines = idx_lines_raw[:B_cols]
        B_rows = mat_lines[0][1]
        coo = []
        for col in range(B_cols):
            rows_in_col = idx_lines[col]
            for row in rows_in_col:
                v = (row * 37 + col * 13 + 1) % 7 + 1
                coo.append((row, col, v))
        coo.sort()
        Bd = [b_desc(0, 0) for _ in range(B_rows)]
        Bc_all = [[] for _ in range(B_rows)]
        Bv_all = [[] for _ in range(B_rows)]
        off = 0
        r = 0
        for row_idx, col_idx, val in coo:
            while r < row_idx:
                Bd[r] = b_desc(off, len(Bc_all[r]))
                r += 1
            Bc_all[r].append(col_idx)
            Bv_all[r].append(int_to_fp16_bits(val))
        while r < B_rows:
            Bd[r] = b_desc(off, len(Bc_all[r]))
            r += 1
        Bc = []; Bv = []; total_nnz = 0
        for i in range(B_rows):
            Bc.extend(Bc_all[i]); Bv.extend(Bv_all[i])
            total_nnz += len(Bc_all[i])
        return Bd, Bc, Bv, total_nnz, B_rows, B_cols


def _read_c_from_mmap(memory, M, N, Ad):
    """Read C from mmap matching hardware row-stride formula."""
    cp = {}
    ngroups = (N + 31) // 32
    row_stride = ngroups * 32
    C_BYTE_BASE = 0x0060_0000
    for r in range(M):
        gid = a_desc_crow(Ad[r])
        for j in range(N):
            addr = C_BYTE_BASE + (gid * row_stride + j) * 2
            memory.seek(addr)
            b = memory.read(2)
            if len(b) < 2:
                continue
            val = (b[1] << 8) | b[0]
            if val != 0:
                cp[gid * C_ROW_STRIDE + j] = fp16_from_bits(val)
    return cp


async def _drain_c_from_pe(dut, n_pe, row_counts, N):
    """Read C directly from PE C banks."""
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
    n_pe = int(os.environ.get('N_PE', '3'))
    a_index  = os.environ.get('A_INDEX',  'A_0_Index.txt')
    a_matrix = os.environ.get('A_MATRIX', 'A_0_Matrix.txt')
    b_index  = os.environ.get('B_INDEX',  'B_0_Index.txt')
    b_matrix = os.environ.get('B_MATRIX', 'B_0_Matrix.txt')
    subdir   = os.environ.get('MATRIX_SUBDIR', None)

    # Load matrices — if MATRIX_SUBDIR starts with '.' or is absolute, use as direct path
    if subdir and (subdir.startswith('.') or os.path.isabs(subdir)):
        # Direct path: construct full paths to the index/matrix files
        a_idx = os.path.join(subdir, a_index)
        a_mat = os.path.join(subdir, a_matrix)
        b_idx = os.path.join(subdir, b_index)
        b_mat = os.path.join(subdir, b_matrix)
        # Use a custom loader that doesn't go through test_case_for_reference
        from test_comp import a_desc, int_to_fp16_bits
        Ad, Ac, Av, An, M, K  = _load_matrix_direct(a_idx, a_mat, False)
        Bd, Bc, Bv, Bn, K2, N = _load_matrix_direct(b_idx, b_mat, True)
    elif subdir:
        Ad, Ac, Av, An, M, K  = load_comp_matrix(a_index, a_matrix, False, subdir=subdir)
        Bd, Bc, Bv, Bn, K2, N = load_comp_matrix(b_index, b_matrix, True,  subdir=subdir)
    else:
        Ad, Ac, Av, An, M, K  = load_comp_matrix(a_index, a_matrix, False)
        Bd, Bc, Bv, Bn, K2, N = load_comp_matrix(b_index, b_matrix, True)
    assert K == K2

    gv, gf = compute_golden_c(Ad, Ac, Av, Bd, Bc, Bv, M, N, K)
    pe_desc, pe_val, pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
    row_counts = [len(pe_desc[p]) for p in range(n_pe)]

    dut._log.info("A(%d,%d)×B(%d,%d)→C(%d,%d)  PE rows=%s  Golden nz=%d",
                  M, K, K2, N, M, N,
                  " ".join(str(rc) for rc in row_counts), len(gv))

    # Reset
    dut.aresetn.value = 0; dut.start.value = 0
    dut.ddr_RVALID.value = 0; dut.ddr_RLAST.value = 0
    dut.ddr_RID.value = 0; dut.ddr_RRESP.value = 0; dut.ddr_RDATA.value = 0
    dut.ddr_BVALID.value = 0; dut.ddr_BID.value = 0; dut.ddr_BRESP.value = 0
    dut.ddr_AWREADY.value = 0; dut.ddr_WREADY.value = 0
    dut.c_rd_en.value = 0; dut.c_rd_addr.value = 0
    cocotb.start_soon(Clock(dut.aclk, 10, unit='ns').start())
    await ClockCycles(dut.aclk, 10); dut.aresetn.value = 1
    await ClockCycles(dut.aclk, 5)

    # Pack DDR
    packer = DDRPacker.from_arrays(Ad, Ac, Av, Bd, Bc, Bv, M, K, N, n_pe)
    packer.pack()
    memory = mmap.mmap(-1, 1 << 23)
    memory.seek(0)
    memory.write(packer.mem)

    # Start AXI responders
    slave_rd = AXIReadResponder(dut, memory, prefix="ddr")
    slave_wr = AXIWriteResponder(dut, memory, prefix="ddr")
    cocotb.start_soon(slave_rd.run())
    cocotb.start_soon(slave_wr.run())

    # Launch
    dut.M.value = M; dut.K.value = K; dut.N.value = N
    dut.op_mode.value = 0; dut.op_sub.value = 0
    await RisingEdge(dut.aclk)
    dut.start.value = 1; await RisingEdge(dut.aclk); dut.start.value = 0

    cyc = 0
    while True:
        await RisingEdge(dut.aclk); cyc += 1
        if cyc % 100000 == 0:
            dut._log.info("  [cyc %d]", cyc)
        if int(dut.done.value):
            break
        if cyc > 10000000:
            dut._log.error("TIMEOUT")
            memory.close()
            return

    # Read C from DDR (hardware drain) and PE banks
    cp_ddr = _read_c_from_mmap(memory, M, N, Ad)
    cp_pe  = await _drain_c_from_pe(dut, n_pe, row_counts, N)

    # Verify both paths
    e_ddr, nz_ddr, z_ddr = verify(dut, M, N, Ad, gf, cp_ddr)
    e_pe,  nz_pe,  z_pe  = verify(dut, M, N, Ad, gf, cp_pe)

    total_macs = count_total_macs(Ad, Ac, Bd, M)
    if e_ddr == 0 and e_pe == 0:
        print("RESULT: PASS  DDR=%d/%d  PE=%d/%d  cycles=%d  MAC=%d  ops/cyc=%.2f" % (
                      nz_ddr, z_ddr, nz_pe, z_pe, cyc, total_macs,
                      total_macs / cyc if cyc else 0))
    else:
        if e_ddr: print("RESULT: DDR FAILED (%d mismatches)" % e_ddr)
        if e_pe:  print("RESULT: PE  FAILED (%d mismatches)" % e_pe)

    memory.close()
    assert e_ddr == 0, f"DDR {e_ddr} mismatches"
    assert e_pe  == 0, f"PE  {e_pe} mismatches"
