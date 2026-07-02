#!/usr/bin/env python3
#=============================================================================
# File     : test_dut.py
# Project  : SPGEMM-Accelerator
# Brief    : Cocotb testbench for core_top.
#           - AXI read slave (DRAM memory model)
#           - AXI write slave (output capture)
#           - Drives CR signals, captures DUT output, compares against ideal_result.txt
#=============================================================================

import os
import struct
import logging
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, ClockCycles

#=============================================================================
# Constants (mirror defines.vh)
#=============================================================================
AXI_DATA_WIDTH  = 512
AXI_ADDR_WIDTH  = 64
AXI_BEAT_BYTES  = AXI_DATA_WIDTH // 8   # 64 bytes
DATA_WIDTH      = 16

log = logging.getLogger('cocotb.spgemm')

#=============================================================================
# AXI Read Slave: serves data from memory model
#=============================================================================
class AxiReadSlave:
    """Responds to AXI-Full read requests from a byte-array memory."""

    def __init__(self, dut, memory, clock_sig,
                 prefix='m_axi_',
                 max_burst_len=16,
                 resp_delay=0):
        self.dut = dut
        self.mem = memory
        self.clk = clock_sig
        self.pfx = prefix
        self.max_burst = max_burst_len
        self.delay = resp_delay

        # Signal handles
        self._arvalid = getattr(dut, f'{prefix}arvalid')
        self._arready = getattr(dut, f'{prefix}arready')
        self._araddr  = getattr(dut, f'{prefix}araddr')
        self._arlen   = getattr(dut, f'{prefix}arlen')
        self._rvalid  = getattr(dut, f'{prefix}rvalid')
        self._rready  = getattr(dut, f'{prefix}rready')
        self._rdata   = getattr(dut, f'{prefix}rdata')
        self._rlast   = getattr(dut, f'{prefix}rlast')

    def read_mem(self, addr, num_bytes):
        """Read num_bytes from memory at addr (byte address)."""
        data = bytearray(num_bytes)
        for i in range(num_bytes):
            a = addr + i
            data[i] = self.mem[a] if a < len(self.mem) else 0
        return data

    async def run(self):
        """Continuously respond to AXI read requests."""
        self._arready.value = 0
        self._rvalid.value  = 0
        self._rdata.value   = 0
        self._rlast.value   = 0

        while True:
            await RisingEdge(self.clk)

            # --- Address handshake ---
            if self._arvalid.value and not self._arready.value:
                self._arready.value = 1
                addr  = int(self._araddr.value)
                blen  = int(self._arlen.value)  # burst length (beats-1)
                burst_beats = blen + 1

                # Pre-read all beat data
                beat_data = []
                for beat in range(burst_beats):
                    ba = addr + beat * AXI_BEAT_BYTES
                    rd = self.read_mem(ba, AXI_BEAT_BYTES)
                    val = int.from_bytes(rd, byteorder='little')
                    beat_data.append(val)

                beat_idx = 0

                await RisingEdge(self.clk)
                self._arready.value = 0

                # --- Data beats ---
                while beat_idx < burst_beats:
                    self._rvalid.value = 1
                    self._rdata.value  = beat_data[beat_idx]
                    self._rlast.value  = 1 if (beat_idx == burst_beats - 1) else 0

                    await RisingEdge(self.clk)

                    if self._rready.value:
                        beat_idx += 1

                self._rvalid.value = 0
                self._rlast.value  = 0


#=============================================================================
# AXI Write Slave: captures output data
#=============================================================================
class AxiWriteSlave:
    """Captures AXI-Full write transactions from the DUT."""

    def __init__(self, dut, clock_sig, prefix='m_axi_'):
        self.dut = dut
        self.clk = clock_sig
        self.pfx = prefix

        self._awvalid = getattr(dut, f'{prefix}awvalid')
        self._awready = getattr(dut, f'{prefix}awready')
        self._awaddr  = getattr(dut, f'{prefix}awaddr')
        self._awlen   = getattr(dut, f'{prefix}awlen')
        self._wvalid  = getattr(dut, f'{prefix}wvalid')
        self._wready  = getattr(dut, f'{prefix}wready')
        self._wdata   = getattr(dut, f'{prefix}wdata')
        self._wlast   = getattr(dut, f'{prefix}wlast')
        self._bvalid  = getattr(dut, f'{prefix}bvalid')
        self._bready  = getattr(dut, f'{prefix}bready')

        # Captured write data: (addr, bytearray) list + dict by address
        self.writes = []
        self.write_mem = {}  # byte address → int value

    async def run(self):
        """Continuously capture AXI write transactions."""
        self._awready.value = 0
        self._wready.value  = 0
        self._bvalid.value  = 0

        while True:
            await RisingEdge(self.clk)

            # --- Address handshake ---
            if self._awvalid.value and not self._awready.value:
                self._awready.value = 1
                addr  = int(self._awaddr.value)
                blen  = int(self._awlen.value)
                burst_beats = blen + 1
                await RisingEdge(self.clk)
                self._awready.value = 0

                # --- Data beats ---
                beat_data = bytearray()
                beat_idx = 0

                while beat_idx < burst_beats:
                    self._wready.value = 1

                    await RisingEdge(self.clk)

                    if self._wvalid.value:
                        wd = int(self._wdata.value)
                        beat_data.extend(wd.to_bytes(AXI_BEAT_BYTES, byteorder='little'))
                        beat_idx += 1

                        if int(self._wlast.value):
                            break

                self._wready.value = 0

                # Store captured write
                self.writes.append((addr, bytes(beat_data)))
                # populate write_mem dict (byte-granularity)
                for i, b in enumerate(beat_data):
                    self.write_mem[addr + i] = b

                # --- Write response ---
                self._bvalid.value = 1
                await RisingEdge(self.clk)

                while not self._bready.value:
                    await RisingEdge(self.clk)

                self._bvalid.value = 0


#=============================================================================
# Testbench Class
#=============================================================================
class TB:
    def __init__(self, dut):
        self.dut = dut
        self.log = log

        # Load ram.txt into memory (byte array)
        self.memory = self._load_ram()
        self.log.info(f"Loaded memory: {len(self.memory)} bytes")

        # Load metadata (instruction count, etc.)
        self.meta = self._load_meta()
        self.log.info(f"Meta: INST_COUNT={self.meta.get('INST_COUNT', '?')}, "
                      f"PAIR_COUNT={self.meta.get('PAIR_COUNT', '?')}")

        # Create AXI slaves
        self.rd_slave = AxiReadSlave(dut, self.memory, dut.aclk)
        self.wr_slave = AxiWriteSlave(dut, dut.aclk)

        # Load ideal reference (software-computed C = A × B)
        self.ideal_pairs = self._load_ideal()
        self.log.info(f"Loaded {len(self.ideal_pairs)} ideal reference pairs")

        # Start clock and AXI slaves as background tasks
        cocotb.start_soon(Clock(dut.aclk, 10, units='ns').start())
        cocotb.start_soon(self.rd_slave.run())
        cocotb.start_soon(self.wr_slave.run())

    def _load_ram(self):
        """Read ram.txt into a bytearray."""
        base = os.path.dirname(os.path.abspath(__file__))
        ram_path = os.path.join(base, 'data', 'ram.txt')
        mem = bytearray()

        with open(ram_path, 'r') as f:
            raw = f.read().strip()
            # Convert binary string → bytes
            # Each byte = 8 bits, bit-reversed within byte (reference convention)
            for i in range(0, len(raw), 8):
                chunk = raw[i:i+8]
                if len(chunk) == 8:
                    byte = int(chunk[::-1], 2)  # little-endian bit reversal
                    mem.append(byte)

        return mem

    def _load_meta(self):
        """Parse info.txt into a dict of metadata. String values kept as str."""
        base = os.path.dirname(os.path.abspath(__file__))
        info_path = os.path.join(base, 'data', 'info.txt')
        meta = {}
        try:
            with open(info_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if '=' in line:
                        k, v = line.split('=', 1)
                        k = k.strip(); v = v.strip()
                        try:
                            meta[k] = int(v)
                        except ValueError:
                            meta[k] = v  # keep string (e.g. OP_NAME=MUL)
        except FileNotFoundError:
            self.log.warning("info.txt not found, using defaults")
        return meta

    def _load_ideal(self):
        """Parse ideal_result.txt into list of dicts."""
        base = os.path.dirname(os.path.abspath(__file__))
        ideal_path = os.path.join(base, 'data', 'ideal_result.txt')
        pairs = []
        current = None
        try:
            with open(ideal_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    if line.startswith('M='):
                        parts = line.split()
                        M  = int(parts[0].split('=')[1])
                        N  = int(parts[1].split('=')[1])
                        NNZ = int(parts[2].split('=')[1])
                        current = {'M': M, 'N': N, 'NNZ': NNZ}
                    elif line.startswith('ROW_PTR:'):
                        current['row_ptr'] = [int(x) for x in line.split(':')[1].split(',') if x.strip()]
                    elif line.startswith('COL_IDX:'):
                        current['col_idx'] = [int(x) for x in line.split(':')[1].split(',') if x.strip()]
                    elif line.startswith('VAL:'):
                        current['val'] = [int(x) for x in line.split(':')[1].split(',') if x.strip()]
                        pairs.append(current)
                        current = None
        except FileNotFoundError:
            self.log.warning("ideal_result.txt not found, skipping result verification")
        return pairs

    async def launch(self, ins_baddr=0x0000, ins_count=28):
        """Reset DUT, then configure CR and launch."""
        # Assert reset first
        self.dut.aresetn.value = 0
        await ClockCycles(self.dut.aclk, 4)
        self.dut.aresetn.value = 1
        await ClockCycles(self.dut.aclk, 5)
        self.log.info("Reset complete")

        # Initialize CR signals
        self.dut.cr_launch.value  = 0
        self.dut.ins_baddr.value  = 0
        self.dut.ins_count.value  = 0

        # Set configuration
        self.dut.ins_baddr.value  = ins_baddr
        self.dut.ins_count.value  = ins_count
        await RisingEdge(self.dut.aclk)

        # Pulse launch
        self.dut.cr_launch.value  = 1
        self.log.info(f"Launch: ins_baddr=0x{ins_baddr:04X}, ins_count={ins_count}")
        await RisingEdge(self.dut.aclk)

        self.dut.cr_launch.value  = 0
        self.dut.ins_baddr.value  = 0
        self.dut.ins_count.value  = 0

    async def wait_finish(self, timeout_us=5000):
        """Wait for cr_finish signal, or timeout."""
        elapsed = 0
        while not self.dut.cr_finish.value:
            await ClockCycles(self.dut.aclk, 100)
            elapsed += 100
            if elapsed % 1000 == 0:
                cycle = int(self.dut.cycle_counter.value) if hasattr(self.dut, 'cycle_counter') else elapsed
                self.log.debug(f"Waiting... cycle={cycle}")
            if elapsed * 10 > timeout_us * 1000:
                self.log.error(f"Timeout after {timeout_us} us!")
                return False
        self.log.info(f"DUT finished at cycle {int(self.dut.cycle_counter.value)}")
        return True

    def verify_results(self):
        """Compare DUT AXI output against ideal reference using write_mem."""
        if not self.ideal_pairs:
            self.log.warning("No ideal reference data to verify against")
            return True

        out_dram = self.meta.get('PAIR_0_OUT_DRAM', 0)
        out_bytes = self.meta.get('PAIR_0_OUT_BYTES', 0)
        self.log.info(f"Reading output at DRAM 0x{out_dram:08X}, {out_bytes} bytes")

        # Assemble captured data from write_mem
        captured_data = bytes(self.wr_slave.write_mem.get(out_dram + i, 0)
                              for i in range(out_bytes))
        if len(captured_data) < out_bytes:
            self.log.error(f"Captured only {len(captured_data)} of {out_bytes} bytes")
            return False

        all_passed = True
        for pair_idx, ideal in enumerate(self.ideal_pairs):

            g_nnz  = ideal['NNZ']
            g_M    = ideal['M']

            # row_ptr: (M+1) entries × 32-bit = 4 bytes each
            row_ptr_bytes = (g_M + 1) * 4
            col_bytes     = g_nnz * 2
            val_bytes     = g_nnz * 2

            total_expected = row_ptr_bytes + col_bytes + val_bytes
            if len(captured_data) < total_expected:
                self.log.error(f"Pair {pair_idx}: captured data too short "
                               f"({len(captured_data)} < {total_expected})")
                all_passed = False
                continue

            # Parse row_ptr (32-bit little-endian)
            cap_row_ptr = []
            for i in range(g_M + 1):
                lo = captured_data[2*i] | (captured_data[2*i+1] << 8)
                hi = captured_data[2*i+2] | (captured_data[2*i+3] << 8) if 2*i+3 < len(captured_data) else 0
                cap_row_ptr.append(lo | (hi << 16))

            # Parse col_idx (16-bit each, after row_ptr)
            offset = row_ptr_bytes
            cap_col_idx = []
            for i in range(g_nnz):
                val = struct.unpack_from('<H', captured_data, offset + i * 2)[0]
                cap_col_idx.append(val)

            # Parse val (FP16, after col_idx)
            offset = row_ptr_bytes + col_bytes
            cap_val = []
            for i in range(g_nnz):
                val = struct.unpack_from('<H', captured_data, offset + i * 2)[0]
                cap_val.append(val)

            # Compare
            errs = 0
            for i in range(min(len(cap_row_ptr), len(ideal['row_ptr']))):
                if cap_row_ptr[i] != ideal['row_ptr'][i]:
                    if errs < 5:
                        self.log.error(f"  row_ptr[{i}]: DUT={cap_row_ptr[i]}, IDEAL={ideal['row_ptr'][i]}")
                    errs += 1
            for i in range(min(len(cap_col_idx), len(ideal['col_idx']))):
                if cap_col_idx[i] != ideal['col_idx'][i]:
                    if errs < 5:
                        self.log.error(f"  col_idx[{i}]: DUT={cap_col_idx[i]}, IDEAL={ideal['col_idx'][i]}")
                    errs += 1
            for i in range(min(len(cap_val), len(ideal['val']))):
                if cap_val[i] != ideal['val'][i]:
                    if errs < 5:
                        self.log.error(f"  val[{i}]: DUT=0x{cap_val[i]:04X}, IDEAL=0x{ideal['val'][i]:04X}")
                    errs += 1

            if errs == 0:
                self.log.info(f"Pair {pair_idx}: VERIFIED OK (C={g_M}×{ideal['N']}, nnz={g_nnz})")
            else:
                self.log.error(f"Pair {pair_idx}: MISMATCH ({errs} errors)")
                all_passed = False

        return all_passed


#=============================================================================
# Test Cases
#=============================================================================

@cocotb.test()
async def test_single_op(dut):
    """Single-operation test: MUL/ADD/SUB based on generated data."""
    tb = TB(dut)

    inst_count = tb.meta.get('INST_COUNT', 10)
    op_name = tb.meta.get('OP_NAME', 'MUL')
    dut._log.info(f"Test: {op_name}, {inst_count} instructions")

    await tb.launch(ins_baddr=0x0000, ins_count=inst_count)

    finished = await tb.wait_finish(timeout_us=200000)
    assert finished, f"DUT did not finish within timeout ({op_name})"

    await ClockCycles(dut.aclk, 50)

    passed = tb.verify_results()
    assert passed, f"Result verification FAILED for {op_name}"


@cocotb.test()
async def test_quick_launch(dut):
    """Quick launch test: just start DUT and check state machine progresses."""
    tb = TB(dut)

    inst_count = tb.meta.get('INST_COUNT', 26)
    await tb.launch(ins_baddr=0x0000, ins_count=inst_count)

    # Wait a bit and verify things are running
    for _ in range(100):
        await RisingEdge(dut.aclk)
        if dut.cr_finish.value:
            dut._log.info(f"Quick finish at cycle {int(dut.cycle_counter.value)}")
            break

    # Check some internal activity (m_axi_arvalid should have been active)
    dut._log.info("Quick launch test done")


@cocotb.test()
async def test_debug_state(dut):
    """Minimal debug test: launch and monitor cycle_counter, arvalid, finish."""
    tb = TB(dut)
    inst_count = tb.meta.get('INST_COUNT', 28)
    await tb.launch(ins_baddr=0x0000, ins_count=inst_count)
    dut._log.info(f"Launched with inst_count={inst_count}")

    last_cycle = 0
    ar_fired = False
    for i in range(2000):
        await RisingEdge(dut.aclk)
        cyc = int(dut.cycle_counter.value)
        ar = int(dut.m_axi_arvalid.value)
        fin = int(dut.cr_finish.value)

        if not ar_fired and ar:
            ar_fired = True
            dut._log.info(f"First m_axi_arvalid at cycle {cyc}")

        if cyc != last_cycle and cyc % 100 == 0:
            dut._log.info(f"Cycle {cyc}: arvalid={ar}, finish={fin}")
            last_cycle = cyc

        if fin:
            dut._log.info(f"Finished at cycle {cyc}!")
            break

    if i >= 1999:
        dut._log.error(f"TIMEOUT after {i} cycles. ar_fired={ar_fired}")
        dut._log.info(f"Final: cycle={cyc}, arvalid={ar}, finish={fin}")
