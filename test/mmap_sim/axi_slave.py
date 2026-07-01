"""
AXI4 Read + Write Slave — Cocotb coroutines that respond to AXI requests.

AXIReadResponder:  monitors AR channel, serves read data from mmap on R channel.
AXIWriteResponder: monitors AW/W channels, writes data to mmap, responds on B channel.

All AXI addresses use BYTE addressing (AXI4 standard).
ARSIZE/AWSIZE = 6 → 64-byte beats (512-bit data bus).
Consecutive burst beats are 64 bytes apart.
"""

from cocotb.triggers import RisingEdge


class AXIReadResponder:
    """
    Monitors AR channel, serves read data from a byte-addressable memory
    (e.g. mmap.mmap or bytearray) on the R channel.

    AXI burst beats are spaced 64 bytes apart (512-bit bus, ARSIZE=6).
    Each beat contains 32 consecutive 16-bit words (little-endian).
    """

    def __init__(self, dut, memory, prefix="ddr"):
        self.dut = dut
        self.mem = memory

        # AR channel (master → slave)
        self.arvalid = getattr(dut, f"{prefix}_ARVALID")
        self.arready = getattr(dut, f"{prefix}_ARREADY")
        self.araddr  = getattr(dut, f"{prefix}_ARADDR")
        self.arlen   = getattr(dut, f"{prefix}_ARLEN")
        self.arid    = getattr(dut, f"{prefix}_ARID")

        # R channel (slave → master)
        self.rvalid  = getattr(dut, f"{prefix}_RVALID")
        self.rready  = getattr(dut, f"{prefix}_RREADY")
        self.rdata   = getattr(dut, f"{prefix}_RDATA")
        self.rlast   = getattr(dut, f"{prefix}_RLAST")
        self.rid     = getattr(dut, f"{prefix}_RID")
        self.rresp   = getattr(dut, f"{prefix}_RRESP")

    def reset(self):
        self.arready.value = 1
        self.rvalid.value  = 0
        self.rdata.value   = 0
        self.rlast.value   = 0
        self.rid.value     = 0
        self.rresp.value   = 0

    def _read_beat(self, byte_addr):
        """Read a 512-bit beat (32 consecutive 16-bit words, little-endian)."""
        beat = 0
        for w in range(32):
            addr = byte_addr + w * 2   # 2 bytes per FP16 word
            self.mem.seek(addr)
            b = self.mem.read(2)
            if len(b) < 2:
                val = 0
            else:
                val = (b[1] << 8) | b[0]  # little-endian 16-bit
            beat |= (val & 0xFFFF) << (w * 16)
        return beat

    async def run(self):
        """Main responder loop. Run as a background coroutine."""
        self.reset()

        while True:
            await RisingEdge(self.dut.aclk)

            # --- AR handshake ---
            if int(self.arvalid.value) and int(self.arready.value):
                byte_addr = int(self.araddr.value)   # byte address
                length    = int(self.arlen.value)    # burst beats - 1
                rid_in    = int(self.arid.value)
                beat_count = length + 1

                # Beats are 64 bytes apart (AXI4 standard, ARSIZE=6, 512-bit bus)
                beats = [self._read_beat(byte_addr + i * 64) for i in range(beat_count)]

                for i, beat in enumerate(beats):
                    # Deassert RVALID from previous beat for one clean cycle
                    if i > 0:
                        self.rvalid.value = 0
                        await RisingEdge(self.dut.aclk)

                    # Assert RVALID with new beat data
                    self.rvalid.value = 1
                    self.rid.value    = rid_in
                    self.rdata.value  = beat
                    self.rresp.value  = 0       # OKAY
                    self.rlast.value  = 1 if i == (beat_count - 1) else 0

                    # Let Verilog see rvalid=1 for one full clock cycle
                    await RisingEdge(self.dut.aclk)

                    # Wait for master to accept (RREADY) while keeping RVALID high
                    while not int(self.rready.value):
                        await RisingEdge(self.dut.aclk)

                # --- End of burst ---
                self.rvalid.value = 0
                self.rlast.value  = 0


class AXIWriteResponder:
    """
    Monitors AW/W channels, writes data to memory, responds on B channel.

    AXI burst beats are spaced 64 bytes apart (512-bit bus, AWSIZE=6).
    Supports single-beat bursts (AWLEN=0) and multi-beat bursts.
    """

    def __init__(self, dut, memory, prefix="ddr"):
        self.dut = dut
        self.mem = memory

        # AW channel (master → slave)
        self.awvalid = getattr(dut, f"{prefix}_AWVALID")
        self.awready = getattr(dut, f"{prefix}_AWREADY")
        self.awaddr  = getattr(dut, f"{prefix}_AWADDR")
        self.awlen   = getattr(dut, f"{prefix}_AWLEN")
        self.awid    = getattr(dut, f"{prefix}_AWID")

        # W channel (master → slave)
        self.wvalid  = getattr(dut, f"{prefix}_WVALID")
        self.wready  = getattr(dut, f"{prefix}_WREADY")
        self.wdata   = getattr(dut, f"{prefix}_WDATA")
        self.wstrb   = getattr(dut, f"{prefix}_WSTRB")
        self.wlast   = getattr(dut, f"{prefix}_WLAST")

        # B channel (slave → master)
        self.bvalid  = getattr(dut, f"{prefix}_BVALID")
        self.bready  = getattr(dut, f"{prefix}_BREADY")
        self.bid     = getattr(dut, f"{prefix}_BID")
        self.bresp   = getattr(dut, f"{prefix}_BRESP")

    def reset(self):
        self.awready.value = 1
        self.wready.value  = 1
        self.bvalid.value  = 0
        self.bid.value     = 0
        self.bresp.value   = 0

    def _write_beat(self, byte_addr, data, wstrb_val):
        """Write one 512-bit beat to memory, honouring byte strobes."""
        for w in range(32):
            byte0_en = (wstrb_val >> (w * 2 + 0)) & 1
            byte1_en = (wstrb_val >> (w * 2 + 1)) & 1
            if byte0_en or byte1_en:
                val = (data >> (w * 16)) & 0xFFFF
                addr = byte_addr + w * 2
                b0 = val & 0xFF
                b1 = (val >> 8) & 0xFF
                self.mem.seek(addr)
                if byte0_en and byte1_en:
                    self.mem.write(bytes([b0, b1]))
                elif byte0_en:
                    self.mem.write(bytes([b0]))
                elif byte1_en:
                    self.mem.seek(addr + 1)
                    self.mem.write(bytes([b1]))

    def _val(self, signal):
        """Safely convert a LogicArray to int, returning 0 for X/Z values."""
        try:
            return int(signal.value)
        except ValueError:
            return 0

    async def run(self):
        """Main responder loop. Run as a background coroutine."""
        self.reset()

        aw_pending = False
        aw_addr   = 0
        aw_len    = 0
        w_count   = 0
        b_pending = False

        while True:
            await RisingEdge(self.dut.aclk)

            # --- AW handshake ---
            if not aw_pending and self._val(self.awvalid) and self._val(self.awready):
                aw_addr   = self._val(self.awaddr)
                aw_len    = self._val(self.awlen)
                aw_pending = True
                w_count    = 0

            # --- W handshake ---
            if aw_pending and self._val(self.wvalid) and self._val(self.wready):
                data      = self._val(self.wdata)
                wstrb_val = self._val(self.wstrb)
                beat_addr = aw_addr + w_count * 64
                self._write_beat(beat_addr, data, wstrb_val)
                w_count += 1
                if self._val(self.wlast):
                    aw_pending = False
                    b_pending  = True
                    self.bvalid.value = 1
                    self.bid.value    = 0
                    self.bresp.value  = 0

            # --- B handshake ---
            if b_pending and self._val(self.bvalid) and self._val(self.bready):
                self.bvalid.value = 0
                b_pending = False
