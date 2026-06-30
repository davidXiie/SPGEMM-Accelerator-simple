"""
AXI4 Read Slave — Cocotb coroutine that responds to AXI read requests.

Analogous to the reference's AXI4Slave usage:
    AXI4Slave(dut, "m_axi_gmem", clk, memory)

Our custom version works with Icarus VPI (cocotb-bus AXI4Slave requires
submodule access which Icarus doesn't support).  All signals are top-level
ports driven via getattr(dut, signal_name).

Supports:
  - AR channel handshake (ARVALID/ARREADY)
  - R channel burst response (RVALID/RDATA/RLAST/RRESP/RID)
  - 512-bit data beats (32 × 16-bit words per beat)
  - Single-cycle latency (ideal DDR, no CAS/RAS delay)
"""

from cocotb.triggers import RisingEdge


class AXIReadResponder:
    """
    Monitors AR channel, serves read data from a byte-addressable memory
    (e.g. mmap.mmap or bytearray) on the R channel.
    """

    def __init__(self, dut, memory, prefix="ddr"):
        self.dut = dut
        self.mem = memory       # must support .seek(addr) + .read(n)  or be bytearray

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
        """Initialize AXI signals (call before starting coroutine)."""
        self.arready.value = 1   # always ready
        self.rvalid.value  = 0
        self.rdata.value   = 0
        self.rlast.value   = 0
        self.rid.value     = 0
        self.rresp.value   = 0

    def _read_beat(self, word_addr):
        """Read a 512-bit beat (32 consecutive 16-bit words) from memory."""
        beat = 0
        for w in range(32):
            addr = (word_addr + w) * 2
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
                addr   = int(self.araddr.value)    # word address (ADDR * 2 = byte offset)
                length = int(self.arlen.value)      # burst beats - 1
                rid_in = int(self.arid.value)

                # --- Read burst from memory ---
                beat_count = length + 1
                for i in range(beat_count):
                    beat = self._read_beat(addr + i)

                    # Drive R channel
                    if i > 0:
                        await RisingEdge(self.dut.aclk)

                    self.rvalid.value = 1
                    self.rid.value    = rid_in
                    self.rdata.value  = beat
                    self.rresp.value  = 0       # OKAY
                    self.rlast.value  = 1 if i == (beat_count - 1) else 0

                    # Wait for master to accept (RREADY)
                    while not int(self.rready.value):
                        await RisingEdge(self.dut.aclk)

                # --- End of burst ---
                await RisingEdge(self.dut.aclk)
                self.rvalid.value = 0
                self.rlast.value  = 0
