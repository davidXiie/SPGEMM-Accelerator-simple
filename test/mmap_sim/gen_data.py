#!/usr/bin/env python3
"""
Data generator — analogous to reference Dram.py.

Pre-computes load-balanced partitioned A and broadcast B, packs them into
a binary blob (ram.txt) matching the DDR layout expected by axi_loader.v.

DDR layout (16-bit word addresses, non-overlapping per-PE zones):
  Offset   Size    Content
  ────────────────────────────────────────
  0x0000    6      Header: pe_rows[0..2], pe_nnz[0..2]
  0x0100   var     PE0: A_desc (4w/row) | A_col (nnz) | A_val (nnz)
  0x2000   var     PE1: A_desc | A_col | A_val
  0x4000   var     PE2: A_desc | A_col | A_val
  0x8000   var     B:   B_desc (2w/row) | B_col (nnz) | B_val (nnz)

Usage:
  python gen_data.py                    # generates ram.txt
  python gen_data.py A_0_Index.txt ...  # custom matrix files
"""
import sys, os, struct

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pe_sim'))
from test_comp import load_comp_matrix, partition_a

# DDR zone constants (word addresses, must match axi_loader.v layout).
# AXI addresses in hardware use BYTE addressing = word_addr * 2.
# PE zone = 0x12000 words each = 0x24000 bytes.
PE_STRIDE_WORDS = 0x012000                     # 73,728 words per PE zone
DDR_HEADER      = 0x000000

# PE bases computed dynamically in DDRPacker.__init__():
#   PE0 = 0x000100 (special, leaves room for header padding)
#   PE_i (i>0) = i * PE_STRIDE_WORDS
# B base = N_PE * PE_STRIDE_WORDS

TOTAL_SIZE = 1 << 22  # 4M words = 8MB


class DDRPacker:
    """Packs partitioned A/B into DDR binary layout."""

    def __init__(self, a_index='A_0_Index.txt', a_matrix='A_0_Matrix.txt',
                 b_index='B_0_Index.txt', b_matrix='B_0_Matrix.txt', n_pe=3):
        self.n_pe = n_pe
        self.mem = bytearray(TOTAL_SIZE * 2)  # 2 bytes per word

        # Compute PE base addresses (word addresses)
        # PE0 at 0x000100 (header padding); PE_i (i>0) at i * PE_STRIDE_WORDS
        self.pe_bases = []
        for i in range(n_pe):
            if i == 0:
                self.pe_bases.append(0x000100)
            else:
                self.pe_bases.append(i * PE_STRIDE_WORDS)
        self.b_base = n_pe * PE_STRIDE_WORDS  # B zone starts after all PE zones
        Ad, Ac, Av, An, M, K   = load_comp_matrix(a_index, a_matrix, False)
        Bd, Bc, Bv, Bn, K2, N = load_comp_matrix(b_index, b_matrix, True)
        assert K == K2
        self.M, self.K, self.N, self.n_pe = M, K, N, n_pe
        # partition_a returns (pe_desc, pe_val, pe_col) — note val BEFORE col!
        self.pe_desc, self.pe_val, self.pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
        self.Bd, self.Bc, self.Bv = Bd, Bc, Bv

    def w16(self, addr, val):
        """Write 16-bit word at byte offset addr*2."""
        self.mem[addr * 2 : addr * 2 + 2] = struct.pack('<H', val & 0xFFFF)

    def r16(self, addr):
        """Read 16-bit word for verification."""
        return struct.unpack('<H', self.mem[addr * 2 : addr * 2 + 2])[0]

    def pack(self):
        """Pack all data into memory buffer."""
        row_counts = [len(self.pe_desc[p]) for p in range(self.n_pe)]
        nnz_counts  = [len(self.pe_col[p])  for p in range(self.n_pe)]

        # -- Header (2 words per PE: rows, nnz) --
        print(f"Header @ 0x{DDR_HEADER:04X}: rows={row_counts} nnz={nnz_counts}")
        for pid in range(self.n_pe):
            self.w16(DDR_HEADER + pid * 2 + 0, row_counts[pid])
            self.w16(DDR_HEADER + pid * 2 + 1, nnz_counts[pid])

        # -- Per-PE A data --
        for pid in range(self.n_pe):
            base = self.pe_bases[pid]
            desc = self.pe_desc[pid]
            col  = self.pe_col[pid]
            val  = self.pe_val[pid]

            # A_desc: 4 words per row (36-bit padded to 64-bit)
            for ri, d in enumerate(desc):
                off = base + ri * 4
                self.w16(off + 0, (d >>  0) & 0xFFFF)
                self.w16(off + 1, (d >> 16) & 0xFFFF)
                self.w16(off + 2, (d >> 32) & 0xFFFF)
                self.w16(off + 3, 0)

            # A_col: starts at base + 0x0400 (gap after 1K desc region)
            col_off = base + 0x0400
            for i, v in enumerate(col):
                self.w16(col_off + i, v & 0xFFFF)

            # A_val: starts at base + 0x9000 (large gap after col, up to 32K nnz)
            val_off = base + 0x9000
            for i, v in enumerate(val):
                self.w16(val_off + i, v & 0xFFFF)

            print(f"PE{pid} @ 0x{base:04X}: {len(desc)} rows, {len(col)} nnz")

        # -- B data (broadcast) --
        b_base = self.b_base
        # B_desc: 2 words per row
        for k in range(self.K):
            d = self.Bd[k]
            self.w16(b_base + k * 2 + 0, (d >>  0) & 0xFFFF)
            self.w16(b_base + k * 2 + 1, (d >> 16) & 0xFFFF)

        # B_col at base + 0x400 words
        col_off = b_base + 0x400
        for i, v in enumerate(self.Bc):
            self.w16(col_off + i, v & 0xFFFF)

        # B_val at base + 0x8000 words
        val_off = b_base + 0x8000
        for i, v in enumerate(self.Bv):
            self.w16(val_off + i, v & 0xFFFF)

        print(f"B @ 0x{self.b_base:04X}: {self.K} rows, {len(self.Bc)} nnz")

    def write(self, filename='ram.txt'):
        """Write packed binary to file (same format as reference ram.txt)."""
        # Convert memory to binary string (LSB-first bit order, matching reference)
        # Each byte: binary_repr(byte, 8)[::-1] → reversed bit string
        result = ''
        for byte in self.mem:
            bits = format(byte, '08b')[::-1]  # LSB-first
            result += bits

        with open(filename, 'w') as f:
            f.write(result)
        print(f"Wrote {len(result)//8} bytes → {filename}")

    def get_counts(self):
        """Return row counts for verification."""
        return ([len(self.pe_desc[p]) for p in range(self.n_pe)],
                [len(self.pe_col[p])  for p in range(self.n_pe)])


# =========================================================================
# For loading ram.txt back into mmap (test.py uses this)
# =========================================================================
def load_ram_to_mmap(filename, memory):
    """Load binary ram.txt into an mmap object (matching init_ram() pattern)."""
    addr = 0
    with open(filename, 'r') as f:
        while True:
            chunk = f.read(8)  # 8 bits = 1 byte in LSB-first bit string
            if not chunk:
                break
            byte_val = int(chunk[::-1], 2)  # reverse bits to get actual byte
            memory.seek(addr)
            memory.write(byte_val.to_bytes(1, 'little'))
            addr += 1
    return addr  # total bytes written


# =========================================================================
# CLI
# =========================================================================
if __name__ == '__main__':
    if len(sys.argv) >= 3:
        packer = DDRPacker(a_file=sys.argv[1], b_file=sys.argv[2])
    else:
        packer = DDRPacker()
    packer.pack()
    packer.write()
    print(f"Done. M={packer.M} K={packer.K} N={packer.N} PE={packer.n_pe}")
