#!/usr/bin/env python3
"""
Data generator — packed DDR layout for axi_loader.v.
Extended with from_arrays() for in-memory matrix generation.
"""
import sys, os, struct

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pe_sim'))
from test_comp import load_comp_matrix, partition_a

PE_STRIDE_WORDS = 0x012000
DDR_HEADER      = 0x000000
TOTAL_SIZE      = 1 << 22   # 4M words = 8 MB


class DDRPacker:
    """Packs partitioned A/B into DDR binary layout."""

    def __init__(self, a_index='A_0_Index.txt', a_matrix='A_0_Matrix.txt',
                 b_index='B_0_Index.txt', b_matrix='B_0_Matrix.txt', n_pe=3):
        self.n_pe = n_pe
        self.mem = bytearray(TOTAL_SIZE * 2)
        self.pe_bases = []
        for i in range(n_pe):
            self.pe_bases.append(0x000100 if i == 0 else i * PE_STRIDE_WORDS)
        self.b_base = n_pe * PE_STRIDE_WORDS

        Ad, Ac, Av, An, M, K   = load_comp_matrix(a_index, a_matrix, False)
        Bd, Bc, Bv, Bn, K2, N = load_comp_matrix(b_index, b_matrix, True)
        assert K == K2
        self._init_from_arrays(Ad, Ac, Av, Bd, Bc, Bv, M, K, N, n_pe)

    @classmethod
    def from_arrays(cls, Ad, Ac, Av, Bd, Bc, Bv, M, K, N, n_pe=3):
        """Construct DDRPacker from pre-loaded descriptor arrays (no file I/O)."""
        packer = cls.__new__(cls)
        packer.n_pe = n_pe
        packer.mem = bytearray(TOTAL_SIZE * 2)
        packer.pe_bases = []
        for i in range(n_pe):
            packer.pe_bases.append(0x000100 if i == 0 else i * PE_STRIDE_WORDS)
        packer.b_base = n_pe * PE_STRIDE_WORDS
        packer._init_from_arrays(Ad, Ac, Av, Bd, Bc, Bv, M, K, N, n_pe)
        return packer

    def _init_from_arrays(self, Ad, Ac, Av, Bd, Bc, Bv, M, K, N, n_pe):
        self.M, self.K, self.N = M, K, N
        self.pe_desc, self.pe_val, self.pe_col = partition_a(Ad, Ac, Av, M, n_pe, Bd)
        self.Bd, self.Bc, self.Bv = Bd, Bc, Bv

    def w16(self, addr, val):
        self.mem[addr * 2 : addr * 2 + 2] = struct.pack('<H', val & 0xFFFF)

    def pack(self):
        row_counts = [len(self.pe_desc[p]) for p in range(self.n_pe)]
        nnz_counts  = [len(self.pe_col[p])  for p in range(self.n_pe)]
        print(f"  Header: rows={row_counts} nnz={nnz_counts}")
        for pid in range(self.n_pe):
            self.w16(DDR_HEADER + pid * 2 + 0, row_counts[pid])
            self.w16(DDR_HEADER + pid * 2 + 1, nnz_counts[pid])

        for pid in range(self.n_pe):
            base = self.pe_bases[pid]
            desc, col, val = self.pe_desc[pid], self.pe_col[pid], self.pe_val[pid]
            for ri, d in enumerate(desc):
                off = base + ri * 4
                self.w16(off + 0, (d >>  0) & 0xFFFF)
                self.w16(off + 1, (d >> 16) & 0xFFFF)
                self.w16(off + 2, (d >> 32) & 0xFFFF)
                self.w16(off + 3, 0)
            col_off = base + 0x0400
            for i, v in enumerate(col):
                self.w16(col_off + i, v & 0xFFFF)
            val_off = base + 0x9000
            for i, v in enumerate(val):
                self.w16(val_off + i, v & 0xFFFF)

        b_base = self.b_base
        for k in range(self.K):
            d = self.Bd[k]
            self.w16(b_base + k * 2 + 0, (d >>  0) & 0xFFFF)
            self.w16(b_base + k * 2 + 1, (d >> 16) & 0xFFFF)
        col_off = b_base + 0x400
        for i, v in enumerate(self.Bc):
            self.w16(col_off + i, v & 0xFFFF)
        val_off = b_base + 0x8000
        for i, v in enumerate(self.Bv):
            self.w16(val_off + i, v & 0xFFFF)

    def get_counts(self):
        return ([len(self.pe_desc[p]) for p in range(self.n_pe)],
                [len(self.pe_col[p])  for p in range(self.n_pe)])
