#!/usr/bin/env python3
"""Generate the PEAK-demand worst-case dataset (512x512x512, 30% density).

  A: max ROW weight  -> every row has exactly floor(512*0.30)=153 nonzeros
                        (random distinct columns in [0,512), sorted).  CSR.
  B: max COLUMN weight-> every column has exactly 153 nonzeros
                        (random distinct rows in [0,512), sorted).  CSC.

This is the structural worst case for the per-PE A/B buffers (153/row and
153/col at full 512 dimensions).  Values are synthesized by load_comp_matrix,
so only the index/weight structure is written here.

File format (matches TC1_RAW / load_comp_matrix):
  A_0_Matrix.txt : M lines, each "<row_nnz> <K>"
  A_0_Index.txt  : M lines, each = that row's sorted column indices
  B_0_Matrix.txt : N lines, each "<col_nnz> <K>"   (K = B row count)
  B_0_Index.txt  : N lines, each = that column's sorted row indices
"""
import os, random

DIM     = 512
DENSITY = 0.30
W       = int(DIM * DENSITY)          # 153 = max row/col weight
SEED    = 20260629
here = os.path.dirname(os.path.abspath(__file__))
rng = random.Random(SEED)

def write_csx(matrix_name, index_name, n_lines, idx_range):
    with open(os.path.join(here, matrix_name), 'w') as fm, \
         open(os.path.join(here, index_name), 'w') as fi:
        for _ in range(n_lines):
            idx = sorted(rng.sample(range(idx_range), W))
            fm.write(f"{W} {DIM}\n")
            fi.write(" ".join(map(str, idx)) + "\n")

# A: M=512 rows, each 153 distinct columns in [0,512)
write_csx('A_0_Matrix.txt', 'A_0_Index.txt', DIM, DIM)
# B: N=512 columns, each 153 distinct rows in [0,512)
write_csx('B_0_Matrix.txt', 'B_0_Index.txt', DIM, DIM)

print(f"Wrote PEAK dataset: A {DIM}x{DIM} ({W}/row), B {DIM}x{DIM} ({W}/col), "
      f"A_nnz=B_nnz={DIM*W} (seed {SEED}) to {here}")
