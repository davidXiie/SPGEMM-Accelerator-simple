#!/usr/bin/env python3
"""
Matrix generator for automated SPGEMM testing.

Generates A(row-sparse, CSR) and B(column-sparse, CSC) matrices
with configurable dimensions and density, writing them in the
competition-format Matrix.txt + Index.txt expected by load_comp_matrix.
"""
import sys, os, random, struct

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pe_sim'))
from test_comp import gen_sparse_rows, pack_csr


def generate_matrix_files(name, M, K, N, density, seed=42, out_dir=None):
    """
    Generate A(M,K) row-sparse and B(K,N) column-sparse matrix files.

    Parameters
    ----------
    name     : case name, e.g. 'TC_N121'
    M, K, N  : matrix dimensions (A is M×K, B is K×N, C is M×N)
    density  : sparsity factor (0..1), actual row/col nnz ≤ floor(dim * density)
    seed     : random seed (deterministic output)
    out_dir  : output directory (default: ./generated_cases/<name>/)
    """
    if out_dir is None:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'generated_cases', name)
    os.makedirs(out_dir, exist_ok=True)

    # --- A matrix: row-sparse CSR ---
    random.seed(seed)
    A_rows = gen_sparse_rows(M, K, density, seed)
    _, A_col, A_val = pack_csr(A_rows, is_B=False)

    # Write A_Matrix.txt:  one line per row: <nnz> <K>
    with open(os.path.join(out_dir, 'A_Matrix.txt'), 'w') as f:
        for row in A_rows:
            f.write(f"{len(row)}  {K}\n")

    # Write A_Index.txt:  one line per row: space-separated column indices
    with open(os.path.join(out_dir, 'A_Index.txt'), 'w') as f:
        for row in A_rows:
            cols = [str(c) for c, _ in row]
            f.write(" ".join(cols) + "\n")

    # --- B matrix: column-sparse CSC ---
    # We generate B^T as row-sparse (N rows × K cols), then store as CSC.
    random.seed(seed + 1)
    Bt_rows = gen_sparse_rows(N, K, density, seed + 1)
    _, Bt_col, Bt_val = pack_csr(Bt_rows, is_B=False)

    # B_Matrix.txt: one line per COLUMN of B = per row of B^T: <nnz> <K>
    with open(os.path.join(out_dir, 'B_Matrix.txt'), 'w') as f:
        for col_nnz_list in Bt_rows:
            f.write(f"{len(col_nnz_list)}  {K}\n")

    # B_Index.txt: one line per COLUMN of B = per row of B^T: row indices
    with open(os.path.join(out_dir, 'B_Index.txt'), 'w') as f:
        for col_nnz_list in Bt_rows:
            rows_in_col = [str(r) for r, _ in col_nnz_list]
            f.write(" ".join(rows_in_col) + "\n")

    total_a_nnz = sum(len(r) for r in A_rows)
    total_b_nnz = sum(len(r) for r in Bt_rows)
    print(f"[GEN] {name}: A({M},{K}→{total_a_nnz}nnz) × B({K},{N}→{total_b_nnz}nnz) @ {density*100:.0f}% dens → {out_dir}")

    return out_dir


# =========================================================================
# CLI:  python matrix_gen.py <name> <M> <K> <N> <density> [seed]
# =========================================================================
if __name__ == '__main__':
    if len(sys.argv) < 6:
        print("Usage: python matrix_gen.py <name> <M> <K> <N> <density> [seed]")
        print("Example: python matrix_gen.py TC_N121 64 128 121 0.15")
        sys.exit(1)

    name    = sys.argv[1]
    M       = int(sys.argv[2])
    K       = int(sys.argv[3])
    N       = int(sys.argv[4])
    density = float(sys.argv[5])
    seed    = int(sys.argv[6]) if len(sys.argv) > 6 else 42
    generate_matrix_files(name, M, K, N, density, seed)
