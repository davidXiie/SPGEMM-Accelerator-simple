#!/usr/bin/env python3
"""
host_scheduler.py — 主机端软件调度器

功能: 计算 PE 任务描述符, 替代原先硬件 Scheduler (scheduler.v) 的所有逻辑。

算法:
  1. B Row Length Generator: b_row_nnz[k] = B_row_ptr[k+1] - B_row_ptr[k]
  2. A Row Workload Analyzer (Row-Block): row_cyc[i] = ceil(sum(b_row_nnz[col]) / N_MAC)
  3. Remaining-Aware Row Partitioner: dynamic_target = ceil(remaining_work / remaining_pe)
     最近边界判断 → 生成 row_start, row_end per PE
  4. Task Descriptor Generator: 生成每个 PE 的 {row_start, row_end, a_ptr_start, a_ptr_end, valid}

用法:
  python host_scheduler.py

输入: (通过函数参数或文件读取)
  A_row_ptr[], A_col_idx[], B_row_ptr[]  — 来自 test_case 数据
  N_PE, N_MAC, M, K

输出:
  task_descriptors[N_PE][5] = {row_start, row_end, a_ptr_start, a_ptr_end, valid}
  直接写入 DDR 约定地址 (或输出为二进制文件供 testbench 加载)
"""

import math
import struct

# ============================================================================
# Configuration (匹配 defines.vh)
# ============================================================================
N_PE  = 8       # PE 数量
N_MAC = 4       # 每 PE 乘法器数
M_MAX = 512     # A 最大行数
K_MAX = 512     # K = B 最大行数

# ============================================================================
# 1. B Row Length Generator
#    b_row_nnz[k] = B_row_ptr[k+1] - B_row_ptr[k]
# ============================================================================
def gen_b_row_nnz(b_row_ptr, K):
    """输入 B_row_ptr 数组, 输出 b_row_nnz 数组 (每行非零元素数)"""
    b_row_nnz = [0] * K
    for k in range(K):
        b_row_nnz[k] = b_row_ptr[k + 1] - b_row_ptr[k]
    return b_row_nnz

# ============================================================================
# 2. A Row Workload Analyzer (Row-Block formula)
#    row_cyc[i] = ceil(sum(b_row_nnz[A_col_idx[p]]) / N_MAC)
# ============================================================================
def analyze_a_workload(a_row_ptr, a_col_idx, b_row_nnz, M):
    """
    输入: A CSR 数据, b_row_nnz
    输出: row_cyc[i], total_cycle_work (每行计算周期, 总周期)
    """
    row_cyc = [0] * M
    total_cycle_work = 0

    for i in range(M):
        row_eff = 0  # sum of b_row_nnz for all elements in row i
        a_start = a_row_ptr[i]
        a_end   = a_row_ptr[i + 1]

        for p in range(a_start, a_end):
            k = a_col_idx[p]  # A(i,k)
            if k < len(b_row_nnz):
                row_eff += b_row_nnz[k]

        # Row-Block formula: single ceil per row
        row_cyc[i] = (row_eff + N_MAC - 1) // N_MAC  # ceil(row_eff / N_MAC)
        total_cycle_work += row_cyc[i]

    return row_cyc, total_cycle_work

# ============================================================================
# 3. Remaining-Aware Row Partitioner
#    dynamic_target = ceil(remaining_work / remaining_pe)
#    最近边界判断
# ============================================================================
def partition_rows(row_cyc, total_cycle_work, M):
    """
    动态剩余目标 + 最近边界切分
    输出: tasks = [(row_start, row_end, estimated_cycle), ...] per PE
    """
    tasks = []
    part_row_idx = 0
    assigned_work = 0
    cur_pe = 0
    cur_pe_load = 0
    cur_pe_start = 0

    while part_row_idx < M and cur_pe < N_PE:
        remaining_work = total_cycle_work - assigned_work
        remaining_pe   = N_PE - cur_pe
        dynamic_target = (remaining_work + remaining_pe - 1) // remaining_pe  # ceil

        w = row_cyc[part_row_idx]
        new_load = cur_pe_load + w

        cross_boundary = (new_load >= dynamic_target) and (remaining_pe > 1)

        if not cross_boundary:
            # Take this row
            cur_pe_load = new_load
            part_row_idx += 1
        else:
            # Boundary decision: which is closer?
            err_before = dynamic_target - cur_pe_load
            err_after  = new_load - dynamic_target

            if err_after <= err_before and cur_pe_load > 0:
                # Assign current row then move to next PE
                cur_pe_load = new_load
                part_row_idx += 1

            # Close current PE
            tasks.append((cur_pe_start, part_row_idx - 1, cur_pe_load))
            assigned_work += cur_pe_load

            # Move to next PE
            cur_pe += 1
            if cur_pe < N_PE:
                cur_pe_start = part_row_idx
                cur_pe_load = 0

    # Close last PE
    if cur_pe < N_PE and cur_pe_load > 0:
        tasks.append((cur_pe_start, part_row_idx - 1, cur_pe_load))
        cur_pe += 1

    # Assign remaining rows to last PE (if any)
    if part_row_idx < M and len(tasks) > 0:
        tasks[-1] = (tasks[-1][0], M - 1, tasks[-1][2])

    return tasks

# ============================================================================
# 4. Task Descriptor Generator
#    生成每个 PE 的完整任务描述符
# ============================================================================
def gen_task_descriptors(tasks, a_row_ptr, M):
    """
    输入: tasks from partition_rows, A_row_ptr
    输出: list of per-PE descriptors
      descriptor[i] = {
          'row_start': int,
          'row_end':   int,
          'a_ptr_start': int,
          'a_ptr_end':   int,
          'valid':       0/1
      }
    """
    descriptors = []
    for pe_id, (row_start, row_end, est_cyc) in enumerate(tasks):
        valid = 1 if row_end >= row_start else 0
        a_ptr_start = a_row_ptr[row_start] if valid else 0
        a_ptr_end   = a_row_ptr[row_end + 1] if valid else 0

        descriptors.append({
            'pe_id': pe_id,
            'row_start': row_start,
            'row_end': row_end,
            'a_ptr_start': a_ptr_start,
            'a_ptr_end': a_ptr_end,
            'estimated_cycle': est_cyc,
            'valid': valid
        })

    # Pad remaining PEs with invalid entries
    for pe_id in range(len(tasks), N_PE):
        descriptors.append({
            'pe_id': pe_id,
            'row_start': 0,
            'row_end': 0,
            'a_ptr_start': 0,
            'a_ptr_end': 0,
            'estimated_cycle': 0,
            'valid': 0
        })

    return descriptors

# ============================================================================
# 5. 打包为二进制 (写入 DDR / GlobalBuffer)
#    Task descriptor 格式 (每 PE 5 个 16-bit elements):
#     [0]: row_start (16-bit, lower bits in [MAX_DIM_BITS-1:0])
#     [1]: row_end   (16-bit)
#     [2]: a_ptr_start (16-bit)
#     [3]: a_ptr_end   (16-bit)
#     [4]: {15'd0, valid} (16-bit, bit[0] = valid)
# ============================================================================
def pack_task_descriptors(descriptors, output_file=None):
    """打包为 DDR 二进制镜像格式"""
    packed = b''
    for desc in descriptors:
        elems = [
            desc['row_start'] & 0xFFFF,
            desc['row_end']   & 0xFFFF,
            desc['a_ptr_start'] & 0xFFFF,
            desc['a_ptr_end']   & 0xFFFF,
            desc['valid'] & 0x1,
        ]
        for e in elems:
            packed += struct.pack('<H', e)

    if output_file:
        with open(output_file, 'wb') as f:
            f.write(packed)

    return packed

# ============================================================================
# 顶层: 一步完成调度
# ============================================================================
def host_schedule(a_row_ptr, a_col_idx, b_row_ptr, M, K):
    """
    主机端完整调度流程

    参数:
      a_row_ptr : list[int] — A CSR row pointer 数组 (长度 M+1)
      a_col_idx : list[int] — A CSR column index 数组
      b_row_ptr : list[int] — B CSR row pointer 数组 (长度 K+1)
      M : int — A 行数
      K : int — B 行数 (A的列数)

    返回:
      descriptors : list[dict] — 每个 PE 的任务描述符
      stats : dict — 调度统计信息
    """
    # 1. B Row Length
    b_row_nnz = gen_b_row_nnz(b_row_ptr, K)

    # 2. A Workload Analysis
    row_cyc, total_cycle_work = analyze_a_workload(a_row_ptr, a_col_idx, b_row_nnz, M)

    # 3. Partition
    tasks = partition_rows(row_cyc, total_cycle_work, M)

    # 4. Task Descriptors
    descriptors = gen_task_descriptors(tasks, a_row_ptr, M)

    # Statistics
    stats = {
        'K': K,
        'M': M,
        'N_PE': N_PE,
        'N_MAC': N_MAC,
        'total_cycle_work': total_cycle_work,
        'b_row_nnz_max': max(b_row_nnz) if b_row_nnz else 0,
        'b_row_nnz_total': sum(b_row_nnz),
        'tasks': [(t[0], t[1], t[2]) for t in tasks],
    }

    return descriptors, stats

# ============================================================================
# 打印函数
# ============================================================================
def print_descriptors(descriptors):
    print(f"\n{'PE':<4} {'valid':<6} {'row_start':<11} {'row_end':<9} {'a_ptr_start':<12} {'a_ptr_end':<10} {'est_cyc':<9}")
    print("-" * 75)
    for d in descriptors:
        print(f"{d['pe_id']:<4} {d['valid']:<6} {d['row_start']:<11} {d['row_end']:<9} "
              f"{d['a_ptr_start']:<12} {d['a_ptr_end']:<10} {d['estimated_cycle']:<9}")


# ============================================================================
# Self-test
# ============================================================================
if __name__ == "__main__":
    # 小例子: M=5, K=4 (简化版)
    # A = sparse 5×4
    # Row 0: A[0,0]=x, A[0,2]=x
    # Row 1: A[1,1]=x
    # Row 2: empty
    # Row 3: A[3,0]=x, A[3,1]=x, A[3,3]=x
    # Row 4: A[4,2]=x
    a_row_ptr = [0, 2, 3, 3, 6, 7]  # M+1=6
    a_col_idx = [0, 2, 1, 0, 1, 3, 2]  # nnz=7

    # B = sparse 4×3
    # B_row_ptr: [0, 2, 3, 3, 5] → Row0:2 nz, Row1:1 nz, Row2:0 nz, Row3:2 nz
    b_row_ptr = [0, 2, 3, 3, 5]

    descriptors, stats = host_schedule(a_row_ptr, a_col_idx, b_row_ptr, M=5, K=4)

    print("=" * 75)
    print("Host Scheduler Results")
    print("=" * 75)
    print(f"\nStats: M={stats['M']}, K={stats['K']}, N_PE={stats['N_PE']}, N_MAC={stats['N_MAC']}")
    print(f"Total cycle work: {stats['total_cycle_work']}")
    print(f"B nnz total: {stats['b_row_nnz_total']}")
    print(f"B nnz per row: {[gen_b_row_nnz(b_row_ptr, 4)[k] for k in range(4)]}")
    print(f"Row cyc per A row: {stats.get('row_cyc', 'not available')}")

    print_descriptors(descriptors)

    # Pack to binary
    binary = pack_task_descriptors(descriptors)
    print(f"\nBinary packed: {len(binary)} bytes ({len(binary)//2} × 16-bit elements)")
    print(f"Expected: {N_PE} PEs × 5 elements = {N_PE*5*2} bytes")
