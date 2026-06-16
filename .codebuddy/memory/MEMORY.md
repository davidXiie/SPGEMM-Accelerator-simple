# Project Memory

## 项目概述
SPGEMM 稀疏矩阵乘法加速器，初始基于 Chisel SPMM 加速器改造，现按「精简架构.md」重构为 SpGEMM-only 简化版。

## 关键决策
- 2025-06-15: 采用精简架构.md 方案，重写核心控制逻辑，Phase 1 = N_PE=1, N_MAC=1
- 2025-06-16: PE调试进展 — 4个关键bug修复（序列化器握手、MAC col/val同步、row_idx双递增、a_nnz下溢）。PE所有行正确运行，2×2测试 2/4正确，4×4测试 4/12精确匹配。剩余：部分乘积累积丢失，需波形级调试。

## 目录结构 (v2)
- `rtl/` — 新版 RTL（精简架构，14个 Verilog 文件）
- `rtl_old/` — 旧版 RTL（ISA + CSR 输出）
- `note/` — 设计文档，关键: `精简架构.md`，`pe架构设计2026年6月16日.md`

## 文件变更 (2025-06-16)
- PE 完全重写: pe_top(FSM+A/B reg数组), pe_task_packer, pe_serializer, pe_accumulator(含acc_buf), pe_mul_array(4-MAC)
- 删除旧: pe_decompress.v, pe_aggregation.v
- 加载模块更新: a_group_loader/b_broadcast_loader 改为3组独立写端口
- core_top/c_dense_write_arbiter 适配新 PE 接口
- 编译状态: Icarus 编译通过 (0 errors, 0 warnings)
