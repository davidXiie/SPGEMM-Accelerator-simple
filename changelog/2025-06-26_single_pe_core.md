# 2025-06-26 — 创建单 PE FPGA 验证版本

## 目的
构建可上 FPGA 板验证的单 PE 完整加速器。

## 新增文件
- `rtl/core/pe_c_to_densebuf.v` — PE C buffer 读出 → c_dense_buffer 写入
- `rtl/core/core_top_1pe.v` — 单 PE 顶层，8 态 FSM + 全部子模块

## 恢复文件
| 文件 | 操作 |
|---|---|
| `rtl/core/pe_top.v` | 反注释 c_bank/c_rd 全部 6 处 |
| `rtl/core/pe_cluster.v` | 反注释 c_rd 端口声明 + 实例连接 |
| `rtl/sim/tb_pe_top.v` | 反注释 c_rd reg/wire + 实例连接 |
| `rtl/sim/tb_pe_cluster.v` | 反注释 c_rd reg/wire + 实例连接 |
| `test/pe_sim/test_comp.py` | 恢复 read_c_buffer/read_c_buffer_pe/verify/assert |

## core_top_1pe.v 结构

```
FSM: IDLE → LOAD_DESC → LOAD_B → LOAD_A
     → START_PE → WAIT_PE → WRITE_C_DENSE → WRITE_C_DDR → FINISH

子模块:
├── descriptor_loader       (DDR → M/K/N)
├── a_group_loader           (DDR → PE A buffer)
├── b_broadcast_loader       (DDR → PE B buffer)
├── pe_top ×1                (带 c_bank)
├── pe_c_to_densebuf         (PE c_rd → c_dense_buffer)
├── c_dense_buffer           (512×512 FP16 = 512KB)
└── c_dense_ddr_writer       (c_dense_buffer → AXI → DDR)
```

## C 输出两阶段
1. S_WRITE_C_DENSE: pe_c_to_densebuf 逐行逐列读出 PE 内部 c_bank，写入 c_dense_buffer
2. S_WRITE_C_DDR: c_dense_ddr_writer 从 c_dense_buffer 批量读出，通过 AXI 写回 DDR

---

# 附加：core_top_1pe 的 cocotb 测试平台

## 新建文件
- `test/pe_sim/tb_core_top_1pe.v` — AXI4 Slave DDR 模型 (1 MB)
- `test/pe_sim/test_core_top_1pe.py` — cocotb 测试 (A 16×16 × B 16×4)
- `test/pe_sim/run_core_top_1pe.bat` — 编译运行脚本

## 数据流
```
Python 预填 DDR model → core_top_1pe (AXI read)
  → loader → PE 计算 → c_dense_buffer → ddr_writer (AXI write)
  → Python 读回 DDR model → 对比 golden
```
