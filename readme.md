# SPGEMM-Accelerator v2

FP16 稀疏矩阵乘法加速器（SpGEMM: C = A × B），硬件在线指令生成，单 PE + 4 MAC，AXI DDR 接口。

---

## 目录结构

```
├── rtl/                              # 当前版本 RTL
│   ├── include/
│   │   └── defines.vh                # 全局参数
│   ├── infrastructure/
│   │   └── scratchpad.v              # sync_fifo (BRAM 推断)
│   ├── core/
│   │   ├── core_top_1pe.v            # 单 PE 顶层 FSM (8 态)
│   │   ├── pe_top.v                  # PE 核心 (硬件在线指令生成)
│   │   ├── pe_mul_array.v            # 4-lane FP16×FP16→FP16 乘法阵列
│   │   ├── row_accumulator_4bank.v   # 4-bank 行累加器 (乒乓)
│   │   ├── accum_bank.v              # 单 bank 累加器 (RMW 流水线)
│   │   ├── fp16_mul.v                # FP16 乘法器 (组合逻辑)
│   │   ├── fp16_add.v                # FP16 加法器 (组合逻辑)
│   │   ├── descriptor_loader.v       # 描述符加载 (AXI→参数)
│   │   ├── a_group_loader.v          # A 矩阵加载 (DDR→PE A buffer)
│   │   ├── b_broadcast_loader.v      # B 矩阵广播加载 (DDR→PE B buffer)
│   │   ├── pe_c_to_densebuf.v        # PE C buffer→c_dense_buffer
│   │   ├── c_dense_buffer.v          # 片上 Dense C 缓冲 (512×512 FP16)
│   │   └── c_dense_ddr_writer.v      # Dense C 写回 DDR
│   └── filelist.f
│
├── test/                             # 测试 (当前版本)
│   └── pe_sim/
│       ├── tb_core_top_1pe.v         # AXI DDR 模型 Testbench
│       ├── test_core_top_1pe.py      # cocotb 测试 (DDR→PE→DDR)
│       └── run_core_top_1pe.bat      # 一键运行
│
├── rtl_old/                          # 旧版 RTL (多 PE / CSR 输出 / ISA)
├── test_old/                         # 旧版测试 (PE 单元测试 / 赛题用例)
│
├── changelog/                        # 修改记录
│   ├── 2025-06-25_disable_cbank.md   # C bank 禁用/恢复 + FIFO BRAM 优化
│   └── 2025-06-26_single_pe_core.md  # 单 PE core_top_1pe 创建
│
└── note/                             # 设计文档
    ├── 精简架构.md
    ├── pe架构设计2026年6月16日.md
    └── PE性能分析报告.md
```

---

## 架构特点

| 特性 | 说明 |
|------|------|
| A/B 输入格式 | Compact row-desc (64-bit desc + 16-bit col/val) |
| C 输出格式 | Dense FP16 (512×512 stride)，通过 c_dense_buffer 缓存后 AXI 写回 DDR |
| PE 数量 | **1** |
| MAC/PE | **4** (4-lane 并行乘法) |
| 数据类型 | **FP16** (乘/加均为 FP16) |
| 指令生成 | **硬件在线生成** (Generator FSM 遍历 A_col→B_desc) |
| 累加器 | **4-bank 乒乓** (row_accumulator_4bank ×2，epoch/tag 免清零) |
| FIFO | **BRAM** (同步读，ram_style=block) |
| 接口 | **AXI4-Full** (512-bit 读/写) |

---

## PE 数据流

```
A_col_buf ──┐
A_val_buf ──┤
B_desc_buf ─┤──→ [Generator FSM] ──→ task_fifo (BRAM) ──→ pe_mul_array
            └──  硬件在线生成 4-wide task          (260b)    (4×FP16×FP16)
                                                               │
                                                        product_fifo (BRAM)
                                                               │
                                                row_accumulator_4bank ×2 (乒乓)
                                                               │
                                                            c_bank[4] (FP16, 256KB)
                                                               │
                                                        pe_c_to_densebuf
                                                               │
                                                        c_dense_buffer (512KB)
                                                               │
                                                        c_dense_ddr_writer → AXI → DDR
```

---

## 快速开始

### core_top_1pe 全链路仿真

```bash
cd test/pe_sim
run_core_top_1pe.bat
```

测试 A(16,16)×B(16,4) 小矩阵，覆盖 DDR→loader→PE 计算→c_dense_buffer→DDR 全链路。

### Vivado 综合

以 `core_top_1pe.v` 为顶层，添加 rtl/core/ 下全部 .v 文件和 rtl/infrastructure/scratchpad.v。

---

## 配置参数 (defines.vh)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MAX_M/K/N` | 512 | 最大矩阵维度 |
| `DATA_WIDTH` | 16 | 数据位宽 (FP16) |
| `N_MAC` | 4 | 每 PE 乘法器数 |
| `MUL_LAT` | 1 | 乘法流水线级数 |
| `TASK_FIFO_DEPTH` | 16 | 任务 FIFO 深度 |
| `PROD_FIFO_DEPTH` | 16 | 产品 FIFO 深度 |
| `A_ROW_SLOT_PER_PE` | 256 | A 最大行数/PE |
| `A_NNZ_SLOT_PER_PE` | 16384 | A 最大 nnz/PE |
| `B_ROW_SLOT` | 512 | B 最大行数 |
| `B_NNZ_SLOT` | 78848 | B 最大 nnz |
| `AXI_DATA_WIDTH` | 512 | AXI 数据位宽 |

---
