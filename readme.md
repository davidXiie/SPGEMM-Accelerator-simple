# SPGEMM-Accelerator v2

基于 Verilog 的稀疏矩阵乘法加速器（SpGEMM: C = A × B），compact row-desc 输入，dense FP16 输出。

---

## 目录结构

```
├── rtl/                          # Verilog RTL
│   ├── include/
│   │   └── defines.vh            # 全局参数 (PE=1, MAC=4, MAX_DIM=512, FIFO=256)
│   ├── infrastructure/
│   │   ├── axi_interface.v       # AXI-Lite CR 从设备 + AXI-Full 读 MUX/写通道
│   │   └── scratchpad.v          # SRAM 模型 (std_scratchpad / banked / sync_fifo)
│   ├── core/                     # SpGEMM 核心
│   │   ├── core_top.v            # 顶层 FSM (Load_B→Load_A→Start_PE→Wait→Write_C)
│   │   ├── pe_top.v              # PE 顶层 (FSM + A/B buffer + 全子模块集成)
│   │   ├── pe_task_packer.v      # 任务打包 (4 task → 1 group)
│   │   ├── pe_mul_array.v        # 4-lane 乘法器阵列 (3-stage pipeline)
│   │   ├── pe_serializer.v       # 产品串行化 (4-lane → 1 product/cycle)
│   │   ├── pe_accumulator.v      # 串行累加器 (acc_buf[N], IDLE→ADD→WRITE)
│   │   ├── a_group_loader.v      # A 矩阵加载 (DDR → PE A buffer)
│   │   ├── b_broadcast_loader.v  # B 矩阵广播加载 (DDR → PE B buffer)
│   │   ├── descriptor_loader.v   # 描述符加载
│   │   ├── c_dense_buffer.v      # 片上 Dense C 缓冲 (512×512×16b)
│   │   ├── c_dense_write_arbiter.v # 多 PE 写仲裁
│   │   └── c_dense_ddr_writer.v  # Dense C 写回 DDR
│   ├── sim/
│   │   └── tb_pe_top.v           # PE 单元测试 Testbench
│   ├── wrapper.v                 # 顶层封装 (CR slave + core_top)
│   └── filelist.f                # 仿真文件列表
│
├── test/                         # 测试
│   ├── pe_sim/                   # PE 单元仿真
│   │   ├── test_pe.py            # 随机矩阵测试 (2×2 ~ 50×50)
│   │   ├── test_comp.py          # 赛题参考用例测试
│   │   ├── Makefile.pe / Makefile.comp
│   │   └── run_pe_test.bat / run_comp.bat
│   └── test_case_for_reference/  # 赛题参考矩阵
│
├── note/                         # 设计文档
│   ├── 精简架构.md               # 架构方案
│   ├── pe架构设计2026年6月16日.md # PE 详细设计
│   └── PE性能分析报告.md         # 性能分析报告
│
└── rtl_old/                      # 旧版 RTL (ISA + CSR 输出, 已停用)
```

---

## 架构特点

| 特性 | 说明 |
|------|------|
| A 输入格式 | Compact row-desc (64-bit desc + 16-bit col/val) |
| B 输入格式 | Compact row-desc (B 广播到所有 PE) |
| C 输出格式 | Dense FP16 (512×512 stride) |
| PE 数量 | 1 (可配, 最大 8) |
| MAC/PE  | 4 |
| 数据宽度 | 16-bit integer (仿真) / FP16 (目标) |
| 调度器 | Host 端预计算, PE 端固定 row_count |
| acc_buf | 串行单端口, 3 cycles/product |

---

## PE 数据流

```
A_row_desc ──→ A iterator ──→ task_packer ──→ task_fifo(256) ──→ 4×MAC
A_col/val      B streamer ←── B_row_desc                            │
B_col/val                  B_col/val                         product_fifo(256)
                                                                   │
                                                             serializer(4→1)
                                                                   │
                                                             accumulator
                                                             (IDLE→ADD→WRITE, 3c/prod)
                                                                   │
                                                             row_writeback → C_dense
```

---

## 快速开始

### PE 单元仿真

```bash
cd test/pe_sim

# 50×50 随机矩阵测试
run_pe_test.bat

# 赛题用例测试
run_comp.bat

# 或使用 cocotb Makefile
make -f Makefile.pe TESTCASE=test_pe_50x50
```

### 全系统编译（Icarus Verilog）

```bash
cd rtl
iverilog -g2012 -I include -o compile_test.vvp -f filelist.f
```

---

## 配置参数 (defines.vh)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `N_PE` | 1 | PE 数量 |
| `N_MAC` | 4 | 每 PE 乘法器数 |
| `MAX_M/K/N` | 512 | 最大矩阵维度 |
| `DATA_WIDTH` | 16 | 数据位宽 |
| `TASK_FIFO_DEPTH` | 256 | 任务 FIFO 深度 |
| `PROD_FIFO_DEPTH` | 256 | 产品 FIFO 深度 |
| `A_ROW_SLOT_PER_PE` | 128 | A 最大行数/PE |
| `A_NNZ_SLOT_PER_PE` | 16384 | A 最大 nnz/PE |
| `B_ROW_SLOT` | 512 | B 最大行数 |
| `B_NNZ_SLOT` | 78848 | B 最大 nnz |

---

## 测试结果

| 测试 | 规模 | 周期 | MAC利用率 | 状态 |
|------|------|------|----------|------|
| 2×2 | A(2,3)×B(3,2) | 2,095 | — | ✅ |
| 4×4 | 30% sparse | 4,277 | — | ✅ |
| 20×20 | 30% sparse | 24,541 | — | ✅ |
| 50×50 | 30% sparse | 40,451 | 7.9% | ✅ |
| 赛题 Case1 P1 | A(32,317)×B(317,6) | 48,237 | 2.2% | ⚠️ 97.4% |

---

## 性能瓶颈

| 瓶颈 | 占比 | 提升空间 |
|------|------|---------|
| 串行累加器 (3c/product) | 42% | 4-bank → 4× |
| FSM 排空等待 | 42% | 更大 FIFO + 连续流 |
| CLEAR + WRITE | 12% | 已按 N 裁剪 |

**当前主要限制**: 累加器单端口 SRAM → 1 product/3 cycles，是 MAC 产出速度 (4/cycle) 的 1/12。

---

## 已修复的 Bug（开发历程）

| # | 根因 | 症状 |
|---|------|------|
| 1 | 序列化器握手时序 | acc_in_valid 永不为 1 |
| 2 | MAC col/val 同步 | 列号错位 |
| 3 | state_stable 双递增 | row_idx 跳行 |
| 4 | a_nnz_left 下溢 | 0→0xFFFF 死循环 |
| 5 | write_global_row 延迟 | C 行列地址错 |
| 6 | a_nnz 连减两次 | 部分 product 丢失 |
| 7 | MAC 读组合 FIFO 旧数据 | product FIFO 重复填充死锁 |
| 8 | flush_done 死锁 | pack_count=0 永不等 |
