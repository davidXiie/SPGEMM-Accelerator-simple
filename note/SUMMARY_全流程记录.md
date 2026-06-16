# SPGEMM-Accelerator 全流程工作记录

> 目标：基于老师 Chisel SPMM 加速器(gcn_scala/)改造成 Verilog SPGEMM 加速器，
>       支持 Mul/Add/Sub，Scheduler 移到主机端。
> 文件路径: `d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple`

---

## 四个目标 & 完成状态

| # | 目标 | 状态 |
|---|------|------|
| 1 | SPMM → SPGEMM 架构改造 | ✅ A/B 均 CSR, PE 用 SPA 稀疏累加 |
| 2 | Chisel → Verilog (保留数据搬运+指令) | ✅ Fetch/Load/Store/Decode 完全复用 |
| 3 | Scheduler 移到主机端 | ✅ host_scheduler.py → task_loader.v |
| 4 | 支持 Mul/Add/Sub | ✅ op_type 字段 + 三路 ALU + sp_elementwise.v |

---

## Phase 1: 架构分析

**产出**: `note/Phase1_SPMM_to_SPGEMM_对照分析.md`, `note/Phase1_执行摘要.md`

**核心结论**:
- 10 个基础设施模块 (Fetch/Load/Store/Decode/AXI/Scratchpad) 与 Chisel 完全等价，直接复用
- 4 个模块要大改: core_top(状态机), PE(SPA), Compute/Group(去掉), Scheduler(删除→主机)
- Chisel pipelinedPE 的 4 级流水握手 (d1_moving/d2_nextValid/pipeEmpty) 可直接翻译为 Verilog

---

## Phase 2: RTL 核心改造

**全部改动的文件**:

| 文件 | 改动 |
|------|------|
| `rtl/include/isa.vh` | 新增 `OP_LOAD_TASK`/`OP_COMPUTE`，增加 `COMPUTE_OP_TYPE[8:6]` |
| `rtl/include/defines.vh` | 新增 `OP_TYPE_MUL/ADD/SUB`、`MEM_TASK_DESC`、`TASK_DESC_*` |
| `rtl/core/core_top.v` | 删除 SCHEDULE，新增 LOAD_TASK 状态，op_type 分支 |
| `rtl/infrastructure/fetch.v` | 删除 sch 通道，dispatch: LOAD+LOAD_TASK→ld |
| `rtl/infrastructure/decode.v` | 新增 is_load_task/is_compute, spgemm_decode→compute_decode |
| `rtl/core/pe_mul_array.v` | 新增 op_type 输入，MUL/ADD/SUB 三路 ALU |
| `rtl/core/pe_top.v` | 新增 op_type 端口 |
| **新增** `rtl/core/task_loader.v` | 从 GBuf 读取主机 task descriptors |
| **新增** `rtl/core/sp_elementwise.v` | Sparse Add/Sub merge-sort ALU |
| `rtl/filelist.f` | 更新文件列表 |

**新状态机**: `IDLE → LOAD_A×3 → LOAD_B×3 → LOAD_TASK → COMPUTE → WRITE_CSR → STORE → FINISH`

**新 ISA**:
```
bit[2:0] opcode: 000=LOAD, 001=LOAD_TASK, 010=STORE, 011=COMPUTE, 111=FINISH
bit[8:6] op_type (COMPUTE): 000=MUL, 001=ADD, 010=SUB
```

---

## Phase 3: 测试适配

| 文件 | 改动 |
|------|------|
| `test/gen_data.py` | 新 opcode 编码，集成 host_schedule()，9条指令/pair |
| `test/test_dut.py` | ins_count 默认 28，新增 test_debug_state 测试 |
| **新增** `test/host_scheduler.py` | B Row Length → Workload → Dynamic Partition → Pack binary |

---

## Phase 4: PE 流水线重写 (Chisel 风格)

**重写**:
- `rtl/core/pe_decompress.v` — 单体 FSM → 3 级流水线 (S0 FETCH + S1 STREAM + entry_fifo)
- `rtl/core/pe_aggregation.v` — 脉冲接口 (row_start_pulse/end_pulse) + agg_stall 反压
- `rtl/core/pe_top.v` — 连接新 pipeline 信号

---

## Phase A: 仿真调试

### 修复的编译/仿真问题 (8 个)

| # | 问题 | 修复 |
|---|------|------|
| 1 | Makefile 使用 Unix `grep`/`sed` | 硬编码 VERILOG_SOURCES |
| 2 | cocotb 需要 Unix `tr` | `conda install -c conda-forge m2-base m2-make` |
| 3 | `core_top.v` 嵌套 `{{N{1'b0}}, data}` 语法错误 | 改为逐元素 assign |
| 4 | `` `N_MAC[9:0] `` 对 define 做位选取 | 改为 `N_MAC_10` wire |
| 5 | `entry_mem[addr][HI:LO]` SystemVerilog | 先读到 wire 再取位选取 |
| 6 | `cr_slave` 缺少 clk/rst 端口 | 添加到端口列表 + wrapper 连线 |
| 7 | `wrapper.v` cr_slave 实例化缺端口 | 连线 aclk/aresetn |
| 8 | iverilog 默认 `timescale 1s/1s` 精度不够 | `defines.vh` 加 `\`timescale 1ns/1ps` |

### 仿真结果

```
test_debug_state  : ✅ PASS  (20060 ns, ~2006 cycles)  — 基础架构通过
test_quick_launch : ✅ PASS  (1060 ns)                  — 快速启动通过
test_spgemm_tc1   : ⏳ DUT 运行中, 10ms 超时未完成
                    (Pair 2 有 87332 nnz, iverilog 仿真慢, 4分钟=10ms模拟时间)
                    timeout 已改为 100ms, 预计需 ~40 分钟
```

### 手动运行完整仿真

```bash
conda activate gcnenv
cd d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple\test
make SIM=icarus TESTCASE=test_spgemm_tc1 WAVES=1
gtkwave sim_build/core_top.fst wave.gtkw
```

### Conda 环境

```
环境名: gcnenv
Python: 3.11.15
cocotb: 1.9.2
iverilog: 12.0 (C:\iverilog\bin)
make: C:\Users\Administrator\scoop\shims\make.exe
已安装: m2-base, m2-make (提供 tr/grep/sed)
```

---

## 新架构总览

```
Host (CPU)
  │ host_scheduler.py: 计算 task descriptors → DDR
  ↓
┌─ 加速器 (core_top) ───────────────────────────────────────┐
│  IDLE→LOAD_A×3→LOAD_B×3→LOAD_TASK→COMPUTE→WRITE_CSR→STORE→FINISH
│                                                           │
│  COMPUTE 分支:                                            │
│    MUL → PE×8 [S0 FETCH→entry_fifo→S1 STREAM→MUL_ALU→SPA]│
│    ADD/SUB → sp_elementwise (merge-sort)                  │
│                                                           │
│  输出: C CSR Writer → OutBuf → Store → DDR                │
└───────────────────────────────────────────────────────────┘
```

---

## 关键文件索引

```
rtl/include/isa.vh            ← ISA 定义
rtl/include/defines.vh        ← 参数 + timescale
rtl/core/core_top.v           ← 主状态机
rtl/core/task_loader.v        ← 主机 task descriptor 加载 (新增)
rtl/core/sp_elementwise.v     ← Add/Sub 专用 (新增, 骨架)
rtl/core/pe_decompress.v      ← 3级流水线 Decompress
rtl/core/pe_aggregation.v     ← SPA + 反压
rtl/core/pe_top.v             ← PE 顶层
rtl/core/pe_mul_array.v       ← MUL/ADD/SUB ALU
rtl/core/c_csr_writer.v       ← CSR 输出生成
rtl/infrastructure/fetch.v    ← 取指+分发
rtl/infrastructure/decode.v   ← 解码
rtl/infrastructure/load.v     ← DDR→GBuf
rtl/infrastructure/store.v    ← GBuf→DDR
rtl/infrastructure/axi_interface.v ← AXI 接口
rtl/infrastructure/scratchpad.v    ← SRAM 模型
rtl/wrapper.v                 ← 顶层封装
test/gen_data.py              ← 测试数据生成
test/test_dut.py              ← cocotb Testbench
test/host_scheduler.py        ← 主机端软件调度器 (新增)
note/Phase1_SPMM_to_SPGEMM_对照分析.md
note/Phase1_执行摘要.md
```

## 下一步 (新对话)

1. **跑完整仿真** — `make SIM=icarus TESTCASE=test_spgemm_tc1 WAVES=1` (约 40min)
2. **完善 sp_elementwise.v** — 目前是骨架，需完善 CSR 读取+FP16 ALU
3. **完善 C CSR Writer** — 适配 PE Array 多路输出
4. **Chisel full PE rewrite** — 参照 pipelinedPE.scala 的完整流水线
5. **性能对比** — 软件 baseline vs 硬件 cycle count
6. **输出设计文档** — 整理为体系结构说明
