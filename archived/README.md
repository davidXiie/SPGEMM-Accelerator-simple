# Archived Files

这些文件是 SPGEMM-Accelerator 项目早期开发阶段的遗留模块，已被 `mmap_sim` 平台（test/mmap_sim/）的新架构替代。保留用于参考。

## rtl/core/ — 旧 RTL 模块

| 文件 | 原始位置 | 简要说明 |
|------|---------|---------|
| `accelerator_top.v` | rtl/core/ | 旧全加速器顶层，FSM: IDLE→LOAD_A→LOAD_B→COMPUTE→DRAIN→DONE，使用 3 个 global buffer (a/b/c_global_buffer) 代替 DDR |
| `a_global_buffer.v` | rtl/core/ | A 矩阵全局 BRAM buffer |
| `b_global_buffer.v` | rtl/core/ | B 矩阵全局 BRAM buffer |
| `c_global_buffer.v` | rtl/core/ | C 矩阵全局 dense BRAM buffer (512×512 16-bit) |
| `pe_load_ctrl.v` | rtl/core/ | PE 加载控制器：从 global buffer 读 A（轮询分区）、B（广播），直写 PE 端口。含 1 周期读延迟 bug |
| `pe_drain_ctrl.v` | rtl/core/ | C 回读控制器：按 PE→local_row→gaddr 三层循环从 PE C bank 读出，按 global row 顺序写入 c_global_buffer |
| `ddr_model.v` | rtl/core/ | 旧 Verilog BRAM DDR 模型（8MB, 带 AXI4 Read Slave）。被 Python mmap + AXIReadResponder 替代 |
| `core_top.v` | rtl/core/ | 更早期顶层，含预计算指令调度器 |
| `a_group_loader.v` | rtl/core/ | A 矩阵分组加载器（旧，已弃用） |
| `b_broadcast_loader.v` | rtl/core/ | B 矩阵广播加载器（旧，已弃用） |
| `c_dense_buffer.v` | rtl/core/ | C 矩阵 dense 写回 buffer（旧，已弃用） |
| `pe_task_packer.v` | rtl/core/ | PE 任务打包器（旧，已弃用） |

## rtl/sim/ — 旧 Testbench

| 文件 | 原始位置 | 简要说明 |
|------|---------|---------|
| `tb_accelerator.v` | rtl/sim/ | acc_sim 平台的 testbench wrapper，例化 accelerator_top，暴露 a/b/c global buffer host 端口 |
| `tb_accelerator_axi.v` | rtl/sim/ | axi_sim 平台的 testbench wrapper，例化 accelerator_axi_top + ddr_model，暴露 host_wr_* 端口 |
| `tb_pe_cluster.v` | rtl/sim/ | pe_sim 平台的 testbench wrapper，例化 pe_cluster，暴露每 PE 独立的 A_desc_we_N 端口 |
| `tb_pe_top.v` | rtl/sim/ | 单 PE testbench wrapper，用于 run_comp.bat 和 run_pe_test.bat |

## test/acc_sim/ — 全加速器仿真平台

| 文件 | 简要说明 |
|------|---------|
| `test_accelerator.py` | cocotb 测试：通过 global buffer host 端口加载 A/B，读 C 结果 |
| `run_accel.bat/.sh` | 编译运行脚本，top=tb_accelerator，含 accelerator_top + load/drain ctrl + 3×global_buffer |

## test/axi_sim/ — AXI-DDR 仿真平台（Verilog BRAM 版本）

| 文件 | 简要说明 |
|------|---------|
| `test_accelerator_axi.py` | cocotb 测试（v1）：通过 ddr_model host 端口写数据，axi_loader 通过 AXI 读入 PE（B 加载为 stub） |
| `run_axi.bat/.sh` | 编译运行脚本，top=tb_accelerator_axi，含 ddr_model + axi_loader |

## test/pe_sim/ — PE 单元/集群仿真平台（Python 直连写）

| 文件 | 简要说明 |
|------|---------|
| `test_pe.py` | 随机矩阵冒烟测试（3×20 到 50×50） |
| `test_accelerator.py` | 全加速器测试（早期版本） |
| `analyze_state.py` | PE FSM 状态分析工具 |
| `run_cluster.bat/.sh` | PE 集群测试，top=tb_pe_cluster，已验证 PASSED |
| `run_comp.bat/.sh` | 单 PE 测试，top=tb_pe_top |
| `run_pe_test.bat/.sh` | 单 PE 随机矩阵测试 |
| `Makefile.comp / Makefile.pe` | Cocotb Makefile |

> ⚠️ `test/pe_sim/test_comp.py` 保留在原位。它是 mmap_sim 的共享库（load_comp_matrix, partition_a, verify, slice_bits, fp16_*）

## test/ (顶层) — 旧独立文件

| 文件 | 原始位置 | 简要说明 |
|------|---------|---------|
| `gen_data.py` | test/ | 顶层数据生成器（旧版），mmap_sim 有独立复本 |
| `host_scheduler.py` | test/ | 主机端 A 行调度器（预计算指令） |
| `test_dut.py` | test/ | 另一个 DUT 的 cocotb 测试（core_top） |
| `Makefile` | test/ | Cocotb Makefile |
| `run.bat` | test/ | 旧编译运行脚本 |
| `tb_row_accumulator_4bank.v` | test/ | 4-bank row accumulator testbench |
| `wave.gtkw` | test/ | GTKWave 配置文件 |
| `data/` | test/ | 生成的数据文件 |
