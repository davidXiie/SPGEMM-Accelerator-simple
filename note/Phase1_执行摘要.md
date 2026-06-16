# Phase 1 执行摘要：SPMM→SPGEMM 关键决策

## 三条核心结论

### 1. 老师的 Chisel 代码 → 你的 Verilog：基础设施完全对应 ✅
```
Fetch.scala     ≡  fetch.v      (5状态取指+分发)
Load.scala      ≡  load.v       (AXI burst→GBuf展开)
Store.scala     ≡  store.v      (OutputBuf→AXI写回)
Decode.scala    ≡  decode.v     (指令解码+分发)
CR.scala        ≡  wrapper.v    (AXI-Lite控制寄存器)
Wrapper.scala   ≡  wrapper.v    (顶层封装)
```
**只需要微改不需要重写。**

### 2. 核心差异就四个模块需要大改
```
pipelinedPE.scala → pe_decompress.v  + pe_mul_array.v + pe_aggregation.v
  SPMM: Dense累加器, B用Dense索引
  SPGEMM: SPA稀疏累加器, B用CSR解压

Compute.scala    → core_top.v 状态机
  删除: sDataMoveRow/Col/Val/Den (Group切分)
  删除: CombineGroup/Combine (跨组聚合/PR/VR)
  新增: LOAD_TASK 状态, op_type(ADD/SUB/MUL)分支

Group.scala      → pe_top.v
  保留: 4级流水握手 (d1_moving, d2_nextValid)
  改变: 密集累加 → SPA稀疏累加

scheduler.v      → 删除, 移到主机端
```

### 3. ISA 改动是三件事合一
```
① B 矩阵从 Dense → CSR: 多了2条LOAD指令 (B_col_idx, B_val)
② Scheduler 移到主机: 新增 LOAD_TASK 指令类型
③ 支持 Add/Sub: SPGEMM 指令增加 bit[8:6] op_type 字段

新 opcode:
  3'b000: LOAD_DATA (不变)
  3'b001: LOAD_TASK (新增)
  3'b010: STORE (不变)
  3'b011: COMPUTE (原SPGEMM, 增加op_type子字段)
  3'b111: FINISH (不变)

新 op_type (COMPUTE指令 bit[8:6]):
  3'b000: MUL (SpGEMM)
  3'b001: ADD (SpAdd)
  3'b010: SUB (SpSubtract)
```

## 改造顺序

| 顺序 | 文件 | 工作量 | 依赖 |
|------|------|--------|------|
| 1 | `isa.vh` | 小 | 无 |
| 2 | `defines.vh` | 小 | 无 |
| 3 | `core_top.v` 状态机 | 中 | 1,2 |
| 4 | `host_scheduler.c` | 中 | 无 (独立) |
| 5 | `task_loader.v` | 小 | 3 |
| 6 | `pe_top.v` | 大 | 1,2 |
| 7 | `pe_decompress.v` | 大 | 6 (参考 Chisel pipelinePE) |
| 8 | `pe_mul_array.v` (+ADD/SUB) | 中 | 6 |
| 9 | `sp_elementwise.v` | 中 | 3 |
| 10 | `test/` 测试适配 | 中 | 全部 |

## Chisel pipelinePE → Verilog 关键对照速查

| Chisel 信号 | 含义 | Verilog 等效 |
|------------|------|-------------|
| `d1_state_q` | D1级状态: Idle/RowPtr1/RowPtr2 | `d1_state` |
| `d1_moving` | D1可前进到下一行 (=!d2_nextValid) | `d1_moving = !d2_next_valid` |
| `d2_nextValid_q` | D2级有自循环数据 | `d2_next_valid_r` |
| `d2_valid_q` | D2级当前有效 | `d2_valid_r` |
| `d2_endOfRow_q` | D2级行末标志 | `d2_end_of_row_r` |
| `dr_valid_q` | DR级延迟一拍的valid | `dr_valid_r` |
| `dr_isNewOutput_q` | 当前是否新行首元素 | `dr_is_new_row_r` |
| `m_acc_q` | MAC累加器 (密集数组) | → `acc_val[512]` (SPA) |
| `pipeEmpty` | 流水线完全排空 | `pipe_empty` |
| `io.free` | PE空闲可接收新任务 | `pe_free` |
