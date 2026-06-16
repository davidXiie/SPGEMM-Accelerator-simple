# Phase 1: SPMM → SPGEMM 逐模块对照分析

> 目标：基于老师的 Chisel SPMM 加速器（gcn_scala/），在保留数据搬运逻辑和指令结构的前提下，
> 用 Verilog 改造成 SPGEMM 加速器，去掉硬件 Scheduler（移到主机），支持 Mul/Add/Sub。

---

## 一、顶层架构对照

```
┌─────────────────── SPMM (Chisel) ───────────────────┐    ┌────────── SPGEMM (Verilog 目标) ──────────┐
│                                                       │    │                                            │
│  CR (AXI-Lite) → Wrapper ← DDR (AXI-Full 512bit)     │    │  CR (AXI-Lite) → wrapper ← DDR (AXI-Full)  │
│    ↓                    ↓                             │    │    ↓                    ↓                  │
│  Core (5状态)          ME (Mux)                       │    │  core_top (7→简化到5状态)  axi_read_mux    │
│    ↓                                                    │    │    ↓                                       │
│  Fetch → Decode → Load/Compute/Store                  │    │  Fetch → Decode → Load/Compute/Store       │
│                          ↑ 原来4轮Load                 │    │                          ↑ 变为6轮Load      │
│  Compute内部Group×32调度                               │    │  ← 主机端 Scheduler (硬件Scheduler删除)     │
│    ↓                                                    │    │    ↓                                       │
│  Group → D1→D2→DR→M (4级流水)                         │    │  PE×8 → Decompress→MUL×4→SPA              │
│    ↓                                                    │    │    ↓                                       │
│  Dense Output → Store                                  │    │  C CSR Writer → CSR Output → Store         │
└───────────────────────────────────────────────────────┘    └────────────────────────────────────────────┘
```

---

## 二、逐模块 Chisel → Verilog 对照

### 2.1 Wrapper（顶层封装）

| Chisel | Verilog (已有) | 改动 |
|--------|---------------|------|
| `Wrapper.scala` - CR + Core + ME 三模块直连 | `wrapper.v` - CR slave + core_top + axi_read_mux | ✅ 无需改动 |
| AXI-Lite CR 寄存器: 0x00启动, 0x04指令基址, 0x08指令数, 0x0C完成 | 同左 | ✅ 无需改动 |
| AXI-Full 512-bit AR/AW/W/B/R 直连 | 同左 | ✅ 无需改动 |
| ME 读写仲裁 (2读1写) | axi_read_mux + 直连写 | ✅ 逻辑等价 |

**结论：Wrapper 完全可复用，Verilog 版与 Chisel 版等价。**

---

### 2.2 ISA（指令集）

| 特性 | Chisel SPMM | Verilog SPGEMM (已有) | 需要改成 |
|------|-------------|----------------------|---------|
| 位宽 | 256-bit | 256-bit | 不变 |
| Opcode [1:0] / task | 2-bit (00=Load,01=Store,10=SPMM,11=Finish) | 3-bit [2:0] (001=Load,010=Store,011=SPGEMM,111=Finish) | **对齐到2-bit** 或 保持3-bit兼容 |
| MemID (Load/Store) | 3-bit (000=Col,001=Ptr,010=Val,011=Den,100=Out,101=Psum) | 3-bit [5:3] (同) | 需新增: **110=B_Col, 111=B_Val** (或复用) |
| Load 指令字段 | dram_offset[63:0], sram_offset[31:0], xsize[31:0] | 同 (LOAD_DRAM_BASE / LOAD_SRAM_OFFSET / LOAD_XSIZE) | 不变 |
| Compute/SPMM 指令 | sramCol, sramPtr, sramVal, sramDen + rowSize, colSize, denSize | M, K, N + A/B各三个SRAM基址 | SPGEMM 指令字段足够 |

**关键决策：opcode 位宽对齐方案**

```
Chisel: bit[1:0] = opcode (00=Load, 01=Store, 10=Compute, 11=Finish)
Verilog 已有: bit[2:0] = opcode (001=Load, 010=Store, 011=SPGEMM, 111=Finish)

建议方案A (保守): 保持 Verilog 的 3-bit opcode，与 Chisel 兼容：
  `OP_LOAD   = 3'b001   (对应 Chisel bit[1:0]=00, bit[2]=don't care)
  `OP_STORE  = 3'b010   (对应 Chisel bit[1:0]=01)
  `OP_SPGEMM = 3'b011   (对应 Chisel bit[1:0]=10)
  `OP_FINISH = 3'b111   (对应 Chisel bit[1:0]=11)

建议方案B (精简): 改为 2-bit opcode 与 Chisel 完全一致
```

---

### 2.3 Fetch（取指 + 分发）

| Chisel `Fetch.scala` | Verilog `fetch.v` | 对照 |
|---------------------|-------------------|------|
| 5状态: sIdle→sReadCmd→sRead→sDrain→sSplit | S_IDLE→S_READ_CMD→S_READ_DATA→S_DRAIN→S_SPLIT | ✅ 完全等价 |
| `SyncQueue(entries_q)` 指令队列 | `inst_queue[0:15]` 寄存器队列 | ✅ 等价 |
| `insPerTransfer = mp.dataBits / INST_BITS = 2` | `INS_PER_TRANSFER = 2` | ✅ 一致 |
| `launch` 上升沿脉冲启动 | `launch_pulse` 边沿检测 | ✅ 一致 |
| 3通道分发: `io.inst.ld / .co / .st` | 4通道: `ld_inst / sp_inst / st_inst / sch_inst` | 多了 sch 通道 (对应用户的Scheduler) |
| `FetchDecode` 模块: ListLookup 匹配 BitPat → isLoad/isCompute/isStore | 组合逻辑: `inst_opcode == OP_LOAD` 等 | ✅ 等价 |

**结论：fetch.v 已完美对应 Fetch.scala。唯一差异是多了一个 sch_inst 通道（后续删除）。**

---

### 2.4 Decode（指令解码）

| Chisel | Verilog |
|--------|---------|
| `FetchDecode` - 仅分派用 (isLoad/isCompute/isStore) | fetch.v 内联组合逻辑 | ✅ 等价 |
| `LoadDecode` - dramOffset, sramOffset, xSize, isVal/isCol/isPtr/isSeq | `load_decode` 子模块 | ✅ 等价 |
| `ComputeDecode` (SpMMDecode) - sramVal/Col/Ptr/Den, denSize, colSize, rowSize | `spgemm_decode` 子模块 | 字段不同但结构等价 |
| `StoreDecode` - dramOffset, sramOffset, xSize | store.v 内联 | ✅ 等价 |
| `MemDecode` Bundle: empty[122:0] + ysize + xsize + sram + dram + memID + op | isa.vh `LOAD_*` 宏 | ✅ 等价 |

**需要修改的：ComputeDecode → SPGEMM Decode**

```
Chisel SpMMDecode:
  sram_offset_col[31:0]  →  A_col_idx SRAM base
  sram_offset_ptr[31:0]  →  A_row_ptr SRAM base
  sram_offset_val[31:0]  →  A_val SRAM base
  sram_offset_den[31:0]  →  B Dense SRAM base (只有1个基址)
  den_size[31:0]         →  Dense 总元素数
  col_size[31:0]         →  A 非零元素数
  row_size[31:0]         →  A 行数

Verilog SPGEMM Decode (isa.vh 已定义):
  A_row_ptr SRAM + A_col_idx SRAM + A_val SRAM
  B_row_ptr SRAM + B_col_idx SRAM + B_val SRAM  ← B需要3个基址
  M + K + N  (维度)
```

---

### 2.5 Load（数据加载）

| Chisel `Load.scala` | Verilog `load.v` | 对照 |
|---------------------|------------------|------|
| 6状态: sIdle→sStride→sSeq→sSeqCmd→sSeqReadData→sDelay | 5状态: IDLE→READ_CMD→READ_DATA→DRAIN_BEAT→DELAY | ✅ 等价 (无 stride 模式) |
| `inst_q` 指令队列 | `inst_q_valid` (1深寄存器) | ✅ 简化但等效 |
| `data_q` 数据队列 | `beat_reg` + `beat_valid_r` + `elem_idx` | 等效: Chisel用Queue缓冲, Verilog用节拍展开 |
| AXI: arvalid/arready/arlen/rvalid/rdata | 同 | ✅ 一致 |
| 写入: `io.spWrite (GlobalBuffer)` | `gbuf_wr_en/addr/data` 展开为单元素写 | ✅ 等价 (Verilog更明确) |
| scratchSel 选择目标暂存器 | Load 指令中的 mem_id | 需扩展 mem_id |

**结论：load.v 逻辑与 Load.scala 等价。关键改动：需要新增 S_STATE_LOAD_B 子状态或在 core_top 中循环调度 6 次 Load。**

---

### 2.6 PR (PageRank) / VR (Virtual Row) 聚合 — 仅 SPMM 需要

Chisel 的 `Compute.scala` 中有复杂的 `sCombine / sCombineGroup` 状态用于跨 Group 行聚合：

```scala
// Chisel: 32 Groups 各自计算，结果需要按行聚合
sCombineGroup → sCombine  // 逐行读取VR表判断跨组依赖
aggWithPrevGroup  // 前一组最后一行 + 当前组第一行累加
prRowAgg           // PageRank 场景的特殊累加
```

**SPGEMM 完全不需要这个逻辑**，因为：
- 每个 C 行只由一个 PE 负责（行级切分）
- 不需要跨 PE 聚合同一行
- 不需要 VR/PR 机制

**删除模块：VRTable, VRTableEntry, CombineGroup 状态机, PR 部分和**

---

### 2.7 Core / Core_top（主状态机）— 最大改动点

```
Chisel Core.scala:
  sIdle → sLoad(×4轮: col/ptr/val/den) → sCompute → sStore → 循环/Finish
  ctr: 0..3 循环计数切换 Load 子类型

Verilog core_top.v (当前):
  IDLE → LOAD_A(3子: row/col/val) → LOAD_B(3子: row/col/val) 
  → SCHEDULE → COMPUTE → WRITE_CSR → STORE → FINISH

目标 core_top.v (改造后):
  IDLE → LOAD_A(3子) → LOAD_B(3子) → LOAD_TASK → COMPUTE → WRITE_CSR → STORE → FINISH
                                      ↑ 新增：主机算好的 task descriptor 加载
                    SCHEDULE 模块删除
```

**对照表：**

| Chisel 状态 | 你的 Verilog 当前 | 目标 Verilog | 对照 |
|------------|-----------------|-------------|------|
| sIdle | STATE_IDLE | STATE_IDLE | 相同 |
| sLoad (ctr=0:col) | LOAD_A row | LOAD_A row | A CSR 首次 |
| sLoad (ctr=1:ptr) | LOAD_A col | LOAD_A col | A CSR 二次 |
| sLoad (ctr=2:val) | LOAD_A val | LOAD_A val | A CSR 三次 |
| sLoad (ctr=3:den) | LOAD_B row | LOAD_B row | **B Dense → B CSR row** |
| sLoad (done→sCompute) | LOAD_B col | LOAD_B col | **新增 B CSR col** |
| — | LOAD_B val | LOAD_B val | **新增 B CSR val** |
| — | SCHEDULE | **LOAD_TASK** (新增) | 主机端算好的 PE 任务表 |
| sCompute | COMPUTE | COMPUTE | PE 内部从 SPMM→SPGEMM |
| — | WRITE_CSR | WRITE_CSR | **新增: CSR 格式输出** |
| sStore | STORE | STORE | 相同（输出变为CSR） |
| →sLoad 循环 | →LOAD_A 循环 | →LOAD_A 循环 | 相同 |

---

### 2.8 Group / PE（计算核心）— 第二大改动点

#### Chisel Group.scala (4级流水线 D1→D2→DR→M)

```
D1 (2周期): 读取 RowPtr[i] 和 RowPtr[i+1] → 得到行范围
    d1_state_q: sIdle → sRowPtr1 → sRowPtr2
    输入: peReq (rowIdx), 输出: rowPtr1Data, rowPtr2Data

D2 (1周期): 读取 A_col_idx[currPtr] → 得到列号 k
    d2_nextValid: D1有数据或D2自己循环
    d2_endOfRow: currPtr == rowPtr2Data-1
    输出: colIdx, denCol

DR (1周期): 读取 A_val[currPtr] 和 B_den[colIdx, denCol]
    从 spVal 读稀疏值, 从 spDen 读稠密值 (按colIdx索引)
    (SPMM中 B 是 dense: B_den[colIdx * denXSize + denCol])

M (1周期): MAC 累加
    m_mac[i] = (isNewRow) ? A_val * B_den[i] : acc[i] + A_val * B_den[i]
    输出到 spOut (BankedScratchpad)
```

**关键手信号对照：**
```
d1_moving     = !d2_nextValid         // D1流水线stall条件
d2_nextValid  = !d2_endOfColRow && d2_valid_q  // D2下一周期有效
dr_valid_q    = RegNext(d2_valid_q)   // DR延迟一拍
m_valid_q     = RegNext(dr_valid_q)   // M延迟一拍
pipeEmpty     = d1_ready && !dr_valid_q && !d2_valid_q && !m_valid_q
```

#### Verilog pe_decompress.v (已有 SPGEMM)

你当前的 PE 设计已经实现了 SPGEMM 的核心逻辑，与 Chisel Group 对照：

| 阶段 | Chisel Group | Verilog pe_decompress |
|------|-------------|----------------------|
| 行指针读取 | D1: sRowPtr1→sRowPtr2 读两个RowPtr | A Row Controller: 从A_row_ptr读范围 |
| 列索引读取 | D2: 读 A_col_idx[currPtr] → k | A Element Fetcher: 读A_col_idx[p] → k |
| B 矩阵访问 | DR: B_den[colIdx, denCol] (Dense索引) | **B Row Controller: B_row_ptr[k]→B_row_ptr[k+1] 范围** |
| B 元素流 | 循环 denCol 0..denXSize-1 | **B Element Streamer: 顺序读取 B_col_idx[q], B_val[q]** |
| MAC 计算 | M: A_val × B_den[i] | MUL Array: A_val × B_val (流式) |
| 累加器 | 密集数组 acc[0..nColInDense-1] | **SPA: acc_valid[j], acc_val[j], touched_cols** |
| 输出 | 按行写 BankedScratchpad | 按行输出 row_id, row_nnz, col_idx[], val[] |

#### 关键改动：Dense累加器 → SPA稀疏累加器

```verilog
// Chisel SPMM: 密集累加
m_mac = (dr_isNewOutput_q) ? m_multiply : m_acc_q + m_multiply;
// 目标列 = denCol (0..denXSize-1 顺序递增), 所有列都被覆盖

// Verilog SPGEMM: 稀疏累加
if (!acc_valid[j]) begin
    acc_valid[j] <= 1;
    acc_val[j]   <= partial;
    touched_cols.push(j);   // 记录该行哪些列被写过
end else begin
    acc_val[j] <= acc_val[j] + partial;
end
// 目标列 = B_col_idx[q], 只有B非零列才被覆盖
```

---

### 2.9 数据搬运对照 (Compute.scala → core_top.v)

Chisel `Compute.scala` 的核心功能之一是从 GlobalBuffer 把 CSR 数据搬运到每个 Group 的 Scratchpad：

```scala
// Chisel: 按 Group 拆分 CSR 数据
sDataMoveRow: 遍历 row_ptr，按非零元素边界切分 → 写入 Group[i].spPtr
sDataMoveCol: 遍历 col_idx，按 blockSize 搬运 → 写入 Group[i].spCol
sDataMoveVal: 遍历 val，同样切分 → 写入 Group[i].spVal
sDataMoveDen: 遍历 dense，全部复制 → 写入 Group[i].spDen (每个Group完整Dense)
```

在 Verilog SPGEMM 中，这个数据搬运逻辑对应 `core_top.v` 的 Load 阶段：

```
Chisel sDataMoveRow → 对应 LOAD_A row:     Load A_row_ptr 到 GlobalBuffer
Chisel sDataMoveCol → 对应 LOAD_A col:     Load A_col_idx 到 GlobalBuffer
Chisel sDataMoveVal → 对应 LOAD_A val:     Load A_val 到 GlobalBuffer
Chisel sDataMoveDen → 对应 LOAD_B *3:     Load B_row_ptr/col_idx/val 到 GlobalBuffer
                                            然后 PE 从 GlobalBuffer 读入本地 B Buffer
```

**SPMM 有 Group 间切割逻辑（rowPtrFin/colFin/valFin 检测 Group 边界），SPGEMM 不需要**，因为每个 PE 加载完整 B 或各自范围的 A。

---

### 2.10 Store（数据写回）

| Chisel | Verilog | 改动 |
|--------|---------|------|
| 5状态: sIdle→sWriteCmd→sWriteData→sReadMem→sWriteAck | 同逻辑 | ✅ 不变 |
| 从 OutputScratchpad 读 → AXI 写 DDR | 同 | ✅ 不变 |
| Dense 输出: 按行 stride 写回 | CSR 输出: 按 CSR 三数组写回 | 输出数据内容变，但 Store 模块本身逻辑不变 |

**结论：Store 模块完全可复用不变。写回内容由 C CSR Writer 准备好即可。**

---

## 三、ISA 扩展方案

### 3.1 当前 ISA 编码

```
256-bit 指令布局:
┌──────────────────────────────────────────────────────┬───────┬──────────┐
│              指令字段 (250 bits)                       │ MemID │  Opcode  │
│              取决于指令类型                             │ [5:3] │  [2:0]   │
└──────────────────────────────────────────────────────┴───────┴──────────┘

Opcode:
  3'b001: LOAD   - 从 DDR 加载数据到 GlobalBuffer
  3'b010: STORE  - 从 OutputScratchpad 写回 DDR
  3'b011: SPGEMM - 触发 SpGEMM 计算
  3'b111: FINISH - 终止指令序列

MemID (仅 LOAD/STORE):
  3'b000: A_col_idx / B_col_idx (通过不同 LOAD 地址区分 A/B)
  3'b001: A_row_ptr / B_row_ptr
  3'b010: A_val / B_val

现有方案通过 Core 状态机区分 A/B:
  LOAD_A_ROW → LOAD_A_COL → LOAD_A_VAL → LOAD_B_ROW → LOAD_B_COL → LOAD_B_VAL
  六条连续的 LOAD 指令，不需区分 A/B mem_id
```

### 3.2 需要新增的字段

#### (a) 操作类型 (支持 Mul/Add/Sub)
```
方案A: 复用 MemID 高位
  bit[7:6] = op_type:
    2'b00: MUL (SpGEMM)
    2'b01: ADD (SpAdd)
    2'b10: SUB (SpSubtract)
    2'b11: reserved

方案B: 在 SPGEMM 指令中增加字段
  bit[253:251] = op_type (3 bits, 直接扩展预留位)

推荐方案A: 因为 MemID 只需要 3 bits (8种), 我们只需要 col/ptr/val 三种 (A/B通过顺序区分)
```

#### (b) 主机端 Task Descriptor 指令 (新增)
```
新增指令类型: TASKDESC (opcode = 3'b100, 或复用 Load 的一种 MemID)

建议: 使用 LOAD 指令的保留 MemID:
  LOAD MemID=3'b110: 加载任务描述符
  LOAD MemID=3'b111: 加载完成信号

或新增 opcode:
  OP_TASK = 3'b100  → task descriptor load from DRAM
```

#### (c) Task Descriptor 格式 (主机→加速器)
```
每个 PE 的任务描述符 (建议打包在 256-bit 指令中):
  bit[7:0]   : pe_id
  bit[17:8]  : row_start
  bit[27:18] : row_end
  bit[43:28] : a_ptr_start
  bit[59:44] : a_ptr_end
  bit[255:60]: reserved

或者批量传输方案: 8个PE的描述符连续存放在DRAM，一条 TASKDESC_LOAD 指令加载全部：
  LOAD_TASK dram_offset=xxx, sram_offset=xxx, xsize=8×4=32 elements
```

### 3.3 最终 ISA 规划

```
Opcode 定义 (3-bit):
  3'b000: LOAD_DATA   - 加载 CSR 数据 (A_row/A_col/A_val/B_row/B_col/B_val)
  3'b001: LOAD_TASK   - 加载主机算好的 PE 任务描述符  ← 新增
  3'b010: STORE       - 写回结果 (不变)
  3'b011: SPGEMM      - 触发计算 (增加 op_type 子字段)
  3'b100: SPADD       - 稀疏矩阵加法  ← 新增
  3'b101: SPSUB       - 稀疏矩阵减法  ← 新增
  3'b111: FINISH      - 终止 (不变)

MemID 定义 (3-bit, 仅 LOAD_DATA 有效):
  3'b000: CSR_COL
  3'b001: CSR_PTR
  3'b010: CSR_VAL
  3'b011: TASK_DESC  ← 新增
  3'b100: reserved
```

---

## 四、数据流改造总览

### 4.1 SPMM 原始数据流

```
┌─ CPU ─┐
│ 写CR寄存器(启动, 指令地址, 指令数)
│ 指令序列: LOAD_COL→LOAD_PTR→LOAD_VAL→LOAD_DEN→SPMM→SOUT (×N次GCN层)
└───────┘
         ↓
┌─ 加速器 ──────────────────────────────────────────────────────────┐
│ Fetch: 从DDR取指令 → 解码 → 分发
│   LOAD_COL: Load模块 AXI读 DDR→GlobalBuffer(spCol)
│   LOAD_PTR: Load模块 AXI读 DDR→GlobalBuffer(spPtr)
│   LOAD_VAL: Load模块 AXI读 DDR→GlobalBuffer(spVal)
│   LOAD_DEN: Load模块 AXI读 DDR→GlobalBuffer(spDen)  ← Dense B
│   SPMM:    Compute模块启动
│             ① 按Group拆分CSR数据 (rowPtr/col/val/den)
│             ② 32个Group并行计算 (D1→D2→DR→M流水)
│             ③ Dense输出写 OutputScratchpad
│   SOUT:    Store模块 OutputScratchpad→AXI写DDR
└──────────────────────────────────────────────────────────────────┘
```

### 4.2 SPGEMM 目标数据流

```
┌─ CPU ────────────────────────────────────────────────────────┐
│ 1. 写CR寄存器(启动, 指令地址, 指令数)                          │
│ 2. 软件 Scheduler:                                           │
│    - 读取 A CSRs, B_row_ptr                                  │
│    - 计算 b_row_nnz[k], row_cyc[i]                           │
│    - 动态剩余目标切分 → 生成8个PE的 task descriptors           │
│    - 将 task descriptors 写到 DDR 约定地址                     │
│ 3. 指令序列:                                                 │
│    LOAD(PTR) → LOAD(COL) → LOAD(VAL)    ← A CSR             │
│    LOAD(PTR) → LOAD(COL) → LOAD(VAL)    ← B CSR             │
│    LOAD(TASK)                            ← 主机 TaskDesc     │
│    SPGEMM / SPADD / SPSUB               ← 计算               │
│    STORE                                 ← 写回               │
└──────────────────────────────────────────────────────────────┘
         ↓
┌─ 加速器 ───────────────────────────────────────────────────────────┐
│ Fetch/Decode: 不变                                                │
│ Load A CSR ×3: 不变                                               │
│ Load B CSR ×3: 不变 (B从Dense变为CSR, 多两次Load)                  │
│ Load Task: Load模块 AXI读 DDR→GlobalBuffer (task descriptors) 新增 │
│                                                                   │
│ core_top 状态机:                                                   │
│   IDLE → LOAD_A(×3) → LOAD_B(×3) → LOAD_TASK → COMPUTE →         │
│   WRITE_CSR → STORE → FINISH/(循环)                               │
│                                                                   │
│ COMPUTE 阶段 (PE Array ×8):                                       │
│   每个 PE:                                                        │
│     ① 从 task descriptor 读取 row_start/row_end                   │
│     ② 从 GlobalBuffer 加载 A 行范围 + B 完整 CSR 到本地 Buffer     │
│     ③ for row in row_start..row_end:                             │
│          for each A(i,k):                                         │
│            read B_row_ptr[k]..B_row_ptr[k+1] (B第k行范围)         │
│            for each B(k,j):                                       │
│              partial = A_val × B_val  (或 + / -)                  │
│              SPA[j] += partial                                    │
│          output C row: row_nnz + col_idx[] + val[]               │
│                                                                   │
│ WRITE_CSR 阶段 (C CSR Writer):                                    │
│     ① 收集所有 PE 输出的行结果                                     │
│     ② 对 C_row_nnz[] 做前缀和 → C_row_ptr[]                       │
│     ③ 按序写出 C_col_idx[] 和 C_val[]                             │
│                                                                   │
│ STORE 阶段: 不变 (CSR 三数组写回 DDR)                              │
└──────────────────────────────────────────────────────────────────┘
```

---

## 五、Verilog 模块改造清单

### 5.1 可复用 (不改或微改)

| 文件 | 改造程度 | 说明 |
|------|---------|------|
| `wrapper.v` | 不改 | 顶层封装完全等价 |
| `isa.vh` | **微改** | 增加 TASK opcode, 增加 op_type 字段 |
| `defines.vh` | **微改** | 增加 N_PE=8 等参数确认 |
| `fetch.v` | **微改** | 增加 TASK 指令分发通道，删除 SCH 通道 |
| `decode.v` | **微改** | 增加 task_decode 子模块 |
| `load.v` | **不改** | Load 模块逻辑完全通用 |
| `store.v` | **不改** | Store 模块逻辑完全通用 |
| `axi_interface.v` | **不改** | AXI 接口不变 |
| `scratchpad.v` | **不改** | SRAM 模型不变 |

### 5.2 需要大改

| 文件 | 改造程度 | 说明 |
|------|---------|------|
| `core_top.v` | **重写状态机** | SCHEDULE→LOAD_TASK, 去掉Scheduler子状态, 增加 op_type 选择 |
| `pe_top.v` | **重写** | 参考 Chisel pipelinePE 的握手逻辑, 但用 SPA 替代密集累加器 |
| `pe_decompress.v` | **重写** | 保留 Row-Block + preload 机制, 改用 Chisel D1→D2→DR→M 流水风格 |
| `pe_mul_array.v` | **大改** | 增加 ADD/SUB 模式, 改为可配置 ALU |
| `pe_aggregation.v` | **中改** | SPA 逻辑保留, 增加 bank conflict 处理, 对齐 Chisel 的 m_acc_q 风格 |

### 5.3 需要删除

| 文件 | 原因 |
|------|------|
| `scheduler.v` | 移到主机端 |

### 5.4 需要新增

| 文件 | 功能 |
|------|------|
| `task_loader.v` | 从 GlobalBuffer 读取主机 task descriptors, 分发给各 PE |
| `sp_elementwise.v` | SpAdd/SpSub element-wise merge-sort 计算单元 |
| `host_scheduler.c / .py` | 主机端软件: 计算 b_row_nnz, row_cyc, 动态切分, 生成 task descriptor |

---

## 六、关键 Chisel 模式 → Verilog 转换参考

### 6.1 Decoupled 握手 → valid/ready

```scala
// Chisel: Decoupled
val io = Decoupled(UInt(32.W))
io.valid := ...  // 生产者驱动
io.ready := ...  // 消费者驱动
io.bits  := ...  // 数据

// Verilog:
output reg        valid;
input  wire       ready;
output reg [31:0] data;
// 握手条件: valid && ready
```

### 6.2 RegEnable 带使能

```scala
// Chisel
val reg = RegEnable(data, enable)

// Verilog
always @(posedge clk)
    if (enable)
        reg <= data;
```

### 6.3 流水线级间 valid

```scala
// Chisel: pipelinedPE 流水线
d2_valid_q := d2_nextValid || d1_valid  // 本级=上一级有效 OR 本级自循环
d1_moving  := !d2_nextValid             // 上家等待下家消费完

// Verilog: 等效
wire d1_moving = !d2_nextValid;
always @(posedge clk)
    d2_valid_q <= d2_nextValid || d1_valid;
```

### 6.4 SyncReadMem → reg array

```scala
// Chisel
val mem = SyncReadMem(512, UInt(32.W))
val rdata = mem.read(addr, ren)

// Verilog
reg [31:0] mem [0:511];
wire [31:0] rdata = mem[addr];
```

---

## 七、下一步行动 (Phase 2 入口)

按以下顺序开始写 Verilog 代码：

1. **修改 `isa.vh`** — 增加 TASK opcode, op_type 字段, 统一编码方案
2. **修改 `core_top.v`** — 去掉 SCHEDULE 状态, 增加 LOAD_TASK 状态, 增加 op_type 分支
3. **新增 `task_loader.v`** — 读取主机 task descriptor 分发给 PE
4. **重写 `pe_top.v` + `pe_decompress.v`** — 参照 Chisel pipelinePE 流水风格
5. **修改 `pe_mul_array.v`** — 增加 ADD/SUB 模式
6. **新增 `sp_elementwise.v`** — 加/减法专用模块
7. **写主机端 `host_scheduler.c`** — 软件调度器
8. **修改 `test/`** — 适配新 ISA, 新增 ADD/SUB 测试用例
