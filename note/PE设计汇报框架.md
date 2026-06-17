# SpGEMM 加速器 PE（Processing Element）设计汇报框架



---

## 一、PE 核心目标与应用场景

### 1.1 解决什么问题

$$C = A \times B$$

A、B 为稀疏矩阵（非零率 ~30%），C 输出为 dense 矩阵。FP16 精度。

### 1.2 为什么稀疏乘法需要专用硬件

```
Dense GEMM:     每个 C[i][j] = ∑ A[i][k] × B[k][j]   所有 k 都算
SpGEMM:        每个 C[i][j] = ∑ A[i][k] × B[k][j]   只算 A 非零的 k
                                                     且只算 B 对应行非零的 j
```

CPU/GPU 处理稀疏乘法时，不规则访存导致 cache miss 严重。专用加速器将稀疏访问转为连续流式处理。

### 1.3 应用场景

- 图神经网络（GCN）前向传播
- 稀疏矩阵分解
- 社交网络分析中的矩阵运算
- 赛题：给定稀疏 A/B，计算 dense C

---

## 二、整体架构与数据流

### 2.1 PE 在系统中的位置

```
Host (CPU)                          FPGA 加速器
   │                                    │
   ├─ 数据预处理（CSR→compact row-desc） │
   ├─ 写入 DDR 固定地址                  │
   └─ 写入 CR 寄存器启动 ──────────────→ core_top ──→ PE Array ──→ C_dense → DDR
```

### 2.2 PE 内部模块与数据流

```
                    ┌──────────────────────────────────────────┐
                    │              pe_top                      │
                    │                                          │
  A_row_desc[128]──→│ A_iterator ──→ task_packer ──→ task_fifo │
  A_col[16K]        │   ↓ cur_k              ↓ pack 4tasks     │ depth=256
  A_val[16K]        │ B_streamer ←── B_row_desc[512]           │
                    │   ↓ b_ptr          B_col[78K]            │
                    │   B_val[78K]                             │
                    │                                          │
  task_fifo ──→ 4×MAC (mul_array) ──→ product_fifo ──→        │
                │ 3-stage pipeline      depth=256     serializer│
                │                                       (4→1)  │
                │                                          ↓   │
                └──────────────────────→ accumulator ←─────────┘
                                         acc_buf[N]
                                         IDLE→ADD→WRITE (3c)
                                              │
                                         row_writeback → C_dense
```

**四层流水**：
1. **生成层**：A_iterator + B_streamer → 生成 (col, a_val, b_val) 三元组
2. **打包层**：task_packer → 4 个 task 打包为 1 个 task_group
3. **计算层**：4×MAC 并行乘法 → product_group
4. **归约层**：serializer + accumulator → 按列累加到 acc_buf

---

## 三、输入数据格式与处理流程

### 3.1 Compact Row-Desc 格式

Host 端将原始 CSR 矩阵转换为以下格式：

**A 矩阵（128 行/PE）**：
```
A_row_desc[ri] = {start_offset[31:0], row_nnz[15:0], global_row_id[15:0]}  64-bit
A_col[start+t] = k           ← A(i,k) 的列索引
A_val[start+t] = value       ← A(i,k) 的值
```

**B 矩阵（完整副本/PE）**：
```
B_row_desc[k] = {start_offset[31:0], reserved[15:0], row_nnz[15:0]}  64-bit
B_col[start+u] = j           ← B(k,j) 的列索引
B_val[start+u] = value       ← B(k,j) 的值
```

### 3.2 一行 A 的处理流程（逐步）

**Step 1: 读 A 行描述符**
```
row_desc = A_row_desc_buf[row_idx]
global_row_id = row_desc[15:0]    // C 行号
a_nnz = row_desc[31:16]           // 该行非零数
a_start = row_desc[63:32]         // col/val 数组起始偏移
```

**Step 2: 遍历每个 A(i,k)**
```
for t in 0..a_nnz-1:
    k = A_col[a_start + t]        // 列号
    a_val = A_val[a_start + t]    // A 的值
    
    // 读 B 的第 k 行
    b_desc = B_row_desc_buf[k]
    b_nnz = b_desc[15:0]
    b_start = b_desc[63:32]
    
    // 遍历 B(k,:) 所有非零
    for u in 0..b_nnz-1:
        j = B_col[b_start + u]    // B 的列号
        b_val = B_val[b_start + u]
        
        task = {j, a_val, b_val}  // 送入 task_packer
```

**Step 3: task_packer 打包**
```
task 逐个进入 → pack_count 0→1→2→3
pack_count=3 时: pack_task3 到位 → 写 1 个满 group (4 个有效 lane) 入 FIFO
行末: FLUSH → 写部分 group (1~3 个有效 lane) 入 FIFO
```

**Step 4: MAC 计算**
```
从 task_fifo 读 1 个 group:
  lane0: {col0, a_val0, b_val0} → product0 = a0 × b0
  lane1: {col1, a_val1, b_val1} → product1 = a1 × b1
  lane2: {col2, a_val2, b_val2} → product2 = a2 × b2
  lane3: {col3, a_val3, b_val3} → product3 = a3 × b3
  
product_group = {col0,val0, col1,val1, col2,val2, col3,val3} → product_fifo
```

**Step 5: 归约累加**
```
serializer 从 product_fifo 读 product_group:
  逐 lane 取出 {col, val} → acc_in_valid=1

accumulator:
  ACC_IDLE:  读 acc_buf[col] → acc_old_reg
  ACC_ADD:   acc_new_reg = acc_old_reg + val
  ACC_WRITE:  acc_buf[col] ← acc_new_reg
  → 回到 ACC_IDLE (3 周期 / product)
```

**Step 6: 写回**
```
行处理完毕后:
  for col in 0..N-1:
    C_dense[global_row_id][col] = acc_buf[col]
```

---

## 四、关键约束与设计决策

### 4.1 为什么用 4 个 MAC 但累加器串行？

```
MAC 产出:     4 product / cycle
累加器消费:   1 product / 3 cycle  (读→加→写, 单端口 SRAM)

速度比 = 1/12 → MAC 利用率仅 8.3%
```

**原因**：acc_buf 为单端口 reg array，读和写不能同时进行。避免 bank 冲突简化了第一版设计。

### 4.2 为什么 A 元素间要 FLUSH+DRAIN？

每个 A(i,k) 处理完后，task_packer 内有残留 task（不足 4 个满组）。FLUSH 把它们写成部分 group 送入 FIFO，DRAIN 等待 product FIFO 和累加器排空。确保下一 A 元素开始时数据不乱。

**代价**：WT_PROD 占 42% 总时间（见性能分析报告）。

### 4.3 FIFO 深度为什么是 256？

```
每 A 元素 ~15 task → 15/4 ≈ 4 groups
50×50 总共: 225 groups

FIFO=32 时: 中间必须频繁 FLUSH+DRAIN 防止溢出
FIFO=256 时: 可以去掉中间 DRAIN，流水线更连续
             → 速度提升 10.2%
```

### 4.4 CLEAR_ACC 和 WRITE_ROW 为什么按实际 N 裁剪？

```
最初设计: 固定清除/写回 MAX_N=512 列，无论实际矩阵多大
优化后:   按实际 N 清除/写回，50×50 从 512→50 列/行
          → 省 46,200 周期，整体提速 58%
```

---

## 五、各模块协同逻辑

### 5.1 生产者-消费者握手链

```
B_streamer ──valid/ready──→ task_packer ──wr_en──→ task_fifo
                                                      │ rd_en
                                                 4×MAC ──wr_en──→ product_fifo
                                                                      │ rd_en
                                                              serializer ──valid/ready──→ accumulator
```

每个 FIFO 提供反压：满时上游自动暂停，空时下游自动等待。无需中央调度器。

### 5.2 关键状态机（pe_top FSM）

```
IDLE → LOAD_ROW_DESC → CLEAR_ACC → LOAD_A_ELEM → LOAD_B_DESC
  → STREAM_B_ROW ──→ FLUSH ──→ WT_TASK ──→ WT_PROD
       ↑                                    │
       └────── (a_nnz > 0) ←───────────────┘
                                  │ (a_nnz = 0)
                                  ↓
                              WRITE_ROW → NEXT_ROW → DONE
```

**中间 FLUSH+DRAIN** 为当前设计，已确认正确性。去掉中间 DRAIN（FIFO=256）提速 10.2%。

### 5.3 task_packer 与 serializer 的对称设计

```
task_packer:                         serializer:
  task_in → pack 4 tasks → group      group_in → 拆 4 products → acc_in
  64bit → 260bit                      132bit → 32bit
  (打包)                              (拆包)
```

两者互为逆过程，形成对称的流水线接口。

---

## 六、性能总结

| 指标 | 值 |
|------|-----|
| MAC 数量 | 4 @ 100MHz |
| 50×50 30% 稀疏 | 40,451 周期 |
| MAC 利用率 (计算段) | 7.9% |
| 吞吐率 | ~56 MFLOPS |
| 片上存储 | ~389 KB reg + 512 KB C_dense |
| 已验证规模 | 2×2 ~ 50×50 (100% 正确率) |

**主要瓶颈**：串行累加器 (3c/product, 42% 总时间)

**优化方向**：Banked accumulator (4-bank) → MAC 利用率可达 ~60%

---

## 七、开发历程总结

共修复 8 个关键 bug，覆盖握手协议、流水线同步、FSM 死锁、FIFO 反压等典型硬件设计问题。最终版本通过 7 种规模测试验证，具备向 FPGA 综合迁移的基础。
