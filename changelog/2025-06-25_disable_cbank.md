# 2025-06-25 — 注释掉 pe_top.v 中的 C buffer（c_bank0~3）

## 目的
暂时禁用 C buffer 存储以节省 FPGA 资源（256 KB），方便综合验证其他逻辑。

## 修改文件
- `rtl/core/pe_top.v`
- `rtl/core/pe_cluster.v`
- `rtl/sim/tb_pe_top.v`
- `rtl/sim/tb_pe_cluster.v`
- `test/pe_sim/test_comp.py`

## 改动内容

### pe_top.v（6 处）
| # | 位置 | 内容 | 效果 |
|---|---|---|---|
| 1 | 端口声明 | `c_rd_en, c_rd_addr, c_rd_data` | 3 个端口注释掉 |
| 2 | localparam | `C_BANK_ADDR_W, C_BANK_DEPTH, C_RD_ADDR_W` | 注释掉 |
| 3 | SRAM 声明 | `c_bank0~c_bank3` (32768×4×16b) | 省 ~256 KB |
| 4 | `COCOTB_SIM initial` | 初始化循环 | 注释掉 |
| 5 | C buffer write | `c_wr_*` 连线 + always 写 | 注释掉 |
| 6 | C buffer read | `rd_bank/rd_baddr` + always 读回 | 注释掉 |

### pe_cluster.v（4 处）
- 端口声明：`c_rd_en, c_rd_addr, c_rd_data` 注释掉
- pe_top 实例化：`.c_rd_en/addr/data` 连接注释掉
- 尾逗号修复：端口声明 `b_desc_wdata,` → `b_desc_wdata`
- 尾逗号修复：实例连接 `b_desc_wdata(b_desc_wdata),` 去逗号

### tb_pe_top.v（3 处）
- reg/wire 声明：`c_rd_en, c_rd_addr, c_rd_data` 注释掉
- pe_top 实例化：`.c_rd_en/addr/data` 连接注释掉
- 尾逗号修复：实例连接 `b_desc_wdata(b_desc_wdata),` 去逗号

### tb_pe_cluster.v（3 处）
- reg/wire 声明：`c_rd_en, c_rd_addr, c_rd_data` 注释掉
- pe_cluster 实例化：`.c_rd_en/addr/data` 连接注释掉
- 尾逗号修复：实例连接 `b_desc_wdata(b_desc_wdata),` 去逗号

### test_comp.py（4 处）
- `rst()` / `rst_cluster()`：注释掉 `dut.c_rd_en/c_rd_addr` 赋值
- `read_c_buffer()` / `read_c_buffer_pe()`：改为直接 `return {}`
- 三个测试函数：注释掉 verify + assert，输出 `SKIPPED (c_bank disabled)`

## 剩余存储（约 397 KB）
- A buffer: 66 KB
- B buffer: 312 KB
- FIFO + 累加器: ~19 KB

---

# 附加修复：sync_fifo 读改同步（BRAM 推断优化）

## 目的
task_fifo 原消耗 12,000 LUT（组合读导致寄存器+MUX 树实现），改为同步读以启用 BRAM 推断。

## 修改文件
- `rtl/infrastructure/scratchpad.v`
- `rtl/core/pe_top.v`

## 改动内容

### scratchpad.v（4 次迭代）
1. `assign rd_data = mem[...]` → `reg rd_data`，在 `rd_en` 时钟沿同步更新
2. 端口 `output wire rd_data` → `output reg rd_data`，去内部重复 `reg` 声明
3. 读操作从条件 `if (rd_en && !rd_empty)` 内改为无条件每拍执行 → 启用 BRAM 推断
4. 加 `initial` 块全零初始化 `mem` 数组 → 消除仿真 X 传播
5. 加 `(* ram_style = "block" *)` 属性

### pe_top.v（2 次迭代）
1. task_fifo 消费者：加 `task_fifo_rd_en_d1`/`task_fifo_rd_data_d1` 1 拍延迟
2. product_fifo 消费者：加 `prod_fifo_rd_en_d1`/`prod_fifo_rd_data_d1` 1 拍延迟
3. 两对 d1 寄存器加 `negedge aresetn` 异步复位 → 消除仿真 X 传播

## 预期效果
- task_fifo: 12,000 LUT → ~200 LUT + 8 BRAM36
- product_fifo: ~800 LUT → ~150 LUT + 6 BRAM36

