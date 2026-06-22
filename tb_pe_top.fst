$date
	Sun Jun 21 19:54:11 2026
$end
$version
	Icarus Verilog
$end
$timescale
	1ps
$end
$scope module tb_pe_top $end
$var wire 1 ! done $end
$var wire 1 " cbuf_wr_valid $end
$var wire 16 # cbuf_wr_data [15:0] $end
$var wire 18 $ cbuf_wr_addr [17:0] $end
$var reg 10 % K [9:0] $end
$var reg 10 & M [9:0] $end
$var reg 10 ' N [9:0] $end
$var reg 16 ( a_col_waddr [15:0] $end
$var reg 16 ) a_col_wdata [15:0] $end
$var reg 1 * a_col_we $end
$var reg 8 + a_desc_waddr [7:0] $end
$var reg 64 , a_desc_wdata [63:0] $end
$var reg 1 - a_desc_we $end
$var reg 16 . a_val_waddr [15:0] $end
$var reg 16 / a_val_wdata [15:0] $end
$var reg 1 0 a_val_we $end
$var reg 1 1 aclk $end
$var reg 1 2 aresetn $end
$var reg 18 3 b_col_waddr [17:0] $end
$var reg 16 4 b_col_wdata [15:0] $end
$var reg 1 5 b_col_we $end
$var reg 10 6 b_desc_waddr [9:0] $end
$var reg 64 7 b_desc_wdata [63:0] $end
$var reg 1 8 b_desc_we $end
$var reg 18 9 b_val_waddr [17:0] $end
$var reg 16 : b_val_wdata [15:0] $end
$var reg 1 ; b_val_we $end
$var reg 1 < cbuf_wr_ready $end
$var reg 16 = row_count [15:0] $end
$var reg 1 > start $end
$scope module u_pe $end
$var wire 10 ? K [9:0] $end
$var wire 10 @ M [9:0] $end
$var wire 10 A N [9:0] $end
$var wire 14 B a_col_waddr [13:0] $end
$var wire 16 C a_col_wdata [15:0] $end
$var wire 1 * a_col_we $end
$var wire 7 D a_desc_waddr [6:0] $end
$var wire 64 E a_desc_wdata [63:0] $end
$var wire 1 - a_desc_we $end
$var wire 14 F a_val_waddr [13:0] $end
$var wire 16 G a_val_wdata [15:0] $end
$var wire 1 0 a_val_we $end
$var wire 1 H acc_inp_done $end
$var wire 1 I acc_out_ready $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 1 J b_batch_done $end
$var wire 17 K b_col_waddr [16:0] $end
$var wire 16 L b_col_wdata [15:0] $end
$var wire 1 5 b_col_we $end
$var wire 9 M b_desc_waddr [8:0] $end
$var wire 64 N b_desc_wdata [63:0] $end
$var wire 1 8 b_desc_we $end
$var wire 17 O b_val_waddr [16:0] $end
$var wire 16 P b_val_wdata [15:0] $end
$var wire 1 ; b_val_we $end
$var wire 1 < cbuf_wr_ready $end
$var wire 1 " cbuf_wr_valid $end
$var wire 4 Q mac_lane_valid [3:0] $end
$var wire 1 R prod_fifo_rd_en $end
$var wire 1 S product_group_wr_en $end
$var wire 16 T row_count [15:0] $end
$var wire 1 > start $end
$var wire 1 U task_drain_done $end
$var wire 1 V task_fifo_rd_en $end
$var wire 1 W task_in_valid $end
$var wire 1 X task_packer_ready $end
$var wire 64 Y task_in_data [63:0] $end
$var wire 1 Z task_group_wr_en $end
$var wire 260 [ task_group_wr_data [259:0] $end
$var wire 1 \ task_flush_pack $end
$var wire 1 ] task_flush_done $end
$var wire 260 ^ task_fifo_rd_data [259:0] $end
$var wire 1 _ task_fifo_full $end
$var wire 1 ` task_fifo_empty $end
$var wire 132 a product_group_wr_data [131:0] $end
$var wire 1 b product_fifo_full $end
$var wire 132 c prod_fifo_rd_data [131:0] $end
$var wire 1 d prod_fifo_empty $end
$var wire 4 e mul_valid [3:0] $end
$var wire 128 f mul_product [127:0] $end
$var wire 256 g mac_lane_task [255:0] $end
$var wire 16 h cbuf_wr_data [15:0] $end
$var wire 18 i cbuf_wr_addr [17:0] $end
$var wire 1 j acc_row_start $end
$var wire 1 k acc_row_done $end
$var wire 32 l acc_out_value [31:0] $end
$var wire 1 m acc_out_valid $end
$var wire 16 n acc_out_row_id [15:0] $end
$var wire 9 o acc_out_col_id [8:0] $end
$var wire 4 p acc_lane_valid [3:0] $end
$var wire 64 q acc_lane_product [63:0] $end
$var wire 36 r acc_lane_col_id [35:0] $end
$var wire 1 s acc_issue_ready $end
$var parameter 4 t PE_CLEAR_ACC $end
$var parameter 4 u PE_DONE $end
$var parameter 4 v PE_FLUSH_TASK_PACK $end
$var parameter 32 w PE_ID $end
$var parameter 4 x PE_IDLE $end
$var parameter 4 y PE_LOAD_A_ELEM $end
$var parameter 4 z PE_LOAD_B_DESC $end
$var parameter 4 { PE_LOAD_ROW_DESC $end
$var parameter 4 | PE_NEXT_ROW $end
$var parameter 4 } PE_STREAM_B_ROW $end
$var parameter 4 ~ PE_WAIT_PRODUCT_DRAIN $end
$var parameter 4 !" PE_WAIT_TASK_DRAIN $end
$var parameter 4 "" PE_WRITE_ROW $end
$var reg 16 #" a_nnz_left [15:0] $end
$var reg 32 $" a_ptr [31:0] $end
$var reg 16 %" b_nnz_left [15:0] $end
$var reg 32 &" b_ptr [31:0] $end
$var reg 64 '" b_row_desc_reg [63:0] $end
$var reg 16 (" cur_a_row_nnz [15:0] $end
$var reg 32 )" cur_a_start [31:0] $end
$var reg 16 *" cur_a_val [15:0] $end
$var reg 16 +" cur_global_row [15:0] $end
$var reg 16 ," cur_k [15:0] $end
$var reg 1 ! done $end
$var reg 256 -" mac_lane_task_r [255:0] $end
$var reg 4 ." mac_lane_valid_r [3:0] $end
$var reg 64 /" row_desc_reg [63:0] $end
$var reg 7 0" row_idx [6:0] $end
$var reg 4 1" state [3:0] $end
$var reg 4 2" state_next [3:0] $end
$var reg 1 3" state_stable $end
$var reg 10 4" write_global_row [9:0] $end
$scope module u_mul_array $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 256 5" lane_task [255:0] $end
$var wire 4 6" lane_valid [3:0] $end
$var wire 4 7" mul_valid [3:0] $end
$var wire 128 8" mul_product [127:0] $end
$scope begin gen_lane[0] $end
$var wire 16 9" mac_a [15:0] $end
$var wire 16 :" mac_b [15:0] $end
$var wire 16 ;" mac_col [15:0] $end
$var wire 32 <" product [31:0] $end
$var wire 16 =" mul_comb [15:0] $end
$var parameter 2 >" m $end
$var integer 32 ?" s [31:0] $end
$upscope $end
$scope begin gen_lane[1] $end
$var wire 16 @" mac_a [15:0] $end
$var wire 16 A" mac_b [15:0] $end
$var wire 16 B" mac_col [15:0] $end
$var wire 32 C" product [31:0] $end
$var wire 16 D" mul_comb [15:0] $end
$var parameter 2 E" m $end
$var integer 32 F" s [31:0] $end
$upscope $end
$scope begin gen_lane[2] $end
$var wire 16 G" mac_a [15:0] $end
$var wire 16 H" mac_b [15:0] $end
$var wire 16 I" mac_col [15:0] $end
$var wire 32 J" product [31:0] $end
$var wire 16 K" mul_comb [15:0] $end
$var parameter 3 L" m $end
$var integer 32 M" s [31:0] $end
$upscope $end
$scope begin gen_lane[3] $end
$var wire 16 N" mac_a [15:0] $end
$var wire 16 O" mac_b [15:0] $end
$var wire 16 P" mac_col [15:0] $end
$var wire 32 Q" product [31:0] $end
$var wire 16 R" mul_comb [15:0] $end
$var parameter 3 S" m $end
$var integer 32 T" s [31:0] $end
$upscope $end
$upscope $end
$scope module u_product_fifo $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 132 U" rd_data [131:0] $end
$var wire 1 R rd_en $end
$var wire 132 V" wr_data [131:0] $end
$var wire 1 S wr_en $end
$var wire 1 b wr_full $end
$var wire 1 d rd_empty $end
$var wire 9 W" count [8:0] $end
$var parameter 32 X" DEPTH $end
$var parameter 32 Y" DEPTH_LOG $end
$var parameter 32 Z" WIDTH $end
$var reg 9 [" rd_ptr [8:0] $end
$var reg 9 \" wr_ptr [8:0] $end
$upscope $end
$scope module u_row_acc $end
$var wire 1 ]" all_clr_done $end
$var wire 1 ^" all_fifos_empty $end
$var wire 1 _" all_rmw_done $end
$var wire 1 1 clk $end
$var wire 1 `" cur_valid_entry $end
$var wire 1 a" do_enqueue $end
$var wire 7 b" drain_rd_addr [6:0] $end
$var wire 1 s issue_ready $end
$var wire 1 c" issue_valid $end
$var wire 36 d" lane_col_id [35:0] $end
$var wire 64 e" lane_product [63:0] $end
$var wire 4 f" lane_valid [3:0] $end
$var wire 1 g" last_cell $end
$var wire 1 I out_ready $end
$var wire 16 h" row_id_in [15:0] $end
$var wire 1 H row_input_done $end
$var wire 1 j row_start $end
$var wire 1 2 rst_n $end
$var wire 1 i" tag_clear_pulse $end
$var wire 64 j" wr_data_flat [63:0] $end
$var wire 28 k" wr_addr_flat [27:0] $end
$var wire 16 l" sel_tag [15:0] $end
$var wire 32 m" sel_acc [31:0] $end
$var wire 1 n" rmw_b3 $end
$var wire 1 o" rmw_b2 $end
$var wire 1 p" rmw_b1 $end
$var wire 1 q" rmw_b0 $end
$var wire 3 r" mc3 [2:0] $end
$var wire 3 s" mc2 [2:0] $end
$var wire 3 t" mc1 [2:0] $end
$var wire 3 u" mc0 [2:0] $end
$var wire 4 v" free_b3 [3:0] $end
$var wire 4 w" free_b2 [3:0] $end
$var wire 4 x" free_b1 [3:0] $end
$var wire 4 y" free_b0 [3:0] $end
$var wire 1 z" emp_b3 $end
$var wire 1 {" emp_b2 $end
$var wire 1 |" emp_b1 $end
$var wire 1 }" emp_b0 $end
$var wire 16 ~" dtag_b3 [15:0] $end
$var wire 16 !# dtag_b2 [15:0] $end
$var wire 16 "# dtag_b1 [15:0] $end
$var wire 16 ## dtag_b0 [15:0] $end
$var wire 32 $# dacc_b3 [31:0] $end
$var wire 32 %# dacc_b2 [31:0] $end
$var wire 32 &# dacc_b1 [31:0] $end
$var wire 32 '# dacc_b0 [31:0] $end
$var wire 1 (# clr_b3 $end
$var wire 1 )# clr_b2 $end
$var wire 1 *# clr_b1 $end
$var wire 1 +# clr_b0 $end
$var wire 4 ,# bwv3 [3:0] $end
$var wire 4 -# bwv2 [3:0] $end
$var wire 4 .# bwv1 [3:0] $end
$var wire 4 /# bwv0 [3:0] $end
$var wire 2 0# bid3 [1:0] $end
$var wire 2 1# bid2 [1:0] $end
$var wire 2 2# bid1 [1:0] $end
$var wire 2 3# bid0 [1:0] $end
$var parameter 32 4# ACC_W $end
$var parameter 33 5# BANK_ADDR_W $end
$var parameter 32 6# BANK_DEPTH $end
$var parameter 32 7# BANK_FIFO_DEPTH $end
$var parameter 32 8# BANK_FIFO_LOG $end
$var parameter 33 9# BANK_LAST $end
$var parameter 32 :# COL_W $end
$var parameter 32 ;# EPOCH_W $end
$var parameter 32 <# OUT_COLS $end
$var parameter 32 =# PROD_W $end
$var parameter 32 ># ROW_W $end
$var parameter 3 ?# S_ACCUM $end
$var parameter 3 @# S_CLEAR_TAGS $end
$var parameter 3 A# S_DONE $end
$var parameter 3 B# S_DRAIN $end
$var parameter 3 C# S_IDLE $end
$var parameter 3 D# S_WAIT_DRAIN $end
$var reg 2 E# bank_sel [1:0] $end
$var reg 1 F# busy $end
$var reg 1 G# clr_triggered $end
$var reg 16 H# cur_row_id [15:0] $end
$var reg 1 I# drain_emit $end
$var reg 7 J# group_addr [6:0] $end
$var reg 1 K# input_done_latch $end
$var reg 9 L# out_col_id [8:0] $end
$var reg 16 M# out_row_id [15:0] $end
$var reg 1 m out_valid $end
$var reg 32 N# out_value [31:0] $end
$var reg 9 O# prev_col [8:0] $end
$var reg 1 P# prev_col_valid $end
$var reg 1 k row_done $end
$var reg 16 Q# row_epoch [15:0] $end
$var reg 3 R# state [2:0] $end
$var integer 32 S# ai [31:0] $end
$var integer 32 T# bi [31:0] $end
$scope module u_bank0 $end
$var wire 1 1 clk $end
$var wire 1 U# deq_fire $end
$var wire 32 V# drain_acc [31:0] $end
$var wire 7 W# drain_rd_addr [6:0] $end
$var wire 16 X# drain_tag [15:0] $end
$var wire 16 Y# row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 +# tag_clear_busy $end
$var wire 1 i" tag_clear_en $end
$var wire 3 Z# waddr0 [2:0] $end
$var wire 28 [# wr_addr_flat [27:0] $end
$var wire 64 \# wr_data_flat [63:0] $end
$var wire 4 ]# wr_valid [3:0] $end
$var wire 3 ^# wr_cnt [2:0] $end
$var wire 3 _# waddr3 [2:0] $end
$var wire 3 `# waddr2 [2:0] $end
$var wire 3 a# waddr1 [2:0] $end
$var wire 2 b# slot3 [1:0] $end
$var wire 2 c# slot2 [1:0] $end
$var wire 2 d# slot1 [1:0] $end
$var wire 1 q" rmw_busy $end
$var wire 4 e# free_count [3:0] $end
$var wire 1 }" fifo_empty $end
$var parameter 32 f# ACC_W $end
$var parameter 33 g# BANK_ADDR_W $end
$var parameter 32 h# BANK_DEPTH $end
$var parameter 33 i# BANK_LAST $end
$var parameter 34 j# ENTRY_W $end
$var parameter 32 k# EPOCH_W $end
$var parameter 32 l# FIFO_DEPTH $end
$var parameter 32 m# FIFO_DEPTH_LOG $end
$var parameter 33 n# FIFO_MASK $end
$var parameter 32 o# PROD_W $end
$var parameter 2 p# RMW_ADD $end
$var parameter 2 q# RMW_IDLE $end
$var parameter 2 r# RMW_READ $end
$var parameter 2 s# RMW_WRITE $end
$var reg 1 +# clr_active $end
$var reg 7 t# clr_idx [6:0] $end
$var reg 4 u# fifo_cnt [3:0] $end
$var reg 3 v# fifo_head [2:0] $end
$var reg 3 w# fifo_tail [2:0] $end
$var reg 7 x# rmw_addr [6:0] $end
$var reg 32 y# rmw_new [31:0] $end
$var reg 32 z# rmw_old [31:0] $end
$var reg 16 {# rmw_old_tag [15:0] $end
$var reg 16 |# rmw_prod [15:0] $end
$var reg 2 }# rmw_st [1:0] $end
$var integer 32 ~# ci [31:0] $end
$var integer 32 !$ fi [31:0] $end
$upscope $end
$scope module u_bank1 $end
$var wire 1 1 clk $end
$var wire 1 "$ deq_fire $end
$var wire 32 #$ drain_acc [31:0] $end
$var wire 7 $$ drain_rd_addr [6:0] $end
$var wire 16 %$ drain_tag [15:0] $end
$var wire 16 &$ row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 *# tag_clear_busy $end
$var wire 1 i" tag_clear_en $end
$var wire 3 '$ waddr0 [2:0] $end
$var wire 28 ($ wr_addr_flat [27:0] $end
$var wire 64 )$ wr_data_flat [63:0] $end
$var wire 4 *$ wr_valid [3:0] $end
$var wire 3 +$ wr_cnt [2:0] $end
$var wire 3 ,$ waddr3 [2:0] $end
$var wire 3 -$ waddr2 [2:0] $end
$var wire 3 .$ waddr1 [2:0] $end
$var wire 2 /$ slot3 [1:0] $end
$var wire 2 0$ slot2 [1:0] $end
$var wire 2 1$ slot1 [1:0] $end
$var wire 1 p" rmw_busy $end
$var wire 4 2$ free_count [3:0] $end
$var wire 1 |" fifo_empty $end
$var parameter 32 3$ ACC_W $end
$var parameter 33 4$ BANK_ADDR_W $end
$var parameter 32 5$ BANK_DEPTH $end
$var parameter 33 6$ BANK_LAST $end
$var parameter 34 7$ ENTRY_W $end
$var parameter 32 8$ EPOCH_W $end
$var parameter 32 9$ FIFO_DEPTH $end
$var parameter 32 :$ FIFO_DEPTH_LOG $end
$var parameter 33 ;$ FIFO_MASK $end
$var parameter 32 <$ PROD_W $end
$var parameter 2 =$ RMW_ADD $end
$var parameter 2 >$ RMW_IDLE $end
$var parameter 2 ?$ RMW_READ $end
$var parameter 2 @$ RMW_WRITE $end
$var reg 1 *# clr_active $end
$var reg 7 A$ clr_idx [6:0] $end
$var reg 4 B$ fifo_cnt [3:0] $end
$var reg 3 C$ fifo_head [2:0] $end
$var reg 3 D$ fifo_tail [2:0] $end
$var reg 7 E$ rmw_addr [6:0] $end
$var reg 32 F$ rmw_new [31:0] $end
$var reg 32 G$ rmw_old [31:0] $end
$var reg 16 H$ rmw_old_tag [15:0] $end
$var reg 16 I$ rmw_prod [15:0] $end
$var reg 2 J$ rmw_st [1:0] $end
$var integer 32 K$ ci [31:0] $end
$var integer 32 L$ fi [31:0] $end
$upscope $end
$scope module u_bank2 $end
$var wire 1 1 clk $end
$var wire 1 M$ deq_fire $end
$var wire 32 N$ drain_acc [31:0] $end
$var wire 7 O$ drain_rd_addr [6:0] $end
$var wire 16 P$ drain_tag [15:0] $end
$var wire 16 Q$ row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 )# tag_clear_busy $end
$var wire 1 i" tag_clear_en $end
$var wire 3 R$ waddr0 [2:0] $end
$var wire 28 S$ wr_addr_flat [27:0] $end
$var wire 64 T$ wr_data_flat [63:0] $end
$var wire 4 U$ wr_valid [3:0] $end
$var wire 3 V$ wr_cnt [2:0] $end
$var wire 3 W$ waddr3 [2:0] $end
$var wire 3 X$ waddr2 [2:0] $end
$var wire 3 Y$ waddr1 [2:0] $end
$var wire 2 Z$ slot3 [1:0] $end
$var wire 2 [$ slot2 [1:0] $end
$var wire 2 \$ slot1 [1:0] $end
$var wire 1 o" rmw_busy $end
$var wire 4 ]$ free_count [3:0] $end
$var wire 1 {" fifo_empty $end
$var parameter 32 ^$ ACC_W $end
$var parameter 33 _$ BANK_ADDR_W $end
$var parameter 32 `$ BANK_DEPTH $end
$var parameter 33 a$ BANK_LAST $end
$var parameter 34 b$ ENTRY_W $end
$var parameter 32 c$ EPOCH_W $end
$var parameter 32 d$ FIFO_DEPTH $end
$var parameter 32 e$ FIFO_DEPTH_LOG $end
$var parameter 33 f$ FIFO_MASK $end
$var parameter 32 g$ PROD_W $end
$var parameter 2 h$ RMW_ADD $end
$var parameter 2 i$ RMW_IDLE $end
$var parameter 2 j$ RMW_READ $end
$var parameter 2 k$ RMW_WRITE $end
$var reg 1 )# clr_active $end
$var reg 7 l$ clr_idx [6:0] $end
$var reg 4 m$ fifo_cnt [3:0] $end
$var reg 3 n$ fifo_head [2:0] $end
$var reg 3 o$ fifo_tail [2:0] $end
$var reg 7 p$ rmw_addr [6:0] $end
$var reg 32 q$ rmw_new [31:0] $end
$var reg 32 r$ rmw_old [31:0] $end
$var reg 16 s$ rmw_old_tag [15:0] $end
$var reg 16 t$ rmw_prod [15:0] $end
$var reg 2 u$ rmw_st [1:0] $end
$var integer 32 v$ ci [31:0] $end
$var integer 32 w$ fi [31:0] $end
$upscope $end
$scope module u_bank3 $end
$var wire 1 1 clk $end
$var wire 1 x$ deq_fire $end
$var wire 32 y$ drain_acc [31:0] $end
$var wire 7 z$ drain_rd_addr [6:0] $end
$var wire 16 {$ drain_tag [15:0] $end
$var wire 16 |$ row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 (# tag_clear_busy $end
$var wire 1 i" tag_clear_en $end
$var wire 3 }$ waddr0 [2:0] $end
$var wire 28 ~$ wr_addr_flat [27:0] $end
$var wire 64 !% wr_data_flat [63:0] $end
$var wire 4 "% wr_valid [3:0] $end
$var wire 3 #% wr_cnt [2:0] $end
$var wire 3 $% waddr3 [2:0] $end
$var wire 3 %% waddr2 [2:0] $end
$var wire 3 &% waddr1 [2:0] $end
$var wire 2 '% slot3 [1:0] $end
$var wire 2 (% slot2 [1:0] $end
$var wire 2 )% slot1 [1:0] $end
$var wire 1 n" rmw_busy $end
$var wire 4 *% free_count [3:0] $end
$var wire 1 z" fifo_empty $end
$var parameter 32 +% ACC_W $end
$var parameter 33 ,% BANK_ADDR_W $end
$var parameter 32 -% BANK_DEPTH $end
$var parameter 33 .% BANK_LAST $end
$var parameter 34 /% ENTRY_W $end
$var parameter 32 0% EPOCH_W $end
$var parameter 32 1% FIFO_DEPTH $end
$var parameter 32 2% FIFO_DEPTH_LOG $end
$var parameter 33 3% FIFO_MASK $end
$var parameter 32 4% PROD_W $end
$var parameter 2 5% RMW_ADD $end
$var parameter 2 6% RMW_IDLE $end
$var parameter 2 7% RMW_READ $end
$var parameter 2 8% RMW_WRITE $end
$var reg 1 (# clr_active $end
$var reg 7 9% clr_idx [6:0] $end
$var reg 4 :% fifo_cnt [3:0] $end
$var reg 3 ;% fifo_head [2:0] $end
$var reg 3 <% fifo_tail [2:0] $end
$var reg 7 =% rmw_addr [6:0] $end
$var reg 32 >% rmw_new [31:0] $end
$var reg 32 ?% rmw_old [31:0] $end
$var reg 16 @% rmw_old_tag [15:0] $end
$var reg 16 A% rmw_prod [15:0] $end
$var reg 2 B% rmw_st [1:0] $end
$var integer 32 C% ci [31:0] $end
$var integer 32 D% fi [31:0] $end
$upscope $end
$upscope $end
$scope module u_task_fifo $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 260 E% rd_data [259:0] $end
$var wire 1 V rd_en $end
$var wire 1 _ wr_full $end
$var wire 1 Z wr_en $end
$var wire 260 F% wr_data [259:0] $end
$var wire 1 ` rd_empty $end
$var wire 9 G% count [8:0] $end
$var parameter 32 H% DEPTH $end
$var parameter 32 I% DEPTH_LOG $end
$var parameter 32 J% WIDTH $end
$var reg 9 K% rd_ptr [8:0] $end
$var reg 9 L% wr_ptr [8:0] $end
$upscope $end
$scope module u_task_packer $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 1 M% flush_active $end
$var wire 1 \ flush_pack $end
$var wire 1 _ group_fifo_full $end
$var wire 64 N% task_in_data [63:0] $end
$var wire 1 X task_in_ready $end
$var wire 1 W task_in_valid $end
$var reg 1 ] flush_done $end
$var reg 260 O% group_wr_data [259:0] $end
$var reg 1 Z group_wr_en $end
$var reg 2 P% pack_count [1:0] $end
$var reg 64 Q% pack_task0 [63:0] $end
$var reg 64 R% pack_task1 [63:0] $end
$var reg 64 S% pack_task2 [63:0] $end
$var reg 64 T% pack_task3 [63:0] $end
$var reg 4 U% pack_valid [3:0] $end
$upscope $end
$upscope $end
$upscope $end
$enddefinitions $end
$comment Show the parameter values. $end
$dumpall
b100000100 J%
b1000 I%
b100000000 H%
b11 8%
b1 7%
b0 6%
b10 5%
b10000 4%
b111 3%
b11 2%
b1000 1%
b10000 0%
b10111 /%
b1111111 .%
b10000000 -%
b111 ,%
b100000 +%
b11 k$
b1 j$
b0 i$
b10 h$
b10000 g$
b111 f$
b11 e$
b1000 d$
b10000 c$
b10111 b$
b1111111 a$
b10000000 `$
b111 _$
b100000 ^$
b11 @$
b1 ?$
b0 >$
b10 =$
b10000 <$
b111 ;$
b11 :$
b1000 9$
b10000 8$
b10111 7$
b1111111 6$
b10000000 5$
b111 4$
b100000 3$
b11 s#
b1 r#
b0 q#
b10 p#
b10000 o#
b111 n#
b11 m#
b1000 l#
b10000 k#
b10111 j#
b1111111 i#
b10000000 h#
b111 g#
b100000 f#
b10 D#
b0 C#
b11 B#
b100 A#
b101 @#
b1 ?#
b10000 >#
b10000 =#
b1000000000 <#
b10000 ;#
b1001 :#
b1111111 9#
b11 8#
b1000 7#
b10000000 6#
b111 5#
b100000 4#
b10000100 Z"
b1000 Y"
b100000000 X"
b11 S"
b10 L"
b1 E"
b0 >"
b1001 ""
b111 !"
b1000 ~
b101 }
b1010 |
b1 {
b100 z
b11 y
b0 x
b0 w
b110 v
b1011 u
b10 t
$end
#0
$dumpvars
bx U%
bx T%
bx S%
bx R%
bx Q%
bx P%
bx O%
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx N%
xM%
bx L%
bx K%
bx G%
bx F%
bx E%
bx D%
bx C%
bx B%
bx A%
bx @%
bx ?%
bx >%
bx =%
bx <%
bx ;%
bx :%
bx 9%
bx *%
b0x )%
bx (%
bx '%
bx &%
bx %%
bx $%
bx #%
bx "%
bx !%
bx ~$
bx }$
bx |$
bx {$
bx z$
bx y$
xx$
bx w$
bx v$
bx u$
bx t$
bx s$
bx r$
bx q$
bx p$
bx o$
bx n$
bx m$
bx l$
bx ]$
b0x \$
bx [$
bx Z$
bx Y$
bx X$
bx W$
bx V$
bx U$
bx T$
bx S$
bx R$
bx Q$
bx P$
bx O$
bx N$
xM$
bx L$
bx K$
bx J$
bx I$
bx H$
bx G$
bx F$
bx E$
bx D$
bx C$
bx B$
bx A$
bx 2$
b0x 1$
bx 0$
bx /$
bx .$
bx -$
bx ,$
bx +$
bx *$
bx )$
bx ($
bx '$
bx &$
bx %$
bx $$
bx #$
x"$
bx !$
bx ~#
bx }#
bx |#
bx {#
bx z#
bx y#
bx x#
bx w#
bx v#
bx u#
bx t#
bx e#
b0x d#
bx c#
bx b#
bx a#
bx `#
bx _#
bx ^#
bx ]#
bx \#
bx [#
bx Z#
bx Y#
bx X#
bx W#
bx V#
xU#
bx T#
bx S#
bx R#
bx Q#
xP#
bx O#
bx N#
bx M#
bx L#
xK#
bx J#
xI#
bx H#
xG#
xF#
bx E#
bx 3#
bx 2#
bx 1#
bx 0#
bx /#
bx .#
bx -#
bx ,#
x+#
x*#
x)#
x(#
bx '#
bx &#
bx %#
bx $#
bx ##
bx "#
bx !#
bx ~"
x}"
x|"
x{"
xz"
bx y"
bx x"
bx w"
bx v"
bx u"
bx t"
bx s"
bx r"
xq"
xp"
xo"
xn"
bx m"
bx l"
bx k"
bx j"
xi"
bx h"
xg"
bx f"
bx e"
bx d"
xc"
bx b"
xa"
x`"
x_"
x^"
x]"
bx \"
bx ["
bx W"
bx V"
bx U"
bx T"
bx R"
bx Q"
bx P"
bx O"
bx N"
bx M"
bx K"
bx J"
bx I"
bx H"
bx G"
bx F"
bx D"
bx C"
bx B"
bx A"
bx @"
bx ?"
bx ="
bx <"
bx ;"
bx :"
bx 9"
bx 8"
bx 7"
bx 6"
bx 5"
bx 4"
x3"
bx 2"
bx 1"
bx 0"
bx /"
bx ."
bx -"
bx ,"
bx +"
bx *"
bx )"
bx ("
bx '"
bx &"
bx %"
bx $"
bx #"
xs
bx r
bx q
bx p
bx o
bx n
xm
bx l
xk
xj
bx i
bx h
bx g
bx f
bx e
xd
bx c
xb
bx a
x`
x_
bx ^
x]
x\
bx [
xZ
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx Y
xX
xW
xV
xU
bx T
xS
xR
bx Q
bx P
bx O
bx N
bx M
bx L
bx K
xJ
xI
xH
bx G
bx F
bx E
bx D
bx C
bx B
bx A
bx @
bx ?
x>
bx =
x<
x;
bx :
bx 9
x8
bx 7
bx 6
x5
bx 4
bx 3
x2
x1
x0
bx /
bx .
x-
bx ,
bx +
x*
bx )
bx (
bx '
bx &
bx %
bx $
bx #
x"
x!
$end
#10000000000
