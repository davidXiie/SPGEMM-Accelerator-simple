$date
	Tue Jun 23 21:09:22 2026
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
$var wire 8 D a_desc_waddr [7:0] $end
$var wire 64 E a_desc_wdata [63:0] $end
$var wire 1 - a_desc_we $end
$var wire 14 F a_val_waddr [13:0] $end
$var wire 16 G a_val_wdata [15:0] $end
$var wire 1 0 a_val_we $end
$var wire 1 H acc_inp_done $end
$var wire 1 I acc_inp_done_0 $end
$var wire 1 J acc_inp_done_1 $end
$var wire 1 K acc_out_ready_0 $end
$var wire 1 L acc_out_ready_1 $end
$var wire 1 M acc_row_start_0 $end
$var wire 1 N acc_row_start_1 $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 1 O b_batch_done $end
$var wire 17 P b_col_waddr [16:0] $end
$var wire 16 Q b_col_wdata [15:0] $end
$var wire 1 5 b_col_we $end
$var wire 9 R b_desc_waddr [8:0] $end
$var wire 64 S b_desc_wdata [63:0] $end
$var wire 1 8 b_desc_we $end
$var wire 1 T b_last_group_fires $end
$var wire 17 U b_val_waddr [16:0] $end
$var wire 16 V b_val_wdata [15:0] $end
$var wire 1 ; b_val_we $end
$var wire 16 W bc0 [15:0] $end
$var wire 16 X bc1 [15:0] $end
$var wire 16 Y bc2 [15:0] $end
$var wire 16 Z bc3 [15:0] $end
$var wire 16 [ bv0 [15:0] $end
$var wire 16 \ bv1 [15:0] $end
$var wire 16 ] bv2 [15:0] $end
$var wire 16 ^ bv3 [15:0] $end
$var wire 1 < cbuf_wr_ready $end
$var wire 1 " cbuf_wr_valid $end
$var wire 1 _ issue_valid_0 $end
$var wire 1 ` issue_valid_1 $end
$var wire 1 a mac_pipeline_idle $end
$var wire 1 b prod_fifo_rd_en $end
$var wire 1 c product_group_wr_en $end
$var wire 16 d row_count [15:0] $end
$var wire 1 > start $end
$var wire 1 e stream_group_valid $end
$var wire 1 f task_fifo_rd_en $end
$var wire 1 g task_group_wr_en $end
$var wire 260 h task_group_wr_data [259:0] $end
$var wire 260 i task_fifo_rd_data [259:0] $end
$var wire 1 j task_fifo_full $end
$var wire 1 k task_fifo_empty $end
$var wire 4 l stream_lane_valid [3:0] $end
$var wire 64 m sg3 [63:0] $end
$var wire 64 n sg2 [63:0] $end
$var wire 64 o sg1 [63:0] $end
$var wire 64 p sg0 [63:0] $end
$var wire 132 q product_group_wr_data [131:0] $end
$var wire 1 r product_fifo_full $end
$var wire 9 s product_fifo_cnt [8:0] $end
$var wire 132 t prod_fifo_rd_data [131:0] $end
$var wire 1 u prod_fifo_empty $end
$var wire 1 v other_acc_busy $end
$var wire 4 w mul_valid [3:0] $end
$var wire 128 x mul_product [127:0] $end
$var wire 4 y mac_lane_valid [3:0] $end
$var wire 256 z mac_lane_task [255:0] $end
$var wire 32 { drain_out_value [31:0] $end
$var wire 1 | drain_out_valid $end
$var wire 16 } drain_out_row_id [15:0] $end
$var wire 9 ~ drain_out_col_id [8:0] $end
$var wire 16 !" cbuf_wr_data [15:0] $end
$var wire 18 "" cbuf_wr_addr [17:0] $end
$var wire 15 #" b_group [14:0] $end
$var wire 1 $" acc_row_done_1 $end
$var wire 1 %" acc_row_done_0 $end
$var wire 32 &" acc_out_value_1 [31:0] $end
$var wire 32 '" acc_out_value_0 [31:0] $end
$var wire 1 (" acc_out_valid_1 $end
$var wire 1 )" acc_out_valid_0 $end
$var wire 16 *" acc_out_row_id_1 [15:0] $end
$var wire 16 +" acc_out_row_id_0 [15:0] $end
$var wire 9 ," acc_out_col_id_1 [8:0] $end
$var wire 9 -" acc_out_col_id_0 [8:0] $end
$var wire 4 ." acc_lane_valid [3:0] $end
$var wire 64 /" acc_lane_product [63:0] $end
$var wire 36 0" acc_lane_col_id [35:0] $end
$var wire 1 1" acc_issue_ready_1 $end
$var wire 1 2" acc_issue_ready_0 $end
$var wire 1 3" acc_issue_ready $end
$var wire 1 4" acc_busy_1 $end
$var wire 1 5" acc_busy_0 $end
$var wire 14 6" a_ptr_next [13:0] $end
$var parameter 32 7" B_BANK_DEPTH $end
$var parameter 4 8" PE_CLEAR_ACC $end
$var parameter 4 9" PE_DONE $end
$var parameter 32 :" PE_ID $end
$var parameter 4 ;" PE_IDLE $end
$var parameter 4 <" PE_LOAD_A_ELEM $end
$var parameter 4 =" PE_LOAD_B_DESC $end
$var parameter 4 >" PE_LOAD_ROW_DESC $end
$var parameter 4 ?" PE_NEXT_ROW $end
$var parameter 4 @" PE_STREAM_B_ROW $end
$var parameter 4 A" PE_WAIT_PRODUCT_DRAIN $end
$var parameter 4 B" PE_WAIT_TASK_DRAIN $end
$var reg 16 C" a_nnz_left [15:0] $end
$var reg 32 D" a_ptr [31:0] $end
$var reg 16 E" b_nnz_left [15:0] $end
$var reg 32 F" b_ptr [31:0] $end
$var reg 64 G" b_row_desc_reg [63:0] $end
$var reg 1 H" comp_sel $end
$var reg 16 I" cur_a_row_nnz [15:0] $end
$var reg 32 J" cur_a_start [31:0] $end
$var reg 16 K" cur_a_val [15:0] $end
$var reg 16 L" cur_global_row [15:0] $end
$var reg 16 M" cur_k [15:0] $end
$var reg 1 ! done $end
$var reg 256 N" mac_lane_task_r [255:0] $end
$var reg 4 O" mac_lane_valid_r [3:0] $end
$var reg 64 P" row_desc_reg [63:0] $end
$var reg 8 Q" row_idx [7:0] $end
$var reg 4 R" state [3:0] $end
$var reg 4 S" state_next [3:0] $end
$var reg 1 T" state_stable $end
$scope module u_mul_array $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 256 U" lane_task [255:0] $end
$var wire 4 V" lane_valid [3:0] $end
$var wire 4 W" mul_valid [3:0] $end
$var wire 128 X" mul_product [127:0] $end
$scope begin gen_lane[0] $end
$var wire 16 Y" mac_a [15:0] $end
$var wire 16 Z" mac_b [15:0] $end
$var wire 16 [" mac_col [15:0] $end
$var wire 32 \" product [31:0] $end
$var wire 16 ]" mul_comb [15:0] $end
$var parameter 2 ^" m $end
$var integer 32 _" s [31:0] $end
$upscope $end
$scope begin gen_lane[1] $end
$var wire 16 `" mac_a [15:0] $end
$var wire 16 a" mac_b [15:0] $end
$var wire 16 b" mac_col [15:0] $end
$var wire 32 c" product [31:0] $end
$var wire 16 d" mul_comb [15:0] $end
$var parameter 2 e" m $end
$var integer 32 f" s [31:0] $end
$upscope $end
$scope begin gen_lane[2] $end
$var wire 16 g" mac_a [15:0] $end
$var wire 16 h" mac_b [15:0] $end
$var wire 16 i" mac_col [15:0] $end
$var wire 32 j" product [31:0] $end
$var wire 16 k" mul_comb [15:0] $end
$var parameter 3 l" m $end
$var integer 32 m" s [31:0] $end
$upscope $end
$scope begin gen_lane[3] $end
$var wire 16 n" mac_a [15:0] $end
$var wire 16 o" mac_b [15:0] $end
$var wire 16 p" mac_col [15:0] $end
$var wire 32 q" product [31:0] $end
$var wire 16 r" mul_comb [15:0] $end
$var parameter 3 s" m $end
$var integer 32 t" s [31:0] $end
$upscope $end
$upscope $end
$scope module u_product_fifo $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 132 u" rd_data [131:0] $end
$var wire 1 b rd_en $end
$var wire 132 v" wr_data [131:0] $end
$var wire 1 c wr_en $end
$var wire 1 r wr_full $end
$var wire 1 u rd_empty $end
$var wire 9 w" count [8:0] $end
$var parameter 32 x" DEPTH $end
$var parameter 32 y" DEPTH_LOG $end
$var parameter 32 z" WIDTH $end
$var reg 9 {" rd_ptr [8:0] $end
$var reg 9 |" wr_ptr [8:0] $end
$upscope $end
$scope module u_row_acc_0 $end
$var wire 1 }" all_clr_done $end
$var wire 1 ~" all_fifos_empty $end
$var wire 1 !# all_rmw_done $end
$var wire 1 1 clk $end
$var wire 1 "# cur_valid_entry $end
$var wire 1 ## do_enqueue $end
$var wire 10 $# drain_cols [9:0] $end
$var wire 7 %# drain_rd_addr [6:0] $end
$var wire 1 &# grp_any $end
$var wire 1 '# grp_v0 $end
$var wire 1 (# grp_v1 $end
$var wire 1 )# grp_v2 $end
$var wire 1 *# grp_v3 $end
$var wire 1 2" issue_ready $end
$var wire 1 _ issue_valid $end
$var wire 36 +# lane_col_id [35:0] $end
$var wire 64 ,# lane_product [63:0] $end
$var wire 4 -# lane_valid [3:0] $end
$var wire 1 .# nv1 $end
$var wire 1 /# nv2 $end
$var wire 1 0# nv3 $end
$var wire 1 1# nv_has_more $end
$var wire 1 K out_ready $end
$var wire 16 2# row_id_in [15:0] $end
$var wire 1 I row_input_done $end
$var wire 1 M row_start $end
$var wire 1 2 rst_n $end
$var wire 1 3# tag_clear_pulse $end
$var wire 64 4# wr_data_flat [63:0] $end
$var wire 28 5# wr_addr_flat [27:0] $end
$var wire 16 6# sel_tag [15:0] $end
$var wire 32 7# sel_acc [31:0] $end
$var wire 1 8# rmw_b3 $end
$var wire 1 9# rmw_b2 $end
$var wire 1 :# rmw_b1 $end
$var wire 1 ;# rmw_b0 $end
$var wire 2 <# nv_next [1:0] $end
$var wire 3 =# mc3 [2:0] $end
$var wire 3 ># mc2 [2:0] $end
$var wire 3 ?# mc1 [2:0] $end
$var wire 3 @# mc0 [2:0] $end
$var wire 7 A# last_group_addr [6:0] $end
$var wire 1 B# last_group $end
$var wire 1 C# last_cell $end
$var wire 6 D# free_b3 [5:0] $end
$var wire 6 E# free_b2 [5:0] $end
$var wire 6 F# free_b1 [5:0] $end
$var wire 6 G# free_b0 [5:0] $end
$var wire 1 H# emp_b3 $end
$var wire 1 I# emp_b2 $end
$var wire 1 J# emp_b1 $end
$var wire 1 K# emp_b0 $end
$var wire 16 L# dtag_b3 [15:0] $end
$var wire 16 M# dtag_b2 [15:0] $end
$var wire 16 N# dtag_b1 [15:0] $end
$var wire 16 O# dtag_b0 [15:0] $end
$var wire 10 P# drain_cols_m1 [9:0] $end
$var wire 32 Q# dacc_b3 [31:0] $end
$var wire 32 R# dacc_b2 [31:0] $end
$var wire 32 S# dacc_b1 [31:0] $end
$var wire 32 T# dacc_b0 [31:0] $end
$var wire 9 U# cur_col_id [8:0] $end
$var wire 1 V# clr_b3 $end
$var wire 1 W# clr_b2 $end
$var wire 1 X# clr_b1 $end
$var wire 1 Y# clr_b0 $end
$var wire 4 Z# bwv3 [3:0] $end
$var wire 4 [# bwv2 [3:0] $end
$var wire 4 \# bwv1 [3:0] $end
$var wire 4 ]# bwv0 [3:0] $end
$var wire 2 ^# bid3 [1:0] $end
$var wire 2 _# bid2 [1:0] $end
$var wire 2 `# bid1 [1:0] $end
$var wire 2 a# bid0 [1:0] $end
$var parameter 32 b# ACC_W $end
$var parameter 33 c# BANK_ADDR_W $end
$var parameter 32 d# BANK_DEPTH $end
$var parameter 32 e# BANK_FIFO_DEPTH $end
$var parameter 32 f# BANK_FIFO_LOG $end
$var parameter 33 g# BANK_LAST $end
$var parameter 32 h# COL_W $end
$var parameter 32 i# EPOCH_W $end
$var parameter 32 j# OUT_COLS $end
$var parameter 32 k# PROD_W $end
$var parameter 32 l# ROW_W $end
$var parameter 3 m# S_ACCUM $end
$var parameter 3 n# S_CLEAR_TAGS $end
$var parameter 3 o# S_DONE $end
$var parameter 3 p# S_DRAIN $end
$var parameter 3 q# S_IDLE $end
$var parameter 3 r# S_WAIT_DRAIN $end
$var reg 2 s# bank_sel [1:0] $end
$var reg 1 5" busy $end
$var reg 1 t# clr_triggered $end
$var reg 16 u# cur_row_id [15:0] $end
$var reg 1 v# drain_emit $end
$var reg 7 w# group_addr [6:0] $end
$var reg 1 x# input_done_latch $end
$var reg 9 y# out_col_id [8:0] $end
$var reg 16 z# out_row_id [15:0] $end
$var reg 1 )" out_valid $end
$var reg 32 {# out_value [31:0] $end
$var reg 1 %" row_done $end
$var reg 16 |# row_epoch [15:0] $end
$var reg 3 }# state [2:0] $end
$scope module u_bank0 $end
$var wire 1 1 clk $end
$var wire 32 ~# drain_acc [31:0] $end
$var wire 7 !$ drain_rd_addr [6:0] $end
$var wire 16 "$ drain_tag [15:0] $end
$var wire 1 ;# rmw_busy $end
$var wire 16 #$ row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 $$ s12_hazard $end
$var wire 1 Y# tag_clear_busy $end
$var wire 1 3# tag_clear_en $end
$var wire 5 %$ waddr0 [4:0] $end
$var wire 28 &$ wr_addr_flat [27:0] $end
$var wire 64 '$ wr_data_flat [63:0] $end
$var wire 4 ($ wr_valid [3:0] $end
$var wire 3 )$ wr_cnt [2:0] $end
$var wire 5 *$ waddr3 [4:0] $end
$var wire 5 +$ waddr2 [4:0] $end
$var wire 5 ,$ waddr1 [4:0] $end
$var wire 2 -$ slot3 [1:0] $end
$var wire 2 .$ slot2 [1:0] $end
$var wire 2 /$ slot1 [1:0] $end
$var wire 32 0$ s1_old_acc [31:0] $end
$var wire 32 1$ s1_new_val [31:0] $end
$var wire 1 2$ s1_epoch_hit $end
$var wire 6 3$ free_count [5:0] $end
$var wire 1 K# fifo_empty $end
$var wire 1 4$ deq_fire $end
$var parameter 32 5$ ACC_W $end
$var parameter 33 6$ BANK_ADDR_W $end
$var parameter 32 7$ BANK_DEPTH $end
$var parameter 33 8$ BANK_LAST $end
$var parameter 34 9$ ENTRY_W $end
$var parameter 32 :$ EPOCH_W $end
$var parameter 32 ;$ FIFO_DEPTH $end
$var parameter 32 <$ FIFO_DEPTH_LOG $end
$var parameter 33 =$ FIFO_MASK $end
$var parameter 32 >$ PROD_W $end
$var reg 1 Y# clr_active $end
$var reg 7 ?$ clr_idx [6:0] $end
$var reg 6 @$ fifo_cnt [5:0] $end
$var reg 5 A$ fifo_head [4:0] $end
$var reg 5 B$ fifo_tail [4:0] $end
$var reg 7 C$ s1_addr [6:0] $end
$var reg 16 D$ s1_prod [15:0] $end
$var reg 1 E$ s1_valid $end
$var reg 7 F$ s2_addr [6:0] $end
$var reg 32 G$ s2_new_val [31:0] $end
$var reg 1 H$ s2_valid $end
$upscope $end
$scope module u_bank1 $end
$var wire 1 1 clk $end
$var wire 32 I$ drain_acc [31:0] $end
$var wire 7 J$ drain_rd_addr [6:0] $end
$var wire 16 K$ drain_tag [15:0] $end
$var wire 1 :# rmw_busy $end
$var wire 16 L$ row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 M$ s12_hazard $end
$var wire 1 X# tag_clear_busy $end
$var wire 1 3# tag_clear_en $end
$var wire 5 N$ waddr0 [4:0] $end
$var wire 28 O$ wr_addr_flat [27:0] $end
$var wire 64 P$ wr_data_flat [63:0] $end
$var wire 4 Q$ wr_valid [3:0] $end
$var wire 3 R$ wr_cnt [2:0] $end
$var wire 5 S$ waddr3 [4:0] $end
$var wire 5 T$ waddr2 [4:0] $end
$var wire 5 U$ waddr1 [4:0] $end
$var wire 2 V$ slot3 [1:0] $end
$var wire 2 W$ slot2 [1:0] $end
$var wire 2 X$ slot1 [1:0] $end
$var wire 32 Y$ s1_old_acc [31:0] $end
$var wire 32 Z$ s1_new_val [31:0] $end
$var wire 1 [$ s1_epoch_hit $end
$var wire 6 \$ free_count [5:0] $end
$var wire 1 J# fifo_empty $end
$var wire 1 ]$ deq_fire $end
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
$var reg 1 X# clr_active $end
$var reg 7 h$ clr_idx [6:0] $end
$var reg 6 i$ fifo_cnt [5:0] $end
$var reg 5 j$ fifo_head [4:0] $end
$var reg 5 k$ fifo_tail [4:0] $end
$var reg 7 l$ s1_addr [6:0] $end
$var reg 16 m$ s1_prod [15:0] $end
$var reg 1 n$ s1_valid $end
$var reg 7 o$ s2_addr [6:0] $end
$var reg 32 p$ s2_new_val [31:0] $end
$var reg 1 q$ s2_valid $end
$upscope $end
$scope module u_bank2 $end
$var wire 1 1 clk $end
$var wire 32 r$ drain_acc [31:0] $end
$var wire 7 s$ drain_rd_addr [6:0] $end
$var wire 16 t$ drain_tag [15:0] $end
$var wire 1 9# rmw_busy $end
$var wire 16 u$ row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 v$ s12_hazard $end
$var wire 1 W# tag_clear_busy $end
$var wire 1 3# tag_clear_en $end
$var wire 5 w$ waddr0 [4:0] $end
$var wire 28 x$ wr_addr_flat [27:0] $end
$var wire 64 y$ wr_data_flat [63:0] $end
$var wire 4 z$ wr_valid [3:0] $end
$var wire 3 {$ wr_cnt [2:0] $end
$var wire 5 |$ waddr3 [4:0] $end
$var wire 5 }$ waddr2 [4:0] $end
$var wire 5 ~$ waddr1 [4:0] $end
$var wire 2 !% slot3 [1:0] $end
$var wire 2 "% slot2 [1:0] $end
$var wire 2 #% slot1 [1:0] $end
$var wire 32 $% s1_old_acc [31:0] $end
$var wire 32 %% s1_new_val [31:0] $end
$var wire 1 &% s1_epoch_hit $end
$var wire 6 '% free_count [5:0] $end
$var wire 1 I# fifo_empty $end
$var wire 1 (% deq_fire $end
$var parameter 32 )% ACC_W $end
$var parameter 33 *% BANK_ADDR_W $end
$var parameter 32 +% BANK_DEPTH $end
$var parameter 33 ,% BANK_LAST $end
$var parameter 34 -% ENTRY_W $end
$var parameter 32 .% EPOCH_W $end
$var parameter 32 /% FIFO_DEPTH $end
$var parameter 32 0% FIFO_DEPTH_LOG $end
$var parameter 33 1% FIFO_MASK $end
$var parameter 32 2% PROD_W $end
$var reg 1 W# clr_active $end
$var reg 7 3% clr_idx [6:0] $end
$var reg 6 4% fifo_cnt [5:0] $end
$var reg 5 5% fifo_head [4:0] $end
$var reg 5 6% fifo_tail [4:0] $end
$var reg 7 7% s1_addr [6:0] $end
$var reg 16 8% s1_prod [15:0] $end
$var reg 1 9% s1_valid $end
$var reg 7 :% s2_addr [6:0] $end
$var reg 32 ;% s2_new_val [31:0] $end
$var reg 1 <% s2_valid $end
$upscope $end
$scope module u_bank3 $end
$var wire 1 1 clk $end
$var wire 32 =% drain_acc [31:0] $end
$var wire 7 >% drain_rd_addr [6:0] $end
$var wire 16 ?% drain_tag [15:0] $end
$var wire 1 8# rmw_busy $end
$var wire 16 @% row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 A% s12_hazard $end
$var wire 1 V# tag_clear_busy $end
$var wire 1 3# tag_clear_en $end
$var wire 5 B% waddr0 [4:0] $end
$var wire 28 C% wr_addr_flat [27:0] $end
$var wire 64 D% wr_data_flat [63:0] $end
$var wire 4 E% wr_valid [3:0] $end
$var wire 3 F% wr_cnt [2:0] $end
$var wire 5 G% waddr3 [4:0] $end
$var wire 5 H% waddr2 [4:0] $end
$var wire 5 I% waddr1 [4:0] $end
$var wire 2 J% slot3 [1:0] $end
$var wire 2 K% slot2 [1:0] $end
$var wire 2 L% slot1 [1:0] $end
$var wire 32 M% s1_old_acc [31:0] $end
$var wire 32 N% s1_new_val [31:0] $end
$var wire 1 O% s1_epoch_hit $end
$var wire 6 P% free_count [5:0] $end
$var wire 1 H# fifo_empty $end
$var wire 1 Q% deq_fire $end
$var parameter 32 R% ACC_W $end
$var parameter 33 S% BANK_ADDR_W $end
$var parameter 32 T% BANK_DEPTH $end
$var parameter 33 U% BANK_LAST $end
$var parameter 34 V% ENTRY_W $end
$var parameter 32 W% EPOCH_W $end
$var parameter 32 X% FIFO_DEPTH $end
$var parameter 32 Y% FIFO_DEPTH_LOG $end
$var parameter 33 Z% FIFO_MASK $end
$var parameter 32 [% PROD_W $end
$var reg 1 V# clr_active $end
$var reg 7 \% clr_idx [6:0] $end
$var reg 6 ]% fifo_cnt [5:0] $end
$var reg 5 ^% fifo_head [4:0] $end
$var reg 5 _% fifo_tail [4:0] $end
$var reg 7 `% s1_addr [6:0] $end
$var reg 16 a% s1_prod [15:0] $end
$var reg 1 b% s1_valid $end
$var reg 7 c% s2_addr [6:0] $end
$var reg 32 d% s2_new_val [31:0] $end
$var reg 1 e% s2_valid $end
$upscope $end
$upscope $end
$scope module u_row_acc_1 $end
$var wire 1 f% all_clr_done $end
$var wire 1 g% all_fifos_empty $end
$var wire 1 h% all_rmw_done $end
$var wire 1 1 clk $end
$var wire 1 i% cur_valid_entry $end
$var wire 1 j% do_enqueue $end
$var wire 10 k% drain_cols [9:0] $end
$var wire 7 l% drain_rd_addr [6:0] $end
$var wire 1 m% grp_any $end
$var wire 1 n% grp_v0 $end
$var wire 1 o% grp_v1 $end
$var wire 1 p% grp_v2 $end
$var wire 1 q% grp_v3 $end
$var wire 1 1" issue_ready $end
$var wire 1 ` issue_valid $end
$var wire 36 r% lane_col_id [35:0] $end
$var wire 64 s% lane_product [63:0] $end
$var wire 4 t% lane_valid [3:0] $end
$var wire 1 u% nv1 $end
$var wire 1 v% nv2 $end
$var wire 1 w% nv3 $end
$var wire 1 x% nv_has_more $end
$var wire 1 L out_ready $end
$var wire 16 y% row_id_in [15:0] $end
$var wire 1 J row_input_done $end
$var wire 1 N row_start $end
$var wire 1 2 rst_n $end
$var wire 1 z% tag_clear_pulse $end
$var wire 64 {% wr_data_flat [63:0] $end
$var wire 28 |% wr_addr_flat [27:0] $end
$var wire 16 }% sel_tag [15:0] $end
$var wire 32 ~% sel_acc [31:0] $end
$var wire 1 !& rmw_b3 $end
$var wire 1 "& rmw_b2 $end
$var wire 1 #& rmw_b1 $end
$var wire 1 $& rmw_b0 $end
$var wire 2 %& nv_next [1:0] $end
$var wire 3 && mc3 [2:0] $end
$var wire 3 '& mc2 [2:0] $end
$var wire 3 (& mc1 [2:0] $end
$var wire 3 )& mc0 [2:0] $end
$var wire 7 *& last_group_addr [6:0] $end
$var wire 1 +& last_group $end
$var wire 1 ,& last_cell $end
$var wire 6 -& free_b3 [5:0] $end
$var wire 6 .& free_b2 [5:0] $end
$var wire 6 /& free_b1 [5:0] $end
$var wire 6 0& free_b0 [5:0] $end
$var wire 1 1& emp_b3 $end
$var wire 1 2& emp_b2 $end
$var wire 1 3& emp_b1 $end
$var wire 1 4& emp_b0 $end
$var wire 16 5& dtag_b3 [15:0] $end
$var wire 16 6& dtag_b2 [15:0] $end
$var wire 16 7& dtag_b1 [15:0] $end
$var wire 16 8& dtag_b0 [15:0] $end
$var wire 10 9& drain_cols_m1 [9:0] $end
$var wire 32 :& dacc_b3 [31:0] $end
$var wire 32 ;& dacc_b2 [31:0] $end
$var wire 32 <& dacc_b1 [31:0] $end
$var wire 32 =& dacc_b0 [31:0] $end
$var wire 9 >& cur_col_id [8:0] $end
$var wire 1 ?& clr_b3 $end
$var wire 1 @& clr_b2 $end
$var wire 1 A& clr_b1 $end
$var wire 1 B& clr_b0 $end
$var wire 4 C& bwv3 [3:0] $end
$var wire 4 D& bwv2 [3:0] $end
$var wire 4 E& bwv1 [3:0] $end
$var wire 4 F& bwv0 [3:0] $end
$var wire 2 G& bid3 [1:0] $end
$var wire 2 H& bid2 [1:0] $end
$var wire 2 I& bid1 [1:0] $end
$var wire 2 J& bid0 [1:0] $end
$var parameter 32 K& ACC_W $end
$var parameter 33 L& BANK_ADDR_W $end
$var parameter 32 M& BANK_DEPTH $end
$var parameter 32 N& BANK_FIFO_DEPTH $end
$var parameter 32 O& BANK_FIFO_LOG $end
$var parameter 33 P& BANK_LAST $end
$var parameter 32 Q& COL_W $end
$var parameter 32 R& EPOCH_W $end
$var parameter 32 S& OUT_COLS $end
$var parameter 32 T& PROD_W $end
$var parameter 32 U& ROW_W $end
$var parameter 3 V& S_ACCUM $end
$var parameter 3 W& S_CLEAR_TAGS $end
$var parameter 3 X& S_DONE $end
$var parameter 3 Y& S_DRAIN $end
$var parameter 3 Z& S_IDLE $end
$var parameter 3 [& S_WAIT_DRAIN $end
$var reg 2 \& bank_sel [1:0] $end
$var reg 1 4" busy $end
$var reg 1 ]& clr_triggered $end
$var reg 16 ^& cur_row_id [15:0] $end
$var reg 1 _& drain_emit $end
$var reg 7 `& group_addr [6:0] $end
$var reg 1 a& input_done_latch $end
$var reg 9 b& out_col_id [8:0] $end
$var reg 16 c& out_row_id [15:0] $end
$var reg 1 (" out_valid $end
$var reg 32 d& out_value [31:0] $end
$var reg 1 $" row_done $end
$var reg 16 e& row_epoch [15:0] $end
$var reg 3 f& state [2:0] $end
$scope module u_bank0 $end
$var wire 1 1 clk $end
$var wire 32 g& drain_acc [31:0] $end
$var wire 7 h& drain_rd_addr [6:0] $end
$var wire 16 i& drain_tag [15:0] $end
$var wire 1 $& rmw_busy $end
$var wire 16 j& row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 k& s12_hazard $end
$var wire 1 B& tag_clear_busy $end
$var wire 1 z% tag_clear_en $end
$var wire 5 l& waddr0 [4:0] $end
$var wire 28 m& wr_addr_flat [27:0] $end
$var wire 64 n& wr_data_flat [63:0] $end
$var wire 4 o& wr_valid [3:0] $end
$var wire 3 p& wr_cnt [2:0] $end
$var wire 5 q& waddr3 [4:0] $end
$var wire 5 r& waddr2 [4:0] $end
$var wire 5 s& waddr1 [4:0] $end
$var wire 2 t& slot3 [1:0] $end
$var wire 2 u& slot2 [1:0] $end
$var wire 2 v& slot1 [1:0] $end
$var wire 32 w& s1_old_acc [31:0] $end
$var wire 32 x& s1_new_val [31:0] $end
$var wire 1 y& s1_epoch_hit $end
$var wire 6 z& free_count [5:0] $end
$var wire 1 4& fifo_empty $end
$var wire 1 {& deq_fire $end
$var parameter 32 |& ACC_W $end
$var parameter 33 }& BANK_ADDR_W $end
$var parameter 32 ~& BANK_DEPTH $end
$var parameter 33 !' BANK_LAST $end
$var parameter 34 "' ENTRY_W $end
$var parameter 32 #' EPOCH_W $end
$var parameter 32 $' FIFO_DEPTH $end
$var parameter 32 %' FIFO_DEPTH_LOG $end
$var parameter 33 &' FIFO_MASK $end
$var parameter 32 '' PROD_W $end
$var reg 1 B& clr_active $end
$var reg 7 (' clr_idx [6:0] $end
$var reg 6 )' fifo_cnt [5:0] $end
$var reg 5 *' fifo_head [4:0] $end
$var reg 5 +' fifo_tail [4:0] $end
$var reg 7 ,' s1_addr [6:0] $end
$var reg 16 -' s1_prod [15:0] $end
$var reg 1 .' s1_valid $end
$var reg 7 /' s2_addr [6:0] $end
$var reg 32 0' s2_new_val [31:0] $end
$var reg 1 1' s2_valid $end
$upscope $end
$scope module u_bank1 $end
$var wire 1 1 clk $end
$var wire 32 2' drain_acc [31:0] $end
$var wire 7 3' drain_rd_addr [6:0] $end
$var wire 16 4' drain_tag [15:0] $end
$var wire 1 #& rmw_busy $end
$var wire 16 5' row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 6' s12_hazard $end
$var wire 1 A& tag_clear_busy $end
$var wire 1 z% tag_clear_en $end
$var wire 5 7' waddr0 [4:0] $end
$var wire 28 8' wr_addr_flat [27:0] $end
$var wire 64 9' wr_data_flat [63:0] $end
$var wire 4 :' wr_valid [3:0] $end
$var wire 3 ;' wr_cnt [2:0] $end
$var wire 5 <' waddr3 [4:0] $end
$var wire 5 =' waddr2 [4:0] $end
$var wire 5 >' waddr1 [4:0] $end
$var wire 2 ?' slot3 [1:0] $end
$var wire 2 @' slot2 [1:0] $end
$var wire 2 A' slot1 [1:0] $end
$var wire 32 B' s1_old_acc [31:0] $end
$var wire 32 C' s1_new_val [31:0] $end
$var wire 1 D' s1_epoch_hit $end
$var wire 6 E' free_count [5:0] $end
$var wire 1 3& fifo_empty $end
$var wire 1 F' deq_fire $end
$var parameter 32 G' ACC_W $end
$var parameter 33 H' BANK_ADDR_W $end
$var parameter 32 I' BANK_DEPTH $end
$var parameter 33 J' BANK_LAST $end
$var parameter 34 K' ENTRY_W $end
$var parameter 32 L' EPOCH_W $end
$var parameter 32 M' FIFO_DEPTH $end
$var parameter 32 N' FIFO_DEPTH_LOG $end
$var parameter 33 O' FIFO_MASK $end
$var parameter 32 P' PROD_W $end
$var reg 1 A& clr_active $end
$var reg 7 Q' clr_idx [6:0] $end
$var reg 6 R' fifo_cnt [5:0] $end
$var reg 5 S' fifo_head [4:0] $end
$var reg 5 T' fifo_tail [4:0] $end
$var reg 7 U' s1_addr [6:0] $end
$var reg 16 V' s1_prod [15:0] $end
$var reg 1 W' s1_valid $end
$var reg 7 X' s2_addr [6:0] $end
$var reg 32 Y' s2_new_val [31:0] $end
$var reg 1 Z' s2_valid $end
$upscope $end
$scope module u_bank2 $end
$var wire 1 1 clk $end
$var wire 32 [' drain_acc [31:0] $end
$var wire 7 \' drain_rd_addr [6:0] $end
$var wire 16 ]' drain_tag [15:0] $end
$var wire 1 "& rmw_busy $end
$var wire 16 ^' row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 _' s12_hazard $end
$var wire 1 @& tag_clear_busy $end
$var wire 1 z% tag_clear_en $end
$var wire 5 `' waddr0 [4:0] $end
$var wire 28 a' wr_addr_flat [27:0] $end
$var wire 64 b' wr_data_flat [63:0] $end
$var wire 4 c' wr_valid [3:0] $end
$var wire 3 d' wr_cnt [2:0] $end
$var wire 5 e' waddr3 [4:0] $end
$var wire 5 f' waddr2 [4:0] $end
$var wire 5 g' waddr1 [4:0] $end
$var wire 2 h' slot3 [1:0] $end
$var wire 2 i' slot2 [1:0] $end
$var wire 2 j' slot1 [1:0] $end
$var wire 32 k' s1_old_acc [31:0] $end
$var wire 32 l' s1_new_val [31:0] $end
$var wire 1 m' s1_epoch_hit $end
$var wire 6 n' free_count [5:0] $end
$var wire 1 2& fifo_empty $end
$var wire 1 o' deq_fire $end
$var parameter 32 p' ACC_W $end
$var parameter 33 q' BANK_ADDR_W $end
$var parameter 32 r' BANK_DEPTH $end
$var parameter 33 s' BANK_LAST $end
$var parameter 34 t' ENTRY_W $end
$var parameter 32 u' EPOCH_W $end
$var parameter 32 v' FIFO_DEPTH $end
$var parameter 32 w' FIFO_DEPTH_LOG $end
$var parameter 33 x' FIFO_MASK $end
$var parameter 32 y' PROD_W $end
$var reg 1 @& clr_active $end
$var reg 7 z' clr_idx [6:0] $end
$var reg 6 {' fifo_cnt [5:0] $end
$var reg 5 |' fifo_head [4:0] $end
$var reg 5 }' fifo_tail [4:0] $end
$var reg 7 ~' s1_addr [6:0] $end
$var reg 16 !( s1_prod [15:0] $end
$var reg 1 "( s1_valid $end
$var reg 7 #( s2_addr [6:0] $end
$var reg 32 $( s2_new_val [31:0] $end
$var reg 1 %( s2_valid $end
$upscope $end
$scope module u_bank3 $end
$var wire 1 1 clk $end
$var wire 32 &( drain_acc [31:0] $end
$var wire 7 '( drain_rd_addr [6:0] $end
$var wire 16 (( drain_tag [15:0] $end
$var wire 1 !& rmw_busy $end
$var wire 16 )( row_epoch [15:0] $end
$var wire 1 2 rst_n $end
$var wire 1 *( s12_hazard $end
$var wire 1 ?& tag_clear_busy $end
$var wire 1 z% tag_clear_en $end
$var wire 5 +( waddr0 [4:0] $end
$var wire 28 ,( wr_addr_flat [27:0] $end
$var wire 64 -( wr_data_flat [63:0] $end
$var wire 4 .( wr_valid [3:0] $end
$var wire 3 /( wr_cnt [2:0] $end
$var wire 5 0( waddr3 [4:0] $end
$var wire 5 1( waddr2 [4:0] $end
$var wire 5 2( waddr1 [4:0] $end
$var wire 2 3( slot3 [1:0] $end
$var wire 2 4( slot2 [1:0] $end
$var wire 2 5( slot1 [1:0] $end
$var wire 32 6( s1_old_acc [31:0] $end
$var wire 32 7( s1_new_val [31:0] $end
$var wire 1 8( s1_epoch_hit $end
$var wire 6 9( free_count [5:0] $end
$var wire 1 1& fifo_empty $end
$var wire 1 :( deq_fire $end
$var parameter 32 ;( ACC_W $end
$var parameter 33 <( BANK_ADDR_W $end
$var parameter 32 =( BANK_DEPTH $end
$var parameter 33 >( BANK_LAST $end
$var parameter 34 ?( ENTRY_W $end
$var parameter 32 @( EPOCH_W $end
$var parameter 32 A( FIFO_DEPTH $end
$var parameter 32 B( FIFO_DEPTH_LOG $end
$var parameter 33 C( FIFO_MASK $end
$var parameter 32 D( PROD_W $end
$var reg 1 ?& clr_active $end
$var reg 7 E( clr_idx [6:0] $end
$var reg 6 F( fifo_cnt [5:0] $end
$var reg 5 G( fifo_head [4:0] $end
$var reg 5 H( fifo_tail [4:0] $end
$var reg 7 I( s1_addr [6:0] $end
$var reg 16 J( s1_prod [15:0] $end
$var reg 1 K( s1_valid $end
$var reg 7 L( s2_addr [6:0] $end
$var reg 32 M( s2_new_val [31:0] $end
$var reg 1 N( s2_valid $end
$upscope $end
$upscope $end
$scope module u_task_fifo $end
$var wire 1 1 aclk $end
$var wire 1 2 aresetn $end
$var wire 260 O( rd_data [259:0] $end
$var wire 1 f rd_en $end
$var wire 260 P( wr_data [259:0] $end
$var wire 1 g wr_en $end
$var wire 1 j wr_full $end
$var wire 1 k rd_empty $end
$var wire 9 Q( count [8:0] $end
$var parameter 32 R( DEPTH $end
$var parameter 32 S( DEPTH_LOG $end
$var parameter 32 T( WIDTH $end
$var reg 9 U( rd_ptr [8:0] $end
$var reg 9 V( wr_ptr [8:0] $end
$upscope $end
$upscope $end
$upscope $end
$enddefinitions $end
$comment Show the parameter values. $end
$dumpall
b100000100 T(
b1000 S(
b100000000 R(
b10000 D(
b11111 C(
b101 B(
b100000 A(
b10000 @(
b10111 ?(
b1111111 >(
b10000000 =(
b111 <(
b100000 ;(
b10000 y'
b11111 x'
b101 w'
b100000 v'
b10000 u'
b10111 t'
b1111111 s'
b10000000 r'
b111 q'
b100000 p'
b10000 P'
b11111 O'
b101 N'
b100000 M'
b10000 L'
b10111 K'
b1111111 J'
b10000000 I'
b111 H'
b100000 G'
b10000 ''
b11111 &'
b101 %'
b100000 $'
b10000 #'
b10111 "'
b1111111 !'
b10000000 ~&
b111 }&
b100000 |&
b10 [&
b0 Z&
b11 Y&
b100 X&
b101 W&
b1 V&
b10000 U&
b10000 T&
b1000000000 S&
b10000 R&
b1001 Q&
b1111111 P&
b101 O&
b100000 N&
b10000000 M&
b111 L&
b100000 K&
b10000 [%
b11111 Z%
b101 Y%
b100000 X%
b10000 W%
b10111 V%
b1111111 U%
b10000000 T%
b111 S%
b100000 R%
b10000 2%
b11111 1%
b101 0%
b100000 /%
b10000 .%
b10111 -%
b1111111 ,%
b10000000 +%
b111 *%
b100000 )%
b10000 g$
b11111 f$
b101 e$
b100000 d$
b10000 c$
b10111 b$
b1111111 a$
b10000000 `$
b111 _$
b100000 ^$
b10000 >$
b11111 =$
b101 <$
b100000 ;$
b10000 :$
b10111 9$
b1111111 8$
b10000000 7$
b111 6$
b100000 5$
b10 r#
b0 q#
b11 p#
b100 o#
b101 n#
b1 m#
b10000 l#
b10000 k#
b1000000000 j#
b10000 i#
b1001 h#
b1111111 g#
b101 f#
b100000 e#
b10000000 d#
b111 c#
b100000 b#
b10000100 z"
b1000 y"
b100000000 x"
b11 s"
b10 l"
b1 e"
b0 ^"
b110 B"
b111 A"
b101 @"
b1000 ?"
b1 >"
b100 ="
b11 <"
b0 ;"
b0 :"
b1001 9"
b10 8"
b100110100000000 7"
$end
#0
$dumpvars
bx V(
bx U(
bx Q(
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0000000000000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0000000000000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0000000000000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx P(
bx O(
xN(
bx M(
bx L(
xK(
bx J(
bx I(
bx H(
bx G(
bx F(
bx E(
x:(
bx 9(
x8(
bx 7(
bx 6(
b0x 5(
bx 4(
bx 3(
bx 2(
bx 1(
bx 0(
bx /(
bx .(
bx -(
bx ,(
bx +(
x*(
bx )(
bx ((
bx '(
bx &(
x%(
bx $(
bx #(
x"(
bx !(
bx ~'
bx }'
bx |'
bx {'
bx z'
xo'
bx n'
xm'
bx l'
bx k'
b0x j'
bx i'
bx h'
bx g'
bx f'
bx e'
bx d'
bx c'
bx b'
bx a'
bx `'
x_'
bx ^'
bx ]'
bx \'
bx ['
xZ'
bx Y'
bx X'
xW'
bx V'
bx U'
bx T'
bx S'
bx R'
bx Q'
xF'
bx E'
xD'
bx C'
bx B'
b0x A'
bx @'
bx ?'
bx >'
bx ='
bx <'
bx ;'
bx :'
bx 9'
bx 8'
bx 7'
x6'
bx 5'
bx 4'
bx 3'
bx 2'
x1'
bx 0'
bx /'
x.'
bx -'
bx ,'
bx +'
bx *'
bx )'
bx ('
x{&
bx z&
xy&
bx x&
bx w&
b0x v&
bx u&
bx t&
bx s&
bx r&
bx q&
bx p&
bx o&
bx n&
bx m&
bx l&
xk&
bx j&
bx i&
bx h&
bx g&
bx f&
bx e&
bx d&
bx c&
bx b&
xa&
bx `&
x_&
bx ^&
x]&
bx \&
bx J&
bx I&
bx H&
bx G&
bx F&
bx E&
bx D&
bx C&
xB&
xA&
x@&
x?&
bx >&
bx =&
bx <&
bx ;&
bx :&
bx 9&
bx 8&
bx 7&
bx 6&
bx 5&
x4&
x3&
x2&
x1&
bx 0&
bx /&
bx .&
bx -&
x,&
x+&
bx *&
bx )&
bx (&
bx '&
bx &&
bx %&
x$&
x#&
x"&
x!&
bx ~%
bx }%
bx |%
bx {%
xz%
bx y%
xx%
xw%
xv%
xu%
bx t%
bx s%
bx r%
xq%
xp%
xo%
xn%
xm%
bx l%
bx k%
xj%
xi%
xh%
xg%
xf%
xe%
bx d%
bx c%
xb%
bx a%
bx `%
bx _%
bx ^%
bx ]%
bx \%
xQ%
bx P%
xO%
bx N%
bx M%
b0x L%
bx K%
bx J%
bx I%
bx H%
bx G%
bx F%
bx E%
bx D%
bx C%
bx B%
xA%
bx @%
bx ?%
bx >%
bx =%
x<%
bx ;%
bx :%
x9%
bx 8%
bx 7%
bx 6%
bx 5%
bx 4%
bx 3%
x(%
bx '%
x&%
bx %%
bx $%
b0x #%
bx "%
bx !%
bx ~$
bx }$
bx |$
bx {$
bx z$
bx y$
bx x$
bx w$
xv$
bx u$
bx t$
bx s$
bx r$
xq$
bx p$
bx o$
xn$
bx m$
bx l$
bx k$
bx j$
bx i$
bx h$
x]$
bx \$
x[$
bx Z$
bx Y$
b0x X$
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
xH$
bx G$
bx F$
xE$
bx D$
bx C$
bx B$
bx A$
bx @$
bx ?$
x4$
bx 3$
x2$
bx 1$
bx 0$
b0x /$
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
x$$
bx #$
bx "$
bx !$
bx ~#
bx }#
bx |#
bx {#
bx z#
bx y#
xx#
bx w#
xv#
bx u#
xt#
bx s#
bx a#
bx `#
bx _#
bx ^#
bx ]#
bx \#
bx [#
bx Z#
xY#
xX#
xW#
xV#
bx U#
bx T#
bx S#
bx R#
bx Q#
bx P#
bx O#
bx N#
bx M#
bx L#
xK#
xJ#
xI#
xH#
bx G#
bx F#
bx E#
bx D#
xC#
xB#
bx A#
bx @#
bx ?#
bx >#
bx =#
bx <#
x;#
x:#
x9#
x8#
bx 7#
bx 6#
bx 5#
bx 4#
x3#
bx 2#
x1#
x0#
x/#
x.#
bx -#
bx ,#
bx +#
x*#
x)#
x(#
x'#
x&#
bx %#
bx $#
x##
x"#
x!#
x~"
x}"
bx |"
bx {"
bx w"
bx v"
bx u"
bx t"
bx r"
bx q"
bx p"
bx o"
bx n"
bx m"
bx k"
bx j"
bx i"
bx h"
bx g"
bx f"
bx d"
bx c"
bx b"
bx a"
bx `"
bx _"
bx ]"
bx \"
bx ["
bx Z"
bx Y"
bx X"
bx W"
bx V"
bx U"
xT"
bx S"
bx R"
bx Q"
bx P"
bx O"
bx N"
bx M"
bx L"
bx K"
bx J"
bx I"
xH"
bx G"
bx F"
bx E"
bx D"
bx C"
bx 6"
x5"
x4"
x3"
x2"
x1"
bx 0"
bx /"
bx ."
bx -"
bx ,"
bx +"
bx *"
x)"
x("
bx '"
bx &"
x%"
x$"
bx #"
bx ""
bx !"
bx ~
bx }
x|
bx {
bx z
bx y
bx x
bx w
xv
xu
bx t
bx s
xr
bx q
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx p
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx o
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx n
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx m
bx l
xk
xj
bx i
b0xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0000000000000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0000000000000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0000000000000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx h
xg
xf
xe
bx d
xc
xb
xa
x`
x_
bx ^
bx ]
bx \
bx [
bx Z
bx Y
bx X
bx W
bx V
bx U
xT
bx S
bx R
bx Q
bx P
xO
xN
xM
xL
xK
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
