`include "cpu_defs.vh"

module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
reg  reset;
always @(posedge clk) reset <= ~resetn;

wire        valid_fetch;
wire        valid_decode;
wire        valid_execute;
wire        valid_memory;

wire        stall_fetch;
wire        flush_decode;
wire        stall_decode;
wire        flush_execute;
wire        stall_execute;
wire        flush_memory;
wire        stall_memory;
wire        flush_writeback;

wire        bj_taken;
wire [31:0] bj_target;

wire [ 1:0] forward_rs1_select_e;
wire [ 1:0] forward_rs2_select_e;
wire [ 1:0] forward_rs1_select_d;
wire [ 1:0] forward_rs2_select_d;
wire [31:0] forward_result_execute;
wire [31:0] forward_result_memory;
wire [31:0] forward_result_writeback;

wire [`W_TO_RF_BUS_WD-1:0] w_to_rf_bus;
wire [ `F_TO_D_BUS_WD-1:0] f_to_d_bus;
wire [ `D_TO_E_BUS_WD-1:0] d_to_e_bus;
wire [ `E_TO_M_BUS_WD-1:0] e_to_m_bus;
wire [ `M_TO_W_BUS_WD-1:0] m_to_w_bus;
wire [ `D_TO_H_BUS_WD-1:0] d_to_h_bus;
wire [ `E_TO_H_BUS_WD-1:0] e_to_h_bus;
wire [ `M_TO_H_BUS_WD-1:0] m_to_h_bus;
wire [ `W_TO_H_BUS_WD-1:0] w_to_h_bus;

fetch u_fetch(
    .clk                            (clk                            ),
    .reset                          (reset                          ),
    .current_valid_o                (valid_fetch                    ),
    .stall_fetch                    (stall_fetch                    ),
    .bj_taken_i                     (bj_taken                       ),
    .bj_target_i                    (bj_target                      ),
    .f_to_d_bus                     (f_to_d_bus                     ),
    .inst_sram_en                   (inst_sram_en                   ),
    .inst_sram_we                   (inst_sram_we                   ),
    .inst_sram_addr                 (inst_sram_addr                 ),
    .inst_sram_wdata                (inst_sram_wdata                ),
    .inst_sram_rdata                (inst_sram_rdata                )
);

decode u_decode(
    .clk                            (clk                            ),
    .reset                          (reset                          ),
    .previous_valid_i               (valid_fetch                    ),
    .current_valid_o                (valid_decode                   ),
    .stall_decode                   (stall_decode                   ),
    .flush_decode                   (flush_decode                   ),
    .bj_taken_o                     (bj_taken                       ),
    .bj_target_o                    (bj_target                      ),
    .forward_rs1_select_d           (forward_rs1_select_d           ),
    .forward_rs2_select_d           (forward_rs2_select_d           ),
    .forward_result_memory          (forward_result_memory          ),
    .forward_result_writeback       (forward_result_writeback       ),
    .f_to_d_bus                     (f_to_d_bus                     ),
    .d_to_e_bus                     (d_to_e_bus                     ),
    .d_to_h_bus                     (d_to_h_bus                     ),
    .w_to_rf_bus                    (w_to_rf_bus                    )
);

execute u_execute(
    .clk                            (clk                            ),
    .reset                          (reset                          ),
    .previous_valid_i               (valid_decode                   ),
    .current_valid_o                (valid_execute                  ),
    .flush_execute                  (flush_execute                  ),
    .stall_execute                  (stall_execute                  ),
    .forward_rs1_select_e           (forward_rs1_select_e           ),
    .forward_rs2_select_e           (forward_rs2_select_e           ),
    .forward_result_memory          (forward_result_memory          ),
    .forward_result_writeback       (forward_result_writeback       ),
    .d_to_e_bus                     (d_to_e_bus                     ),
    .e_to_m_bus                     (e_to_m_bus                     )
);

memory u_memory(
    .clk                            (clk                            ),
    .reset                          (reset                          ),
    .previous_valid_i               (valid_execute                  ),
    .current_valid_o                (valid_memory                   ),
    .stall_memory                   (stall_memory                   ),
    .flush_memory                   (flush_memory                   ),
    .m_to_h_bus                     (m_to_h_bus                     ),
    .e_to_m_bus                     (e_to_m_bus                     ),
    .m_to_w_bus                     (m_to_w_bus                     ),
    .forward_result_memory          (forward_result_memory          ),
    .data_sram_en                   (data_sram_en                   ),
    .data_sram_we                   (data_sram_we                   ),
    .data_sram_addr                 (data_sram_addr                 ),
    .data_sram_wdata                (data_sram_wdata                ),
    .data_sram_rdata                (data_sram_rdata                )
);

write_back u_write_back(
    .clk                            (clk                            ),
    .reset                          (reset                          ),
    .previous_valid_i               (valid_memory                   ),
    .flush_writeback                (flush_writeback                ),
    .forward_result_writeback       (forward_result_writeback       ),
    .w_to_h_bus                     (w_to_h_bus                     ),
    .m_to_w_bus                     (m_to_w_bus                     ),
    .w_to_rf_bus                    (w_to_rf_bus                    ),
    .debug_wb_pc                    (debug_wb_pc                    ),
    .debug_wb_rf_we                 (debug_wb_rf_we                 ),
    .debug_wb_rf_wnum               (debug_wb_rf_wnum               ),
    .debug_wb_rf_wdata              (debug_wb_rf_wdata              )
);

hazard_unit u_hazard_unit(
    .clk                            (clk                            ),
    .reset                          (reset                          ),
    .d_to_h_bus                     (d_to_h_bus                     ),
    .e_to_h_bus                     (e_to_h_bus                     ),
    .m_to_h_bus                     (m_to_h_bus                     ),
    .w_to_h_bus                     (w_to_h_bus                     ),
    .stall_fetch                    (stall_fetch                    ),
    .stall_decode                   (stall_decode                   ),
    .flush_decode                   (flush_decode                   ),
    .forward_rs1_select_d           (forward_rs1_select_d           ),
    .forward_rs2_select_d           (forward_rs2_select_d           ),
    .flush_execute                  (flush_execute                  ),
    .stall_execute                  (stall_execute                  ),
    .forward_rs1_select_e           (forward_rs1_select_e           ),
    .forward_rs2_select_e           (forward_rs2_select_e           ),
    .flush_memory                   (flush_memory                   ),
    .stall_memory                   (stall_memory                   ),
    .flush_writeback                (flush_writeback                )
);

endmodule