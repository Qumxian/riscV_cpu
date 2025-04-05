`include "cpu_defs.vh"

module hazard_unit (
    input  wire                      clk,
    input  wire                      reset,
    input  wire [`D_TO_H_BUS_WD-1:0] d_to_h_bus,
    input  wire [`E_TO_H_BUS_WD-1:0] e_to_h_bus,
    input  wire [`M_TO_H_BUS_WD-1:0] m_to_h_bus,
    input  wire [`W_TO_H_BUS_WD-1:0] w_to_h_bus,

    input  wire                      bj_taken,

    output wire                      stall_fetch,

    output wire                      stall_decode,
    output wire                      flush_decode,
    output wire[1:0]                 forward_rs1_select_d,
    output wire[1:0]                 forward_rs2_select_d,

    output wire                      flush_execute,
    output wire                      stall_execute,
    output wire[1:0]                 forward_rs1_select_e,
    output wire[1:0]                 forward_rs2_select_e,

    output wire                      flush_memory,
    output wire                      stall_memory,

    output wire                      flush_writeback
);
wire [4:0] rf_dest_e;
wire [4:0] rf_dest_m;
wire [4:0] rf_dest_w;
wire [4:0] rs1_addr_d;
wire [4:0] rs2_addr_d;
wire [4:0] rs1_addr_e;
wire [4:0] rs2_addr_e;

wire       mem_load_e;
wire       mem_load_m;
wire       need_use_rs;
wire       rf_write_en_e;
wire       rf_write_en_m;
wire       rf_write_en_w;

wire       ld_stall;
wire       br_stall;

assign {
    need_use_rs,
    rs1_addr_d,
    rs2_addr_d
} = d_to_h_bus;

assign {
    mem_load_e,
    rf_dest_e,
    rf_write_en_e,
    rs1_addr_e,
    rs2_addr_e
} = e_to_h_bus;

assign {
    mem_load_m,
    rf_dest_m,
    rf_write_en_m
} = m_to_h_bus;

assign {
    rf_dest_w,
    rf_write_en_w
} = w_to_h_bus;

assign forward_rs1_select_d = (rf_dest_m != 5'h0 && rf_write_en_m && (rs1_addr_d == rf_dest_m)) ? 2'b10 :
                        (rf_dest_w != 5'h0 && rf_write_en_w && (rs1_addr_d == rf_dest_w)) ? 2'b01 : 2'b00;

assign forward_rs2_select_d = (rf_dest_m != 5'h0 && rf_write_en_m && (rs2_addr_d == rf_dest_m)) ? 2'b10 :
                        (rf_dest_w != 5'h0 && rf_write_en_w && (rs2_addr_d == rf_dest_w)) ? 2'b01 : 2'b00;

assign forward_rs1_select_e = (rf_dest_m != 5'h0 && rf_write_en_m && (rs1_addr_e == rf_dest_m)) ? 2'b10 :
                        (rf_dest_w != 5'h0 && rf_write_en_w && (rs1_addr_e == rf_dest_w)) ? 2'b01 : 2'b00;

assign forward_rs2_select_e = (rf_dest_m != 5'h0 && rf_write_en_m && (rs2_addr_e == rf_dest_m)) ? 2'b10 :
                        (rf_dest_w != 5'h0 && rf_write_en_w && (rs2_addr_e == rf_dest_w)) ? 2'b01 : 2'b00;

assign ld_stall = (rf_dest_e != 5'h0 && mem_load_e && (rs1_addr_d == rf_dest_e || rs2_addr_d == rf_dest_e));

assign br_stall = need_use_rs && ((rf_dest_e != 5'h0 && rf_write_en_e && (rs1_addr_d == rf_dest_e || rs2_addr_d == rf_dest_e)) ||
                 (rf_dest_m != 5'h0 && mem_load_m && (rs1_addr_d == rf_dest_m || rs2_addr_d == rf_dest_m)));

assign stall_fetch  = stall_decode;

assign stall_decode  = ld_stall || br_stall || stall_execute;
assign flush_decode  = (bj_taken && !stall_decode);

assign flush_execute  = (stall_decode && !stall_execute);
assign stall_execute  = 1'b0;

assign stall_memory  = 1'b0;
assign flush_memory  = stall_execute;

assign flush_writeback = 1'b0;

endmodule