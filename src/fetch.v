`include "cpu_defs.vh"

module fetch (
    input  wire clk,
    input  wire reset,
    input  wire current_valid_o,
    input  wire stall_fetch,

    input  wire bj_taken_i,
    input  wire[31:0] bj_target_i,
    output wire [`F_TO_D_BUS_WD-1:0] f_to_d_bus,

    output wire       inst_sram_en,
    output wire[3:0]  inst_sram_we,
    output wire[31:0] inst_sram_addr,
    output wire[31:0] inst_sram_wdata,
    output wire[31:0] inst_sram_rdata
);
/**************** valid ******************/
reg valid_fetch;
always@(posedge clk) begin
    if(reset) begin
        valid_fetch <= 1'b0;
    end else begin
        valid_fetch <= 1'b1;
    end
end

assign current_valid_o = valid_fetch;

reg[31:0] pc;
wire[31:0] pc_fetch;
wire[31:0] inst_decode;
wire[31:0] next_pc, seq_pc;

assign inst_sram_en = 1'b1;
assign inst_sram_we = 4'h0;
assign inst_sram_addr = pc;
assign inst_sram_wdata = 32'b0;

assign seq_pc = pc + 32'h4;
assign next_pc = bj_taken_i ? bj_target_i : seq_pc;

assign pc_fetch   = pc;
assign inst_decode = inst_sram_rdata;

always @(posedge clk) begin
    if(reset) begin 
        pc <= 32'b0;
    end else if(!stall_fetch) begin
        pc <= next_pc;
    end
end

assign fetch_to_decode_bus = {
    pc_fetch        ,//[63:32]
    inst_decode      //[31:0]
};
endmodule