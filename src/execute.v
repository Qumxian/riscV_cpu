`include "cpu_defs.vh"

module execute (
    input  wire                      clk,
    input  wire                      reset,
    input  wire                      previous_valid_i,
    output wire                      current_valid_o,

    input  wire                      flush_execute,
    input  wire                      stall_execute,

    input  wire [ 1:0]               forward_rs1_select_e,
    input  wire [ 1:0]               forward_rs2_select_e,
    input  wire [31:0]               forward_result_memory,
    input  wire [31:0]               forward_result_writeback,

    input  wire [`D_TO_E_BUS_WD-1:0] d_to_e_bus,
    output wire [`E_TO_M_BUS_WD-1:0] e_to_m_bus,
    output wire [`E_TO_H_BUS_WD-1:0] e_to_h_bus
);
/**************** input bus ****************/
wire[4:0] rs1_addr_execute;
wire[4:0] rs2_addr_execute;
wire mem_load_execute;
wire mem_store_execute;
wire[1:0] mem_access_size_execute;
wire load_signext_execute;
wire rf_write_en_execute;
wire[4:0] rf_dest_execute;
wire[10:0] alu_op_execute;
wire src1_is_pc_execute;
wire src2_is_4_execute;
wire src2_is_imm_execute;
wire[31:0] imm_execute;
wire[31:0] rs1_data_execute;
wire[31:0] rs2_data_execute;
wire[31:0] pc_execute;

reg  [`D_TO_E_BUS_WD-1:0] d_to_e_bus_r;
always @(posedge clk) begin
    if(reset || flush_execute) 
    begin
        d_to_e_bus_r <= 0;
    end else if(!stall_execute) 
    begin
        d_to_e_bus_r <= d_to_e_bus;
    end
end

assign {rs1_addr_execute        ,
        rs2_addr_execute        ,
        mem_store_execute       ,
        mem_load_execute        ,
        mem_access_size_execute ,
        load_signext_execute    ,
        rf_write_en_execute     ,
        rf_dest_execute         ,
        alu_op_execute          ,
        src1_is_pc_execute      , 
        src2_is_4_execute       ,
        src2_is_imm_execute     ,
        imm_execute             ,
        rs1_data_execute        , 
        rs2_data_execute        ,
        pc_execute
} = d_to_e_bus_r;
/**************** valid ******************/
reg valid_execute;
always@(posedge clk) begin
    if(reset || flush_execute) begin
        valid_execute <= 1'b0;
    end else if(!stall_execute)
    begin
        valid_execute <= previous_valid_i;
    end
end

assign current_valid_o = valid_execute;

/******************* alu *******************/
// determine oprands
wire[31:0] right_rs1_data;
wire[31:0] right_rs2_data;
assign right_rs1_data = forward_rs1_select_e[1] ? forward_result_memory:
                        forward_rs1_select_e[0] ? forward_result_writeback:
                        rs1_data_execute;
assign right_rs2_data = forward_rs2_select_e[1] ? forward_result_memory:
                        forward_rs2_select_e[0] ? forward_result_writeback:
                        rs2_data_execute;

wire[31:0] src1_execute;
wire[31:0] src2_execute;
assign src1_execute =   src1_is_pc_execute ? pc_execute : right_rs1_data;
assign src2_execute =   src2_is_4_execute  ? 32'h4      :
                        src2_is_imm_execute? imm_execute:
                        right_rs2_data; 
// alu
wire[31:0] alu_result;
alu alu(
    .alu_op(alu_op_execute),
    .alu_src1(src1_execute),
    .alu_src2(src2_execute),
    .alu_result(alu_result)
);

/******************** output bus ********************/
assign e_to_m_bus = {
    mem_load_execute       ,//[106:106] 
    mem_store_execute      ,//[105:105] 
    mem_access_size_execute,//[104:103] 
    load_signext_execute   ,//[102:102] 
    rf_write_en_execute    ,//[101:101] 
    rf_dest_execute        ,//[100: 96] 
    right_rs2_data         ,//[ 95: 64] 
    alu_result             ,//[ 63: 32]  
    pc_execute              //[ 31:  0]  
};

assign e_to_h_bus = {
    mem_load_execute       ,//[16:16]
    rf_dest_execute        ,//[15:11]
    rf_write_en_execute    ,//[10:10]
    rs1_addr_execute       ,//[ 9: 5]
    rs2_addr_execute        //[ 4: 0]
};
endmodule