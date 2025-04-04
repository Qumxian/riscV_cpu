`include "cpu_defs.vh"
`include "riscv_defs.vh"

module decode (
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       previous_valid_i,
    output wire                       current_valid_o,

    input  wire                       stall_decode,
    input  wire                       flush_decode,

    output wire                       bj_taken_o,
    output wire[31:0]                 bj_target_o,

    input  wire[ 1:0]                 forward_rs1_select_d,
    input  wire[ 1:0]                 forward_rs2_select_d,
    input  wire[31:0]                 forward_result_memory,
    input  wire[31:0]                 forward_result_writeback,

    input  wire [`F_TO_D_BUS_WD-1:0]  f_to_d_bus,
    output wire [`D_TO_E_BUS_WD-1:0]  d_to_e_bus,
    output wire [`D_TO_H_BUS_WD-1:0]  d_to_h_bus,
    input  wire [`W_TO_RF_BUS_WD-1:0] w_to_rf_bus
);
/******************** input bus *******************/
wire[31:0] inst_decode, pc_decode;
reg  [`F_TO_D_BUS_WD-1:0] f_to_d_bus_r;
always @(posedge clk) begin
    if(reset || flush_decode)
    begin
        f_to_d_bus_r <= 0;
    end else if(!stall_decode) 
    begin
        f_to_d_bus_r <= f_to_d_bus;
    end
end

reg hold_inst_sign;
reg[31:0] inst_buffer;
always @(posedge clk) begin
    if(reset) begin
        inst_buffer <= 32'h0;
        hold_inst_sign <= 1'b0;
    end else if(stall_decode)
    begin
        inst_buffer <= inst_decode;
        hold_inst_sign <= 1'b1;
    end else if(hold_inst_sign)
    begin
        hold_inst_sign <= 1'b0;
    end
end

assign {pc_decode} = f_to_d_bus_r[63:32];
assign inst_decode = hold_inst_sign ? inst_buffer : f_to_d_bus[31:0];

/**************** valid ******************/
reg valid_decode;
always @(posedge clk) begin
    if(reset || flush_decode) begin
        valid_decode <= 1'b0;
    end else if(!stall_decode) 
    begin
        valid_decode <= previous_valid_i;
    end
end

assign current_valid_o = valid_decode;

wire opcode_op;
wire opcode_opimm;
wire opcode_load;
wire opcode_store;
wire opcode_branch;
wire opcode_jal;
wire opcode_jalr;
wire opcode_lui;
wire opcode_auipc;
wire opcode_system;
wire opcode_fence;

wire funct3_000;
wire funct3_001;
wire funct3_010;
wire funct3_011;
wire funct3_100;
wire funct3_101;
wire funct3_110;
wire funct3_111;

wire funct7_00H;
wire funct7_20H;

wire[6:0] opcode;
wire[2:0] funct3;
wire[4:0] funct7;
wire[4:0] dest;
wire[4:0] rd_addr_decode;
wire[4:0] rs1_addr_decode;
wire[4:0] rs2_addr_decode;

wire[31:0] i_imm;
wire[31:0] s_imm;
wire[31:0] b_imm;
wire[31:0] u_imm;
wire[31:0] j_imm;

assign i_imm = {{21{inst_decode[31]}}, inst_decode[30:25], inst_decode[24:21], inst_decode[20]};
assign s_imm = {{21{inst_decode[31]}}, inst_decode[30:25], inst_decode[11: 8], inst_decode[ 7]};
assign b_imm = {{20{inst_decode[31]}}, inst_decode[7], inst_decode[30:25], inst_decode[11:8], 1'b0};
assign u_imm = {inst_decode[31], inst_decode[30:20], inst_decode[19:12], 12'b0};
assign j_imm = {{12{inst_decode[31]}}, inst_decode[19:12], inst_decode[20], inst_decode[30:25], inst_decode[24:21], 1'b0};

assign opcode = inst_decode[6:0];
assign funct3 = inst_decode[14:12];
assign funct7 = inst_decode[31:25];
assign dest   = inst_decode[11:7];
assign rd_addr_decode  = inst_decode[11:7];
assign rs1_addr_decode = inst_decode[19:15];
assign rs2_addr_decode = inst_decode[24:20];

assign opcode_lui   = opcode == `OPCODE_LUI;
assign opcode_auipc = opcode == `OPCODE_AUIPC;
assign opcode_jal   = opcode == `OPCODE_JAL;
assign opcode_jalr  = opcode == `OPCODE_JALR;
assign opcode_branch= opcode == `OPCODE_BRANCH;
assign opcode_load  = opcode == `OPCODE_LOAD;
assign opcode_store = opcode == `OPCODE_STORE;
assign opcode_opimm = opcode == `OPCODE_OPIMM;
assign opcode_op    = opcode == `OPCODE_OP;
assign opcode_fence = opcode == `OPCODE_FENCE;
assign opcode_system= opcode == `OPCODE_SYSTEM;

assign funct3_000 = funct3 == 3'b000;
assign funct3_001 = funct3 == 3'b001;
assign funct3_010 = funct3 == 3'b010;
assign funct3_011 = funct3 == 3'b011;
assign funct3_100 = funct3 == 3'b100;
assign funct3_101 = funct3 == 3'b101;
assign funct3_110 = funct3 == 3'b110;
assign funct3_111 = funct3 == 3'b111;

assign funct7_00H = funct7 == 7'h00;
assign funct7_20H = funct7 == 7'h20;

// rv32i
wire inst_lui;
wire inst_auipc;
wire inst_jal;
wire inst_jalr;
wire inst_beq;
wire inst_bne;
wire inst_blt;
wire inst_bge;
wire inst_bltu;
wire inst_bgeu;
wire inst_lb;
wire inst_lh;
wire inst_lw;
wire inst_lbu;
wire inst_lhu;
wire inst_sb;
wire inst_sh;
wire inst_sw;
wire inst_addi;
wire inst_slti;
wire inst_sltiu;
wire inst_xori;
wire inst_ori;
wire inst_andi;
wire inst_slli;
wire inst_srli;
wire inst_srai;
wire inst_add;
wire inst_sub;
wire inst_sll;
wire inst_slt;
wire inst_sltu;
wire inst_xor;
wire inst_srl;
wire inst_sra;
wire inst_or;
wire inst_and;
wire inst_fence;
wire inst_fence_tso;
wire inst_pause;
wire inst_ecall;
wire inst_ebreak;

/************* identify instructions *************/
// lui
assign inst_lui     = opcode_lui;
// auipc
assign inst_auipc   = opcode_auipc;
// jal
assign inst_jal     = opcode_jal;
// jalr
assign inst_jalr    = opcode_jalr;
// branch
assign inst_beq     = opcode_branch & funct3_000;
assign inst_bne     = opcode_branch & funct3_001;
assign inst_blt     = opcode_branch & funct3_100;
assign inst_bge     = opcode_branch & funct3_101;
assign inst_bltu    = opcode_branch & funct3_110;
assign inst_bgeu    = opcode_branch & funct3_111;
// load
assign inst_lb      = opcode_load & funct3_000;
assign inst_lh      = opcode_load & funct3_001;
assign inst_lw      = opcode_load & funct3_010;
assign inst_lbu     = opcode_load & funct3_100;
assign inst_lhu     = opcode_load & funct3_101;
// store
assign inst_sb      = opcode_store & funct3_000;
assign inst_sh      = opcode_store & funct3_001;
assign inst_sw      = opcode_store & funct3_010;
// i-type
assign inst_addi    = opcode_opimm & funct3_000;
assign inst_slti    = opcode_opimm & funct3_010;
assign inst_sltiu   = opcode_opimm & funct3_011;
assign inst_xori    = opcode_opimm & funct3_100;
assign inst_ori     = opcode_opimm & funct3_110;
assign inst_andi    = opcode_opimm & funct3_111;
assign inst_slli    = opcode_opimm & funct3_001 & funct7_00H;
assign inst_srli    = opcode_opimm & funct3_101 & funct7_00H;
assign inst_srai    = opcode_opimm & funct3_101 & funct7_20H;
// r-type
assign inst_add     = opcode_op & funct3_000 & funct7_00H;
assign inst_sub     = opcode_op & funct3_000 & funct7_20H;
assign inst_sll     = opcode_op & funct3_001 & funct7_00H;
assign inst_slt     = opcode_op & funct3_010 & funct7_00H;
assign inst_sltu    = opcode_op & funct3_011 & funct7_00H;
assign inst_xor     = opcode_op & funct3_100 & funct7_00H;
assign inst_srl     = opcode_op & funct3_101 & funct7_00H;
assign inst_sra     = opcode_op & funct3_101 & funct7_20H;
assign inst_or      = opcode_op & funct3_110 & funct7_00H;
assign inst_add     = opcode_op & funct3_111 & funct7_00H;
// fence 
assign inst_fence   = opcode_fence;
/************** instruction format **************/
wire r_type;
wire i_type;
wire s_type;
wire b_type;
wire u_type;
wire j_type;
assign r_type = opcode_op;
assign i_type = opcode_opimm | opcode_load | opcode_jalr;
assign s_type = opcode_store;
assign b_type = opcode_branch;
assign u_type = opcode_lui | opcode_auipc;
assign j_type = opcode_jal;

/************** alu options **************/
wire[`ALU_OP_WD-1:0] alu_op;
assign alu_op[`ALU_ADD] = opcode_load || opcode_store || inst_add || inst_addi;
assign alu_op[`ALU_SUB] = inst_sub;
assign alu_op[`ALU_SLT] = inst_slt || inst_slti;
assign alu_op[`ALU_SLTU]= inst_sltu || inst_sltiu;
assign alu_op[`ALU_XOR] = inst_xor || inst_xori;
assign alu_op[`ALU_OR]  = inst_or || inst_ori;
assign alu_op[`ALU_AND] = inst_and || inst_andi;
assign alu_op[`ALU_SLL] = inst_sll || inst_slli;
assign alu_op[`ALU_SRL] = inst_srl || inst_srli;
assign alu_op[`ALU_SRA] = inst_sra || inst_srai;
assign alu_op[`ALU_LUI] = inst_lui;

/*************************** memory access control ***************************/
wire mem_store;
wire mem_load;
wire load_signext;
wire[1:0] mem_access_size;

assign mem_load  = (inst_lb || inst_lh || inst_lw || inst_lbu || inst_lhu) & valid_decode;
assign mem_store = (inst_sb || inst_sh || inst_sw) & valid_decode;
assign mem_access_size[0] = inst_lb || inst_lbu || inst_sb;
assign mem_access_size[1] = inst_lh || inst_lhu || inst_sh;
assign load_signext = inst_lb || inst_lh;

/************************** regfile **************************/
// regfile access
wire rf_write_en;
wire[4:0] rf_dest;
wire[31:0] rf_write_data;
wire[31:0] rs1_data_o;
wire[31:0] rs2_data_o;
wire[31:0] rs1_data_decode;
wire[31:0] rs2_data_decode;

assign {
    rf_write_en,
    rf_dest,
    rf_write_data
} = w_to_rf_bus;

regfile u_regfile(
    .clk            (clk),
    .write_en       (rf_write_en),
    .rs1_addr_i     (rs1_addr_decode),
    .rs2_addr_i     (rs2_addr_decode),
    .rs1_data_o     (rs1_data_o),
    .rs2_data_o     (rs2_data_o),
    .write_addr_i   (rf_dest),
    .write_data_i   (rf_write_data)
);
assign rs1_data_decode = forward_rs1_select_d[1] ? forward_result_memory :
                         forward_rs1_select_d[0] ? forward_result_writeback:
                         rs1_data_o;
assign rs2_data_decode = forward_rs2_select_d[1] ? forward_result_memory :
                         forward_rs2_select_d[0] ? forward_result_writeback:
                         rs2_data_o;
// decode stage data for regfile
wire rf_write_en_decode;
wire[4:0] rf_dest_decode;
assign rf_write_en_decode = (opcode_auipc || opcode_lui || opcode_jal || opcode_jalr || opcode_load ||
                            opcode_op || opcode_opimm) & valid_decode;
assign rf_dest_decode = dest;

/************************ oprands select ************************/
wire src1_is_pc;
wire src2_is_imm;
wire src2_is_4;
wire imm_decode;
assign imm_decode =     ({32{i_type}} & i_imm) |
                        ({32{s_type}} & s_imm) |
                        ({32{b_type}} & b_imm) |
                        ({32{u_type}} & u_imm) |
                        ({32{j_type}} & j_imm);

assign src1_is_pc   =   inst_auipc || inst_jal || inst_jalr;
assign src2_is_4    =   inst_jal || inst_jalr;
assign src2_is_imm  =   opcode_lui || opcode_auipc || opcode_load || opcode_store ||  
                        opcode_opimm;

/********************** branch and jump **********************/
// to handle hazard
wire branch_inst;
assign branch_inst = opcode_branch;
// get target
wire[31:0] jal_target;
wire[31:0] jalr_target;
wire[31:0] branch_target;
assign jal_target   = pc_decode + j_imm;
assign jalr_target  = (rs1_data_decode + i_imm) & ~32'b1;
assign branch_target= pc_decode + b_imm;
// condition judgment
wire rs1_eq_rs2;
wire rs1_lt_rs2;
wire rs1_lt_rs2_u;
assign rs1_eq_rs2   =   rs1_data_decode == rs2_data_decode;
assign rs1_lt_rs2_u =   rs1_data_decode <  rs2_data_decode;
assign rs1_lt_rs2   =   ( rs1_data_decode[31] & ~rs2_data_decode[31]) ? 1'b1 :
                        (~rs1_data_decode[31] &  rs2_data_decode[31]) ? 1'b0 :
                        rs1_lt_rs2_u;
// result
assign bj_taken_o   =   ((inst_beq && rs1_eq_rs2) || (inst_bne && !rs1_eq_rs2) ||
                         (inst_blt && rs1_lt_rs2) || (inst_bge && !rs1_lt_rs2) ||
                         (inst_bltu && rs1_lt_rs2_u) || (inst_bgeu && rs1_lt_rs2_u) ||
                         (inst_jal || inst_jalr)
                        ) && valid_decode;
assign bj_target_o  =   ({32{opcode_jal }} & jal_target ) |
                        ({32{opcode_jalr}} & jalr_target) |
                        ({32{opcode_branch}} & branch_target);

/****************************** data bus ******************************/
assign d_to_e_bus = {
    rs1_addr_decode    ,//[162:158]
    rs2_addr_decode    ,//[157:153]
    mem_store          ,//[152:152]
    mem_load           ,//[151:151]
    mem_access_size    ,//[150:149]
    load_signext       ,//[148:148]
    rf_write_en_decode ,//[147:147]
    rf_dest_decode     ,//[146:142]
    alu_op             ,//[141:131]
    src1_is_pc         ,//[130:130]
    src2_is_4          ,//[129:129]
    src2_is_imm        ,//[128:128]
    imm_decode         ,//[127: 96]
    rs1_data_decode    ,//[ 95: 64]
    rs2_data_decode    ,//[ 63: 32]
    pc_decode           //[ 31:  0]
};

assign d_to_h_bus = {
    branch_inst        ,//[11:11]
    bj_taken_o         ,//[10:10]
    rs1_addr_decode    ,//[ 9: 5]
    rs2_addr_decode     //[ 4: 0]
};
// decode context end
endmodule