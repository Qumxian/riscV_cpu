`include "cpu_defs.vh"
module alu(
  input  wire [10:0] alu_op,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire [31:0] alu_result
);

wire [31:0] add_sub_result;
wire [31:0] slt_result;
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;
wire [31:0] sr_result;


wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

assign adder_a   = alu_src1;
assign adder_b   =  (alu_op[`ALU_SUB] | alu_op[`ALU_SLT] | alu_op[`ALU_SLTU]) ? ~alu_src2
                    : alu_src2;
assign adder_cin =  (alu_op[`ALU_SUB] | alu_op[`ALU_SLT] | alu_op[`ALU_SLTU]) ? 1'b1 : 1'b0;
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

assign add_sub_result = adder_result;

assign slt_result[31:1] = 31'b0;
assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])
                        | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);

assign sltu_result[31:1] = 31'b0;
assign sltu_result[0]    = ~adder_cout;

assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

assign sll_result = alu_src1 << alu_src2[4:0];

assign sr64_result = {{32{alu_op[`ALU_SRA] & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0];

assign sr_result   = sr64_result[31:0];

assign alu_result = ({32{alu_op[`ALU_ADD]|alu_op[`ALU_SUB]  }} & add_sub_result)
                  | ({32{alu_op[`ALU_SLT]                   }} & slt_result)
                  | ({32{alu_op[`ALU_SLTU]                  }} & sltu_result)
                  | ({32{alu_op[`ALU_AND]                   }} & and_result)
                  | ({32{alu_op[`ALU_OR]                    }} & or_result)
                  | ({32{alu_op[`ALU_XOR]                   }} & xor_result)
                  | ({32{alu_op[`ALU_LUI]                   }} & lui_result)
                  | ({32{alu_op[`ALU_SLL]                   }} & sll_result)
                  | ({32{alu_op[`ALU_SRL]|alu_op[`ALU_SRA]  }} & sr_result);

endmodule
