module regfile (
    input wire clk,
    input wire write_en,
    input wire[4:0] rs1_addr_i,
    input wire[4:0] rs2_addr_i,
    input wire[4:0] write_addr_i,
    input wire[31:0] write_data_i,
    output[31:0] rs1_data_o,
    output[31:0] rs2_data_o
);

    reg [31:0] x[31:0];

    assign rs1_data_o = (rs1_addr_i == 5'b00000) ? 32'b0 : x[rs1_addr_i];
    assign rs2_data_o = (rs2_addr_i == 5'b00000) ? 32'b0 : x[rs2_addr_i];

    always @(posedge clk) begin
        if (write_en & (write_addr_i != 5'b00000))
            x[write_addr_i] <= write_data_i;
    end

endmodule
