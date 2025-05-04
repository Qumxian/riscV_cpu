`include "cpu_defs.vh"

module myCPU(
    input  wire cpu_rst,
    input  wire cpu_clk,
    output wire [31:0] irom_addr,
    input  wire [31:0] irom_data,

    output wire [31:0] perip_addr,
    output wire perip_wen,
    output wire [1 :0]perip_mask,
    output wire [31:0] perip_wdata,
    input  wire [31:0] perip_rdata
);
wire [3:0] raw_we;

riscv_core u_riscv_core (
    .clk(cpu_clk),
    .reset(cpu_rst),
    .inst_sram_addr(irom_addr),
    .inst_sram_rdata(irom_data),
    .data_sram_addr(perip_addr),
    .data_sram_we(raw_we),
    .data_sram_mask(perip_mask),
    .data_sram_wdata(perip_wdata),
    .data_sram_rdata(perip_rdata)
);

assign perip_wen = |raw_we;

endmodule