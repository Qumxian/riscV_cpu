module miniRV_SoC (
    input  wire         fpga_rst,   // High active
    input  wire         fpga_clk,

    output wire         debug_wb_have_inst, // 当前时钟周期是否有指令写回 (对单周期CPU，可在复位后恒置1)
    output wire [31:0]  debug_wb_pc,        // 当前写回的指令的PC (若wb_have_inst=0，此项可为任意值)
    output wire         debug_wb_ena,       // 指令写回时，寄存器堆的写使能 (若wb_have_inst=0，此项可为任意值)
    output wire [ 4:0]  debug_wb_reg,       // 指令写回时，写入的寄存器号 (若wb_ena或wb_have_inst=0，此项可为任意值)
    output wire [31:0]  debug_wb_value      // 指令写回时，写入寄存器的值 (若wb_ena或wb_have_inst=0，此项可为任意值)

);
    wire cpu_clk;
    assign cpu_clk = fpga_clk;
    reg  resetn;
    always @(posedge cpu_clk) resetn <= ~fpga_rst;

    wire        inst_sram_en;
    wire [3:0]  inst_sram_we;
    wire [31:0] inst_sram_addr;
    wire [31:0] inst_sram_wdata;
    wire [31:0] inst_sram_rdata;

    wire        data_sram_en;
    wire [3:0]  data_sram_we;
    wire [31:0] data_sram_addr;
    wire [31:0] data_sram_wdata;
    wire [31:0] data_sram_rdata;
    
    riscv_core u_riscv_core (
        .clk                    (cpu_clk            ),
        .resetn                 (resetn             ),
        .inst_sram_en           (inst_sram_en       ),
        .inst_sram_we           (inst_sram_we       ),
        .inst_sram_addr         (inst_sram_addr     ),
        .inst_sram_wdata        (inst_sram_wdata    ),
        .inst_sram_rdata        (inst_sram_rdata    ),

        .data_sram_en           (data_sram_en       ),
        .data_sram_we           (data_sram_we       ),
        .data_sram_addr         (data_sram_addr     ),
        .data_sram_wdata        (data_sram_wdata    ),
        .data_sram_rdata        (data_sram_rdata    ),

        .debug_wb_have_inst     (debug_wb_have_inst ),
        .debug_wb_pc            (debug_wb_pc        ),
        .debug_wb_ena           (debug_wb_ena       ),
        .debug_wb_reg           (debug_wb_reg       ),
        .debug_wb_value         (debug_wb_value     )
    );
    
    // 下面两个模块，只需要实例化代码和连线代码，不需要创建IP核
    wire[15:0] irom_addr;
    assign irom_addr = inst_sram_addr[17:2];
    IROM Mem_IROM (
        .a          (irom_addr),
        .spo        (inst_sram_rdata)
    );

    wire[15:0] dram_addr;
    assign dram_addr = data_sram_addr[17:2];
    DRAM Mem_DRAM (
        .clk        (cpu_clk),
        .a          (dram_addr),
        .spo        (data_sram_rdata),
        .we         (data_sram_we),
        .d          (data_sram_wdata)
    );

endmodule
