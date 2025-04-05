`include "cpu_defs.vh"

module write_back (
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       previous_valid_i,
    input  wire                       flush_writeback,

    output wire [31:0] forward_result_writeback,

    output wire [`W_TO_H_BUS_WD-1 :0] w_to_h_bus,
    input  wire [`M_TO_W_BUS_WD-1 :0] m_to_w_bus,
    output wire [`W_TO_RF_BUS_WD-1:0] w_to_rf_bus,

    // trace debug interface
    output wire        debug_wb_have_inst,
    output wire [31:0] debug_wb_pc,
    output wire        debug_wb_ena,
    output wire [ 4:0] debug_wb_reg,
    output wire [31:0] debug_wb_value
);
/**************** input bus ****************/
wire mem_load_writeback;
wire[31:0] load_result_writeback;
wire[31:0] alu_result_writeback;
wire rf_write_en_writeback;
wire[4:0] rf_dest_writeback;
wire[31:0] pc_writeback;

reg  [`M_TO_W_BUS_WD-1:0] m_to_w_bus_r;
always @(posedge clk) begin
    if(reset || flush_writeback) begin
        m_to_w_bus_r <= 0;
    end else begin
        m_to_w_bus_r <= m_to_w_bus;
    end
end

assign {
    mem_load_writeback      ,
    load_result_writeback   ,
    alu_result_writeback    ,
    rf_write_en_writeback   ,
    rf_dest_writeback       ,
    pc_writeback            
} = m_to_w_bus_r;
//} = {m_to_w_bus_r[102], m_to_w_bus_r[69:0]};

//assign load_result_writeback = m_to_w_bus[101:70];

/**************** valid ******************/
reg  valid_writeback;
always @(posedge clk) begin
    if(reset || flush_writeback) begin
        valid_writeback <= 1'b0;
    end else begin
        valid_writeback <= previous_valid_i;
    end
end

/**************** write back ********************/
wire rf_write_en_o;
wire[4:0]  rf_dest_o;
wire[31:0] rf_write_data_o;

assign rf_write_en_o    = rf_write_en_writeback & valid_writeback;
assign rf_dest_o        = rf_dest_writeback;
assign rf_write_data_o  = mem_load_writeback ? load_result_writeback : alu_result_writeback;

/******************* output *******************/
assign forward_result_writeback = rf_write_data_o;

assign w_to_rf_bus = {
    rf_write_en_o       ,//[37:37]
    rf_dest_o           ,//[36:32]
    rf_write_data_o      //[31: 0]
};

assign w_to_h_bus = {
    rf_dest_writeback       ,//[5:1]
    rf_write_en_writeback    //[0:0]
};
/****************** debug ******************/
assign debug_wb_have_inst = valid_writeback;
assign debug_wb_pc        = pc_writeback;
assign debug_wb_ena       = rf_write_en_o;
assign debug_wb_reg       = rf_dest_o;
assign debug_wb_value     = rf_write_data_o;

endmodule