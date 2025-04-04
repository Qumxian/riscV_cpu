`include "cpu_defs.vh"

module memory(
    input  wire                      clk,
    input  wire                      reset,
    input  wire                      previous_valid_i,
    output wire                      current_valid_o,

    input  wire                      stall_memory,
    input  wire                      flush_memory,

    output wire[`M_TO_H_BUS_WD-1:0]  m_to_h_bus,
    input  wire[`E_TO_M_BUS_WD-1:0]  e_to_m_bus,
    output wire[`M_TO_W_BUS_WD-1:0]  m_to_w_bus,
    output wire[31:0]                forward_result_memory,

    output wire                      data_sram_en,
    output wire[ 3:0]                data_sram_we,
    output wire[31:0]                data_sram_addr,
    output wire[31:0]                data_sram_wdata,
    input  wire[31:0]                data_sram_rdata
);
/****************** input bus ******************/
wire mem_load;
wire mem_store;
wire[1:0] mem_access_size;
wire load_signext;
wire rf_write_en_memory;
wire[31:0] rf_dest_memory;
wire[31:0] rs2_data_memory;
wire[31:0] alu_result_memory;
wire[31:0] pc_memory;

reg  [`E_TO_M_BUS_WD-1:0] e_to_m_bus_r;
always @(posedge clk) begin
    if(reset || flush_memory) begin
        e_to_m_bus_r <= 0;
    end else if(!stall_memory) 
    begin
        e_to_m_bus_r <= e_to_m_bus;
    end
end

assign {mem_load                , 
        mem_store               , 
        mem_access_size         , 
        load_signext            , 
        rf_write_en_memory      , 
        rf_dest_memory          , 
        rs2_data_memory         , 
        alu_result_memory       , 
        pc_memory               
} = e_to_m_bus_r;

/**************** valid ******************/
reg  valid_memory;
always @(posedge clk) begin
    if(reset || flush_memory) begin
        valid_memory <= 1'b0;
    end else if(!stall_memory) 
    begin
        valid_memory <= previous_valid_i;
    end
end

assign current_valid_o = valid_memory;
/****************** memory access ******************/
wire[31:0] mem_addr;
wire[31:0] mem_read_data;
wire[1:0]  addr_low2bit;

assign mem_addr         = alu_result_memory;
assign addr_low2bit     = mem_addr[1:0];
assign data_sram_en     = (mem_load || mem_store) & valid_memory;
assign data_sram_addr   = alu_result_memory;
assign mem_read_data    = data_sram_rdata;
// load -- shit solution
reg mem_load_delay;
reg load_signext_delay;
reg[1:0] mem_access_size_delay;
reg[1:0] addr_low2bit_delay;
always @(posedge clk) begin
    if(reset) begin
        mem_load_delay          <= 1'b0;
        load_signext_delay      <= 1'b0;
        mem_access_size_delay   <= 2'b0;
        addr_low2bit_delay      <= 2'b0;
    end else begin
        mem_load_delay          <= mem_load;
        load_signext_delay      <= load_signext;
        mem_access_size_delay   <= mem_access_size;
        addr_low2bit_delay      <= addr_low2bit;
    end
end
wire load_byte;
wire load_half;
wire load_word;
wire[31:0] load_result;
wire[31:0] load_data_byte;
wire[31:0] load_data_half;

assign load_byte = mem_load_delay & mem_access_size_delay[0];
assign load_half = mem_load_delay & mem_access_size_delay[1];
assign load_word = mem_load_delay & ~load_byte & ~load_half;

assign load_data_byte = ({ 8{addr_low2bit_delay == 2'b00}} & mem_read_data[ 7: 0]) |
                        ({ 8{addr_low2bit_delay == 2'b01}} & mem_read_data[15: 8]) |
                        ({ 8{addr_low2bit_delay == 2'b10}} & mem_read_data[23:16]) |
                        ({ 8{addr_low2bit_delay == 2'b11}} & mem_read_data[31:24]);
assign load_data_half = ({16{addr_low2bit_delay == 2'b00}} & mem_read_data[15: 0]) |
                        ({16{addr_low2bit_delay == 2'b10}} & mem_read_data[31:16]);

assign load_result =  ({32{load_byte &  load_signext}} & {{24{load_data_byte[ 7]}}, load_data_byte}) |
                      ({32{load_byte & ~load_signext}} & {24'b0, load_data_byte}) |
                      ({32{load_half &  load_signext}} & {{16{load_data_half[15]}}, load_data_half}) |
                      ({32{load_half & ~load_signext}} & {16'b0, load_data_half}) |
                      ({32{load_word}} & mem_read_data);

// store 
wire store_byte;
wire store_half;
wire store_word;
wire[3:0] store_we_byte;
wire[3:0] store_we_half;
wire[3:0] store_we;
wire[31:0] store_result;

assign store_byte = mem_store & mem_access_size[0];
assign store_half = mem_store & mem_access_size[1];
assign store_word = mem_store & ~store_byte & ~store_half;

assign store_we_byte      = {addr_low2bit == 2'b11, addr_low2bit == 2'b10, 
                            addr_low2bit == 2'b01, addr_low2bit == 2'b00};
assign store_we_half      = {addr_low2bit == 2'b10, addr_low2bit == 2'b10, 
                            addr_low2bit == 2'b00, addr_low2bit == 2'b00};

assign store_we         = (({4{store_byte}} & store_we_byte) |
                           ({4{store_half}} & store_we_half) |
                           ({4{store_word}}));

assign data_sram_we     = store_we;
assign data_sram_wdata  = rs2_data_memory;

/********************* result select *********************/
wire[31:0] final_result;
assign final_result = alu_result_memory;
/******************** output bus ********************/
assign forward_result_memory = alu_result_memory;
assign load_result_o    = load_result;

assign m_to_w_bus = {
    mem_load             ,//[102:102]
    load_result          ,//[101: 70]
    final_result         ,//[ 69: 38]
    rf_write_en_memory   ,//[ 37: 37]
    rf_dest_memory       ,//[ 36: 32]
    pc_memory             //[ 31:  0]
};

assign m_to_h_bus = {
    mem_load            ,//[6:6]
    rf_dest_memory      ,//[5:1]
    rf_write_en_memory   //[0:0]
};
endmodule