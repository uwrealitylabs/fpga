module ov9281_top #(
    parameter CLK_FREQ = 50000000, //50 MHz
    parameter DATA_WIDTH = 32 //default 32 bits
) ( 
    input i_clk,
    input i_rst,
    /* config ports */
    input i_start_cfg,
    input i_read_cfg,
    input i_write_cfg,
    input logic i_scl_in, 
    input logic i_sda_in, 
    output logic o_scl_out, 
    output logic o_sda_out, 
    output logic o_sda_oe, 
    output logic o_scl_oe

    /* capture ports */

);
localparam I2C_DATA_WIDTH = 32; 
localparam REG_ADDR_WIDTH = 16;
localparam REG_DATA_WIDTH = 8;

logic cfg_busy, cfg_error, cfg_valid;
logic [REG_DATA_WIDTH-1:0] cfg_rdata;

ov9281_cfg #(
    .CLK_FREQ(CLK_FREQ), // 50 MHz
    .I2C_DATA_WIDTH(I2C_DATA_WIDTH), // 32 bits
    .REG_ADDR_WIDTH(REG_ADDR_WIDTH), // 16 bits
    .REG_DATA_WIDTH(REG_DATA_WIDTH) // 8 bits
) cfg_unit (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_start(i_start_cfg), //start signal to begin configuration
    .i_read(i_read_cfg),
    .i_write(i_write_cfg),
    .i_cfg(1'b0), //not used in this module
    .o_busy(cfg_busy), //not used in this module
    .o_valid(cfg_valid), //not used in this module
    .o_error(cfg_error), //not used in this module
    .o_rdata(cfg_rdata), //not used in this module
    .o_cfg_addr(), 
    .o_cfg_data(),
    
    //physical ports to I2C bus
    .i_scl_in(i_scl_in), 
    .i_sda_in(i_sda_in), 
    .o_scl_out(o_scl_out), 
    .o_sda_out(o_sda_out), 
    .o_sda_oe(o_sda_oe), 
    .o_scl_oe(o_scl_oe)  
);



endmodule
