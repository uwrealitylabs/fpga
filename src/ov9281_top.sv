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

logic done_r, error_r;
ov9281_cfg  #(
    .DATA_WIDTH(DATA_WIDTH),
    .CLK_FREQ(CLK_FREQ)
) cfg_unit (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_start(i_start_config), 
    .i_read(i_read_cfg),
    .i_write(i_write_cfg),
    .o_done(done_r), 
    .o_error(error_r), 
    //physical ports to I2C bus
    .i_scl_in(i_scl_in), 
    .i_sda_in(i_sda_in), 
    .o_scl_out(o_scl_out), 
    .o_sda_out(o_sda_out), 
    .o_sda_oe(o_sda_oe), 
    .o_scl_oe(o_scl_oe)
);




endmodule
