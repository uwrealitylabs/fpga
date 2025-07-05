module ov9281_config #(
    parameter CLK_SPEED = 50000000, //50 MHz
    parameter DATA_WIDTH = 32, //default 32 bits
) ( 
    input i_clk,
    input i_rst,
    /* config ports */
    input logic i_scl_in, 
    input logic i_sda_in, 
    output logic o_scl_out, 
    output logic o_sda_out, 
    output logic o_sda_oe, 
    output logic o_scl_oe

    /* capture ports */

);
ov9281_config config_inst #(
    .DATA_WIDTH(DATA_WIDTH),
    .CLK_SPEED(CLK_SPEED)
) (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_scl_in(i_scl_in),
    .i_sda_in(i_sda_in),   
    .o_scl_out(o_scl_out),
    .o_sda_out(o_sda_out),
    .o_sda_oe(o_sda_oe),
    .o_scl_oe(o_scl_oe)
);


endmodule
