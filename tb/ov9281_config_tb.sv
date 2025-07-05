`timescale 1ns / 1ps

module ov9281_config_tb;

logic clk;
logic rst;

localparam CLK_SPEED = 50000000; // 50 MHz
localparam DATA_WIDTH = 32; // 32 bits
ov92821_config #(
    .CLK_SPEED(CLK_SPEED), // 50 MHz
    .DATA_WIDTH(DATA_WIDTH) // 32 bits
) dut (
    .i_clk(clk),
    .i_rst(rst),
    .i_scl_in(1'b0), // Placeholder for SCL input
    .i_sda_in(1'b0), // Placeholder for SDA input
    .o_scl_out(), // SCL output
    .o_sda_out(), // SDA output
    .o_sda_oe(), // SDA output enable
    .o_scl_oe()  // SCL output enable
);

initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
end

initial begin  

end 


