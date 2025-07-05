`timescale 1ns / 1ps

module ov9281_cfg_tb;

logic i_clk, i_rst;
logic i_read, i_write, o_done, i_start, o_error;
logic i_scl_in, i_sda_in;
logic o_scl_out, o_sda_out, o_sda_oe, o_scl_oe;
localparam CLK_FREQ = 50000000; 
localparam DATA_WIDTH = 32; // 32 bits

localparam SCRIPT_LEN = 96;
localparam CYCLES_PER_SECOND = 16;
ov92821_config #(
    .CLK_FREQ(CLK_FREQ), // 50 MHz
    .DATA_WIDTH(DATA_WIDTH) // 32 bits
) dut (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_start(i_start), //start signal to begin configuration
    .i_read(i_read),
    .i_write(i_write),
    .o_done(o_done),
    .o_error(o_error),
    //physical ports to I2C bus
    .i_scl_in(i_scl_in), 
    .i_sda_in(i_sda_in), 
    .o_scl_out(o_scl_out), 
    .o_sda_out(o_sda_out), 
    .o_sda_oe(o_sda_oe), 
    .o_scl_oe(o_scl_oe)
);

initial begin
    clk = 1'b0;
    forever #(CYCLES_PER_SECOND/2) clk = ~clk;
end

initial begin  
    rst = 1'b1;
    #(CLK_FREQ * 25);
    rst = 1'b0; 


end 
endmodule

