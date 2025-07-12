//======================================================================
// MODULE: ov9281_cfg_tb.sv
//
// DESCRIPTION:
// This testbench initiates the ov9281_cfg
//======================================================================

`timescale 1ns / 1ps

module ov9281_cfg_tb;

localparam CLK_FREQ = 50000000; 
localparam I2C_DATA_WIDTH = 32; // 32 bits
localparam REG_ADDR_WIDTH = 16;
localparam REG_DATA_WIDTH = 8;
localparam SCRIPT_LEN = 109;
localparam CLK_PERIOD = 4;
localparam NUM_TESTS = 50;

logic clk, rst;
logic i_start, i_read, i_write, i_cfg, o_busy, o_error, o_led;
logic i_scl_in, i_sda_in;
logic o_scl_out, o_sda_out, o_sda_oe, o_scl_oe;
logic [REG_ADDR_WIDTH-1:0] i_raddr, i_waddr, o_cfg_addr;
logic [REG_DATA_WIDTH-1:0] i_wdata, o_cfg_data;
logic sim_failed;

ov92821_config #(
    .CLK_FREQ(CLK_FREQ), // 50 MHz
    .I2C_DATA_WIDTH(I2C_DATA_WIDTH), // 32 bits
    .REG_ADDR_WIDTH(REG_ADDR_WIDTH), // 16 bits
    .REG_DATA_WIDTH(REG_DATA_WIDTH) // 8 bits
) dut (
    .i_clk(clk),
    .i_rst(rst),
    .i_start(i_start), //start signal to begin configuration
    .i_read(i_read),
    .i_write(i_write),
    .i_cfg(i_cfg),
    .i_raddr(i_raddr),
    .i_waddr(i_waddr),
    .i_wdata(i_wdata),
    .o_busy(o_busy),
    .o_valid(o_valid),
    .o_error(o_error),
    .o_rdata(o_rdata),
    .o_cfg_addr(o_cfg_addr), 
    .o_cfg_data(o_cfg_data),
    .o_led(o_led), 

    //physical ports to I2C bus
    .i_scl_in(i_scl_in), 
    .i_sda_in(i_sda_in), 
    .o_scl_out(o_scl_out), 
    .o_sda_out(o_sda_out), 
    .o_sda_oe(o_sda_oe), 
    .o_scl_oe(o_scl_oe)
);

typedef struct packed {
    logic [REG_ADDR_WIDTH-1:0] addr; //register address
    logic [REG_DATA_WIDTH-1:0] val; //data to write
} reg_write_t;

const reg_write_t script [SCRIPT_LEN] = {
    '{16'h0103, 8'h01}, //software reset
    '{16'h030A, 8'h00}, //prediv 0
    '{16'h0300, 8'h01}, //prediv 1
    '{16'h0301, 8'h00},
    '{16'h0302, 8'h32}, //PLL_CTRL_02
    '{16'h0305, 8'h02},
    '{16'h0306, 8'h01},
    '{16'h0303, 8'h00},
    '{16'h0304, 8'h03},
    '{16'h0314, 8'h00},
    '{16'h030b, 8'h04},
    '{16'h030c, 8'h00},
    '{16'h030d, 8'h50}, //PLL_CTRL_OD
    '{16'h030e, 8'h02}, //PLL_CTRL_OE
    '{16'h030f, 8'h03}, //PLL_CTRL_OE
    '{16'h0313, 8'h01}, //PLL_CTRL_OE
    '{16'h0312, 8'h07}, //PLL_CTRL_OE
    '{16'h3001, 8'h00}, //SC_CTRL_01
    '{16'h3004, 8'h00}, //SC_CTRL_04
    '{16'h3005, 8'h00}, //SC_CTRL_05
    '{16'h3006, 8'h04}, //SC_CTRL_06
    '{16'h3011, 8'h0a}, //SC_CTRL_11
    '{16'h3013, 8'h18},
    '{16'h3022, 8'h01},
    '{16'h3023, 8'h00},
    '{16'h302c, 8'h00},
    '{16'h302f, 8'h00},
    '{16'h3030, 8'h04},
    '{16'h3039, 8'h32},
    '{16'h303a, 8'h00},
    '{16'h303f, 8'h01},
    '{16'h3500, 8'h00},
    '{16'h3501, 8'h2a},
    '{16'h3502, 8'h90},
    '{16'h3503, 8'h08},
    '{16'h3505, 8'h8c},
    '{16'h3507, 8'h03},
    '{16'h3508, 8'h00},
    '{16'h3509, 8'h10},
    '{16'h3610, 8'h80},
    '{16'h3611, 8'ha0},
    '{16'h3620, 8'h6f},
    '{16'h3632, 8'h56},
    '{16'h3633, 8'h78},
    '{16'h3662, 8'h05},
    '{16'h3666, 8'h00},
    '{16'h366f, 8'h5a},
    '{16'h3680, 8'h84},
    '{16'h3712, 8'h80},
    '{16'h372d, 8'h22},
    '{16'h3731, 8'h80},
    '{16'h3732, 8'h30},
    '{16'h3778, 8'h00},
    '{16'h377d, 8'h22},
    '{16'h3788, 8'h02},
    '{16'h3789, 8'ha4},
    '{16'h378a, 8'h00},
    '{16'h378b, 8'h4a},
    '{16'h3799, 8'h20},
    '{16'h3800, 8'h00},
    '{16'h3801, 8'h00},
    '{16'h3802, 8'h00},
    '{16'h3803, 8'h00},
    '{16'h3804, 8'h05},
    '{16'h3805, 8'h0f},
    '{16'h3806, 8'h03},
    '{16'h3807, 8'h2f},
    '{16'h3808, 8'h05},
    '{16'h3809, 8'h00},
    '{16'h380a, 8'h03},
    '{16'h380b, 8'h20},
    '{16'h380c, 8'h02},
    '{16'h380d, 8'hd8},
    '{16'h380e, 8'h03},
    '{16'h380f, 8'h8e},
    '{16'h3810, 8'h00},
    '{16'h3811, 8'h08},
    '{16'h3812, 8'h00},
    '{16'h3813, 8'h08},
    '{16'h3814, 8'h11},
    '{16'h3815, 8'h11},
    '{16'h3820, 8'h40},
    '{16'h3821, 8'h00},
    '{16'h3881, 8'h42},
    '{16'h38b1, 8'h00},
    '{16'h3920, 8'hff},
    '{16'h4003, 8'h40},
    '{16'h4008, 8'h04},
    '{16'h4009, 8'h0b},
    '{16'h400c, 8'h00},
    '{16'h400d, 8'h07},
    '{16'h4010, 8'h40},
    '{16'h4043, 8'h40},
    '{16'h4307, 8'h30},
    '{16'h4317, 8'h00},
    '{16'h4501, 8'h00},
    '{16'h4507, 8'h00},
    '{16'h4509, 8'h00},
    '{16'h450a, 8'h08},
    '{16'h4601, 8'h04},
    '{16'h470f, 8'h00},
    '{16'h4f07, 8'h00},
    '{16'h4800, 8'h10}, //enable sync line for debugging
    '{16'h5000, 8'h9f},
    '{16'h5001, 8'h00},
    '{16'h5e00, 8'h80}, //enable test pattern
    '{16'h5d00, 8'h07},
    '{16'h5d01, 8'h00},
    '{16'hFFFF, 8'h00}  // Sentinel value to mark the end of the script
};

reg_write_t current_cmd; 
//test read and write operation
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end


initial begin  
    rst = 1'b1;
    i_start = 1'b0;
    i_read = 1'b0;
    i_write = 1'b0;
    sim_failed = 1'b0;
    #(CLK_PERIOD * 25);
    $display("Starting OV9281 Configuration Test...");
    rst = 1'b0; 
    i_write = 1'b1;
    i_read = 1'b0;

    // --- OV9281 CONFIGURATION OPERATION ---
    i_cfg = 1'b1;
    i_start = 1'b1;
    #(CLK_PERIOD);
    i_start = 1'b0; 
    
    wait(o_busy);

    //simulate a configure operation
    for (int i = 0; i < SCRIPT_LEN; i++) begin
        current_cmd = script[i];
        if (o_cfg_addr != current_cmd.addr || o_cfg_data != current_cmd.val
            || !o_busy || o_rdata) begin
            $display("Error: Configuration mismatch at index %0d o_cfg_addr: %0d o_cfg_data: %0d", i, o_cfg_addr, o_cfg_data);
        end
    end

    wait (|o_busy);

    // --- OV9281 WRITE OPERATION ---
    $display("Starting OV9281 WRITE Test...");
    i_cfg = 1'b0;
    i_waddr = 16'h0100;
    i_wdata = {0};
    i_start = 1'b1;
    #(CLK_PERIOD);
    i_start = 1'b0;
    wait(o_busy);
    if (o_cfg_addr != i_waddr || o_cfg_data != i_wdata || !o_busy) begin
        $display("Error: Write operation failed. Expected addr: %0d, data: %0d, got addr: %0d, data: %0d", 
                 i_waddr, i_wdata, o_cfg_addr, o_cfg_data);
    end else begin
        $display("Write operation successful. Addr: %0d, Data: %0d", o_cfg_addr, o_cfg_data);
    end


    // --- OV9281 READ OPERATION ---
    $display("Starting OV9281 READ Test...");
    i_read = 1'b1;
    i_raddr = 16'h300A; //SC CHIP ID HIGH
    i_start = 1'b1;
    #(CLK_PERIOD);
    i_start = 1'b0; 
    wait(o_busy);

    #(CLK_PERIOD * 25);
    $stop; 

end



