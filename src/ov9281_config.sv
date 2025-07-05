module ov9281_config #(
    parameter CLK_FREQ = 50 //50 MHz 
    parameter DATA_WIDTH = 32 //default 32 bits
)
(
    input i_clk,
    input i_rst,
    input i_start, //start signal to begin configuration
    input read,
    input write,
    input num_bytes_rw, //num of bytes to r/w
    output logic o_done, //indicates configuration is done

    //physical ports to I2C bus
    input logic i_scl_in, 
    input logic i_sda_in, 
    output logic o_scl_out, 
    output logic o_sda_out, 
    output logic o_sda_oe, 
    output logic o_scl_oe
);

localparam SCRIPT_LEN = 95;
typedef struct packed {
    logic [15:0] addr; //register address
    logic [7:0] val; //data to write
} reg_write_t;



const reg_write_t script [SCRIPT_LEN] = ' {
    '{16'h0103, 8'h01}, //software reset
    '{16'h0302, 8'h32}, //PLL_CTRL_02
    '{16'h030d, 8'h50}, //PLL_CTRL_OD
    '{16'h030e, 8'h02}, //PLL_CTRL_OE
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

/* Register Addresses */ */
localparam SLAVE_ADDR_WRITE = 8'hC0; //I2C write address  
localparam SLAVE_ADDR_READ = 8'hC1; //I2C read address with ACK

localparam SC_SOFTWARE_RESET = 16'h0103; //software reset
localparam SC_MODE_SELECT = 16'h0100; //mode select register
localparam SC_CTRL_38 = 16'h3038; //MIPI debug control
localparam SC_CTRL_39 = 16'h3039; //MIPI lane control

//read 0x9281 chip ID to verify correctness
localparam SC_CHIP_ID_HIGH = 16'h300A; 
localparam SC_CHIP_ID_LOW = 16'h300B;
localparam SC_CHIP_ID = 16'h9281; //chip ID value


localparam MIPI_CTRL_00 = 16'h4800; //MIPI control register 0
localparam MIPI_CTRL_01 = 16'h4802; //MIPI control register 1

localparam PRE_CTRL00 = 16'h5E00; //pre control register - enable debugging

localparam ANA_CORE_2 = 16'h3662; //analog core register 2 - enable debugging, 2-lane and format

// PLL control registers
localparam PLL_CTRL_00 = 16'h0300; 
localparam PLL_CTRL_01 = 16'h0301; 
localparam PLL_CTRL_02 = 16'h0302; 
localparam PLL_CTRL_03 = 16'h0303; 
localparam PLL_CTRL_04 = 16'h0304; 
localparam PLL_CTRL_05 = 16'h0305; 
localparam PLL_CTRL_06 = 16'h0306; 
localparam PLL_CTRL_07 = 16'h0307;
localparam PLL_CTRL_08 = 16'h0308; 
localparam PLL_CTRL_09 = 16'h0309;
localparam PLL_CTRL_0A = 16'h030A;

// Values  
localparam SOFTWARE_STANDBY_MODE = 8'h00; 
localparam STREAMING_MODE = 8'h01; 


enum {IDLE, SEND, WRITE, READ, DONE} state, next_state;
//state machine for I2C communication

//I2C regs
logic mst_scl_in, mst_sda_in, 
       mst_scl_out, mst_sda_out, 
       mst_sda_oe, mst_scl_oe;
logic i2c_busy, i2c_soft_rst, i2c_rxak, 
        i2c_arb_lost, i2c_arb_lost_clr;
logic mst_read, mst_write, mst_write_done, 
        mst_data_out_valid;
logic [7:0] i2c_slave_addr, mst_command_byte, mst_num_bytes;
logic [DATA_WIDTH-1:0] mst_din, mst_data_out;
logic [$clog2(SCRIPT_LEN)-1:0] cmd_index;

localparam I2C_FAST_MODE = 1; //0 - 100 kHz operating speed for i2c, 1 - 400 kHz operating speed for i2c
localparam SPIKE_FILTER_CYCLE = 2; //deglitches SDA and SCL for 2 cycles on inputs. Inputs must stay stable for this num of cycles
localparam SLAVE_ADDR = 8'h00; //set to dummy 0. only useful in SLAVE I2C_MODE
localparam I2C_MODE = "MASTER";

i2c_ov9281_en #(
    .CLOCK_FREQ(CLK_FREQ);
    .DATA_BYTE_WIDTH(DATA_WIDTH);
    .I2C_FAST_MODE(I2C_FAST_MODE); 
    .SPIKE_FILTER_CYCLE(SPIKE_FILTER_CYCLE);
    .SLAVE_ADDR(SLAVE_ADDR);
    .I2C_MODE(I2C_MODE);
) i2c_inst (
    .clk ( i_clk ),
    .rst ( i_rst ),
    .mst_scl_in ( i_scl_in ),
    .mst_sda_in ( i_sda_in ),
    .mst_scl_out ( o_scl_out ),
    .mst_sda_out ( o_sda_out ),
    .mst_sda_oe ( o_sda_oe ),
    .mst_scl_oe ( o_scl_oe ),
    .i2c_busy ( i2c_busy ),
    .i2c_soft_rst ( i2c_soft_rst ),
    .i2c_rxak ( i2c_rxak ),
    .i2c_arb_lost ( i2c_arb_lost ),
    .i2c_arb_lost_clr ( i2c_arb_lost_clr ),
    .i2c_slave_addr ( i2c_slave_addr ),
    .mst_command_byte ( mst_command_byte ),
    .mst_num_bytes ( mst_num_bytes ),
    .mst_read ( mst_read ),
    .mst_write ( mst_write ),
    .mst_write_done ( mst_write_done ),
    .mst_data_out_valid ( mst_data_out_valid ),
    .mst_data_out ( mst_data_out ),
    //assume mst_din  read the lower bytes of the wide bus first
    .mst_din ( mst_din )
);

always_ff @(posedge clk) begin
    if (i_rst) begin
        state <= IDLE;
        cmd_index <= 'd0;
    end else begin
        state <= next_state;
        if (next_state == IDLE) begin 
            cmd_index <= 'd0; 
            //only increment in SEND STATE
        end else if (next_state == SEND) begin 
            cmd_index <= cmd_index + 1; 
        end 
    end
end

always_comb begin: state_decoder
    case (state)
        IDLE: begin
            if (i_start) begin
                if (i2c_busy) begin
                    next_state = IDLE; //stay in idle if I2C is busy
                end else begin
                    next_state = SEND; //move to send address state
                end
            end else begin
                next_state = IDLE;
            end
        end

        SEND: begin
            //change state when busy is asserted
            if (i2c_busy) begin 
                next_state = WAIT;
            end else begin 
                next_state = SEND; //stay in send address state until acknowledged
            end 

        end 
        WAIT: 
            if (write_done) begin 
                //not the end of the list of values
                if (cmd_index == SCRIPT_LEN - 1) begin 
                    next_state = WAIT_DONE;
                end else begin 
                    //go back to sending if this is not ready
                    next_state = SEND;
                end
            end else begin 
                next_state = WAIT; 
            end
        WAIT_DONE:
            if (!i2c_busy) begin 
                if (!i2c_arb_lost && !i2c_rxak) begin 
                    next_state = DONE;
                end else begin 
                    next_state = WAIT_DONE;
                end
            end else begin 
                next_state = WAIT_DONE;
            end 

        DONE:
        default: next_state = IDLE;
    endcase
end

always_comb begin: output_decoder
    case (state)
        IDLE: begin 
            i2c_slave_addr = 16'h0000;
            mst_command_byte = 8'h00; //default command byte
            o_done = 1'b0; 
        end

        SEND: begin 
            reg_write_t current_cmd = script[cmd_index];
            mst_write_done = 1'b0; //not done writing yet
            mst_data_out_valid = 1'b0; //data not valid yet
            if (current_cmd.addr != 16'hFFFF) begin 
                mst_command_byte = current_cmd.addr[15:8]; //high byte
                num_bytes = 8'd3; //higher byte + lower byte + val
                //pack low address byte, data value onto 32-bit din bus
                mst_din <= {16'h0000, current_cmd.val, current_cmd.addr[7:0]};
                //assert r/w signal
                if (read) begin
                    mst_read = 1'b1; //set read flag
                    mst_write = 1'b0; //clear write flag
                    i2c_slave_addr = SLAVE_ADDR_READ;
                end else if (write) begin
                    mst_read = 1'b0; //clear read flag
                    mst_write = 1'b1; //set write flag
                    i2c_slave_addr = SLAVE_ADDR_WRITE; //set write address
                end else begin
                    mst_read = 1'b0; 
                    mst_write = 1'b0; 
                end
            end


        end

        WAIT: begin 

        end

        DONE:
        default:
    endcase
end
