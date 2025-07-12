
module FPGA_HSPI (
	//tx signal
	HTCLK,
	HTREQ,
	HTRDY,
	HTVLD,
	HD,
	tx_act,
	wire_led,
	wire_led_o,
	//rx signal
	HRCLK,
	HRACT,
	HRVLD,
	HTACK,
	//global signal
	rstn,
	clk,
	dat_mod
);

output 	HTCLK;					//tx clock(max:120MHz)	
output 	HTREQ;					//tx request
input	HTRDY;						//tx ready
output	HTVLD;					//tx data valid
output	tx_act;					//tx trigger
output	wire_led;
output	wire_led_o;

input	HRCLK;						//rx clock(max:120MHz)
input	HRACT;						//rx active
input	HRVLD;						//rx data valid
output	HTACK;					//ack to transmitter

inout 	[15:0] HD;			//HSPI data

input	rstn;							//global reset signal, low action
input	clk;							//global clock
input 	[1:0] dat_mod;	//HSPI data mode, 00 - 8bits, 01 - 16bits, 1x - 32bits


//HSPI
wire	[31:0] wire_pif_tdata; 
wire	[2:0] wire_pif_toe;   
wire	[31:0] wire_pif_txin;  
wire	[31:0] wire_pif_rxin; 

//ram signal
wire	ram_clk;	
wire 	ram_csn;	
wire	ram_wen;	
wire	[8:0] ram_addr;	
wire	[31:0]ram_wdata;
wire	[31:0]ram_rdata;	
wire	ram_wren;
wire	ram_rden;
wire	[15:0] wdata;
wire	[15:0] rdata;

wire	wire_clk;
reg		reg_rstn;

//HSPI instance
EXM_HSPI M_HSPI (
	//tx signal
	.HTCLK			( ),//HTCLK				),
	.HTREQ			( HTREQ				),
	.HTRDY			( HTRDY				),
	.HTVLD			( HTVLD				),
	.HTD				( wire_pif_tdata	),
	.HTOE				( wire_pif_toe		),
	.tx_act			( tx_act			),
	//rx signal
	.HRCLK			( HRCLK				),
	.HRACT			( HRACT				),
	.HRVLD			( HRVLD				),
	.HTACK			( HTACK				),
	.HRD				( wire_pif_rxin		),
	//global signal
	.rstn				( reg_rstn			),
	.clk				( wire_clk			),
	.dat_mod		( 2'b01 ),//dat_mod			),
	.ram_clk		( ram_clk			),
	.ram_csn		( ram_csn			),
	.ram_wen		( ram_wen			),
	.ram_addr		( ram_addr			),
	.ram_wdata	( ram_wdata			),
	.ram_rdata	( ram_rdata			)
);

assign ram_wren = ~ram_csn & ~ram_wen;
assign ram_rden = ~ram_csn & ram_wen;
assign ram_rdata = {16'd0, rdata};
assign wdata = ram_wdata[15:0];

//RAM instance (16bits)
SRAM1KB m_ram (
	.address 	( ram_addr 			),
	.clock 		( ram_clk 			),
	.data 		( wdata				),
	.wren 		( ram_wren 			),
	.rden		( ram_rden			),
	.q 			( rdata				)
);

//PLL instance
wire	wire_80m;
PLL pll (
		.inclk0	( clk 			),	// OSC CLK		// 24MHz
		.c0 	( wire_clk 	 	),	// HSPI CLK		// 120MHz, HSPI clock
		.c1 	( wire_80m 	 	),	// HSPI CLK		// 80MHz, Signaltap sample clock (function check)
		.c2 	( HTCLK 	 	)	// HSPI TCLK	// 120MHz, note: clock phase shift 225 deg
);


//GPIO instance
GPIO PIF_D0   ( .O( wire_pif_rxin[0] ),  .I( wire_pif_tdata[0] ),  .IO( HD[0] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D1   ( .O( wire_pif_rxin[1] ),  .I( wire_pif_tdata[1] ),  .IO( HD[1] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D2   ( .O( wire_pif_rxin[2] ),  .I( wire_pif_tdata[2] ),  .IO( HD[2] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D3   ( .O( wire_pif_rxin[3] ),  .I( wire_pif_tdata[3] ),  .IO( HD[3] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D4   ( .O( wire_pif_rxin[4] ),  .I( wire_pif_tdata[4] ),  .IO( HD[4] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D5   ( .O( wire_pif_rxin[5] ),  .I( wire_pif_tdata[5] ),  .IO( HD[5] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D6   ( .O( wire_pif_rxin[6] ),  .I( wire_pif_tdata[6] ),  .IO( HD[6] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D7   ( .O( wire_pif_rxin[7] ),  .I( wire_pif_tdata[7] ),  .IO( HD[7] ),  .E( wire_pif_toe[0] ) );
GPIO PIF_D8   ( .O( wire_pif_rxin[8] ),  .I( wire_pif_tdata[8] ),  .IO( HD[8] ),  .E( wire_pif_toe[1] ) );
GPIO PIF_D9   ( .O( wire_pif_rxin[9] ),  .I( wire_pif_tdata[9] ),  .IO( HD[9] ),  .E( wire_pif_toe[1] ) );
GPIO PIF_D10  ( .O( wire_pif_rxin[10] ), .I( wire_pif_tdata[10] ), .IO( HD[10] ), .E( wire_pif_toe[1] ) );
GPIO PIF_D11  ( .O( wire_pif_rxin[11] ), .I( wire_pif_tdata[11] ), .IO( HD[11] ), .E( wire_pif_toe[1] ) );
GPIO PIF_D12  ( .O( wire_pif_rxin[12] ), .I( wire_pif_tdata[12] ), .IO( HD[12] ), .E( wire_pif_toe[1] ) );
GPIO PIF_D13  ( .O( wire_pif_rxin[13] ), .I( wire_pif_tdata[13] ), .IO( HD[13] ), .E( wire_pif_toe[1] ) );
GPIO PIF_D14  ( .O( wire_pif_rxin[14] ), .I( wire_pif_tdata[14] ), .IO( HD[14] ), .E( wire_pif_toe[1] ) );
GPIO PIF_D15  ( .O( wire_pif_rxin[15] ), .I( wire_pif_tdata[15] ), .IO( HD[15] ), .E( wire_pif_toe[1] ) );

//test
parameter	CNT_VLU = 16'd15000;

reg		[15:0] reg_ms_cnt;
wire	[15:0] wire_ms_cnt_nx;
assign wire_ms_cnt_nx = reg_ms_cnt + 1'b1;
always @(posedge wire_clk or negedge rstn) begin
	if (!rstn) reg_ms_cnt <= 16'd0;
	else if (wire_ms_cnt_nx == CNT_VLU) reg_ms_cnt <= 16'd0;
	else reg_ms_cnt <= wire_ms_cnt_nx;
end

reg		reg_led;
reg		[9:0] reg_base_cnt;
wire	[9:0] wire_base_nx;
assign wire_base_nx = reg_base_cnt + 1'b1;
always @(posedge wire_clk or negedge rstn) begin
	if (!rstn) begin
		reg_base_cnt <= 10'd0;
		reg_led <= 1'b0;
	end
	else if (wire_ms_cnt_nx == CNT_VLU) begin
`ifndef		SIM_TEST
		if (wire_base_nx == 10'd20) begin
`else 
		if (wire_base_nx == 10'd1) begin
`endif
			reg_base_cnt <= 10'd0;
			reg_led <= ~reg_led;
		end
		else reg_base_cnt <= wire_base_nx;
	end
end

assign wire_led = reg_led;
assign wire_led_o = reg_led;

always @(negedge wire_clk or negedge rstn) begin
	if (!rstn) reg_rstn <= 1'b0;
`ifndef		SIM_TEST
	else if ((wire_ms_cnt_nx == CNT_VLU) & (wire_base_nx == 10'd20)) reg_rstn <= 1'b1;
`else
	else if ((wire_ms_cnt_nx == CNT_VLU) & (wire_base_nx == 10'd1)) reg_rstn <= 1'b1;
`endif
end

reg		reg_act;
always @(negedge wire_clk or negedge reg_rstn) begin
	if (!reg_rstn) reg_act <= 1'b0;
`ifndef		SIM_TEST
	else reg_act <= (wire_ms_cnt_nx == CNT_VLU) & (wire_base_nx == 10'd20);
`else
	else reg_act <= (wire_ms_cnt_nx == CNT_VLU) & (wire_base_nx == 10'd1);
`endif
end

wire	wire_tx_req_neg;
reg		reg_tx_req_dly;
always @(posedge wire_clk or negedge reg_rstn) begin
	if (!reg_rstn) reg_tx_req_dly <= 1'b0;
	else reg_tx_req_dly <= HTREQ;
end
assign wire_tx_req_neg = !HTREQ & reg_tx_req_dly;

reg		[7:0] reg_idle_cnt;
always @(posedge wire_clk or negedge reg_rstn) begin
	if (!reg_rstn) reg_idle_cnt <= 8'd0;
	else if (wire_tx_req_neg) reg_idle_cnt <= 8'd0;
	else if (reg_idle_cnt != 8'd50) reg_idle_cnt <= reg_idle_cnt + 1'b1;
end

assign tx_act = reg_idle_cnt==8'd32;	
//assign tx_act = reg_act;	

reg		[7:0] reg_act_cnt/*synthesis noprune*/;
always @(posedge wire_clk or negedge reg_rstn) begin
	if (!reg_rstn) reg_act_cnt <= 8'd0;
	else if(tx_act) reg_act_cnt <= reg_act_cnt + 1'b1;
end

endmodule

module GPIO ( O, I, IO, E ) ;
	output	O;
	input	I;
	inout	IO;
	input	E;

	assign O = IO;
	assign IO = E ? I : 1'bz;
endmodule

