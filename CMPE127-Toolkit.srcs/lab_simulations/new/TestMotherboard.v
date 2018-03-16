`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2018 04:15:49 PM
// Design Name: 
// Module Name: TestMotherboard
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module TestMotherboard;

integer clock_count;
reg clk;
reg rst;
reg ps2_clk;
reg ps2_data;

wire hsync;
wire vsync;
wire [3:0] r;
wire [3:0] g;
wire [3:0] b;

Motherboard #(.CLOCK_DIVIDER(1)) motherboard(
	//// input 100 MHz clock
    .clk100Mhz(clk),
    .rst(rst),
    .ps2_clk(ps2_clk),
    .ps2_data(ps2_data),
    //// Horizontal sync pulse for VGA controller
    .hsync(hsync),
    //// Vertical sync pulse for VGA controller
    .vsync(vsync),
    //// RGB 4-bit singal that go to a DAC (range 0V <-> 0.7V) to generate a color intensity
    .r(r),
    .g(g),
    .b(b),
    .clk_select(1'b1),
    .button_clock(1'b1)
);

task RESET;
begin
    rst = 0;
    clock_count = 0;
    clk = 0;
    ps2_clk = 0;
    ps2_data = 0;
    #5
    rst = 1;
    #5
    rst = 0;
    clk = 0;
end
endtask;

task CLOCK;
	input [31:0] count;
	integer k;
begin
	for (k=0; k < count; k = k+1)
	begin
		#5
		clk = 1;
		clock_count = clock_count + 1;
		#5
		clk = 0;
	end
end
endtask

parameter FULL_CYCLE = 32'd3000;

initial begin
    #10
    #10
	RESET;
	CLOCK(FULL_CYCLE);

    #10 $stop;
    #5 $finish;
end

endmodule

