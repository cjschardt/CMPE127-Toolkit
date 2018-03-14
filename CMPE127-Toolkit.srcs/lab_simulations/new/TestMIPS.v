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

module TestMIPS;

reg clk;
reg rst;

wire [31:0] AddressBus, DataBus, ProgramCounter, Instruction;
wire BusCycle;

MIPS mips(
    .clk(clk),
    .rst(rst),
    .BusCycle(BusCycle),
    .AddressBus(AddressBus),
    .DataBus(DataBus),
    .ProgramCounter(ProgramCounter),
    .Instruction(Instruction)
);

task RESET;
begin
    rst = 0;
    clk = 0;
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
		#5
		clk = 0;
	end
end
endtask

parameter FULL_CYCLE = 32'd60;

initial begin
    #10
    #10
	RESET;
	CLOCK(FULL_CYCLE);

    #10 $stop;
    #5 $finish;
end

endmodule

