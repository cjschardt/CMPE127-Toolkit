`timescale 1ns / 1ps
//------------------------------------------------
// Source Code for a Single-cycle MIPS Processor (supports partial instruction)
// Developed by D. Hung, D. Herda and G. Gerken,
// based on the following source code provided by
// David_Harris@hmc.edu (9 November 2005):
//    mipstop.v
//    mipsmem.v
//    mips.v
//    mipsparts.v
//------------------------------------------------

// Main Decoder
module maindec(
	input	[ 5:0]	op,
	output			memtoreg, memwrite, branch, alusrc, regdst, regwrite, jump, pcspr, rawr, nbranch,
	output	[ 1:0]	aluop );

	reg 	[ 11:0]	controls;

	assign {nbranch, regwrite, regdst, alusrc, branch, memwrite, memtoreg, jump,  pcspr, rawr, aluop} = controls;

	always @(*)
		case(op)
			6'b000000: controls <= 12'b011000001010; //Rtype
			6'b100011: controls <= 12'b010100100000; //LW
			6'b101011: controls <= 12'b000101000000; //SW
			6'b000100: controls <= 12'b000010000001; //BEQ
			6'b000101: controls <= 12'b100000000001; //BNE
			6'b001000: controls <= 12'b010100001000; //ADDI
			6'b000010: controls <= 12'b000000010000; //J
			6'b000011: controls <= 12'b010000010100; //JAL
			default:   controls <= 12'bxxxxxxxxxxxx; //???
		endcase
endmodule

// ALU Decoder
module aludec(
	input		[5:0]	funct,
	input		[1:0]	aluop,
	output reg	[6:0]	alucontrol );

	always @(*)       //sprwr, sprrd, sprmux, jrmux,
		case(aluop)
			2'b00: alucontrol <= 7'b0000010;  // add
			2'b01: alucontrol <= 7'b0000110;  // sub
			default: case(funct)          // RTYPE
				6'b100000: alucontrol <= 7'b0000010; // ADD
				6'b100010: alucontrol <= 7'b0000110; // SUB
				6'b100100: alucontrol <= 7'b0000000; // AND
				6'b100101: alucontrol <= 7'b0000001; // OR
				6'b101010: alucontrol <= 7'b0000111; // SLT
				6'b011001: alucontrol <= 7'b1000000; //multu
				6'b010000: alucontrol <= 7'b0110000; //mfhi
				6'b010010: alucontrol <= 7'b0010000; //mflo
				6'b001000: alucontrol <= 7'b0001000; //jr
				default:   alucontrol <= 7'b0000000; // ???
			endcase
		endcase
endmodule
// ALU
module alu(
	input		[31:0]	a, b,
	input		[ 2:0]	alucont,
	output reg	[31:0]	result,
	output			zero );

	wire	[31:0]	b2, sum, slt;

	assign b2 = alucont[2] ? ~b:b;
	assign sum = a + b2 + alucont[2];
	assign slt = sum[31];

	always@(*)
		case(alucont[1:0])
			2'b00: result <= a & b;
			2'b01: result <= a | b;
			2'b10: result <= sum;
			2'b11: result <= slt;
		endcase

	assign zero = (result == 32'b0);
endmodule

// Adder
module adder(
	input	[31:0]	a, b,
	output	[31:0]	y );

	assign y = a + b;
endmodule

// Two-bit left shifter
module sl2(
	input	[31:0]	a,
	output	[31:0]	y );

	// shift left by 2
	assign y = {a[29:0], 2'b00};
endmodule

// Sign Extension Unit
module signext(
	input	[15:0]	a,
	output	[31:0]	y );

	assign y = {{16{a[15]}}, a};
endmodule

//multiplier
module mult (input [31:0] a, input [31:0] b, output [63:0] out);
    assign out = a * b;
endmodule

// Parameterized Register
module flopr #(parameter WIDTH = 8) (
	input					clk, reset,
	input		[WIDTH-1:0]	d,
	output reg	[WIDTH-1:0]	q);

	always @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else       q <= d;
endmodule

// commented out since flopenr is not used
//module flopenr #(parameter WIDTH = 8) (
//	input					clk, reset,
//	input					en,
//	input		[WIDTH-1:0]	d,
//	output reg	[WIDTH-1:0]	q);
//
//	always @(posedge clk, posedge reset)
//		if      (reset) q <= 0;
//		else if (en)    q <= d;
//endmodule

// Parameterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8) (
	input	[WIDTH-1:0]	d0, d1,
	input				s,
	output	[WIDTH-1:0]	y );

	assign y = s ? d1 : d0;
endmodule

module mux4 #(parameter WIDTH=8)(
	input wire [WIDTH-1:0] in1,
	input wire [WIDTH-1:0] in2,
    input wire [WIDTH-1:0] in3,
    input wire [WIDTH-1:0] in4,
    input wire [1:0] select,
	output reg [WIDTH-1:0] out
);

initial begin
	out = 0;
end

always @(*) begin
	case(select)
		0: out = in1;
		1: out = in2;
		2: out = in3;
		3: out = in4;
		default: out = 0;
	endcase
end
endmodule

module mux8 #(parameter WIDTH=8)(
	input wire [WIDTH-1:0] in1,
	input wire [WIDTH-1:0] in2,
    input wire [WIDTH-1:0] in3,
    input wire [WIDTH-1:0] in4,
	input wire [WIDTH-1:0] in5,
	input wire [WIDTH-1:0] in6,
    input wire [WIDTH-1:0] in7,
    input wire [WIDTH-1:0] in8,
    input wire [2:0] select,
	output reg [WIDTH-1:0] out
);

initial begin
	out = 0;
end

always @(*) begin
	case(select)
		0: out = in1;
		1: out = in2;
		2: out = in3;
		3: out = in4;
		4: out = in5;
		5: out = in6;
		6: out = in7;
		7: out = in8;
		default: out = 0;
	endcase
end
endmodule

module register #(parameter WIDTH=8)(
	input wire clk,
	input wire [WIDTH-1:0] in,
    input wire enable,
	output reg [WIDTH-1:0] out
);

initial begin
	out = 0;
end

always @(posedge clk) begin
	if(enable) begin
		out = in;
	end
end

endmodule

module srlatch (
	input wire clk,
	input wire s,
    input wire r,
	output reg out
);

initial begin
	out = 0;
end

always @(posedge clk) begin
	if(s & !r) begin
		out = 1;
	end
	else if(!s & r) begin
		out = 0;
	end
	// else if() <- add other conditions
end

endmodule

// register file with one write port and three read ports
// the 3rd read port is for prototyping dianosis
module regfile(
	input			clk,
	input			we3,
	input 	[ 4:0]	ra1, ra2, wa3,
	input	[31:0] 	wd3,
	output 	[31:0] 	rd1, rd2,
	input	[ 4:0] 	ra4,
	output 	[31:0] 	rd4);

	reg		[31:0]	rf[31:0];
	integer			n;

	//initialize registers to all 0s
	initial begin
		for (n=0; n<32; n=n+1) begin
			rf[n] = 32'h00;
		end
		rf[29] = 63;
	end

	//write first order, include logic to handle special case of $0
    always @(posedge clk)
        if (we3)
			if (~ wa3[4])
				rf[{27'b0,wa3[3:0]}] <= wd3;
			else
				rf[{27'b1,wa3[3:0]}] <= wd3;

			// this leads to 72 warnings
			//rf[wa3] <= wd3;

			// this leads to 8 warnings
			//if (~ wa3[4])
			//	rf[{0,wa3[3:0]}] <= wd3;
			//else
			//	rf[{1,wa3[3:0]}] <= wd3;

	assign rd1 = (ra1 != 0) ? rf[ra1[4:0]] : 0;
	assign rd2 = (ra2 != 0) ? rf[ra2[4:0]] : 0;
	assign rd4 = (ra4 != 0) ? rf[ra4[4:0]] : 0;
endmodule

module spreg(
	input			clk,
	input			we,
	input 		    re,
	input	[63:0] 	wd,
	output 	[31:0] 	rd);

	reg [31:0] rf [1:0];
	integer			n;

	//initialize registers to all 0s
	initial
		for (n=0; n<2; n=n+1)
			rf[n] = 32'h00;

	//write first order, include logic to handle special case of $0
    always @(posedge clk)
        if (we) begin
			rf[1] <= wd[63:32];
			rf[0] <= wd[31:0];
	    end

			// this leads to 72 warnings
			//rf[wa3] <= wd3;

			// this leads to 8 warnings
			//if (~ wa3[4])
			//	rf[{0,wa3[3:0]}] <= wd3;
			//else
			//	rf[{1,wa3[3:0]}] <= wd3;

	assign rd = re ? rf[1] : rf[0];
endmodule

// Control Unit
module controller(
	input	[5:0]	op, funct,
	input			zero,
	output			memtoreg, memwrite, pcsrc, alusrc, regdst, regwrite, jump, sprwr, sprrd, sprmux, jrmux, pcspr, rawr,
	output	[2:0]	alucontrol );

	wire	[1:0]	aluop;
	wire			branch, nbranch;

	maindec	md(op, memtoreg, memwrite, branch, alusrc, regdst, regwrite, jump, pcspr, rawr, nbranch, aluop);
	aludec	ad(funct, aluop, {sprwr, sprrd, sprmux, jrmux, alucontrol});

	assign pcsrc = (branch & zero || nbranch & ~zero);
endmodule

// Data Path (excluding the instruction and data memories)
module datapath(
	input			clk, reset, memtoreg, pcsrc, alusrc, regdst, regwrite, jump, sprwr, sprrd, sprmux, jrmux, pcspr, rawr,
	input	[2:0]	alucontrol,
	output			zero,
	output	[31:0]	pc,
	input	[31:0]	instr,
	output	[31:0]	aluout, writedata,
	input	[31:0]	readdata,
	input	[ 4:0]	dispSel,
	output	[31:0]	dispDat );

	wire [4:0]  writereg, writerega;
	wire [31:0] pcnext, pcnext2, pcnextbr, pcplus4, pcbranch, signimm, signimmsh, srca, srcb, result, pcresult, mux2reg, spr2mux, hold;
    wire [63:0] multspr;
z
	// next PC logic
	flopr #(32) pcreg(clk, reset, pcnext2, pc);
	adder       pcadd1(pc, 32'b100, pcplus4);
	sl2         immsh(signimm, signimmsh);
	adder       pcadd2(pcplus4, signimmsh, pcbranch);
	mux2 #(32)  pcbrmux(pcplus4, pcbranch, pcsrc, pcnextbr);
	assign hold = {pcplus4[31:28], instr[25:0], 2'b00};
	mux2 #(32)  pcmux(pcnextbr, hold, jump, pcnext);
	mux2 #(32)  pcrg(pcnext, srca, jrmux, pcnext2);

	// register file logic
	regfile		rf(clk, regwrite, instr[25:21], instr[20:16], writerega, pcresult, srca, writedata, dispSel, dispDat);
	mux2 #(5)	wrmux(instr[20:16], instr[15:11], regdst, writereg);
	mux2 #(5)   wrmuxra(writereg, 5'b11111, rawr, writerega);
	mux2 #(32)	resmux(pcplus4, readdata, memtoreg, result);
	mux2 #(32)  pcres(result, mux2reg, pcspr, pcresult);
	mux2 #(32)  pcspr1(aluout, spr2mux, sprmux, mux2reg);
	signext		se(instr[15:0], signimm);

	//spregister
	spreg       spr(clk, sprwr, sprrd, multspr, spr2mux);

	//mult
	mult        mul(srca, writedata, multspr);

	// ALU logic
	mux2 #(32)	srcbmux(writedata, signimm, alusrc, srcb);
	alu			alu(srca, srcb, alucontrol, aluout, zero);
endmodule

// The MIPS (excluding the instruction and data memories)
module mips(
	input        	clk, reset,
	output	[31:0]	pc,
	input 	[31:0]	instr,
	output			memwrite,
	output	[31:0]	aluout, writedata,
	input 	[31:0]	readdata,
	input	[ 4:0]	dispSel,
	output	[31:0]	dispDat );

	// deleted wire "branch" - not used
	wire 			memtoreg, pcsrc, zero, alusrc, regdst, regwrite, jump, sprwr, sprrd, sprmux, jrmux, pcspr, rawr;
	wire	[2:0] 	alucontrol;

	controller c(instr[31:26], instr[5:0], zero,
				memtoreg, memwrite, pcsrc,
				alusrc, regdst, regwrite, jump,
				sprwr, sprrd, sprmux, jrmux, pcspr,
				rawr, alucontrol);
	datapath dp(clk, reset, memtoreg, pcsrc,
				alusrc, regdst, regwrite, jump,
				sprwr, sprrd, sprmux, jrmux, pcspr, rawr,
				alucontrol, zero, pc, instr, aluout,
				writedata, readdata, dispSel, dispDat);
endmodule

// Instruction Memory
module imem (
	input	[ 5:0]	a,
	output 	[31:0]	dOut );

	reg		[31:0]	rom[0:63];

	//initialize rom from memfile_s.dat
	initial
		$readmemh("memfile_s.dat", rom);

	//simple rom
    assign dOut = rom[a];
endmodule

// Data Memory
module dmem (
	input			clk,
	input			we,
	input	[31:0]	addr,
	input	[31:0]	dIn,
	output 	[31:0]	dOut );

	reg		[31:0]	ram[63:0];
	integer			n;

	//initialize ram to all FFs
	initial
		for (n=0; n<64; n=n+1)
			ram[n] = 8'hFF;

	assign dOut = ram[addr[31:2]];

	always @(posedge clk)
		if (we)
			ram[addr[31:2]] = dIn;
endmodule