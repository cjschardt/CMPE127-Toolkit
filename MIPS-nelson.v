`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2018 01:45:39 PM
// Design Name: 
// Module Name: MIPS
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

`define OPCODE_WIDTH 6
`define ALU_OPCODE_WIDTH 2
`define REGISTER_WIDTH 32
`define ALU_CONTROL_WIDTH 3

module controller(
	input wire	[5:0]	op, funct,
	input wire			zero,
	output wire			memtoreg, memwrite, pcsrc, alusrc, regdst, regwrite, jump,
	output wire	[2:0]	alucontrol,
	output wire			hilosrc, hilowrite,
	output wire	[1:0]	execsrc, 
	output wire			jumpsrc, ra_we,
	output wire			shamtsrc,
	output wire	[1:0]	shiftcontrol
);

	wire	[1:0]	aluop;
	wire			branch;
	wire			jumpmain;

	maindec	md(
        .op(op),
	    .memtoreg(memtoreg), 
        .memwrite(memwrite), 
        .branch(branch), 
        .alusrc(alusrc), 
        .regdst(regdst), 
        .regwrite(regwrite), 
        .jump(jumpmain), 
	    .aluop(aluop),
	    .ra_we(ra_we)
    );
	aludec	ad(funct, aluop, alucontrol,
			   hilosrc, hilowrite, execsrc,
			   jumpsrc,
			   shamtsrc, shiftcontrol);

	assign pcsrc = branch & zero;
	assign jump = jumpmain | jumpsrc;
endmodule

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
	input wire [ 5:0] op,
	output wire	memtoreg, 
    output wire memwrite, 
    output wire branch, 
    output wire alusrc, 
    output wire regdst, 
    output wire regwrite, 
    output wire jump,
	output wire	[ 1:0] aluop,
	output wire	ra_we
);

reg 	[ 9:0]	controls;

assign {regwrite, regdst, alusrc, branch, memwrite, memtoreg, jump, aluop, ra_we} = controls;

always @(*)
begin
    case(op)
        6'b000000: controls <= 10'b1100000100; //Rtype
        6'b100011: controls <= 10'b1010010000; //LW
        6'b101011: controls <= 10'b0010100000; //SW
        6'b100011: controls <= 10'b1010010000; //LB
        6'b101011: controls <= 10'b0010100000; //SB
        6'b000100: controls <= 10'b0001000010; //BEQ
        6'b001000: controls <= 10'b1010000000; //ADDI
        6'b000010: controls <= 10'b0000001000; //J
        6'b000011: controls <= 10'b0000001001; //JAL
        6'b001111: controls <= 10'b1010000110; //LUI
        default:   controls <= 10'bxxxxxxxxxxx; //???
    endcase
end
endmodule

// ALU Decoder
module aludec(
	input wire	[5:0]	funct,
	input wire	[1:0]	aluop,
	output wire	[2:0]	alucontrol,
	output wire			hilosrc, hilowrite,
	output wire	[1:0]	execsrc,
	output wire			jumpsrc,
	output wire			shamtsrc,
	output wire	[1:0]	shiftcontrol
);

	reg 	[10:0] controls;
	
	assign {alucontrol, hilosrc, hilowrite, execsrc, jumpsrc, shamtsrc, shiftcontrol} = controls;
	
	always @(*)
		case(aluop)
			2'b00: controls <= 11'b01000000000;  // add
			2'b01: controls <= 11'b11000000000;  // sub
			2'b11: controls <= 11'b01100000000;  // lui
			default: case(funct)          // RTYPE
				6'b100000: controls <= 11'b01000000000; // ADD
				6'b100010: controls <= 11'b11000000000; // SUB
				6'b100100: controls <= 11'b00000000000; // AND
				6'b100101: controls <= 11'b00100000000; // OR
				6'b101010: controls <= 11'b11100000000; // SLT
				6'b011000: controls <= 11'b00001000000; // MULT, 18
				6'b011010: controls <= 11'b00011000000; // DIV , 1a
				6'b010000: controls <= 11'b00000010000; // MFHI, 10
				6'b010010: controls <= 11'b00000100000; // MFLO, 12
				6'b001000: controls <= 11'b00000001000; // JR  , 08
				6'b100111: controls <= 11'b10000000000; // NOR , 27
				6'b100110: controls <= 11'b10100000000; // XOR , 26
				6'b000000: controls <= 11'b00000110000; // SLL , 00
				6'b000010: controls <= 11'b00000110001; // SRL , 02
				6'b000011: controls <= 11'b00000110010; // SRA , 03
				6'b000100: controls <= 11'b00000110100; // SLLV, 04
				6'b000110: controls <= 11'b00000110101; // SRLV, 06
				6'b000111: controls <= 11'b00000110110; // SRAV, 07
				default:   controls <= 11'bxxxxxxxxxxx; // ???
			endcase
		endcase
endmodule


//-----------------------------------------------------------------
// Module Name   : clk_gen
// Description   : Generate 4 second and 5KHz clock cycle from
//                 the 50MHz clock on the Nexsys2 board
//------------------------------------------------------------------
module clk_gen(
	input wire			clk50MHz, reset, 
	output reg		clksec
);

	reg 			clk_5KHz;
	integer 		count, count1;
	
	always@(posedge clk50MHz) begin
		if(reset) begin
			count = 0;
			count1 = 0;
			clksec = 0;
			clk_5KHz =0;
		end else begin
			if (count == 50000000) begin
				// Just toggle after certain number of seconds
				clksec = ~clksec;
				count = 0;
			end
			if (count1 == 20000) begin
				clk_5KHz = ~clk_5KHz;
				count1 = 0;
			end
			count = count + 1;
			count1 = count1 + 1;
		end
	end
endmodule

// datapath_submodules.v

// ALU
module alu(
	input wire		[31:0]	a, b, 
	input wire		[ 2:0]	alucont, 
	output reg	[31:0]	result,
	output wire			zero
);

	wire	[31:0]	b2, sum, slt, lui;

	assign b2 = alucont[2] ? ~b:b; 
	assign sum = a + b2 + alucont[2];
	assign slt = sum[31];
	assign lui = {b[15:0], 16'b0};

	always@(*)
		case(alucont[2:0])
			3'b000: result <= a & b;
			3'b001: result <= a | b;
			3'b010: result <= sum;
			3'b011: result <= lui;
			3'b100: result <= ~a & ~b;
			3'b101: result <= a ^ b;
			3'b110: result <= sum;
			3'b111: result <= slt;
		endcase

	assign zero = (result == 32'b0);
endmodule

// Adder
module adder(
	input wire	[31:0]	a, b,
	output wire	[31:0]	y
);

	assign y = a + b;
endmodule

// Two-bit left shifter
module sl2(
	input wire	[31:0]	a,
	output wire	[31:0]	y
);

	// shift left by 2
	assign y = {a[29:0], 2'b00};
endmodule

// Sign Extension Unit
module signext(
	input wire	[15:0]	a,
	output wire	[31:0]	y
);

	assign y = {{16{a[15]}}, a};
endmodule

// Parameterized Register
module flopr #(parameter WIDTH = 8) (
	input wire					clk, reset,
	input wire		[WIDTH-1:0]	d, 
	output reg	[WIDTH-1:0]	q
);

	always @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else       q <= d;
endmodule

// Previously commented out since flopenr was not used
// Parameterized Register with Enable
module flopenr #(parameter WIDTH = 8) (
	input wire					clk, reset,
	input wire					en,
	input wire		[WIDTH-1:0]	d, 
	output reg	[WIDTH-1:0]	q
);

	always @(posedge clk, posedge reset)
		if      (reset) q <= 0;
		else if (en)    q <= d;
endmodule

// Parameterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8) (
	input wire	[WIDTH-1:0]	d0, d1, 
	input wire				s, 
	output wire	[WIDTH-1:0]	y
);

	assign y = s ? d1 : d0; 
endmodule

// Parameterized 4-to-1 MUX
module mux4 #(parameter WIDTH = 8) (
	input wire	[WIDTH-1:0] d0, d1, d2, d3,
	input wire	[1:0]		s,
	output reg [WIDTH-1:0] y
);
	
	always @ (*)
	begin
		case(s)
			2'b00: y <= d0;
			2'b01: y <= d1;
			2'b10: y <= d2;
			default: y <= d3;
		endcase
	end
endmodule

// register file with one write port and three read ports
// the 3rd read port is for prototyping dianosis
module regfile(	
	input wire			clk, 
	input wire			we3, 
	input wire 	[ 4:0]	ra1, ra2, wa3, 
	input wire	[31:0] 	wd3, 
	output wire 	[31:0] 	rd1, rd2,
	input wire	[ 4:0] 	ra4,
	output wire 	[31:0] 	rd4,
	input wire			ra_we,
	input wire	[31:0]	ra_wd
);

	reg		[31:0]	rf[0:31];
	integer			n;
	
	//initialize registers to all 0s
	initial 
		for (n=0; n<32; n=n+1) 
			rf[n] = 32'h00;
			
	//write first order, include logic to handle special case of $0
    always @(posedge clk)
	begin
        if (we3)
			if (~ wa3[4])
				rf[{1'b0,wa3[3:0]}] <= wd3;
			else
				rf[{1'b1,wa3[3:0]}] <= wd3;
		
			// this leads to 72 warnings
			//rf[wa3] <= wd3;
			
			// this leads to 8 warnings
			//if (~ wa3[4])
			//	rf[{0,wa3[3:0]}] <= wd3;
			//else
			//	rf[{1,wa3[3:0]}] <= wd3;
		if (ra_we)
			rf[31] <= ra_wd;
	end
		
	assign rd1 = (ra1 != 0) ? rf[ra1[4:0]] : 0;
	assign rd2 = (ra2 != 0) ? rf[ra2[4:0]] : 0;
	assign rd4 = (ra4 != 0) ? rf[ra4[4:0]] : 0;
endmodule

// Comparator
module compare(
	input wire	[31:0]	a,
	input wire	[31:0]	b,
	output wire			y
);
	assign y = (a == b) ? 1 : 0;
endmodule

// datapath_shift.v
module shift( 
	input wire signed [31:0] in,
	input wire [4:0] shamt,
	input wire [1:0] shiftcontrol,
	output reg [31:0] out
);
always @(*)
begin
	case(shiftcontrol)
		2'b00: out = in << shamt;	//SLL or SLLV
		2'b01: out = in >> shamt;	//SRL or SRLV
		2'b10: out = in >>> shamt;	//SRA or SRAV
	endcase
end
endmodule

// Multiplier
module mult(
	input wire	[31:0]	 a, b,
	output wire	[31:0]	hi,lo
);
	
	assign {hi,lo} = a * b;
endmodule

// Divider placeholder for synthesis
/*
module div(
	input wire [31:0]	a, b,
	output wire [31:0]	hi, lo
);
	
	assign {hi, lo} = {a, b};
endmodule
*/

// Divider, only if implied division is allowed
module div(
	input wire	[31:0]	 a, b,
	output wire	[31:0]	hi,lo
);
	
	assign lo = a / b;
	assign hi = a % b;
endmodule
/*
// Divider, when implied division is not allowed
module div(
	input wire		[31:0]	dividend, divisor,			// a, b
	output wire		[31:0]	remainder, quotient
);		// hi, lo
	wire		[31:0]	dividendWire, divisorWire;
	wire		[ 4:0]	dividendBitCount, divisorBitCount;
	
	div_sl dividend_sl(dividend, dividendWire, dividendBitCount);
	div_sl divisor_sl(divisor, divisorWire, divisorBitCount);
	divider divider(dividendWire, divisorWire, dividendBitCount, divisorBitCount, remainder, quotient);
endmodule

// Division Input Shifter
module div_sl(
	input wire		[31:0]	in,
	output reg	[31:0]	out,
	output reg	[ 4:0]	bitcount
);
	integer				N;
	
	always @ (in)
	begin
		bitcount = 5'bX;
		for (N = 0; N <= 31; N = N + 1)
			if (in[N])
			begin
				bitcount = N;
			end
		out = in << (31 - bitcount);
	end
endmodule

// Divider helper module
module divider(
	input wire		[31:0]	dividend, divisor,
	input wire		[ 4:0]	dividendBitCount, divisorBitCount,
	output reg	[31:0]	remainder, quotient
);
	integer				N, nosub;
	reg			[31:0]	dividendReg;
	
	always @ (*)
	begin
		dividendReg = dividend;
		remainder = 0;
		quotient = 0;
		nosub = 0;
		if (dividendBitCount < divisorBitCount)
			begin
				remainder[31:31-dividendBitCount] = dividendReg[31:31-dividendBitCount];
			end
		else
			begin
				for (N = 31; N >= (dividendBitCount - divisorBitCount + 31); N = N - 1)
				begin
					if (divisor[31:31-divisorBitCount] > dividendReg[nosub+N:N-divisorBitCount]) //no subtract
					begin
						remainder[31:31-divisorBitCount] = dividendReg[N:N-divisorBitCount];
						nosub = nosub + 1;
						quotient[N - dividendBitCount + divisorBitCount] = 0;
					end
					else 																	  //subtract
					begin
						dividendReg[nosub+N:N-divisorBitCount] = dividendReg[nosub+N:N-divisorBitCount] - divisor[31:31-divisorBitCount];
						remainder[31:31-divisorBitCount] = dividendReg[N:N-divisorBitCount];
						nosub = 0;
						quotient[N - dividendBitCount + divisorBitCount] = 1;
					end
				end
			end
	end
endmodule
*/

module fetch(
	input wire clk,
	input wire reset,
    input wire pcsrc,
    input wire jumpsrc,
    input wire jump,
	input wire [31:0] pcbranch,
	input wire [31:0] jumpimm, //jumpimm = {pcplus4[31:28],instr[25:0],2'b00}
	input wire [31:0] srca,
	output wire	[31:0] pc,
    output wire	[31:0] pcplus4
); 
	wire	[31:0]	pcnextbr;
	wire	[31:0]	pcjump;
	wire	[31:0]	pcnext;

	adder       	pcadd1	(	.a(pc[31:0]), 
								.b(32'b100), 
								.y(pcplus4[31:0]));
	mux2 #(32)  	pcbrmux	(	.d0(pcplus4[31:0]), 
								.d1(pcbranch[31:0]), 
								.s(pcsrc), 
								.y(pcnextbr[31:0]));
	mux2 #(32)		pcjmux	(	.d0(jumpimm[31:0]), 
								.d1(srca[31:0]), 
								.s(jumpsrc), 
								.y(pcjump[31:0]));
	mux2 #(32)  	pcmux	(	.d0(pcnextbr[31:0]), 
								.d1(pcjump[31:0]), 
								.s(jump), 
								.y(pcnext[31:0]));
	flopr #(32)		pcreg	(	.clk(clk),
								.reset(reset),
								.d(pcnext[31:0]),
								.q(pc[31:0]));
endmodule

module decode(
	input wire		clk,
					reset,
					regwrite,
					ra_we,
	input wire	[31:0]	instr,
	input wire	[ 4:0]	writereg,
	input wire	[31:0]	result,
	input wire	[31:0]	pcplus4,
	output wire			equal,
	output wire	[31:0]	srca,
					writedata,
	output wire	[ 4:0]	rt,
					rd,
	output wire	[31:0]	signimm,
	output wire	[ 4:0]	instrshamt,		//instr[10:6] = instrshamt[4:0]
	output wire	[31:0]	pcbranch,
	input wire	[ 4:0]	dispSel,
	output wire	[31:0]	dispDat
);
	assign rt[4:0] = instr[20:16];
	assign rd[4:0] = instr[15:11];
	assign instrshamt[4:0] = instr[10:6];
	wire	[31:0]	signimmsh;
	regfile			rf		(	.clk(clk), 
								.we3(regwrite), 
								.ra1(instr[25:21]), 
								.ra2(instr[20:16]), 
								.wa3(writereg[4:0]), 
								.wd3(result[31:0]), 
								.rd1(srca[31:0]), 
								.rd2(writedata[31:0]), 
								.ra4(dispSel), 
								.rd4(dispDat), 
								.ra_we(ra_we), 
								.ra_wd(pcplus4[31:0]));
	signext			se		(	.a(instr[15:0]), 
								.y(signimm[31:0]));
	compare			compare	(	.a(srca[31:0]),
								.b(writedata[31:0]),
								.y(equal));
	sl2         	immsh	(	.a(signimm[31:0]), 
								.y(signimmsh[31:0]));
	adder       	pcadd2	(	.a(pcplus4[31:0]), 
								.b(signimmsh[31:0]), 
								.y(pcbranch[31:0]));
endmodule

module execute(
	input wire			clk,
					reset,
	input wire	[ 2:0]	alucontrol,
	input wire			alusrc,
					regdst,
					hilosrc,
					hilowrite,
					shamtsrc,
	input wire	[ 1:0]	shiftcontrol,
	input wire	[ 1:0]	execsrc,
	input wire	[31:0]	srca,
	input wire	[31:0]	writedata,
	input wire	[ 4:0]	instrshamt,
	input wire	[ 4:0]	rt,
					rd,
	input wire	[31:0]	signimm,
	output wire			zero,
	output wire	[31:0]	execout,
	output wire	[ 4:0]	writereg
);
	wire	[31:0]	srcb,
					aluout,
					divhi,
					divlo,
					multhi,
					multlo,
					hi,
					lo,
					hiout,
					loout;
	wire	[ 4:0]	shamt;
	wire	[31:0]	shiftout;	
	mux2 #(32)		srcbmux	(	.d0(writedata[31:0]), 
								.d1(signimm[31:0]), 
								.s(alusrc), 
								.y(srcb[31:0]));
	alu				alu		(	.a(srca[31:0]), 
								.b(srcb[31:0]), 
								.alucont(alucontrol[2:0]), 
								.result(aluout[31:0]), 
								.zero(zero));
	mult			mult	(	.a(srca[31:0]), 
								.b(srcb[31:0]), 
								.hi(multhi[31:0]), 
								.lo(multlo[31:0]));
	div				div		(	.a(srca[31:0]), 
								.b(srcb[31:0]), 
								.hi(divhi[31:0]), 
								.lo(divlo[31:0]));
	mux2 #(32)		himux	(	.d0(multhi[31:0]), 
								.d1(divhi[31:0]), 
								.s(hilosrc), 
								.y(hi[31:0]));
	mux2 #(32)		lomux	(	.d0(multlo[31:0]), 
								.d1(divlo[31:0]), 
								.s(hilosrc), 
								.y(lo[31:0]));
	flopenr #(32)	hireg	(	.clk(clk), 
								.reset(reset), 
								.en(hilowrite), 
								.d(hi[31:0]), 
								.q(hiout[31:0]));
	flopenr #(32)	loreg	(	.clk(clk), 
								.reset(reset), 
								.en(hilowrite), 
								.d(lo[31:0]), 
								.q(loout[31:0]));
	mux2 #(5)		shiftmux(	.d0(instrshamt[4:0]), 
								.d1(srca[4:0]), 
								.s(shamtsrc), 
								.y(shamt[4:0]));
	shift			shift	(	.in(srcb[31:0]), 
								.shamt(shamt[4:0]), 
								.shiftcontrol(shiftcontrol[1:0]), 
								.out(shiftout[31:0]));
	mux4 #(32)		execmux	(	.d0(aluout[31:0]), 
								.d1(hiout[31:0]), 
								.d2(loout[31:0]), 
								.d3(shiftout[31:0]), 
								.s(execsrc[1:0]), 
								.y(execout[31:0]));
	mux2 #(5)		wrmux	(	.d0(rt[4:0]), 
								.d1(rd[4:0]), 
								.s(regdst), 
								.y(writereg[4:0]));
endmodule

// Data Path (excluding the instruction and data memories)
module datapath(
	input wire			clk, 
	input wire			reset, 
	//
	input wire			alusrcD, 
	input wire	[2:0]	alucontrolD,
	input wire			hilosrcD, 
	input wire			hilowriteD,
	input wire			shamtsrcD,
	input wire	[ 1:0]	shiftcontrolD,
	input wire	[ 1:0]	execsrcD,
	input wire			regdstD, 
	//
	input wire			memwriteD,
	output wire			memwriteM,
	//
	input wire			regwriteD, //regwrite enable
	input wire			memtoregD, //result mux
	//
	input wire			pcsrcD, 
	input wire			jumpsrcD, 
	input wire			jumpD,
	input wire			ra_weD,
	//
	input wire	[31:0]	instrF, 
	output wire	[31:0]	instrD,
	input wire	[31:0]	readdataM,
	output wire	[31:0]	pcF,
	output wire	[31:0]	writedataM,
	output wire			equalD,
	output wire	[31:0]	execoutM, 
	input wire	[ 4:0]	dispSel,
	output wire	[31:0]	dispDat
);
	
	wire		alusrcE; 
	wire [2:0]	alucontrolE;
	wire		hilosrcE; 
	wire		hilowriteE;
	wire		shamtsrcE;
	wire [1:0]	shiftcontrolE;
	wire [1:0]	execsrcE;
	wire		regdstE;
	wire		regwriteE, regwriteM, regwriteW;
	wire		memtoregE, memtoregM, memtoregW;
	wire		memwriteE;
	wire		pcsrcE; 
	wire		jumpsrcE; 
	wire		jumpE;
	
	wire [31:0] instrE;
	wire [31:0] srcaD, srcaE;
	wire [31:0] writedataD, writedataE;
	wire [31:0] signimmD, signimmE;
	wire [31:0] pcplus4F, pcplus4D;
	wire [31:0] pcbranchD;
	wire [31:0] execoutE, execoutW;
	wire [31:0] readdataW;
	wire [31:0] resultW;
	wire [ 4:0] rtD, rtE;
	wire [ 4:0] rdD, rdE;
	wire [ 4:0]	writeregE, writeregM, writeregW;
	wire [ 4:0] instrshamtD, instrshamtE;
	wire		zeroE; //Deprecated
	
	fetch			fetch	(	.clk(clk),
								.reset(reset),
								.pcsrc(pcsrcD),
								.jumpsrc(jumpsrcD),
								.jump(jumpD),
								.pcbranch(pcbranchD[31:0]),
								.jumpimm({pcplus4D[31:28],instrD[25:0],2'b00}),
								.srca(srcaD[31:0]),
								.pc(pcF[31:0]),
								.pcplus4(pcplus4F[31:0])
                            ); 
	flopr #(64)		fdreg	(	.clk(clk),
								.reset(reset),
								.d({instrF[31:0],
									pcplus4F[31:0]}),
								.q({instrD[31:0],
									pcplus4D[31:0]})
                                );
	decode			decode	(	.clk(clk),
								.reset(reset),
								.regwrite(regwriteW),
								.ra_we(ra_weD),
								.instr(instrD[31:0]),
								.writereg(writeregW[4:0]),
								.result(resultW[31:0]),
								.pcplus4(pcplus4D[31:0]),
								.equal(equalD),
								.srca(srcaD[31:0]),
								.writedata(writedataD[31:0]),
								.rt(rtD[4:0]),
								.rd(rdD[4:0]),
								.signimm(signimmD[31:0]),
								.instrshamt(instrshamtD[4:0]),
								.pcbranch(pcbranchD[31:0]),
								.dispSel(dispSel[4:0]),
								.dispDat(dispDat[31:0])
                            );
	flopr #(129)		dereg	(	.clk(clk),
									.reset(reset),
									.d({alusrcD, 
										alucontrolD[2:0],
										hilosrcD, 
										hilowriteD,
										shamtsrcD,
										shiftcontrolD[1:0],
										execsrcD[1:0],
										regdstD,
										memwriteD,
										regwriteD,
										memtoregD,
										pcsrcD, 
										jumpsrcD, 
										jumpD,
										srcaD[31:0],
										writedataD[31:0],
										signimmD[31:0],
										rtD[4:0],
										rdD[4:0],
										instrshamtD[4:0]}),
									.q({alusrcE, 
										alucontrolE[2:0],
										hilosrcE, 
										hilowriteE,
										shamtsrcE,
										shiftcontrolE[1:0],
										execsrcE[1:0],
										regdstE,
										memwriteE,
										regwriteE,
										memtoregE,
										pcsrcE, 
										jumpsrcE, 
										jumpE,
										srcaE[31:0],
										writedataE[31:0],
										signimmE[31:0],
										rtE[4:0],
										rdE[4:0],
										instrshamtE[4:0]})
                                    );
	execute			execute	(	.clk(clk),
								.reset(reset),
								.alucontrol(alucontrolE[2:0]),
								.alusrc(alusrcE),
								.regdst(regdstE),
								.hilosrc(hilosrcE),
								.hilowrite(hilowriteE),
								.shamtsrc(shamtsrcE),
								.shiftcontrol(shiftcontrolE[1:0]),
								.execsrc(execsrcE[1:0]),
								.srca(srcaE[31:0]),
								.writedata(writedataE[31:0]),
								.instrshamt(instrshamtE[4:0]),
								.rt(rtE[4:0]),
								.rd(rdE[4:0]),
								.signimm(signimmE[31:0]),
								.zero(zeroE),
								.execout(execoutE[31:0]),
								.writereg(writeregE[4:0])
                            );
	flopr #(72)		emreg	(	.clk(clk),
								.reset(reset),
								.d({memwriteE,
									regwriteE,
									memtoregE,
									execoutE[31:0],
									writedataE[31:0],
									writeregE[4:0]}),
								.q({memwriteM,
									regwriteM,
									memtoregM,
									execoutM[31:0],
									writedataM[31:0],
									writeregM[4:0]})
                                );
	flopr #(71)		mwreg	(	.clk(clk),
								.reset(reset),
								.d({regwriteM,
									memtoregM,
									execoutM[31:0],
									readdataM[31:0],
									writeregM[4:0]}),
								.q({regwriteW,
									memtoregW,
									execoutW[31:0],
									readdataW[31:0],
									writeregW[4:0]})
                                );
	// register file logic
	mux2 #(32)		resmux	(	.d0(execoutW[31:0]), 
								.d1(readdataW[31:0]), 
								.s(memtoregW), 
								.y(resultW[31:0]));
endmodule

// The MIPS (excluding the instruction and data memories)
module MIPS(
	input wire clk, 
    input wire reset,
	output wire [31:0]	pc,
	input wire [31:0] instruction,
	output wire write_enable,
    inout wire [31:0] data,
	output wire [31:0] execout, 
	input wire  [4:0] dispSel,
	output wire [31:0]	dispDat 
);

// deleted wire "branch" - not used
wire	[31:0]	instrD;
wire 			memtoreg, pcsrc, zero, alusrc, regdst, regwrite, jump;
wire	[2:0] 	alucontrol;
wire			hilosrc, hilowrite;
wire	[1:0]	execsrc;
wire			shamtsrc;
wire	[1:0]	shiftcontrol;
wire			ra_we;
wire			jumpsrc;
wire			memwriteD;

wire [31:0] writedata;
wire [31:0] readdata;

assign data = (write_enable) ? writedata : 32'bZ;
assign readdata = (!write_enable) ? data : 32'b0;

controller c(instrD[31:26], instrD[5:0], zero,
            memtoreg, memwriteD, pcsrc,
            alusrc, regdst, regwrite, jump,
            alucontrol, 
            hilosrc, hilowrite, execsrc,
            jumpsrc, ra_we, shamtsrc, shiftcontrol);
datapath dp(clk, 
            reset, 
            alusrc, 
            alucontrol, 
            hilosrc, 
            hilowrite, 
            shamtsrc, 
            shiftcontrol,
            execsrc,
            regdst,
            memwriteD,
            write_enable,
            regwrite, 
            memtoreg, 
            pcsrc,
            jumpsrc, 
            jump,
            ra_we,
            instruction, 
            instrD, 
            readdata, 
            pc, 
            writedata, 
            zero, 
            execout, 
            dispSel, 
            dispDat
        );
endmodule

