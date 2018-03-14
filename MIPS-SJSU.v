`timescale 1ns / 1ps

`define OP_CODE_WIDTH           6
`define ALU_OP_CODE_WIDTH       2
`define DECODER_CONTROL_WIDTH   12
`define ALU_FUNCTION_WIDTH      6
`define REGISTER_WIDTH          32
`define NUMBER_OF_REGISTERS     32
`define STACK_POINTER           29

// Adder
module adder(
	input wire [`REGISTER_WIDTH-1:0]	a, 
    input wire [`REGISTER_WIDTH-1:0]	b,
	output wire	[`REGISTER_WIDTH-1:0]	y
);

assign y = a + b;

endmodule

// Two-bit left shifter
module sl2(
	input wire [`REGISTER_WIDTH-1:0]	a,
	output wire	[`REGISTER_WIDTH-1:0]	y
);

// shift left by 2
assign y = {a[29:0], 2'b00};

endmodule

// Sign Extension Unit
module signext(
	input wire [15:0]	a,
	output wire	[`REGISTER_WIDTH-1:0]	y
);

assign y = {{16{a[15]}}, a};

endmodule

//multiplier
module mult (
    input wire  [`REGISTER_WIDTH-1:0] a, 
    input wire  [`REGISTER_WIDTH-1:0] b, 
    output wire [(`REGISTER_WIDTH*2)-1:0] out
);

assign out = a * b;

endmodule

// Parameterized Register
module flopr #(parameter WIDTH = 8) (
	input wire clk, 
    input wire reset,
	input wire [WIDTH-1:0]	d,
	output reg [WIDTH-1:0]	q);

	always @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else       q <= d;
endmodule

// Parameterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8) (
	input wire  [WIDTH-1:0] d0, 
    input wire  [WIDTH-1:0] d1,
	input wire  s,
	output wire	[WIDTH-1:0]	y
);

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
	input wire  clk,
	input wire  we3,
	input wire  [$clog2(`REGISTER_WIDTH+1)-1:0] ra1, 
    input wire  [$clog2(`REGISTER_WIDTH+1)-1:0] ra2, 
    input wire  [$clog2(`REGISTER_WIDTH+1)-1:0] wa3,
	input wire  [`REGISTER_WIDTH-1:0] wd3,
	output wire [`REGISTER_WIDTH-1:0] rd1, 
    output wire [`REGISTER_WIDTH-1:0] rd2
);
	reg		[31:0]	rf [0:`NUMBER_OF_REGISTERS];
	
    integer			n;
	//initialize registers to all 0s
	initial 
    begin
		for (n=0; n<`NUMBER_OF_REGISTERS; n=n+1) 
        begin
			rf[n] = `REGISTER_WIDTH'h00;
		end
        //// set stack pointer to position 63
		rf[`STACK_POINTER] = 63;
	end

	//write first order, include logic to handle special case of $0
    always @(posedge clk)
    begin
        if (we3)
        begin
			if (~ wa3[4])
            begin
        		rf[{27'b0,wa3[3:0]}] <= wd3;
            end
			else
            begin
				rf[{27'b1,wa3[3:0]}] <= wd3;
            end
        end
        // this leads to 72 warnings
        //rf[wa3] <= wd3;

        // this leads to 8 warnings
        //if (~ wa3[4])
        //	rf[{0,wa3[3:0]}] <= wd3;
        //else
        //	rf[{1,wa3[3:0]}] <= wd3;
    end

	assign rd1 = (ra1 != 0) ? rf[ra1[4:0]] : 0;
	assign rd2 = (ra2 != 0) ? rf[ra2[4:0]] : 0;
endmodule

module spreg(
	input wire 		clk,
	input wire 		we,
	input wire 		    re,
	input wire [63:0] 	wd,
	output wire 	[31:0] 	rd);

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

//------------------------------------------------------------------------------
// Endianess
// 0 -> little-endian. 1 -> big-endian
//------------------------------------------------------------------------------
`define LITTLE_ENDIAN      0                   //
`define BIG_ENDIAN         1                   //

//------------------------------------------------------------------------------
// Exception Vector
//------------------------------------------------------------------------------
`define VECTOR_BASE_RESET      32'h0000_0010   // MIPS Standard is 0xBFC0_0000. Reset, soft-reset, NMI
`define VECTOR_BASE_BOOT       32'h0000_0000   // MIPS Standard is 0xBFC0_0200. Bootstrap (Status_BEV = 1)
`define VECTOR_BASE_NO_BOOT    32'h0000_0000   // MIPS Standard is 0x8000_0000. Normal (Status_BEV = 0)
`define VECTOR_OFFSET_GENERAL  32'h0000_0000   // MIPS Standard is 0x0000_0180. General exception, but TBL
`define VECTOR_OFFSET_SPECIAL  32'h0000_0008   // MIPS Standard is 0x0000_0200. Interrupts (Cause_IV = 1)

//------------------------------------------------------------------------------
/*
    Encoding for the MIPS Release 1 Architecture

    3 types of instructions:
        - R   : Register-Register
        - I   : Register-Immediate
        - J   : Jump

    Format:
    ------
        - R : Opcode(6) + Rs(5) + Rt(5) + Rd(5) + shamt(5) +  function(6)
        - I : Opcode(6) + Rs(5) + Rt(5) + Imm(16)
        - J : Opcode(6) + Imm(26)
*/
//------------------------------------------------------------------------------
// Opcode field for special instructions
//------------------------------------------------------------------------------
`define OP_TYPE_R               6'b00_0000          // Special
`define OP_TYPE_REGIMM          6'b00_0001          // Branch/Trap

//------------------------------------------------------------------------------
// Instructions fields
//------------------------------------------------------------------------------
`define INSTR_OPCODE       31:26
`define INSTR_RS           25:21
`define INSTR_RT           20:16
`define INSTR_RD           15:11
`define INSTR_SHAMT        10:6
`define INSTR_FUNCT        5:0
`define INSTR_IMM16        15:0
`define INSTR_IMM26        25:0

//------------------------------------------------------------------------------
// Opcode list
//------------------------------------------------------------------------------
`define OP_ADDI                 6'b00_1000
`define OP_ADDIU                6'b00_1001
`define OP_ANDI                 6'b00_1100
`define OP_BEQ                  6'b00_0100
`define OP_BGEZ                 `OP_TYPE_REGIMM
`define OP_BGEZAL               `OP_TYPE_REGIMM
`define OP_BGTZ                 6'b00_0111
`define OP_BLEZ                 6'b00_0110
`define OP_BLTZ                 `OP_TYPE_REGIMM
`define OP_BLTZAL               `OP_TYPE_REGIMM
`define OP_BNE                  6'b00_0101
`define OP_J                    6'b00_0010
`define OP_JAL                  6'b00_0011
`define OP_LB                   6'b10_0000
`define OP_LBU                  6'b10_0100
`define OP_LH                   6'b10_0001
`define OP_LHU                  6'b10_0101
`define OP_LL                   6'b11_0000
`define OP_LUI                  6'b00_1111
`define OP_LW                   6'b10_0011
`define OP_ORI                  6'b00_1101
`define OP_SB                   6'b10_1000
`define OP_SC                   6'b11_1000
`define OP_SH                   6'b10_1001
`define OP_SLTI                 6'b00_1010
`define OP_SLTIU                6'b00_1011
`define OP_SW                   6'b10_1011
`define OP_XORI                 6'b00_1110

//------------------------------------------------------------------------------
// Function field for R(2)-type instructions
//------------------------------------------------------------------------------
`define FUNCTION_OP_ADD         6'b10_0000
`define FUNCTION_OP_ADDU        6'b10_0001
`define FUNCTION_OP_AND         6'b10_0100
`define FUNCTION_OP_BREAK       6'b00_1101
`define FUNCTION_OP_CLO         6'b10_0001
`define FUNCTION_OP_CLZ         6'b10_0000
`define FUNCTION_OP_DIV         6'b01_1010
`define FUNCTION_OP_DIVU        6'b01_1011
`define FUNCTION_OP_JALR        6'b00_1001
`define FUNCTION_OP_JR          6'b00_1000
`define FUNCTION_OP_MFHI        6'b01_0000
`define FUNCTION_OP_MFLO        6'b01_0010
`define FUNCTION_OP_MOVN        6'b00_1011
`define FUNCTION_OP_MOVZ        6'b00_1010
`define FUNCTION_OP_MSUB        6'b00_0100
`define FUNCTION_OP_MSUBU       6'b00_0101
`define FUNCTION_OP_MTHI        6'b01_0001
`define FUNCTION_OP_MTLO        6'b01_0011
`define FUNCTION_OP_MULT        6'b01_1000
`define FUNCTION_OP_MULTU       6'b01_1001
`define FUNCTION_OP_NOR         6'b10_0111
`define FUNCTION_OP_OR          6'b10_0101
`define FUNCTION_OP_SLL         6'b00_0000
`define FUNCTION_OP_SLLV        6'b00_0100
`define FUNCTION_OP_SLT         6'b10_1010
`define FUNCTION_OP_SLTU        6'b10_1011
`define FUNCTION_OP_SRA         6'b00_0011
`define FUNCTION_OP_SRAV        6'b00_0111
`define FUNCTION_OP_SRL         6'b00_0010
`define FUNCTION_OP_SRLV        6'b00_0110
`define FUNCTION_OP_SUB         6'b10_0010
`define FUNCTION_OP_SUBU        6'b10_0011
`define FUNCTION_OP_SYSCALL     6'b00_1100
`define FUNCTION_OP_XOR         6'b10_0110

//------------------------------------------------------------------------------
// Branch >/< zero (and link), traps: Rt
//------------------------------------------------------------------------------
`define RT_OP_BGEZ              5'b00001
`define RT_OP_BGEZAL            5'b10001
`define RT_OP_BLTZ              5'b00000
`define RT_OP_BLTZAL            5'b10000
`define RT_OP_TEQI              5'b01100
`define RT_OP_TGEI              5'b01000
`define RT_OP_TGEIU             5'b01001
`define RT_OP_TLTI              5'b01010
`define RT_OP_TLTIU             5'b01011
`define RT_OP_TNEI              5'b01110

//------------------------------------------------------------------------------
// Rs field for Coprocessor instructions
//------------------------------------------------------------------------------
`define RS_OP_MFC               5'b00000
`define RS_OP_MTC               5'b00100

//------------------------------------------------------------------------------
// ERET
//------------------------------------------------------------------------------
`define RS_OP_ERET              5'b10000
`define FUNCTION_OP_ERET        6'b01_1000

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
module MAIN_DECODER
(
	input wire [`OP_CODE_WIDTH-1:0] op,
	output wire	memtoreg, 
    output wire memwrite, 
    output wire branch, 
    output wire alusrc, 
    output wire regdst, 
    output wire regwrite, 
    output wire jump, 
    output wire pcspr, 
    output wire rawr, 
    output wire nbranch,
	output wire	[`ALU_OP_CODE_WIDTH-1:0] aluop
);

assign {nbranch, regwrite, regdst, alusrc, branch, memwrite, memtoreg, jump,  pcspr, rawr, aluop} = controls;

reg [`DECODER_CONTROL_WIDTH-1:0] controls;

// `OP_ADDI:      controls <= `DECODER_CONTROL_WIDTH'b010100001000;

`define IMMEDIATE_VERSION_OF_RTYPE `DECODER_CONTROL_WIDTH'b010100001010

always @(*)
begin
    case(op)
        `OP_TYPE_R:    controls <= `DECODER_CONTROL_WIDTH'b011000001010;
        `OP_ADDI:      controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        `OP_ADDIU:     controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        `OP_ANDI:      controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        `OP_BEQ:       controls <= `DECODER_CONTROL_WIDTH'b000010000001;
        `OP_BGEZ:      controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_BGEZAL:    controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_BGTZ:      controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_BLEZ:      controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_BLTZ:      controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_BLTZAL:    controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_BNE:       controls <= `DECODER_CONTROL_WIDTH'b100000000001;
        `OP_J:         controls <= `DECODER_CONTROL_WIDTH'b000000010000;
        `OP_JAL:       controls <= `DECODER_CONTROL_WIDTH'b010000010100;
        `OP_LB:        controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_LBU:       controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_LH:        controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_LHU:       controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_LL:        controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_LUI:       controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_LW:        controls <= `DECODER_CONTROL_WIDTH'b010100100000;
        `OP_ORI:       controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        `OP_SB:        controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_SC:        controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_SH:        controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx;
        `OP_SLTI:      controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        `OP_SLTIU:     controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        `OP_SW:        controls <= `DECODER_CONTROL_WIDTH'b000101000000;
        `OP_XORI:      controls <= `IMMEDIATE_VERSION_OF_RTYPE;
        default:       controls <= `DECODER_CONTROL_WIDTH'bxxxxxxxxxxxx; // unsupported opcode
    endcase
end
endmodule

module IMMEDIATE_TO_ALU_CONVERTER (
    input wire [`OP_CODE_WIDTH-1:0]      opcode,
    input wire [`ALU_FUNCTION_WIDTH-1:0] instruction_function,
    output reg [`ALU_FUNCTION_WIDTH-1:0] alu_funct
);

always @(*)
begin
    case (opcode)
        `OP_TYPE_R:  alu_funct <= instruction_function;
        `OP_ADDI:    alu_funct <= `FUNCTION_OP_ADD;
        `OP_ADDIU:   alu_funct <= `FUNCTION_OP_ADD;
        `OP_ANDI:    alu_funct <= `FUNCTION_OP_AND;
        `OP_BEQ:     alu_funct <= `FUNCTION_OP_SUB;
        //`OP_BGEZ:    alu_funct <= `FUNCTION_OP_BGEZ;
        //`OP_BGEZAL:  alu_funct <= `FUNCTION_OP_BGEZAL;
        //`OP_BGTZ:    alu_funct <= `FUNCTION_OP_BGTZ;
        //`OP_BLEZ:    alu_funct <= `FUNCTION_OP_BLEZ;
        //`OP_BLTZ:    alu_funct <= `FUNCTION_OP_BLTZ;
        //`OP_BLTZAL:  alu_funct <= `FUNCTION_OP_BLTZAL;
        //`OP_BNE:     alu_funct <= `FUNCTION_OP_BNE;
        //`OP_JAL:     alu_funct <= `FUNCTION_OP_JAL;
        //`OP_LB:      alu_funct <= `FUNCTION_OP_LB;
        //`OP_LBU:     alu_funct <= `FUNCTION_OP_LBU;
        //`OP_LH:      alu_funct <= `FUNCTION_OP_LH;
        //`OP_LHU:     alu_funct <= `FUNCTION_OP_LHU;
        //`OP_LL:      alu_funct <= `FUNCTION_OP_LL;
        //`OP_LUI:     alu_funct <= `FUNCTION_OP_LUI;
        //`OP_LW:      alu_funct <= `FUNCTION_OP_LW;
        `OP_ORI:     alu_funct <= `FUNCTION_OP_OR;
        //`OP_SB:      alu_funct <= `FUNCTION_OP_SB;
        //`OP_SC:      alu_funct <= `FUNCTION_OP_SC;
        //`OP_SH:      alu_funct <= `FUNCTION_OP_SH;
        //`OP_SLTI:    alu_funct <= `FUNCTION_OP_SLTI;
        //`OP_SLTIU:   alu_funct <= `FUNCTION_OP_SLTIU;
        //`OP_SW:      alu_funct <= `FUNCTION_OP_SW;
        `OP_XORI :   alu_funct <= `FUNCTION_OP_XOR;
        default:     alu_funct <= `ALU_FUNCTION_WIDTH'b0;
    endcase
end

endmodule

// ALU Decoder
module ALU_DECODER(
	input wire 	[`ALU_FUNCTION_WIDTH-1:0]	funct,
	input wire 	[`ALU_OP_CODE_WIDTH-1:0]	aluop,
	output reg	[`ALU_FUNCTION_WIDTH-1:0]	alucontrol,
    output wire sprwr,
    output wire sprrd,
    output wire sprmux,
    output wire jrmux
);
    /* TODO: FIX THE ISSUE WITH ALUCONTROL */
    //sprwr, sprrd, sprmux, jrmux, alucontrol
	always @(*)
    begin
		case(aluop)
			2'b00:      alucontrol <= `FUNCTION_OP_ADD;
			2'b01:      alucontrol <= `FUNCTION_OP_SUB;
			default:
            begin 
                alucontrol <= funct;
                case(funct)
                begin
                    `FUNCTION_OP_JALR: 
                    begin 
                        jrmux <= 1; 
                    `FUNCTION_OP_JR: 
                end
            end
		endcase
    end
endmodule
// ALU
module ALU(
	input wire 	[`REGISTER_WIDTH-1:0]	    a, 
    input wire 	[`REGISTER_WIDTH-1:0]       b,
	input wire 	[`ALU_FUNCTION_WIDTH-1:0]   funct,
	output reg	[`REGISTER_WIDTH-1:0]       result,
	output wire			                    zero
);

wire signed [`REGISTER_WIDTH-1:0] signed_a; 
wire signed [`REGISTER_WIDTH-1:0] signed_b;

assign signed_a = a;
assign signed_b = b;

// assign b2 = alucont[2] ? ~b:b;
// assign sum = a + b2 + alucont[2];
// assign slt = sum[31];
assign zero = (result == 32'b0);

always@(*)
begin
    case(funct)
        // 2'b00: result <= a & b;
        // 2'b01: result <= a | b;
        // 2'b10: result <= sum;
        // 2'b11: result <= slt;
        `FUNCTION_OP_ADD:       result <=  (signed_a + signed_b);
        `FUNCTION_OP_ADDU:      result <=  (a + b);
        `FUNCTION_OP_AND:       result <=  (a & b);
        `FUNCTION_OP_BREAK:     result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_CLO:       result <= `REGISTER_WIDTH'hxxxxxxxx; // count leading ones
        `FUNCTION_OP_CLZ:       result <= `REGISTER_WIDTH'hxxxxxxxx; // count leading zeros
        `FUNCTION_OP_DIV:       result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_DIVU:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_JALR:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_JR:        result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MFHI:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MFLO:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MOVN:      result <= (b == 32'b0);
        `FUNCTION_OP_MOVZ:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MSUB:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MSUBU:     result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MTHI:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MTLO:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MULT:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_MULTU:     result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_NOR:       result <= ~(a | b);
        `FUNCTION_OP_OR:        result <=  (a | b);
        `FUNCTION_OP_SLL:       result <=  (a << b);
        `FUNCTION_OP_SLLV:      result <=  (a << b);
        `FUNCTION_OP_SLT:       result <=  (signed_a < signed_b) ? 32'b1 : 32'b0;
        `FUNCTION_OP_SLTU:      result <=  (a < b) ? 32'b1 : 32'b0;
        `FUNCTION_OP_SRA:       result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_SRAV:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_SRL:       result <=  (a >> b);
        `FUNCTION_OP_SRLV:      result <=  (a >> b);
        `FUNCTION_OP_SUB:       result <=  (signed_a - signed_b);
        `FUNCTION_OP_SUBU:      result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_SYSCALL:   result <= `REGISTER_WIDTH'hxxxxxxxx;
        `FUNCTION_OP_XOR:       result <= `REGISTER_WIDTH'hxxxxxxxx;
        default:                result <= `REGISTER_WIDTH'hxxxxxxxx;
    endcase
end

endmodule

// Control Unit
module controller(
	input wire [5:0] op, 
    input wire [5:0] funct,
	input wire 	     zero,
	output wire		 memtoreg, 
    output wire      memwrite, 
    output wire      pcsrc, 
    output wire      alusrc, 
    output wire      regdst, 
    output wire      regwrite, 
    output wire      jump, 
    output wire      sprwr, 
    output wire      sprrd, 
    output wire      sprmux, 
    output wire      jrmux, 
    output wire      pcspr, 
    output wire      rawr,
	output wire	[2:0] alucontrol
);

wire	[1:0]	aluop;
wire			branch, nbranch;

assign pcsrc = (branch & zero || nbranch & ~zero);

MAIN_DECODER md(
    .op(op),
    .memtoreg(memtoreg), 
    .memwrite(memwrite), 
    .branch(branch), 
    .alusrc(alusrc), 
    .regdst(regdst), 
    .regwrite(regwrite), 
    .jump(jump), 
    .pcspr(pcspr), 
    .rawr(rawr), 
    .nbranch(nbranch),
    .aluop(aluop)
);

ALU_DECODER ad(
    .funct(funct),
    .aluop(aluop),
    .alucontrol(alucontrol),
    .sprwr(sprwr),
    .sprrd(sprrd),
    .sprmux(sprmux),
    .jrmux(jrmux)
);

endmodule

// Data Path (excluding the instruction and data memories)
module datapath(
	input wire          clk, 
    input wire          reset, 
    input wire          memtoreg, 
    input wire          pcsrc, 
    input wire          alusrc, 
    input wire          regdst, 
    input wire          regwrite, 
    input wire          jump, 
    input wire          sprwr, 
    input wire          sprrd, 
    input wire          sprmux, 
    input wire          jrmux, 
    input wire          pcspr, 
    input wire          rawr,
	input wire  [2:0]	alucontrol,
	output wire			zero,
	output wire	[31:0]	pc,
	input wire  [31:0]	instr,
	output wire	[31:0]	aluout, 
    output wire	[31:0]	writedata,
	input wire  [31:0]	readdata
);

	wire [4:0]  writereg, writerega;
	wire [31:0] pcnext, pcnext2, pcnextbr, pcplus4, pcbranch, signimm, signimmsh, srca, srcb, result, pcresult, mux2reg, spr2mux, hold;
    wire [63:0] multspr;

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
	regfile		rf(clk, regwrite, instr[25:21], instr[20:16], writerega, pcresult, srca, writedata);
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

    //// r-type instruction
    wire opcode                 = instr[31:26];
    wire register_source        = instr[25:21];
    wire register_target        = instr[20:16];
    wire register_destination   = instr[15:11];
    wire shift_amount           = instr[10:6];
    wire funct                  = instr[5:0];
    //// immediate section of instruction
    wire immediate              = instr[15:0];
    //// address
    wire jump_address           = instr[25:0];

	// ALU logic
    IMMEDIATE_TO_ALU_CONVERTER alu_converter(
        .opcode(opcode),
        .instruction_function(alucontrol),
        .alu_funct()
    );
	mux2 #(32)	srcbmux(writedata, signimm, alusrc, srcb);
	ALU			alu(srca, srcb, alucontrol, aluout, zero);
endmodule

// The MIPS (excluding the instruction and data memories)
module mips(
	input wire        	clk, reset,
	output wire	[31:0]	pc,
	input wire 	[31:0]	instr,
	output wire			memwrite,
	output wire	[31:0]	aluout, 
	output wire	[31:0]	writedata,
	input wire 	[31:0]	readdata
);

wire memtoreg;
wire pcsrc;
wire zero;
wire alusrc;
wire regdst;
wire regwrite;
wire jump;
wire sprwr;
wire sprrd;
wire sprmux;
wire jrmux;
wire pcspr;
wire rawr;
wire [2:0] alucontrol;

//// r-type instruction
wire opcode                 = instr[31:26];
wire register_source        = instr[25:21];
wire register_target        = instr[20:16];
wire register_destination   = instr[15:11];
wire shift_amount           = instr[10:6];
wire funct                  = instr[5:0];
//// immediate section of instruction
wire immediate              = instr[15:0];
//// address
wire jump_address           = instr[25:0];

controller c(
    .op(opcode), 
    .funct(funct),
    .zero(zero),
    .memtoreg(memtoreg), 
    .memwrite(memwrite), 
    .pcsrc(pcsrc), 
    .alusrc(alusrc), 
    .regdst(regdst), 
    .regwrite(regwrite), 
    .jump(jump), 
    .sprwr(sprwr), 
    .sprrd(sprrd), 
    .sprmux(sprmux), 
    .jrmux(jrmux), 
    .pcspr(pcspr), 
    .rawr(rawr),
    .alucontrol(alucontrol)
);
datapath dp(
    .clk(clk),
    .reset(reset),
    .memtoreg(memtoreg),
    .pcsrc(pcsrc),
    .alusrc(alusrc),
    .regdst(regdst),
    .regwrite(regwrite),
    .jump(jump),
    .sprwr(sprwr),
    .sprrd(sprrd),
    .sprmux(sprmux),
    .jrmux(jrmux),
    .pcspr(pcspr),
    .rawr(rawr),
    .alucontrol(alucontrol),
    .zero(zero),
    .pc(pc),
    .instr(instr),
    .aluout(aluout),
    .writedata(writedata),
    .readdata(readdata)
);

endmodule

// Instruction Memory
module imem (
	input wire  [ 5:0]	a,
	output wire [31:0]	dOut
);

reg		[31:0]	rom[0:63];
assign dOut = rom[a];

//initialize rom from memfile_s.dat
initial
begin
    $readmemh("memfile_s.dat", rom);
end
endmodule

// Data Memory
module dmem (
	input wire 		clk,
	input wire 		we,
	input wire [31:0]	addr,
	input wire [31:0]	dIn,
	output wire 	[31:0]	dOut
);

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