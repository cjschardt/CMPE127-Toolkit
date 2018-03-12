`timescale 1ns / 1ps
`default_nettype none
/*
 * File         : MIPS_Parameters.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   26-May-2012  GEA       Release version.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   Provides a language abstraction for the MIPS32-specific op-codes and
 *   the processor-specific datapath, hazard, and exception bits which
 *   control the processor. These definitions are used extensively
 *   throughout the processor HDL modules.
 */


/*** Exception Vector Locations ***

     When the CPU powers up or is reset, it will begin execution at 'EXC_Vector_Base_Reset'.
     All other exceptions are the sum of a base address and offset:
      - The base address is either a bootstrap or normal value. It is controlled by
        the 'BEV' bit in the CP0 'Status' register. Both base addresses can be mapped to
        the same location.
      - The offset address is either a standard offset (which is always used for
        non-interrupt general exceptions in this processor because it lacks TLB Refill
        and Cache errors), or a special interrupt-only offset for interrupts, which is
        enabled with the 'IV' bit in the CP0 'Cause' register.

     Current Setup:
        General exceptions go to 0x0. Interrupts go to 0x8. Booting starts at 0x10.
*/
`define EXC_Vector_Base_Reset          32'h0000_0010    // MIPS Standard is 0xBFC0_0000
`define EXC_Vector_Base_Other_NoBoot   32'h0000_0000    // MIPS Standard is 0x8000_0000
`define EXC_Vector_Base_Other_Boot     32'h0000_0000    // MIPS Standard is 0xBFC0_0200
`define EXC_Vector_Offset_General      32'h0000_0000    // MIPS Standard is 0x0000_0180
`define EXC_Vector_Offset_Special      32'h0000_0008    // MIPS Standard is 0x0000_0200



/*** Kernel/User Memory Areas ***

     Kernel memory starts at address 0x0. User memory starts at 'UMem_Lower' and extends to
     the end of the address space.

     A distinction is made to protect against accesses to kernel memory while the processor
     is in user mode. Lacking MMU hardware, these addresses are physical, not virtual.
     This simple two-part division of the address space can be extended almost arbitrarily
     in the Data Memory Controller. Note that there is currently no user/kernel space check
     for the Instruction Memory, because it is assumed that instructions are in the kernel space.
*/
`define UMem_Lower 32'h08000000



/*** Processor Endianness ***

     The MIPS Configuration Register (CP0 Register 16 Select 0) specifies the processor's
     endianness. A processor in user mode may switch to reverse endianness, which will be
     the opposite of this `define.
*/
`define Big_Endian 1'b1



/*** Encodings for MIPS32 Release 1 Architecture ***/


/* Op Code Categories */
`define Op_Type_R   6'b00_0000  // Standard R-Type instructions
`define Op_Type_R2  6'b01_1100  // Extended R-Like instructions
`define Op_Type_BI  6'b00_0001  // Branch/Trap extended instructions
`define Op_Type_CP0 6'b01_0000  // Coprocessor 0 instructions
`define Op_Type_CP1 6'b01_0001  // Coprocessor 1 instructions (not implemented)
`define Op_Type_CP2 6'b01_0010  // Coprocessor 2 instructions (not implemented)
`define Op_Type_CP3 6'b01_0011  // Coprocessor 3 instructions (not implemented)
// --------------------------------------
`define Op_Add      `Op_Type_R
`define Op_Addi     6'b00_1000
`define Op_Addiu    6'b00_1001
`define Op_Addu     `Op_Type_R
`define Op_And      `Op_Type_R
`define Op_Andi     6'b00_1100
`define Op_Beq      6'b00_0100
`define Op_Bgez     `Op_Type_BI
`define Op_Bgezal   `Op_Type_BI
`define Op_Bgtz     6'b00_0111
`define Op_Blez     6'b00_0110
`define Op_Bltz     `Op_Type_BI
`define Op_Bltzal   `Op_Type_BI
`define Op_Bne      6'b00_0101
`define Op_Break    `Op_Type_R
`define Op_Clo      `Op_Type_R2
`define Op_Clz      `Op_Type_R2
`define Op_Div      `Op_Type_R
`define Op_Divu     `Op_Type_R
`define Op_Eret     `Op_Type_CP0
`define Op_J        6'b00_0010
`define Op_Jal      6'b00_0011
`define Op_Jalr     `Op_Type_R
`define Op_Jr       `Op_Type_R
`define Op_Lb       6'b10_0000
`define Op_Lbu      6'b10_0100
`define Op_Lh       6'b10_0001
`define Op_Lhu      6'b10_0101
`define Op_Ll       6'b11_0000
`define Op_Lui      6'b00_1111
`define Op_Lw       6'b10_0011
`define Op_Lwl      6'b10_0010
`define Op_Lwr      6'b10_0110
`define Op_Madd     `Op_Type_R2
`define Op_Maddu    `Op_Type_R2
`define Op_Mfc0     `Op_Type_CP0
`define Op_Mfhi     `Op_Type_R
`define Op_Mflo     `Op_Type_R
`define Op_Movn     `Op_Type_R
`define Op_Movz     `Op_Type_R
`define Op_Msub     `Op_Type_R2
`define Op_Msubu    `Op_Type_R2
`define Op_Mtc0     `Op_Type_CP0
`define Op_Mthi     `Op_Type_R
`define Op_Mtlo     `Op_Type_R
`define Op_Mul      `Op_Type_R2
`define Op_Mult     `Op_Type_R
`define Op_Multu    `Op_Type_R
`define Op_Nor      `Op_Type_R
`define Op_Or       `Op_Type_R
`define Op_Ori      6'b00_1101
`define Op_Pref     6'b11_0011  // Prefetch does nothing in this implementation.
`define Op_Sb       6'b10_1000
`define Op_Sc       6'b11_1000
`define Op_Sh       6'b10_1001
`define Op_Sll      `Op_Type_R
`define Op_Sllv     `Op_Type_R
`define Op_Slt      `Op_Type_R
`define Op_Slti     6'b00_1010
`define Op_Sltiu    6'b00_1011
`define Op_Sltu     `Op_Type_R
`define Op_Sra      `Op_Type_R
`define Op_Srav     `Op_Type_R
`define Op_Srl      `Op_Type_R
`define Op_Srlv     `Op_Type_R
`define Op_Sub      `Op_Type_R
`define Op_Subu     `Op_Type_R
`define Op_Sw       6'b10_1011
`define Op_Swl      6'b10_1010
`define Op_Swr      6'b10_1110
`define Op_Syscall  `Op_Type_R
`define Op_Teq      `Op_Type_R
`define Op_Teqi     `Op_Type_BI
`define Op_Tge      `Op_Type_R
`define Op_Tgei     `Op_Type_BI
`define Op_Tgeiu    `Op_Type_BI
`define Op_Tgeu     `Op_Type_R
`define Op_Tlt      `Op_Type_R
`define Op_Tlti     `Op_Type_BI
`define Op_Tltiu    `Op_Type_BI
`define Op_Tltu     `Op_Type_R
`define Op_Tne      `Op_Type_R
`define Op_Tnei     `Op_Type_BI
`define Op_Xor      `Op_Type_R
`define Op_Xori     6'b00_1110

/* Op Code Rt fields for Branches & Traps */
`define OpRt_Bgez   5'b00001
`define OpRt_Bgezal 5'b10001
`define OpRt_Bltz   5'b00000
`define OpRt_Bltzal 5'b10000
`define OpRt_Teqi   5'b01100
`define OpRt_Tgei   5'b01000
`define OpRt_Tgeiu  5'b01001
`define OpRt_Tlti   5'b01010
`define OpRt_Tltiu  5'b01011
`define OpRt_Tnei   5'b01110

/* Op Code Rs fields for Coprocessors */
`define OpRs_MF     5'b00000
`define OpRs_MT     5'b00100

/* Special handling for ERET */
`define OpRs_ERET   5'b10000
`define Funct_ERET  6'b011000

/* Function Codes for R-Type Op Codes */
`define Funct_Add     6'b10_0000
`define Funct_Addu    6'b10_0001
`define Funct_And     6'b10_0100
`define Funct_Break   6'b00_1101
`define Funct_Clo     6'b10_0001    // same as Addu
`define Funct_Clz     6'b10_0000    // same as Add
`define Funct_Div     6'b01_1010
`define Funct_Divu    6'b01_1011
`define Funct_Jr      6'b00_1000
`define Funct_Jalr    6'b00_1001
`define Funct_Madd    6'b00_0000
`define Funct_Maddu   6'b00_0001
`define Funct_Mfhi    6'b01_0000
`define Funct_Mflo    6'b01_0010
`define Funct_Movn    6'b00_1011
`define Funct_Movz    6'b00_1010
`define Funct_Msub    6'b00_0100    // same as Sllv
`define Funct_Msubu   6'b00_0101
`define Funct_Mthi    6'b01_0001
`define Funct_Mtlo    6'b01_0011
`define Funct_Mul     6'b00_0010    // same as Srl
`define Funct_Mult    6'b01_1000
`define Funct_Multu   6'b01_1001
`define Funct_Nor     6'b10_0111
`define Funct_Or      6'b10_0101
`define Funct_Sll     6'b00_0000
`define Funct_Sllv    6'b00_0100
`define Funct_Slt     6'b10_1010
`define Funct_Sltu    6'b10_1011
`define Funct_Sra     6'b00_0011
`define Funct_Srav    6'b00_0111
`define Funct_Srl     6'b00_0010
`define Funct_Srlv    6'b00_0110
`define Funct_Sub     6'b10_0010
`define Funct_Subu    6'b10_0011
`define Funct_Syscall 6'b00_1100
`define Funct_Teq     6'b11_0100
`define Funct_Tge     6'b11_0000
`define Funct_Tgeu    6'b11_0001
`define Funct_Tlt     6'b11_0010
`define Funct_Tltu    6'b11_0011
`define Funct_Tne     6'b11_0110
`define Funct_Xor     6'b10_0110

/* ALU Operations (Implementation) */
`define AluOp_Add    5'd1
`define AluOp_Addu   5'd0
`define AluOp_And    5'd2
`define AluOp_Clo    5'd3
`define AluOp_Clz    5'd4
`define AluOp_Div    5'd5
`define AluOp_Divu   5'd6
`define AluOp_Madd   5'd7
`define AluOp_Maddu  5'd8
`define AluOp_Mfhi   5'd9
`define AluOp_Mflo   5'd10
`define AluOp_Msub   5'd13
`define AluOp_Msubu  5'd14
`define AluOp_Mthi   5'd11
`define AluOp_Mtlo   5'd12
`define AluOp_Mul    5'd15
`define AluOp_Mult   5'd16
`define AluOp_Multu  5'd17
`define AluOp_Nor    5'd18
`define AluOp_Or     5'd19
`define AluOp_Sll    5'd20
`define AluOp_Sllc   5'd21  // Move this if another AluOp is needed
`define AluOp_Sllv   5'd22
`define AluOp_Slt    5'd23
`define AluOp_Sltu   5'd24
`define AluOp_Sra    5'd25
`define AluOp_Srav   5'd26
`define AluOp_Srl    5'd27
`define AluOp_Srlv   5'd28
`define AluOp_Sub    5'd29
`define AluOp_Subu   5'd30
`define AluOp_Xor    5'd31



/*
 * File         : Add.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A simple 32-bit 2-input wire adder.
 */
module Add(
    input wire  [31:0] A,
    input wire  [31:0] B,
    output wire [31:0] C
);

    assign C = (A + B);

endmodule

/*
 * File         : ALU.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *   2.0   26-Jul-2012  GEA       Many changes have been made.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   An Arithmetic Logic Unit for a MIPS32 processor. This module computes all
 *   arithmetic operations, including the following:
 *
 *   Add, Subtract, Multiply, And, Or, Nor, Xor, Shift, Count leading 1s/0s.
 */
module ALU(
    input wire  clock,
    input wire  reset,
    input wire  EX_Stall,
    input wire  EX_Flush,
    input wire  [31:0] A, B,
    input wire  [4:0]  Operation,
    input wire  signed [4:0] Shamt,
    output reg signed [31:0] Result,
    output wire BZero,           // Used for Movc
    output reg EXC_Ov,
    output wire ALU_Stall        // Stalls due to long ALU operations
);


    /***
     Performance Notes:

     The ALU is the longest delay path in the Execute stage, and one of the longest
     in the entire processor. This path varies based on the logic blocks that are
     chosen to implement various functions, but there is certainly room to improve
     the speed of arithmetic operations. The ALU could also be placed in a separate
     pipeline stage after the Execute forwarding has completed.
    ***/


    /***
     Divider Logic:

     The hardware divider requires 32 cycles to complete. Because it writes its
     results to HILO and not to the pipeline, the pipeline can proceed without
     stalling. When a later instruction tries to access HILO, the pipeline will
     stall if the divide operation has not yet completed.
    ***/


    // Internal state registers
    reg  [63:0] HILO;
    reg  HILO_Access;                   // Behavioral; not DFFs
    reg  [5:0] CLO_Result, CLZ_Result;  // Behavioral; not DFFs
    reg  div_fsm;

    // Internal signals
    wire [31:0] HI, LO;
    wire HILO_Commit;
    wire signed [31:0] As, Bs;
    wire AddSub_Add;
    wire signed [31:0] AddSub_Result;
    wire signed [63:0] Mult_Result;
    wire [63:0] Multu_Result;
    wire [31:0] Quotient;
    wire [31:0] Remainder;
    wire Div_Stall;
    wire Div_Start, Divu_Start;
    wire DivOp;
    wire Div_Commit;

    // Assignments
    assign HI = HILO[63:32];
    assign LO = HILO[31:0];
    assign HILO_Commit = ~(EX_Stall | EX_Flush);
    assign As = A;
    assign Bs = B;
    assign AddSub_Add = ((Operation == `AluOp_Add) | (Operation == `AluOp_Addu));
    assign AddSub_Result = (AddSub_Add) ? (A + B) : (A - B);
    assign Mult_Result = As * Bs;
    assign Multu_Result = A * B;
    assign BZero = (B == 32'h00000000);
    assign DivOp = (Operation == `AluOp_Div) || (Operation == `AluOp_Divu);
    assign Div_Commit   = (div_fsm == 1'b1) && (Div_Stall == 1'b0);
    assign Div_Start    = (div_fsm == 1'b0) && (Operation == `AluOp_Div)  && (HILO_Commit == 1'b1);
    assign Divu_Start   = (div_fsm == 1'b0) && (Operation == `AluOp_Divu) && (HILO_Commit == 1'b1);
    assign ALU_Stall    = (div_fsm == 1'b1) && (HILO_Access == 1'b1);

    always @(*) begin
        case (Operation)
            `AluOp_Add  : Result <= AddSub_Result;
            `AluOp_Addu : Result <= AddSub_Result;
            `AluOp_And  : Result <= A & B;
            `AluOp_Clo  : Result <= {26'b0, CLO_Result};
            `AluOp_Clz  : Result <= {26'b0, CLZ_Result};
            `AluOp_Mfhi : Result <= HI;
            `AluOp_Mflo : Result <= LO;
            `AluOp_Mul  : Result <= Mult_Result[31:0];
            `AluOp_Nor  : Result <= ~(A | B);
            `AluOp_Or   : Result <= A | B;
            `AluOp_Sll  : Result <= B << Shamt;
            `AluOp_Sllc : Result <= {B[15:0], 16'b0};
            `AluOp_Sllv : Result <= B << A[4:0];
            `AluOp_Slt  : Result <= (As < Bs) ? 32'h00000001 : 32'h00000000;
            `AluOp_Sltu : Result <= (A < B)   ? 32'h00000001 : 32'h00000000;
            `AluOp_Sra  : Result <= Bs >>> Shamt;
            `AluOp_Srav : Result <= Bs >>> As[4:0];
            `AluOp_Srl  : Result <= B >> Shamt;
            `AluOp_Srlv : Result <= B >> A[4:0];
            `AluOp_Sub  : Result <= AddSub_Result;
            `AluOp_Subu : Result <= AddSub_Result;
            `AluOp_Xor  : Result <= A ^ B;
            default     : Result <= 32'hxxxx_xxxx;
        endcase
    end


    always @(posedge clock) begin
        if (reset) begin
            HILO <= 64'h00000000_00000000;
        end
        else if (Div_Commit) begin
            HILO <= {Remainder, Quotient};
        end
        else if (HILO_Commit) begin
            case (Operation)
                `AluOp_Mult  : HILO <= Mult_Result;
                `AluOp_Multu : HILO <= Multu_Result;
                `AluOp_Madd  : HILO <= HILO + Mult_Result;
                `AluOp_Maddu : HILO <= HILO + Multu_Result;
                `AluOp_Msub  : HILO <= HILO - Mult_Result;
                `AluOp_Msubu : HILO <= HILO - Multu_Result;
                `AluOp_Mthi  : HILO <= {A, LO};
                `AluOp_Mtlo  : HILO <= {HI, B};
                default      : HILO <= HILO;
            endcase
        end
        else begin
            HILO <= HILO;
        end
    end

    // Detect accesses to HILO. RAW and WAW hazards are possible while a
    // divide operation is computing, so reads and writes to HILO must stall
    // while the divider is busy.
    // (This logic could be put into an earlier pipeline stage or into the
    // datapath bits to improve timing.)
    always @(Operation) begin
        case (Operation)
            `AluOp_Div   : HILO_Access <= 1;
            `AluOp_Divu  : HILO_Access <= 1;
            `AluOp_Mfhi  : HILO_Access <= 1;
            `AluOp_Mflo  : HILO_Access <= 1;
            `AluOp_Mult  : HILO_Access <= 1;
            `AluOp_Multu : HILO_Access <= 1;
            `AluOp_Madd  : HILO_Access <= 1;
            `AluOp_Maddu : HILO_Access <= 1;
            `AluOp_Msub  : HILO_Access <= 1;
            `AluOp_Msubu : HILO_Access <= 1;
            `AluOp_Mthi  : HILO_Access <= 1;
            `AluOp_Mtlo  : HILO_Access <= 1;
            default      : HILO_Access <= 0;
        endcase
    end

    // Divider FSM: The divide unit is either available or busy.
    always @(posedge clock) begin
        if (reset) begin
            div_fsm <= 1'd0;
        end
        else begin
            case (div_fsm)
                1'd0 : div_fsm <= (DivOp & HILO_Commit) ? 1'd1 : 1'd0;
                1'd1 : div_fsm <= (~Div_Stall) ? 1'd0 : 1'd1;
            endcase
        end
    end

    // Detect overflow for signed operations. Note that MIPS32 has no overflow
    // detection for multiplication/division operations.
    always @(*) begin
        case (Operation)
            `AluOp_Add : EXC_Ov <= ((A[31] ~^ B[31]) & (A[31] ^ AddSub_Result[31]));
            `AluOp_Sub : EXC_Ov <= ((A[31]  ^ B[31]) & (A[31] ^ AddSub_Result[31]));
            default    : EXC_Ov <= 0;
        endcase
    end

    // Count Leading Ones
    always @(A) begin
        casex (A)
            32'b0xxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd0;
            32'b10xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd1;
            32'b110x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd2;
            32'b1110_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd3;
            32'b1111_0xxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd4;
            32'b1111_10xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd5;
            32'b1111_110x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd6;
            32'b1111_1110_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd7;
            32'b1111_1111_0xxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd8;
            32'b1111_1111_10xx_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd9;
            32'b1111_1111_110x_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd10;
            32'b1111_1111_1110_xxxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd11;
            32'b1111_1111_1111_0xxx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd12;
            32'b1111_1111_1111_10xx_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd13;
            32'b1111_1111_1111_110x_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd14;
            32'b1111_1111_1111_1110_xxxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd15;
            32'b1111_1111_1111_1111_0xxx_xxxx_xxxx_xxxx : CLO_Result <= 6'd16;
            32'b1111_1111_1111_1111_10xx_xxxx_xxxx_xxxx : CLO_Result <= 6'd17;
            32'b1111_1111_1111_1111_110x_xxxx_xxxx_xxxx : CLO_Result <= 6'd18;
            32'b1111_1111_1111_1111_1110_xxxx_xxxx_xxxx : CLO_Result <= 6'd19;
            32'b1111_1111_1111_1111_1111_0xxx_xxxx_xxxx : CLO_Result <= 6'd20;
            32'b1111_1111_1111_1111_1111_10xx_xxxx_xxxx : CLO_Result <= 6'd21;
            32'b1111_1111_1111_1111_1111_110x_xxxx_xxxx : CLO_Result <= 6'd22;
            32'b1111_1111_1111_1111_1111_1110_xxxx_xxxx : CLO_Result <= 6'd23;
            32'b1111_1111_1111_1111_1111_1111_0xxx_xxxx : CLO_Result <= 6'd24;
            32'b1111_1111_1111_1111_1111_1111_10xx_xxxx : CLO_Result <= 6'd25;
            32'b1111_1111_1111_1111_1111_1111_110x_xxxx : CLO_Result <= 6'd26;
            32'b1111_1111_1111_1111_1111_1111_1110_xxxx : CLO_Result <= 6'd27;
            32'b1111_1111_1111_1111_1111_1111_1111_0xxx : CLO_Result <= 6'd28;
            32'b1111_1111_1111_1111_1111_1111_1111_10xx : CLO_Result <= 6'd29;
            32'b1111_1111_1111_1111_1111_1111_1111_110x : CLO_Result <= 6'd30;
            32'b1111_1111_1111_1111_1111_1111_1111_1110 : CLO_Result <= 6'd31;
            32'b1111_1111_1111_1111_1111_1111_1111_1111 : CLO_Result <= 6'd32;
            default : CLO_Result <= 6'd0;
        endcase
    end

    // Count Leading Zeros
    always @(A) begin
        casex (A)
            32'b1xxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd0;
            32'b01xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd1;
            32'b001x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd2;
            32'b0001_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd3;
            32'b0000_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd4;
            32'b0000_01xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd5;
            32'b0000_001x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd6;
            32'b0000_0001_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd7;
            32'b0000_0000_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd8;
            32'b0000_0000_01xx_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd9;
            32'b0000_0000_001x_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd10;
            32'b0000_0000_0001_xxxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd11;
            32'b0000_0000_0000_1xxx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd12;
            32'b0000_0000_0000_01xx_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd13;
            32'b0000_0000_0000_001x_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd14;
            32'b0000_0000_0000_0001_xxxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd15;
            32'b0000_0000_0000_0000_1xxx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd16;
            32'b0000_0000_0000_0000_01xx_xxxx_xxxx_xxxx : CLZ_Result <= 6'd17;
            32'b0000_0000_0000_0000_001x_xxxx_xxxx_xxxx : CLZ_Result <= 6'd18;
            32'b0000_0000_0000_0000_0001_xxxx_xxxx_xxxx : CLZ_Result <= 6'd19;
            32'b0000_0000_0000_0000_0000_1xxx_xxxx_xxxx : CLZ_Result <= 6'd20;
            32'b0000_0000_0000_0000_0000_01xx_xxxx_xxxx : CLZ_Result <= 6'd21;
            32'b0000_0000_0000_0000_0000_001x_xxxx_xxxx : CLZ_Result <= 6'd22;
            32'b0000_0000_0000_0000_0000_0001_xxxx_xxxx : CLZ_Result <= 6'd23;
            32'b0000_0000_0000_0000_0000_0000_1xxx_xxxx : CLZ_Result <= 6'd24;
            32'b0000_0000_0000_0000_0000_0000_01xx_xxxx : CLZ_Result <= 6'd25;
            32'b0000_0000_0000_0000_0000_0000_001x_xxxx : CLZ_Result <= 6'd26;
            32'b0000_0000_0000_0000_0000_0000_0001_xxxx : CLZ_Result <= 6'd27;
            32'b0000_0000_0000_0000_0000_0000_0000_1xxx : CLZ_Result <= 6'd28;
            32'b0000_0000_0000_0000_0000_0000_0000_01xx : CLZ_Result <= 6'd29;
            32'b0000_0000_0000_0000_0000_0000_0000_001x : CLZ_Result <= 6'd30;
            32'b0000_0000_0000_0000_0000_0000_0000_0001 : CLZ_Result <= 6'd31;
            32'b0000_0000_0000_0000_0000_0000_0000_0000 : CLZ_Result <= 6'd32;
            default : CLZ_Result <= 6'd0;
        endcase
    end

    // Multicycle divide unit
    Divide Divider (
        .clock      (clock),
        .reset      (reset),
        .OP_div     (Div_Start),
        .OP_divu    (Divu_Start),
        .Dividend   (A),
        .Divisor    (B),
        .Quotient   (Quotient),
        .Remainder  (Remainder),
        .Stall      (Div_Stall)
    );

endmodule

/*
 * File         : Compare.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   15-Jun-2011  GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   Compares two 32-bit values and output wires the following information about them:
 *      EQ  : A and B are equal
 *      GZ  : A is greater than zero
 *      LZ  : A is less than zero
 *      GEZ : A is greater than or equal to zero
 *      LEZ : A is less than or equal to zero
 */
module Compare(
    input wire  [31:0] A,
    input wire  [31:0] B,
    output wire EQ,
    output wire GZ,
    output wire LZ,
    output wire GEZ,
    output wire LEZ
    );

    wire   ZeroA = (A == 32'b0);

    assign EQ  = ( A == B);
    assign GZ  = (~A[31] & ~ZeroA);
    assign LZ  =   A[31];
    assign GEZ =  ~A[31];
    assign LEZ = ( A[31] |  ZeroA);

endmodule

/*
 * File         : Control.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0    7-Jun-2011  GEA       Initial design.
 *   2.0   26-May-2012  GEA       Release version with CP0.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The Datapath Controller. This module sets the datapath control
 *   bits for an incoming instruction. These control bits follow the
 *   instruction through each pipeline stage as needed, and constitute
 *   the effective operation of the processor through each pipeline stage.
 */
module Control(
    input wire  ID_Stall,
    input wire  [5:0] OpCode,
    input wire  [5:0] Funct,
    input wire  [4:0] Rs,        // used to differentiate mfc0 and mtc0
    input wire  [4:0] Rt,        // used to differentiate bgez,bgezal,bltz,bltzal,teqi,tgei,tgeiu,tlti,tltiu,tnei
    input wire  Cmp_EQ,
    input wire  Cmp_GZ,
    input wire  Cmp_GEZ,
    input wire  Cmp_LZ,
    input wire  Cmp_LEZ,
    //------------
    output wire IF_Flush,
    output reg [7:0] DP_Hazards,
    output wire [1:0] PCSrc,
    output wire SignExtend,
    output wire Link,
    output wire Movn,
    output wire Movz,
    output wire Mfc0,
    output wire Mtc0,
    output wire CP1,
    output wire CP2,
    output wire CP3,
    output wire Eret,
    output wire Trap,
    output wire TrapCond,
    output wire EXC_Sys,
    output wire EXC_Bp,
    output wire EXC_RI,
    output wire ID_CanErr,
    output wire EX_CanErr,
    output wire M_CanErr,
    output wire NextIsDelay,
    output wire RegDst,
    output wire ALUSrcImm,
    output reg [4:0] ALUOp,
    output wire LLSC,
    output wire MemWrite,
    output wire MemRead,
    output wire MemByte,
    output wire MemHalf,
    output wire MemSignExtend,
    output wire Left,
    output wire Right,
    output wire RegWrite,
    output wire MemtoReg
);


    wire Movc;
    wire Branch, Branch_EQ, Branch_GTZ, Branch_LEZ, Branch_NEQ, Branch_GEZ, Branch_LTZ;
    wire Unaligned_Mem;

    reg [15:0] Datapath;
    assign PCSrc[0]      = Datapath[14];
    assign Link          = Datapath[13];
    assign ALUSrcImm     = Datapath[12];
    assign Movc          = Datapath[11];
    assign Trap          = Datapath[10];
    assign TrapCond      = Datapath[9];
    assign RegDst        = Datapath[8];
    assign LLSC          = Datapath[7];
    assign MemRead       = Datapath[6];
    assign MemWrite      = Datapath[5];
    assign MemHalf       = Datapath[4];
    assign MemByte       = Datapath[3];
    assign MemSignExtend = Datapath[2];
    assign RegWrite      = Datapath[1];
    assign MemtoReg      = Datapath[0];

    reg [2:0] DP_Exceptions;
    assign ID_CanErr = DP_Exceptions[2];
    assign EX_CanErr = DP_Exceptions[1];
    assign  M_CanErr = DP_Exceptions[0];

    // Set the main datapath control signals based on the Op Code
    always @(*) begin
        if (ID_Stall)
            Datapath <= `DP_None;
        else begin
            case (OpCode)
                // R-Type
                `Op_Type_R  :
                    begin
                        case (Funct)
                            `Funct_Add     : Datapath <= `DP_Add;
                            `Funct_Addu    : Datapath <= `DP_Addu;
                            `Funct_And     : Datapath <= `DP_And;
                            `Funct_Break   : Datapath <= `DP_Break;
                            `Funct_Div     : Datapath <= `DP_Div;
                            `Funct_Divu    : Datapath <= `DP_Divu;
                            `Funct_Jalr    : Datapath <= `DP_Jalr;
                            `Funct_Jr      : Datapath <= `DP_Jr;
                            `Funct_Mfhi    : Datapath <= `DP_Mfhi;
                            `Funct_Mflo    : Datapath <= `DP_Mflo;
                            `Funct_Movn    : Datapath <= `DP_Movn;
                            `Funct_Movz    : Datapath <= `DP_Movz;
                            `Funct_Mthi    : Datapath <= `DP_Mthi;
                            `Funct_Mtlo    : Datapath <= `DP_Mtlo;
                            `Funct_Mult    : Datapath <= `DP_Mult;
                            `Funct_Multu   : Datapath <= `DP_Multu;
                            `Funct_Nor     : Datapath <= `DP_Nor;
                            `Funct_Or      : Datapath <= `DP_Or;
                            `Funct_Sll     : Datapath <= `DP_Sll;
                            `Funct_Sllv    : Datapath <= `DP_Sllv;
                            `Funct_Slt     : Datapath <= `DP_Slt;
                            `Funct_Sltu    : Datapath <= `DP_Sltu;
                            `Funct_Sra     : Datapath <= `DP_Sra;
                            `Funct_Srav    : Datapath <= `DP_Srav;
                            `Funct_Srl     : Datapath <= `DP_Srl;
                            `Funct_Srlv    : Datapath <= `DP_Srlv;
                            `Funct_Sub     : Datapath <= `DP_Sub;
                            `Funct_Subu    : Datapath <= `DP_Subu;
                            `Funct_Syscall : Datapath <= `DP_Syscall;
                            `Funct_Teq     : Datapath <= `DP_Teq;
                            `Funct_Tge     : Datapath <= `DP_Tge;
                            `Funct_Tgeu    : Datapath <= `DP_Tgeu;
                            `Funct_Tlt     : Datapath <= `DP_Tlt;
                            `Funct_Tltu    : Datapath <= `DP_Tltu;
                            `Funct_Tne     : Datapath <= `DP_Tne;
                            `Funct_Xor     : Datapath <= `DP_Xor;
                            default        : Datapath <= `DP_None;
                        endcase
                    end
                // R2-Type
                `Op_Type_R2 :
                    begin
                        case (Funct)
                            `Funct_Clo   : Datapath <= `DP_Clo;
                            `Funct_Clz   : Datapath <= `DP_Clz;
                            `Funct_Madd  : Datapath <= `DP_Madd;
                            `Funct_Maddu : Datapath <= `DP_Maddu;
                            `Funct_Msub  : Datapath <= `DP_Msub;
                            `Funct_Msubu : Datapath <= `DP_Msubu;
                            `Funct_Mul   : Datapath <= `DP_Mul;
                            default      : Datapath <= `DP_None;
                        endcase
                    end
                // I-Type
                `Op_Addi    : Datapath <= `DP_Addi;
                `Op_Addiu   : Datapath <= `DP_Addiu;
                `Op_Andi    : Datapath <= `DP_Andi;
                `Op_Ori     : Datapath <= `DP_Ori;
                `Op_Pref    : Datapath <= `DP_Pref;
                `Op_Slti    : Datapath <= `DP_Slti;
                `Op_Sltiu   : Datapath <= `DP_Sltiu;
                `Op_Xori    : Datapath <= `DP_Xori;
                // Jumps (using immediates)
                `Op_J       : Datapath <= `DP_J;
                `Op_Jal     : Datapath <= `DP_Jal;
                // Branches and Traps
                `Op_Type_BI :
                    begin
                        case (Rt)
                            `OpRt_Bgez   : Datapath <= `DP_Bgez;
                            `OpRt_Bgezal : Datapath <= `DP_Bgezal;
                            `OpRt_Bltz   : Datapath <= `DP_Bltz;
                            `OpRt_Bltzal : Datapath <= `DP_Bltzal;
                            `OpRt_Teqi   : Datapath <= `DP_Teqi;
                            `OpRt_Tgei   : Datapath <= `DP_Tgei;
                            `OpRt_Tgeiu  : Datapath <= `DP_Tgeiu;
                            `OpRt_Tlti   : Datapath <= `DP_Tlti;
                            `OpRt_Tltiu  : Datapath <= `DP_Tltiu;
                            `OpRt_Tnei   : Datapath <= `DP_Tnei;
                            default      : Datapath <= `DP_None;
                        endcase
                    end
                `Op_Beq     : Datapath <= `DP_Beq;
                `Op_Bgtz    : Datapath <= `DP_Bgtz;
                `Op_Blez    : Datapath <= `DP_Blez;
                `Op_Bne     : Datapath <= `DP_Bne;
                // Coprocessor 0
                `Op_Type_CP0 :
                    begin
                        case (Rs)
                            `OpRs_MF   : Datapath <= `DP_Mfc0;
                            `OpRs_MT   : Datapath <= `DP_Mtc0;
                            `OpRs_ERET : Datapath <= (Funct == `Funct_ERET) ? `DP_Eret : `DP_None;
                            default    : Datapath <= `DP_None;
                        endcase
                    end
                // Memory
                `Op_Lb   : Datapath <= `DP_Lb;
                `Op_Lbu  : Datapath <= `DP_Lbu;
                `Op_Lh   : Datapath <= `DP_Lh;
                `Op_Lhu  : Datapath <= `DP_Lhu;
                `Op_Ll   : Datapath <= `DP_Ll;
                `Op_Lui  : Datapath <= `DP_Lui;
                `Op_Lw   : Datapath <= `DP_Lw;
                `Op_Lwl  : Datapath <= `DP_Lwl;
                `Op_Lwr  : Datapath <= `DP_Lwr;
                `Op_Sb   : Datapath <= `DP_Sb;
                `Op_Sc   : Datapath <= `DP_Sc;
                `Op_Sh   : Datapath <= `DP_Sh;
                `Op_Sw   : Datapath <= `DP_Sw;
                `Op_Swl  : Datapath <= `DP_Swl;
                `Op_Swr  : Datapath <= `DP_Swr;
                default  : Datapath <= `DP_None;
            endcase
        end
    end

    // Set the Hazard Control Signals and Exception Indicators based on the Op Code
    always @(*) begin
        case (OpCode)
            // R-Type
            `Op_Type_R  :
                begin
                    case (Funct)
                        `Funct_Add     : begin DP_Hazards <= `HAZ_Add;     DP_Exceptions <= `EXC_Add;     end
                        `Funct_Addu    : begin DP_Hazards <= `HAZ_Addu;    DP_Exceptions <= `EXC_Addu;    end
                        `Funct_And     : begin DP_Hazards <= `HAZ_And;     DP_Exceptions <= `EXC_And;     end
                        `Funct_Break   : begin DP_Hazards <= `HAZ_Break;   DP_Exceptions <= `EXC_Break;   end
                        `Funct_Div     : begin DP_Hazards <= `HAZ_Div;     DP_Exceptions <= `EXC_Div;     end
                        `Funct_Divu    : begin DP_Hazards <= `HAZ_Divu;    DP_Exceptions <= `EXC_Divu;    end
                        `Funct_Jalr    : begin DP_Hazards <= `HAZ_Jalr;    DP_Exceptions <= `EXC_Jalr;    end
                        `Funct_Jr      : begin DP_Hazards <= `HAZ_Jr;      DP_Exceptions <= `EXC_Jr;      end
                        `Funct_Mfhi    : begin DP_Hazards <= `HAZ_Mfhi;    DP_Exceptions <= `EXC_Mfhi;    end
                        `Funct_Mflo    : begin DP_Hazards <= `HAZ_Mflo;    DP_Exceptions <= `EXC_Mflo;    end
                        `Funct_Movn    : begin DP_Hazards <= `HAZ_Movn;    DP_Exceptions <= `EXC_Movn;    end
                        `Funct_Movz    : begin DP_Hazards <= `HAZ_Movz;    DP_Exceptions <= `EXC_Movz;    end
                        `Funct_Mthi    : begin DP_Hazards <= `HAZ_Mthi;    DP_Exceptions <= `EXC_Mthi;    end
                        `Funct_Mtlo    : begin DP_Hazards <= `HAZ_Mtlo;    DP_Exceptions <= `EXC_Mtlo;    end
                        `Funct_Mult    : begin DP_Hazards <= `HAZ_Mult;    DP_Exceptions <= `EXC_Mult;    end
                        `Funct_Multu   : begin DP_Hazards <= `HAZ_Multu;   DP_Exceptions <= `EXC_Multu;   end
                        `Funct_Nor     : begin DP_Hazards <= `HAZ_Nor;     DP_Exceptions <= `EXC_Nor;     end
                        `Funct_Or      : begin DP_Hazards <= `HAZ_Or;      DP_Exceptions <= `EXC_Or;      end
                        `Funct_Sll     : begin DP_Hazards <= `HAZ_Sll;     DP_Exceptions <= `EXC_Sll;     end
                        `Funct_Sllv    : begin DP_Hazards <= `HAZ_Sllv;    DP_Exceptions <= `EXC_Sllv;    end
                        `Funct_Slt     : begin DP_Hazards <= `HAZ_Slt;     DP_Exceptions <= `EXC_Slt;     end
                        `Funct_Sltu    : begin DP_Hazards <= `HAZ_Sltu;    DP_Exceptions <= `EXC_Sltu;    end
                        `Funct_Sra     : begin DP_Hazards <= `HAZ_Sra;     DP_Exceptions <= `EXC_Sra;     end
                        `Funct_Srav    : begin DP_Hazards <= `HAZ_Srav;    DP_Exceptions <= `EXC_Srav;    end
                        `Funct_Srl     : begin DP_Hazards <= `HAZ_Srl;     DP_Exceptions <= `EXC_Srl;     end
                        `Funct_Srlv    : begin DP_Hazards <= `HAZ_Srlv;    DP_Exceptions <= `EXC_Srlv;    end
                        `Funct_Sub     : begin DP_Hazards <= `HAZ_Sub;     DP_Exceptions <= `EXC_Sub;     end
                        `Funct_Subu    : begin DP_Hazards <= `HAZ_Subu;    DP_Exceptions <= `EXC_Subu;    end
                        `Funct_Syscall : begin DP_Hazards <= `HAZ_Syscall; DP_Exceptions <= `EXC_Syscall; end
                        `Funct_Teq     : begin DP_Hazards <= `HAZ_Teq;     DP_Exceptions <= `EXC_Teq;     end
                        `Funct_Tge     : begin DP_Hazards <= `HAZ_Tge;     DP_Exceptions <= `EXC_Tge;     end
                        `Funct_Tgeu    : begin DP_Hazards <= `HAZ_Tgeu;    DP_Exceptions <= `EXC_Tgeu;    end
                        `Funct_Tlt     : begin DP_Hazards <= `HAZ_Tlt;     DP_Exceptions <= `EXC_Tlt;     end
                        `Funct_Tltu    : begin DP_Hazards <= `HAZ_Tltu;    DP_Exceptions <= `EXC_Tltu;    end
                        `Funct_Tne     : begin DP_Hazards <= `HAZ_Tne;     DP_Exceptions <= `EXC_Tne;     end
                        `Funct_Xor     : begin DP_Hazards <= `HAZ_Xor;     DP_Exceptions <= `EXC_Xor;     end
                        default        : begin DP_Hazards <= 8'hxx;        DP_Exceptions <= 3'bxxx;      end
                    endcase
                end
            // R2-Type
            `Op_Type_R2 :
                begin
                    case (Funct)
                        `Funct_Clo   : begin DP_Hazards <= `HAZ_Clo;   DP_Exceptions <= `EXC_Clo;   end
                        `Funct_Clz   : begin DP_Hazards <= `HAZ_Clz;   DP_Exceptions <= `EXC_Clz;   end
                        `Funct_Madd  : begin DP_Hazards <= `HAZ_Madd;  DP_Exceptions <= `EXC_Madd;  end
                        `Funct_Maddu : begin DP_Hazards <= `HAZ_Maddu; DP_Exceptions <= `EXC_Maddu; end
                        `Funct_Msub  : begin DP_Hazards <= `HAZ_Msub;  DP_Exceptions <= `EXC_Msub;  end
                        `Funct_Msubu : begin DP_Hazards <= `HAZ_Msubu; DP_Exceptions <= `EXC_Msubu; end
                        `Funct_Mul   : begin DP_Hazards <= `HAZ_Mul;   DP_Exceptions <= `EXC_Mul;   end
                        default      : begin DP_Hazards <= 8'hxx;      DP_Exceptions <= 3'bxxx;    end
                    endcase
                end
            // I-Type
            `Op_Addi    : begin DP_Hazards <= `HAZ_Addi;  DP_Exceptions <= `EXC_Addi;  end
            `Op_Addiu   : begin DP_Hazards <= `HAZ_Addiu; DP_Exceptions <= `EXC_Addiu; end
            `Op_Andi    : begin DP_Hazards <= `HAZ_Andi;  DP_Exceptions <= `EXC_Andi;  end
            `Op_Ori     : begin DP_Hazards <= `HAZ_Ori;   DP_Exceptions <= `EXC_Ori;   end
            `Op_Pref    : begin DP_Hazards <= `HAZ_Pref;  DP_Exceptions <= `EXC_Pref;  end
            `Op_Slti    : begin DP_Hazards <= `HAZ_Slti;  DP_Exceptions <= `EXC_Slti;  end
            `Op_Sltiu   : begin DP_Hazards <= `HAZ_Sltiu; DP_Exceptions <= `EXC_Sltiu; end
            `Op_Xori    : begin DP_Hazards <= `HAZ_Xori;  DP_Exceptions <= `EXC_Xori;  end
            // Jumps
            `Op_J       : begin DP_Hazards <= `HAZ_J;     DP_Exceptions <= `EXC_J;     end
            `Op_Jal     : begin DP_Hazards <= `HAZ_Jal;   DP_Exceptions <= `EXC_Jal;   end
            // Branches and Traps
            `Op_Type_BI :
                begin
                    case (Rt)
                        `OpRt_Bgez   : begin DP_Hazards <= `HAZ_Bgez;   DP_Exceptions <= `EXC_Bgez;   end
                        `OpRt_Bgezal : begin DP_Hazards <= `HAZ_Bgezal; DP_Exceptions <= `EXC_Bgezal; end
                        `OpRt_Bltz   : begin DP_Hazards <= `HAZ_Bltz;   DP_Exceptions <= `EXC_Bltz;   end
                        `OpRt_Bltzal : begin DP_Hazards <= `HAZ_Bltzal; DP_Exceptions <= `EXC_Bltzal; end
                        `OpRt_Teqi   : begin DP_Hazards <= `HAZ_Teqi;   DP_Exceptions <= `EXC_Teqi;   end
                        `OpRt_Tgei   : begin DP_Hazards <= `HAZ_Tgei;   DP_Exceptions <= `EXC_Tgei;   end
                        `OpRt_Tgeiu  : begin DP_Hazards <= `HAZ_Tgeiu;  DP_Exceptions <= `EXC_Tgeiu;  end
                        `OpRt_Tlti   : begin DP_Hazards <= `HAZ_Tlti;   DP_Exceptions <= `EXC_Tlti;   end
                        `OpRt_Tltiu  : begin DP_Hazards <= `HAZ_Tltiu;  DP_Exceptions <= `EXC_Tltiu;  end
                        `OpRt_Tnei   : begin DP_Hazards <= `HAZ_Tnei;   DP_Exceptions <= `EXC_Tnei;   end
                        default      : begin DP_Hazards <= 8'hxx;       DP_Exceptions <= 3'bxxx;     end
                    endcase
                end
            `Op_Beq     : begin DP_Hazards <= `HAZ_Beq;  DP_Exceptions <= `EXC_Beq;  end
            `Op_Bgtz    : begin DP_Hazards <= `HAZ_Bgtz; DP_Exceptions <= `EXC_Bgtz; end
            `Op_Blez    : begin DP_Hazards <= `HAZ_Blez; DP_Exceptions <= `EXC_Blez; end
            `Op_Bne     : begin DP_Hazards <= `HAZ_Bne;  DP_Exceptions <= `EXC_Bne;  end
            // Coprocessor 0
            `Op_Type_CP0 :
                begin
                    case (Rs)
                        `OpRs_MF   : begin DP_Hazards <= `HAZ_Mfc0; DP_Exceptions <= `EXC_Mfc0; end
                        `OpRs_MT   : begin DP_Hazards <= `HAZ_Mtc0; DP_Exceptions <= `EXC_Mtc0; end
                        `OpRs_ERET : begin DP_Hazards <= (Funct == `Funct_ERET) ? `HAZ_Eret : 8'hxx; DP_Exceptions <= `EXC_Eret; end
                        default    : begin DP_Hazards <= 8'hxx;     DP_Exceptions <= 3'bxxx;   end
                    endcase
                end
            // Memory
            `Op_Lb   : begin DP_Hazards <= `HAZ_Lb;  DP_Exceptions <= `EXC_Lb;  end
            `Op_Lbu  : begin DP_Hazards <= `HAZ_Lbu; DP_Exceptions <= `EXC_Lbu; end
            `Op_Lh   : begin DP_Hazards <= `HAZ_Lh;  DP_Exceptions <= `EXC_Lh;  end
            `Op_Lhu  : begin DP_Hazards <= `HAZ_Lhu; DP_Exceptions <= `EXC_Lhu; end
            `Op_Ll   : begin DP_Hazards <= `HAZ_Ll;  DP_Exceptions <= `EXC_Ll;  end
            `Op_Lui  : begin DP_Hazards <= `HAZ_Lui; DP_Exceptions <= `EXC_Lui; end
            `Op_Lw   : begin DP_Hazards <= `HAZ_Lw;  DP_Exceptions <= `EXC_Lw;  end
            `Op_Lwl  : begin DP_Hazards <= `HAZ_Lwl; DP_Exceptions <= `EXC_Lwl; end
            `Op_Lwr  : begin DP_Hazards <= `HAZ_Lwr; DP_Exceptions <= `EXC_Lwr; end
            `Op_Sb   : begin DP_Hazards <= `HAZ_Sb;  DP_Exceptions <= `EXC_Sb;  end
            `Op_Sc   : begin DP_Hazards <= `HAZ_Sc;  DP_Exceptions <= `EXC_Sc;  end
            `Op_Sh   : begin DP_Hazards <= `HAZ_Sh;  DP_Exceptions <= `EXC_Sh;  end
            `Op_Sw   : begin DP_Hazards <= `HAZ_Sw;  DP_Exceptions <= `EXC_Sw;  end
            `Op_Swl  : begin DP_Hazards <= `HAZ_Swl; DP_Exceptions <= `EXC_Swl; end
            `Op_Swr  : begin DP_Hazards <= `HAZ_Swr; DP_Exceptions <= `EXC_Swr; end
            default  : begin DP_Hazards <= 8'hxx;    DP_Exceptions <= 3'bxxx;  end
        endcase
    end

    // ALU Assignment
    always @(*) begin
        if (ID_Stall)
            ALUOp <= `AluOp_Addu;  // Any Op that doesn't write HILO or cause exceptions
        else begin
            case (OpCode)
                `Op_Type_R  :
                    begin
                        case (Funct)
                            `Funct_Add     : ALUOp <= `AluOp_Add;
                            `Funct_Addu    : ALUOp <= `AluOp_Addu;
                            `Funct_And     : ALUOp <= `AluOp_And;
                            `Funct_Div     : ALUOp <= `AluOp_Div;
                            `Funct_Divu    : ALUOp <= `AluOp_Divu;
                            `Funct_Jalr    : ALUOp <= `AluOp_Addu;
                            `Funct_Mfhi    : ALUOp <= `AluOp_Mfhi;
                            `Funct_Mflo    : ALUOp <= `AluOp_Mflo;
                            `Funct_Movn    : ALUOp <= `AluOp_Addu;
                            `Funct_Movz    : ALUOp <= `AluOp_Addu;
                            `Funct_Mthi    : ALUOp <= `AluOp_Mthi;
                            `Funct_Mtlo    : ALUOp <= `AluOp_Mtlo;
                            `Funct_Mult    : ALUOp <= `AluOp_Mult;
                            `Funct_Multu   : ALUOp <= `AluOp_Multu;
                            `Funct_Nor     : ALUOp <= `AluOp_Nor;
                            `Funct_Or      : ALUOp <= `AluOp_Or;
                            `Funct_Sll     : ALUOp <= `AluOp_Sll;
                            `Funct_Sllv    : ALUOp <= `AluOp_Sllv;
                            `Funct_Slt     : ALUOp <= `AluOp_Slt;
                            `Funct_Sltu    : ALUOp <= `AluOp_Sltu;
                            `Funct_Sra     : ALUOp <= `AluOp_Sra;
                            `Funct_Srav    : ALUOp <= `AluOp_Srav;
                            `Funct_Srl     : ALUOp <= `AluOp_Srl;
                            `Funct_Srlv    : ALUOp <= `AluOp_Srlv;
                            `Funct_Sub     : ALUOp <= `AluOp_Sub;
                            `Funct_Subu    : ALUOp <= `AluOp_Subu;
                            `Funct_Syscall : ALUOp <= `AluOp_Addu;
                            `Funct_Teq     : ALUOp <= `AluOp_Subu;
                            `Funct_Tge     : ALUOp <= `AluOp_Slt;
                            `Funct_Tgeu    : ALUOp <= `AluOp_Sltu;
                            `Funct_Tlt     : ALUOp <= `AluOp_Slt;
                            `Funct_Tltu    : ALUOp <= `AluOp_Sltu;
                            `Funct_Tne     : ALUOp <= `AluOp_Subu;
                            `Funct_Xor     : ALUOp <= `AluOp_Xor;
                            default        : ALUOp <= `AluOp_Addu;
                        endcase
                    end
                `Op_Type_R2 :
                    begin
                        case (Funct)
                            `Funct_Clo   : ALUOp <= `AluOp_Clo;
                            `Funct_Clz   : ALUOp <= `AluOp_Clz;
                            `Funct_Madd  : ALUOp <= `AluOp_Madd;
                            `Funct_Maddu : ALUOp <= `AluOp_Maddu;
                            `Funct_Msub  : ALUOp <= `AluOp_Msub;
                            `Funct_Msubu : ALUOp <= `AluOp_Msubu;
                            `Funct_Mul   : ALUOp <= `AluOp_Mul;
                            default      : ALUOp <= `AluOp_Addu;
                        endcase
                    end
                `Op_Type_BI  :
                    begin
                        case (Rt)
                            `OpRt_Teqi   : ALUOp <= `AluOp_Subu;
                            `OpRt_Tgei   : ALUOp <= `AluOp_Slt;
                            `OpRt_Tgeiu  : ALUOp <= `AluOp_Sltu;
                            `OpRt_Tlti   : ALUOp <= `AluOp_Slt;
                            `OpRt_Tltiu  : ALUOp <= `AluOp_Sltu;
                            `OpRt_Tnei   : ALUOp <= `AluOp_Subu;
                            default      : ALUOp <= `AluOp_Addu;  // Branches don't matter.
                        endcase
                    end
                `Op_Type_CP0 : ALUOp <= `AluOp_Addu;
                `Op_Addi     : ALUOp <= `AluOp_Add;
                `Op_Addiu    : ALUOp <= `AluOp_Addu;
                `Op_Andi     : ALUOp <= `AluOp_And;
                `Op_Jal      : ALUOp <= `AluOp_Addu;
                `Op_Lb       : ALUOp <= `AluOp_Addu;
                `Op_Lbu      : ALUOp <= `AluOp_Addu;
                `Op_Lh       : ALUOp <= `AluOp_Addu;
                `Op_Lhu      : ALUOp <= `AluOp_Addu;
                `Op_Ll       : ALUOp <= `AluOp_Addu;
                `Op_Lui      : ALUOp <= `AluOp_Sllc;
                `Op_Lw       : ALUOp <= `AluOp_Addu;
                `Op_Lwl      : ALUOp <= `AluOp_Addu;
                `Op_Lwr      : ALUOp <= `AluOp_Addu;
                `Op_Ori      : ALUOp <= `AluOp_Or;
                `Op_Sb       : ALUOp <= `AluOp_Addu;
                `Op_Sc       : ALUOp <= `AluOp_Addu;  // XXX Needs HW implement
                `Op_Sh       : ALUOp <= `AluOp_Addu;
                `Op_Slti     : ALUOp <= `AluOp_Slt;
                `Op_Sltiu    : ALUOp <= `AluOp_Sltu;
                `Op_Sw       : ALUOp <= `AluOp_Addu;
                `Op_Swl      : ALUOp <= `AluOp_Addu;
                `Op_Swr      : ALUOp <= `AluOp_Addu;
                `Op_Xori     : ALUOp <= `AluOp_Xor;
                default      : ALUOp <= `AluOp_Addu;
            endcase
        end
    end

    /***
     These remaining options cover portions of the datapath that are not
     controlled directly by the datapath bits. Note that some refer to bits of
     the opcode or other fields, which breaks the otherwise fully-abstracted view
     of instruction encodings. Make sure when adding custom instructions that
     no false positives/negatives are generated here.
     ***/

    // Branch Detection: Options are mutually exclusive.
    assign Branch_EQ  =  OpCode[2] & ~OpCode[1] & ~OpCode[0] &  Cmp_EQ;
    assign Branch_GTZ =  OpCode[2] &  OpCode[1] &  OpCode[0] &  Cmp_GZ;
    assign Branch_LEZ =  OpCode[2] &  OpCode[1] & ~OpCode[0] &  Cmp_LEZ;
    assign Branch_NEQ =  OpCode[2] & ~OpCode[1] &  OpCode[0] & ~Cmp_EQ;
    assign Branch_GEZ = ~OpCode[2] &  Rt[0] & Cmp_GEZ;
    assign Branch_LTZ = ~OpCode[2] & ~Rt[0] & Cmp_LZ;

    assign Branch = Branch_EQ | Branch_GTZ | Branch_LEZ | Branch_NEQ | Branch_GEZ | Branch_LTZ;
    assign PCSrc[1] = (Datapath[15] & ~Datapath[14]) ? Branch : Datapath[15];

    /* In MIPS32, all Branch and Jump operations execute the Branch Delay Slot,
     * or next instruction, regardless if the branch is taken or not. The exception
     * is the "Branch Likely" instruction group. These are deprecated, however, and not
     * implemented here. "IF_Flush" is defined to allow for the cancelation of a
     * Branch Delay Slot should these be implemented later.
     */
    assign IF_Flush = 0;

    // Indicator that next instruction is a Branch Delay Slot.
    assign NextIsDelay = Datapath[15] | Datapath[14];

    // Sign- or Zero-Extension Control. The only ops that require zero-extension are
    // Andi, Ori, and Xori. The following also zero-extends 'lui', however it does not alter the effect of lui.
    assign SignExtend = (OpCode[5:2] != 4'b0011);

    // Move Conditional
    assign Movn = Movc &  Funct[0];
    assign Movz = Movc & ~Funct[0];

    // Coprocessor 0 (Mfc0, Mtc0) control signals.
    assign Mfc0 = ((OpCode == `Op_Type_CP0) && (Rs == `OpRs_MF));
    assign Mtc0 = ((OpCode == `Op_Type_CP0) && (Rs == `OpRs_MT));
    assign Eret = ((OpCode == `Op_Type_CP0) && (Rs == `OpRs_ERET) && (Funct == `Funct_ERET));

    // Coprocessor 1,2,3 accesses (not implemented)
    assign CP1 = (OpCode == `Op_Type_CP1);
    assign CP2 = (OpCode == `Op_Type_CP2);
    assign CP3 = (OpCode == `Op_Type_CP3);

    // Exceptions found in ID
    assign EXC_Sys = ((OpCode == `Op_Type_R) && (Funct == `Funct_Syscall));
    assign EXC_Bp  = ((OpCode == `Op_Type_R) && (Funct == `Funct_Break));

    // Unaligned Memory Accesses (lwl, lwr, swl, swr)
    assign Unaligned_Mem = OpCode[5] & ~OpCode[4] & OpCode[1] & ~OpCode[0];
    assign Left  = Unaligned_Mem & ~OpCode[2];
    assign Right = Unaligned_Mem &  OpCode[2];

    // TODO: Reserved Instruction Exception must still be implemented
    assign EXC_RI  = 0;

endmodule

/*
 * File         : CPZero.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   16-Sep-2011  GEA       Initial design.
 *   2.0   14-May-2012  GEA       Complete rework.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The MIPS-32 Coprocessor 0 (CP0). This is the processor management unit that allows
 *   interrupts, traps, system calls, and other exceptions. It distinguishes
 *   user and kernel modes, provides status information, and can override program
 *   flow. This processor is designed for "bare metal" memory access and thus does
 *   not have virtual memory hardware as a part of it. However, the subset of CP0
 *   is MIPS-32-compliant.
 */
module CPZero(
    input wire  clock,
    //-- CP0 Functionality --//
    input wire  Mfc0,                    // CPU instruction is Mfc0
    input wire  Mtc0,                    // CPU instruction is Mtc0
    input wire  IF_Stall,
    input wire  ID_Stall,                // Commits are not made during stalls
    input wire  COP1,                    // Instruction for Coprocessor 1
    input wire  COP2,                    // Instruction for Coprocessor 2
    input wire  COP3,                    // Instruction for Coprocessor 3
    input wire  ERET,                    // Instruction is ERET (Exception Return)
    input wire  [4:0] Rd,                // Specifies Cp0 register
    input wire  [2:0] Sel,               // Specifies Cp0 'select'
    input wire  [31:0] Reg_In,           // Data from GP register to replace CP0 register
    output reg [31:0] Reg_Out,      // Data from CP0 register for GP register
    output wire KernelMode,              // Kernel mode indicator for pipeline transit
    output wire ReverseEndian,           // Reverse Endian memory indicator for User Mode
    //-- Hw Interrupts --//
    input wire  [4:0] Int,               // Five hardware interrupts external to the processor
    //-- Exceptions --//
    input wire  reset,                   // Cold Reset (EXC_Reset)
//  input wire  EXC_SReset,              // Soft Reset (not implemented)
    input wire  EXC_NMI,                 // Non-Maskable Interrupt
    input wire  EXC_AdIF,                // Address Error Exception from i-fetch (mapped to AdEL)
    input wire  EXC_AdEL,                // Address Error Exception from data memory load
    input wire  EXC_AdES,                // Address Error Exception from data memory store
    input wire  EXC_Ov,                  // Integer Overflow Exception
    input wire  EXC_Tr,                  // Trap Exception
    input wire  EXC_Sys,                 // System Call Exception
    input wire  EXC_Bp,                  // Breakpoint Exception
    input wire  EXC_RI,                  // Reserved Instruction Exception
    //-- Exception Data --//
    input wire  [31:0] ID_RestartPC,     // PC for exception, whether PC of instruction or of branch (PC-4) if BDS
    input wire  [31:0] EX_RestartPC,     // Same as 'ID_RestartPC' but in EX stage
    input wire  [31:0] M_RestartPC,      // Same as 'ID_RestartPC' but in MEM stage
    input wire  ID_IsFlushed,
    input wire  IF_IsBD,                 // Indicator of IF exception being a branch delay slot instruction
    input wire  ID_IsBD,                 // Indicator of ID exception being a branch delay slot instruction
    input wire  EX_IsBD,                 // Indicator of EX exception being a branch delay slot instruction
    input wire  M_IsBD,                  // Indicator of M  exception being a branch delay slot instruction
    input wire  [31:0] BadAddr_M,        // Bad 'Virtual' Address for exceptions AdEL, AdES in MEM stage
    input wire  [31:0] BadAddr_IF,       // Bad 'Virtual' Address for AdIF (i.e. AdEL) in IF stage
    input wire  ID_CanErr,               // Cumulative signal, i.e. (ID_ID_CanErr | ID_EX_CanErr | ID_M_CanErr)
    input wire  EX_CanErr,               // Cumulative signal, i.e. (EX_EX_CanErr | EX_M_CanErr)
    input wire  M_CanErr,                // Memory stage can error (i.e. cause exception)
    //-- Exception Control Flow --/
    output wire IF_Exception_Stall,
    output wire ID_Exception_Stall,
    output wire EX_Exception_Stall,
    output wire M_Exception_Stall,
    output wire IF_Exception_Flush,
    output wire ID_Exception_Flush,
    output wire EX_Exception_Flush,
    output wire M_Exception_Flush,
    output wire Exc_PC_Sel,              // Mux selector for exception PC override
    output reg [31:0] Exc_PC_Out,   // Address for PC at the beginning of an exception
    output wire [7:0] IP                 // Pending Interrupts from Cause register (for diagnostic purposes)
);



    /***
     Exception Control Flow Notes

     - Exceptions can occur in every pipeline stage. This implies that more than one exception
       can be raised in a single cycle. When this occurs, only the forward-most exception
       (i.e. MEM over EX) is handled. This and the following note guarantee program order.

     - An exception in any pipeline stage must stall that stage until all following stages are 
       exception-free. This is because it only makes sense for exceptions to occur in program order.

     - A pipeline stage which causes an exception must flush, i.e. prevent any commits it would
       have normally made and convert itself to a NOP for the next pipeline stage. Furthermore,
       it must flush all previous pipeline stages as well in order to retain program order.

     - Instructions reading CP0 (mtc0) read in ID without further action. Writes to CP0 (mtc0,
       eret) also write in ID, but only after forward pipeline stages have been cleared
       of possible exceptions. This prevents many insidious bugs, such as switching to User Mode
       in ID when a legitimate memory access in kernel mode is processing in MEM, or conversely
       a switch to Kernel Mode in ID when an instruction in User Mode is attempting a kernel region
       memory access (when a kernel mode signal does not propagate through the pipeline).

     - Commits occur in ID (CP0), EX (HILO), MEM, and WB (registers).

     - Hardware interrupts are detected and inserted in the ID stage, but only when there are no
       other possible exceptions in the pipeline. Because they appear 'asynchronous' to the
       processor, the remaining instructions in forward stages (EX, MEM, WB) can either be
       flushed or completed. It is simplest to have them complete to avoid restarts, but the
       interrupt latency is higher if e.g. the MEM stage stalls on a memory access (this would
       be unavoidable on single-cycle processors). This implementation allows all forward instructions
       to complete, for a greater instruction throughput but higher interrupt latency.

     - Software interrupts should appear synchronous in the program order, meaning that all
       instructions previous to them should complete and no instructions after them should start
       until the interrupts has been processed.

     Exception Name             Short Name          Pipeline Stage
       Address Error Ex         (AdEL, AdES)        MEM, IF
       Integer Overflow Ex      (Ov)                EX
       Trap Ex                  (Tr)                MEM
       Syscall                  (Sys)               ID
       Breakpoint               (Bp)                ID
       Reserved Instruction     (RI)                ID
       Coprocessor Unusable     (CpU)               ID
       Interrupt                (Int)               ID
       Reset, SReset, NMI                           ID
    ***/


    // Exceptions Generated Internally
    wire EXC_CpU;

    // Hardware Interrupt #5, caused by Timer/Perf counter
    wire Int5;

    // Top-level Authoritative Interrupt Signal
    wire EXC_Int;

    // General Exception detection (all but Interrupts, Reset, Soft Reset, and NMI)
    wire EXC_General = EXC_AdIF | EXC_AdEL | EXC_AdES | EXC_Ov | EXC_Tr | EXC_Sys | EXC_Bp | EXC_RI | EXC_CpU;

    // Misc
    wire CP0_WriteCond;
    reg  [3:0] Cause_ExcCode_bits;

    reg reset_r;
    always @(posedge clock) begin
        reset_r <= reset;
    end

    /***
     MIPS-32 COPROCESSOR 0 (Cp0) REGISTERS

     These are defined in "MIPS32 Architecture for Programmers Volume III:
     The MIPS32 Privileged Resource Architecture" from MIPS Technologies, Inc.

     Optional registers are omitted. Changes to the processor (such as adding
     an MMU/TLB, etc. must be reflected in these registers.
    */

    // BadVAddr Register (Register 8, Select 0)
    reg [31:0] BadVAddr;

    // Count Register (Register 9, Select 0)
    reg [31:0] Count;

    // Compare Register (Register 11, Select 0)
    reg [31:0] Compare;

    // Status Register (Register 12, Select 0)
    wire [2:0] Status_CU_321 = 3'b000;
    reg  Status_CU_0;       // Access Control to CPs, [2]->Cp3, ... [0]->Cp0
    wire Status_RP = 0;
    wire Status_FR = 0;
    reg  Status_RE;         // Reverse Endian Memory for User Mode
    wire Status_MX = 0;
    wire Status_PX = 0;
    reg  Status_BEV;        // Exception vector locations (0->Norm, 1->Bootstrap)
    wire Status_TS = 0;
    wire Status_SR = 0;     // Soft reset not implemented
    reg  Status_NMI;        // Non-Maskable Interrupt
    wire Status_RES = 0;
    wire [1:0] Status_Custom = 2'b00;
    reg [7:0] Status_IM;    // Interrupt mask
    wire Status_KX = 0;
    wire Status_SX = 0;
    wire Status_UX = 0;
    reg  Status_UM;         // Base operating mode (0->Kernel, 1->User)
    wire Status_R0 = 0;
    reg  Status_ERL;        // Error Level     (0->Normal, 1->Error (reset, NMI))
    reg  Status_EXL;        // Exception level (0->Normal, 1->Exception)
    reg  Status_IE;         // Interrupt Enable
    wire [31:0] Status = {Status_CU_321, Status_CU_0, Status_RP, Status_FR, Status_RE, Status_MX, 
                                 Status_PX, Status_BEV, Status_TS, Status_SR, Status_NMI, Status_RES,
                                 Status_Custom, Status_IM, Status_KX, Status_SX, Status_UX,
                                 Status_UM, Status_R0, Status_ERL, Status_EXL, Status_IE};

    // Cause Register (Register 13, Select 0)
    reg  Cause_BD;                  // Exception occured in Branch Delay
    reg  [1:0] Cause_CE;            // CP number for CP Unusable exception
    reg  Cause_IV;                  // Indicator of general IV (0->0x180) or special IV (1->0x200)
    wire Cause_WP = 0;
    reg  [7:0] Cause_IP;            // Pending HW Interrupt indicator.
    wire Cause_ExcCode4 = 0;        // Can be made into a register when this bit is needed.
    reg  [3:0] Cause_ExcCode30;     // Description of Exception (only lower 4 bits currently used; see above)
    wire [31:0] Cause  = {Cause_BD, 1'b0, Cause_CE, 4'b0000, Cause_IV, Cause_WP,
                                 6'b000000, Cause_IP, 1'b0, Cause_ExcCode4, Cause_ExcCode30, 2'b00};

    // Exception Program Counter (Register 14, Select 0)
    reg [31:0] EPC;

    // Processor Identification (Register 15, Select 0)
    wire [7:0] ID_Options = 8'b0000_0000;
    wire [7:0] ID_CID = 8'b0000_0000;
    wire [7:0] ID_PID = 8'b0000_0000;
    wire [7:0] ID_Rev = 8'b0000_0001;
    wire [31:0] PRId = {ID_Options, ID_CID, ID_PID, ID_Rev};

    // Configuration Register (Register 16, Select 0)
    wire Config_M = 1;
    wire [14:0] Config_Impl = 15'b000_0000_0000_0000;
    wire Config_BE = `Big_Endian;
    wire [1:0] Config_AT = 2'b00;
    wire [2:0] Config_AR = 3'b000;
    wire [2:0] Config_MT = 3'b000;
    wire [2:0] Config_K0 = 3'b000;
    wire [31:0] Config = {Config_M, Config_Impl, Config_BE, Config_AT, Config_AR, Config_MT,
                                 4'b0000, Config_K0};

    // Configuration Register 1 (Register 16, Select 1)
    wire Config1_M = 0;
    wire [5:0] Config1_MMU = 6'b000000;
    wire [2:0] Config1_IS = 3'b000;
    wire [2:0] Config1_IL = 3'b000;
    wire [2:0] Config1_IA = 3'b000;
    wire [2:0] Config1_DS = 3'b000;
    wire [2:0] Config1_DL = 3'b000;
    wire [2:0] Config1_DA = 3'b000;
    wire Config1_C2 = 0;
    wire Config1_MD = 0;
    wire Config1_PC = 0;    // XXX Performance Counters
    wire Config1_WR = 0;    // XXX Watch Registers
    wire Config1_CA = 0;
    wire Config1_EP = 0;
    wire Config1_FP = 0;
    wire [31:0] Config1 = {Config1_M, Config1_MMU, Config1_IS, Config1_IL, Config1_IA,
                                  Config1_DS, Config1_DL, Config1_DA, Config1_C2,
                                  Config1_MD, Config1_PC, Config1_WR, Config1_CA,
                                  Config1_EP, Config1_FP};

    // Performance Counter Register (Register 25) XXX TODO

    // ErrorEPC Register (Register 30, Select 0)
    reg [31:0] ErrorEPC;

    // Exception Detection and Processing
    wire M_Exception_Detect, EX_Exception_Detect, ID_Exception_Detect, IF_Exception_Detect;
    wire M_Exception_Mask, EX_Exception_Mask, ID_Exception_Mask, IF_Exception_Mask;
    wire M_Exception_Ready, EX_Exception_Ready, ID_Exception_Ready, IF_Exception_Ready;

    assign IP = Cause_IP;

    /*** Coprocessor Unusable Exception ***/
    assign EXC_CpU = COP1 | COP2 | COP3 | ((Mtc0 | Mfc0 | ERET) & ~(Status_CU_0 | KernelMode));

    /*** Kernel Mode Signal ***/
    assign KernelMode = ~Status_UM | Status_EXL | Status_ERL;

    /*** Reverse Endian for User Mode ***/
    assign ReverseEndian = Status_RE;

    /*** Interrupts ***/
    assign Int5 = (Count == Compare);
    wire   Enabled_Interrupt = EXC_NMI | (Status_IE & ((Cause_IP[7:0] & Status_IM[7:0]) != 8'h00));
    assign EXC_Int = Enabled_Interrupt & ~Status_EXL & ~Status_ERL & ~ID_IsFlushed;

    assign CP0_WriteCond = (Status_CU_0 | KernelMode) & Mtc0 & ~ID_Stall;


    /***
     Exception Hazard Flow Control Explanation:
        - An exception at any time in any stage causes its own and any previous stages to
          flush (clear own commits, NOPS to fwd stages).
        - An exception in a stage can also stall that stage (and inherently all previous stages) if and only if:
            1. A forward stage is capable of causing an exception AND
            2. A forward stage is not currently causing an exception.
        - An exception is ready to process when it is detected and not stalled in a stage.

        Flush specifics per pipeline stage:
            MEM: Mask 'MemWrite' and 'MemRead' (for performance) after EX/M and before data memory. NOPs to M/WB.
            EX : Mask writes to HI/LO. NOPs to EX/M.
            ID : Mask writes (reads?) to CP0. NOPs to ID/EX.
            IF : NOP to IF/ID.
    ***/

    /*** Exceptions grouped by pipeline stage ***/
    assign   M_Exception_Detect = EXC_AdEL | EXC_AdES | EXC_Tr;
    assign  EX_Exception_Detect = EXC_Ov;
    assign  ID_Exception_Detect = EXC_Sys | EXC_Bp | EXC_RI | EXC_CpU | EXC_Int;
    assign  IF_Exception_Detect = EXC_AdIF;

    /*** Exception mask conditions ***/

    // A potential bug would occur if e.g. EX stalls, MEM has data, but MEM is not stalled and finishes
    // going through the pipeline so forwarding would fail. This is not a problem however because
    // EX would not need data since it would flush on an exception.
    assign   M_Exception_Mask = IF_Stall;
    assign  EX_Exception_Mask = IF_Stall | M_CanErr;
    assign  ID_Exception_Mask = IF_Stall | M_CanErr | EX_CanErr;
    assign  IF_Exception_Mask = M_CanErr | EX_CanErr | ID_CanErr | EXC_Int;

    /***
     Exceptions which must wait for forward stages. A stage will not stall if a forward stage has an exception.
     These stalls must be inserted as stall conditions in the hazard unit so that it will take care of chaining.
     All writes to CP0 must also wait for forward hazard conditions to clear.
    */
    assign  M_Exception_Stall  =  M_Exception_Detect  & M_Exception_Mask;
    assign  EX_Exception_Stall =  EX_Exception_Detect & EX_Exception_Mask & ~M_Exception_Detect;
    assign  ID_Exception_Stall = (ID_Exception_Detect | ERET | Mtc0) & ID_Exception_Mask & ~(EX_Exception_Detect | M_Exception_Detect);
    assign  IF_Exception_Stall =  IF_Exception_Detect & IF_Exception_Mask & ~(ID_Exception_Detect | EX_Exception_Detect | M_Exception_Detect);


    /*** Exceptions which are ready to process (mutually exclusive) ***/
    // XXX can remove ~ID_Stall since in mask now (?)
    assign   M_Exception_Ready =  ~ID_Stall & M_Exception_Detect  & ~M_Exception_Mask;
    assign  EX_Exception_Ready =  ~ID_Stall & EX_Exception_Detect & ~EX_Exception_Mask;
    assign  ID_Exception_Ready =  ~ID_Stall & ID_Exception_Detect & ~ID_Exception_Mask;
    assign  IF_Exception_Ready =  ~ID_Stall & IF_Exception_Detect & ~IF_Exception_Mask;

    /***
     Flushes. A flush clears a pipeline stage's control signals and prevents the stage from committing any changes.
     Data such as 'RestartPC' and the detected exception must remain.
    */
    assign   M_Exception_Flush = M_Exception_Detect;
    assign  EX_Exception_Flush = M_Exception_Detect | EX_Exception_Detect;
    assign  ID_Exception_Flush = M_Exception_Detect | EX_Exception_Detect | ID_Exception_Detect;
    assign  IF_Exception_Flush = M_Exception_Detect | EX_Exception_Detect | ID_Exception_Detect | IF_Exception_Detect | (ERET & ~ID_Stall) | reset_r;


    /*** Software reads of CP0 Registers ***/
    always @(*) begin
        if (Mfc0 & (Status_CU_0 | KernelMode)) begin
            case (Rd)
                5'd8  : Reg_Out <= BadVAddr;
                5'd9  : Reg_Out <= Count;
                5'd11 : Reg_Out <= Compare;
                5'd12 : Reg_Out <= Status;
                5'd13 : Reg_Out <= Cause;
                5'd14 : Reg_Out <= EPC;
                5'd15 : Reg_Out <= PRId;
                5'd16 : Reg_Out <= (Sel == 3'b000) ? Config : Config1;
                5'd30 : Reg_Out <= ErrorEPC;
                default : Reg_Out <= 32'h0000_0000;
            endcase
        end
        else begin
            Reg_Out <= 32'h0000_0000;
        end
    end

    /*** Cp0 Register Assignments: Non-general exceptions (Reset, Soft Reset, NMI...) ***/
    always @(posedge clock) begin
        if (reset) begin
            Status_BEV <= 1;
            Status_NMI <= 0;
            Status_ERL <= 1;
            ErrorEPC   <= 32'b0;
        end
        else if (ID_Exception_Ready & EXC_NMI) begin
            Status_BEV <= 1;
            Status_NMI <= 1;
            Status_ERL <= 1;
            ErrorEPC   <= ID_RestartPC;
        end
        else begin
            Status_BEV    <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[22]   : Status_BEV;
            Status_NMI    <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[19]   : Status_NMI;
            Status_ERL    <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[2]    : ((Status_ERL & ERET & ~ID_Stall) ? 1'b0 : Status_ERL);
            ErrorEPC      <= (CP0_WriteCond & (Rd == 5'd30) & (Sel == 3'b000)) ? Reg_In       : ErrorEPC;
        end
    end

    /*** Cp0 Register Assignments: All other registers ***/
    always @(posedge clock) begin
        if (reset) begin
            Count         <= 32'b0;
            Compare       <= 32'b0;
            Status_CU_0   <= 0;
            Status_RE     <= 0;
            Status_IM     <= 8'b0;
            Status_UM     <= 0;
            Status_IE     <= 0;
            Cause_IV      <= 0;
            Cause_IP      <= 8'b0;
        end
        else begin
            Count         <= (CP0_WriteCond & (Rd == 5'd9 ) & (Sel == 3'b000)) ? Reg_In       : Count + 1;
            Compare       <= (CP0_WriteCond & (Rd == 5'd11) & (Sel == 3'b000)) ? Reg_In       : Compare;
            Status_CU_0   <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[28]   : Status_CU_0;
            Status_RE     <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[25]   : Status_RE;
            Status_IM     <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[15:8] : Status_IM;
            Status_UM     <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[4]    : Status_UM;
            Status_IE     <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[0]    : Status_IE;
            Cause_IV      <= (CP0_WriteCond & (Rd == 5'd13) & (Sel == 3'b000)) ? Reg_In[23]   : Cause_IV;
            /* Cause_IP indicates 8 interrupts:
               [7]   is set by the timer comparison (Compare == Count), and cleared by writing to Compare.
               [6:2] are set and cleared by external hardware.
               [1:0] are set and cleared by software.
             */
            Cause_IP[7]   <= (CP0_WriteCond & (Rd == 5'd11) & (Sel == 3'b000)) ? 1'b0 : ((Int5) ? 1'b1 : Cause_IP[7]);
            Cause_IP[6:2] <= Int[4:0];
            Cause_IP[1:0] <= (CP0_WriteCond & (Rd == 5'd13) & (Sel == 3'b000)) ? Reg_In[9:8]  : Cause_IP[1:0];
        end
    end

    /*** Cp0 Register Assignments: General Exception and Interrupt Processing ***/
    always @(posedge clock) begin
        if (reset) begin
            Cause_BD <= 0;
            Cause_CE <= 2'b00;
            Cause_ExcCode30 <= 4'b0000;
            Status_EXL <= 0;
            EPC <= 32'h0;
            BadVAddr <= 32'h0;
        end
        else begin
            // MEM stage
            if (M_Exception_Ready) begin
                Cause_BD <= (Status_EXL) ? Cause_BD : M_IsBD;
                Cause_CE <= (COP3) ? 2'b11 : ((COP2) ? 2'b10 : ((COP1) ? 2'b01 : 2'b00));
                Cause_ExcCode30 <= Cause_ExcCode_bits;
                Status_EXL <= 1;
                EPC <= (Status_EXL) ? EPC : M_RestartPC;
                BadVAddr <= BadAddr_M;
            end
            // EX stage
            else if (EX_Exception_Ready) begin
                Cause_BD <= (Status_EXL) ? Cause_BD : EX_IsBD;
                Cause_CE <= (COP3) ? 2'b11 : ((COP2) ? 2'b10 : ((COP1) ? 2'b01 : 2'b00));
                Cause_ExcCode30 <= Cause_ExcCode_bits;
                Status_EXL <= 1;
                EPC <= (Status_EXL) ? EPC : EX_RestartPC;
                BadVAddr <= BadVAddr;
            end
            // ID stage
            else if (ID_Exception_Ready) begin
                Cause_BD <= (Status_EXL) ? Cause_BD : ID_IsBD;
                Cause_CE <= (COP3) ? 2'b11 : ((COP2) ? 2'b10 : ((COP1) ? 2'b01 : 2'b00));
                Cause_ExcCode30 <= Cause_ExcCode_bits;
                Status_EXL <= 1;
                EPC <= (Status_EXL) ? EPC : ID_RestartPC;
                BadVAddr <= BadVAddr;
            end
            // IF stage
            else if (IF_Exception_Ready) begin
                Cause_BD <= (Status_EXL) ? Cause_BD : IF_IsBD;
                Cause_CE <= (COP3) ? 2'b11 : ((COP2) ? 2'b10 : ((COP1) ? 2'b01 : 2'b00));
                Cause_ExcCode30 <= Cause_ExcCode_bits;
                Status_EXL <= 1;
                EPC <= (Status_EXL) ? EPC : BadAddr_IF;
                BadVAddr <= BadAddr_IF;
            end
            // No exceptions this cycle
            else begin
                Cause_BD <= 1'b0;
                Cause_CE <= Cause_CE;
                Cause_ExcCode30 <= Cause_ExcCode30;
                // Without new exceptions, 'Status_EXL' is set by software or cleared by ERET.
                Status_EXL <= (CP0_WriteCond & (Rd == 5'd12) & (Sel == 3'b000)) ? Reg_In[1] : ((Status_EXL & ERET & ~ID_Stall) ? 1'b0 : Status_EXL);
                // The EPC is also writable by software
                EPC <= (CP0_WriteCond & (Rd == 5'd14) & (Sel == 3'b000)) ? Reg_In : EPC;
                BadVAddr <= BadVAddr;
            end
        end
    end


    /*** Program Counter for all Exceptions/Interrupts ***/
    always @(*) begin
        // Following is redundant since PC has initial value now.
        if (reset) begin
            Exc_PC_Out <= `EXC_Vector_Base_Reset;
        end
        else if (ERET & ~ID_Stall) begin
            Exc_PC_Out <= (Status_ERL) ? ErrorEPC : EPC;
        end
        else if (EXC_General) begin
            Exc_PC_Out <= (Status_BEV) ? (`EXC_Vector_Base_Other_Boot   + `EXC_Vector_Offset_General) :
                                         (`EXC_Vector_Base_Other_NoBoot + `EXC_Vector_Offset_General);
        end
        else if (EXC_NMI) begin
            Exc_PC_Out <= `EXC_Vector_Base_Reset;
        end
        else if (EXC_Int & Cause_IV) begin
            Exc_PC_Out <= (Status_BEV) ? (`EXC_Vector_Base_Other_Boot   + `EXC_Vector_Offset_Special) :
                                         (`EXC_Vector_Base_Other_NoBoot + `EXC_Vector_Offset_Special);
        end
        else begin
            Exc_PC_Out <= (Status_BEV) ? (`EXC_Vector_Base_Other_Boot   + `EXC_Vector_Offset_General) :
                                         (`EXC_Vector_Base_Other_NoBoot + `EXC_Vector_Offset_General);
        end
    end

    //assign Exc_PC_Sel = (reset | (ERET & ~ID_Stall) | EXC_General | EXC_Int);
    assign Exc_PC_Sel = reset | (ERET & ~ID_Stall) | IF_Exception_Ready | ID_Exception_Ready | EX_Exception_Ready | M_Exception_Ready;

    /*** Cause Register ExcCode Field ***/
    always @(*) begin
        // Ordered by Pipeline Stage with Interrupts last
        if      (EXC_AdEL) Cause_ExcCode_bits <= 4'h4;     // 00100
        else if (EXC_AdES) Cause_ExcCode_bits <= 4'h5;     // 00101
        else if (EXC_Tr)   Cause_ExcCode_bits <= 4'hd;     // 01101
        else if (EXC_Ov)   Cause_ExcCode_bits <= 4'hc;     // 01100
        else if (EXC_Sys)  Cause_ExcCode_bits <= 4'h8;     // 01000
        else if (EXC_Bp)   Cause_ExcCode_bits <= 4'h9;     // 01001
        else if (EXC_RI)   Cause_ExcCode_bits <= 4'ha;     // 01010
        else if (EXC_CpU)  Cause_ExcCode_bits <= 4'hb;     // 01011
        else if (EXC_AdIF) Cause_ExcCode_bits <= 4'h4;     // 00100
        else if (EXC_Int)  Cause_ExcCode_bits <= 4'h0;     // 00000     // OK that NMI writes this.
        else               Cause_ExcCode_bits <= 4'bxxxx;
    end

endmodule

`timescale 1ns / 1ns
/*
 * File         : Divide.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Neil Russell
 *
 * Modification History:
 *  Rev   Date         Initials  Description of Change
 *  1.0   6-Nov-2012   NJR       Initial design.
 *
 * Description:
 *  A multi-cycle 32-bit divider.
 *
 *  On any cycle that one of OP_div or OP_divu are true, the Dividend and
 *  Divisor will be captured and a multi-cycle divide operation initiated.
 *  Stall will go true on the next cycle and the first cycle of the divide
 *  operation completed.  After some time (about 32 cycles), Stall will go
 *  false on the same cycle that the result becomes valid.  OP_div or OP_divu
 *  will abort any currently running divide operation and initiate a new one.
 */
module Divide(
    input wire           clock,
    input wire           reset,
    input wire           OP_div,     // True to initiate a signed divide
    input wire           OP_divu,    // True to initiate an unsigned divide
    input wire  [31:0]   Dividend,
    input wire  [31:0]   Divisor,
    output wire [31:0]   Quotient,
    output wire [31:0]   Remainder,
    output wire          Stall       // True while calculating
    );


    reg             active;     // True if the divider is running
    reg             neg;        // True if the result will be negative
    reg [4:0]       cycle;      // Number of cycles to go

    reg [31:0]      result;     // Begin with dividend, end with quotient
    reg [31:0]      denom;      // Divisor
    reg [31:0]      work;       // Running remainder

    // Calculate the current digit
    wire [32:0]     sub = { work[30:0], result[31] } - denom;

    // Send the results to our master
    assign Quotient = !neg ? result : -result;
    assign Remainder = work;
    assign Stall = active;

    // The state machine
    always @(posedge clock) begin
        if (reset) begin
            active <= 0;
            neg <= 0;
            cycle <= 0;
            result <= 0;
            denom <= 0;
            work <= 0;
        end
        else begin
            if (OP_div) begin
                // Set up for a signed divide.  Remember the resulting sign,
                // and make the operands positive.
                cycle <= 5'd31;
                result <= (Dividend[31] == 0) ? Dividend : -Dividend;
                denom <= (Divisor[31] == 0) ? Divisor : -Divisor;
                work <= 32'b0;
                neg <= Dividend[31] ^ Divisor[31];
                active <= 1;
            end
            else if (OP_divu) begin
                // Set up for an unsigned divide.
                cycle <= 5'd31;
                result <= Dividend;
                denom <= Divisor;
                work <= 32'b0;
                neg <= 0;
                active <= 1;
            end
            else if (active) begin
                // Run an iteration of the divide.
                if (sub[32] == 0) begin
                    work <= sub[31:0];
                    result <= {result[30:0], 1'b1};
                end
                else begin
                    work <= {work[30:0], result[31]};
                    result <= {result[30:0], 1'b0};
                end

                if (cycle == 0) begin
                    active <= 0;
                end

                cycle <= cycle - 5'd1;
            end
        end
    end

endmodule
/*
 * File         : EXMEM_Stage.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   9-Jun-2011   GEA       Initial design.
 *   2.0   26-Jul-2012  GEA       Many updates have been made.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The Pipeline Register to bridge the Execute and Memory stages.
 */
module EXMEM_Stage(
    input wire  clock,
    input wire  reset,
    input wire  EX_Flush,
    input wire  EX_Stall,
    input wire  M_Stall,
    // Control Signals
    input wire  EX_Movn,
    input wire  EX_Movz,
    input wire  EX_BZero,
    input wire  EX_RegWrite,  // Future Control to WB
    input wire  EX_MemtoReg,  // Future Control to WB
    input wire  EX_ReverseEndian,
    input wire  EX_LLSC,
    input wire  EX_MemRead,
    input wire  EX_MemWrite,
    input wire  EX_MemByte,
    input wire  EX_MemHalf,
    input wire  EX_MemSignExtend,
    input wire  EX_Left,
    input wire  EX_Right,
    // Exception Control/Info
    input wire  EX_KernelMode,
    input wire  [31:0] EX_RestartPC,
    input wire  EX_IsBDS,
    input wire  EX_Trap,
    input wire  EX_TrapCond,
    input wire  EX_M_CanErr,
    // Data Signals
    input wire  [31:0] EX_ALU_Result,
    input wire  [31:0] EX_ReadData2,
    input wire  [4:0]  EX_RtRd,
    // ------------------
    output reg M_RegWrite,
    output reg M_MemtoReg,
    output reg M_ReverseEndian,
    output reg M_LLSC,
    output reg M_MemRead,
    output reg M_MemWrite,
    output reg M_MemByte,
    output reg M_MemHalf,
    output reg M_MemSignExtend,
    output reg M_Left,
    output reg M_Right,
    output reg M_KernelMode,
    output reg [31:0] M_RestartPC,
    output reg M_IsBDS,
    output reg M_Trap,
    output reg M_TrapCond,
    output reg M_M_CanErr,
    output reg [31:0] M_ALU_Result,
    output reg [31:0] M_ReadData2,
    output reg [4:0]  M_RtRd
    );

    /***
     The purpose of a pipeline register is to capture data from one pipeline stage
     and provide it to the next pipeline stage. This creates at least one clock cycle
     of delay, but reduces the combinatorial path length of signals which allows for
     higher clock speeds.

     All pipeline registers update unless the forward stage is stalled. When this occurs
     or when the current stage is being flushed, the forward stage will receive data that
     is effectively a NOP and causes nothing to happen throughout the remaining pipeline
     traversal. In other words:

     A stall masks all control signals to forward stages. A flush permanently clears
     control signals to forward stages (but not certain data for exception purposes).
    ***/

    // Mask of RegWrite if a Move Conditional failed.
    wire MovcRegWrite = (EX_Movn & ~EX_BZero) | (EX_Movz & EX_BZero);

    always @(posedge clock) begin
        M_RegWrite      <= (reset) ? 1'b0  : ((M_Stall) ? M_RegWrite      : ((EX_Stall | EX_Flush) ? 1'b0 : ((EX_Movn | EX_Movz) ? MovcRegWrite : EX_RegWrite)));
        M_MemtoReg      <= (reset) ? 1'b0  : ((M_Stall) ? M_MemtoReg                                      : EX_MemtoReg);
        M_ReverseEndian <= (reset) ? 1'b0  : ((M_Stall) ? M_ReverseEndian                                 : EX_ReverseEndian);
        M_LLSC          <= (reset) ? 1'b0  : ((M_Stall) ? M_LLSC                                          : EX_LLSC);
        M_MemRead       <= (reset) ? 1'b0  : ((M_Stall) ? M_MemRead       : ((EX_Stall | EX_Flush) ? 1'b0 : EX_MemRead));
        M_MemWrite      <= (reset) ? 1'b0  : ((M_Stall) ? M_MemWrite      : ((EX_Stall | EX_Flush) ? 1'b0 : EX_MemWrite));
        M_MemByte       <= (reset) ? 1'b0  : ((M_Stall) ? M_MemByte                                       : EX_MemByte);
        M_MemHalf       <= (reset) ? 1'b0  : ((M_Stall) ? M_MemHalf                                       : EX_MemHalf);
        M_MemSignExtend <= (reset) ? 1'b0  : ((M_Stall) ? M_MemSignExtend                                 : EX_MemSignExtend);
        M_Left          <= (reset) ? 1'b0  : ((M_Stall) ? M_Left                                          : EX_Left);
        M_Right         <= (reset) ? 1'b0  : ((M_Stall) ? M_Right                                         : EX_Right);
        M_KernelMode    <= (reset) ? 1'b0  : ((M_Stall) ? M_KernelMode                                    : EX_KernelMode);
        M_RestartPC     <= (reset) ? 32'b0 : ((M_Stall) ? M_RestartPC                                     : EX_RestartPC);
        M_IsBDS         <= (reset) ? 1'b0  : ((M_Stall) ? M_IsBDS                                         : EX_IsBDS);
        M_Trap          <= (reset) ? 1'b0  : ((M_Stall) ? M_Trap          : ((EX_Stall | EX_Flush) ? 1'b0 : EX_Trap));
        M_TrapCond      <= (reset) ? 1'b0  : ((M_Stall) ? M_TrapCond                                      : EX_TrapCond);
        M_M_CanErr      <= (reset) ? 1'b0  : ((M_Stall) ? M_M_CanErr      : ((EX_Stall | EX_Flush) ? 1'b0 : EX_M_CanErr));
        M_ALU_Result    <= (reset) ? 32'b0 : ((M_Stall) ? M_ALU_Result                                    : EX_ALU_Result);
        M_ReadData2     <= (reset) ? 32'b0 : ((M_Stall) ? M_ReadData2                                     : EX_ReadData2);
        M_RtRd          <= (reset) ? 5'b0  : ((M_Stall) ? M_RtRd                                          : EX_RtRd);
    end

endmodule

/*
 * File         : Hazard_Detection.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   23-Jul-2011  GEA       Initial design.
 *   2.0   26-May-2012  GEA       Release version with CP0.
 *   2.01   1-Nov-2012  GEA       Fixed issue with Jal.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   Hazard Detection and Forward Control. This is the glue that allows a
 *   pipelined processor to operate efficiently and correctly in the presence
 *   of data, structural, and control hazards. For each pipeline stage, it
 *   detects whether that stage requires data that is still in the pipeline,
 *   and whether that data may be forwarded or if the pipeline must be stalled.
 *
 *   This module is heavily commented. Read below for more information.
 */
module Hazard_Detection(
    input wire  [7:0] DP_Hazards,
    input wire  [4:0] ID_Rs,
    input wire  [4:0] ID_Rt,
    input wire  [4:0] EX_Rs,
    input wire  [4:0] EX_Rt,
    input wire  [4:0] EX_RtRd,
    input wire  [4:0] MEM_RtRd,
    input wire  [4:0] WB_RtRd,
    input wire  EX_Link,
    input wire  EX_RegWrite,
    input wire  MEM_RegWrite,
    input wire  WB_RegWrite,
    input wire  MEM_MemRead,
    input wire  MEM_MemWrite,        // Needed for Store Conditional which writes to a register
    input wire  InstMem_Read,
    input wire  InstMem_Ready,
    input wire  Mfc0,                // Using fwd mux; not part of haz/fwd.
    input wire  IF_Exception_Stall,
    input wire  ID_Exception_Stall,
    input wire  EX_Exception_Stall,
    input wire  EX_ALU_Stall,
    input wire  M_Stall_Controller,  // Determined by data memory controller
    output wire IF_Stall,
    output wire ID_Stall,
    output wire EX_Stall,
    output wire M_Stall,
    output wire WB_Stall,
    output wire [1:0] ID_RsFwdSel,
    output wire [1:0] ID_RtFwdSel,
    output wire [1:0] EX_RsFwdSel,
    output wire [1:0] EX_RtFwdSel,
    output wire M_WriteDataFwdSel
    );

    /* Hazard and Forward Detection
     *
     * Most instructions read from one or more registers. Normally this occurs in
     * the ID stage. However, frequently the register file in the ID stage is stale
     * when one or more forward stages in the pipeline (EX, MEM, or WB) contains
     * an instruction which will eventually update it but has not yet done so.
     *
     * A hazard condition is created when a forward pipeline stage is set to write
     * the same register that a current pipeline stage (e.g. in ID) needs to read.
     * The solution is to stall the current stage (and effectively all stages behind
     * it) or bypass (forward) the data from forward stages. Fortunately forwarding
     * works for most combinations of instructions.
     *
     * Hazard and Forward conditions are handled based on two simple rules:
     * "Wants" and "Needs." If an instruction "wants" data in a certain pipeline
     * stage, and that data is available further along in the pipeline, it will
     * be forwarded. If it "needs" data and the data is not yet available for forwarding,
     * the pipeline stage stalls. If it does not want or need data in a certain
     * stage, forwarding is disabled and a stall will not occur. This is important
     * for instructions which insert custom data, such as jal or movz.
     *
     * Currently, "Want" and "Need" conditions are defined for both Rs data and Rt
     * data (the two read registers in MIPS), and these conditions exist in the
     * ID and EX pipeline stages. This is a total of eight condition bits.
     *
     * A unique exception exists with Store instructions, which don't need the
     * "Rt" data until the MEM stage. Because data doesn't change in WB, and WB
     * is the only stage following MEM, forwarding is *always* possible from
     * WB to Mem. This unit handles this situation, and a condition bit is not
     * needed.
     *
     * When data is needed from the MEM stage by a previous stage (ID or EX), the
     * decision to forward or stall is based on whether MEM is accessing memory
     * (stall) or not (forward). Normally store instructions don't write to registers
     * and thus are never needed for a data dependence, so the signal 'MEM_MemRead'
     * is sufficient to determine. Because of the Store Conditional instruction,
     * however, 'MEM_MemWrite' must also be considered because it writes to a register.
     *
     */

    wire WantRsByID, NeedRsByID, WantRtByID, NeedRtByID, WantRsByEX, NeedRsByEX, WantRtByEX, NeedRtByEX;
    assign WantRsByID = DP_Hazards[7];
    assign NeedRsByID = DP_Hazards[6];
    assign WantRtByID = DP_Hazards[5];
    assign NeedRtByID = DP_Hazards[4];
    assign WantRsByEX = DP_Hazards[3];
    assign NeedRsByEX = DP_Hazards[2];
    assign WantRtByEX = DP_Hazards[1];
    assign NeedRtByEX = DP_Hazards[0];

    // Trick allowed by RegDst = 0 which gives Rt. MEM_Rt is only used on
    // Data Memory write operations (stores), and RegWrite is always 0 in this case.
    wire [4:0] MEM_Rt = MEM_RtRd;

    // Forwarding should not happen when the src/dst register is $zero
    wire EX_RtRd_NZ  = (EX_RtRd  != 5'b00000);
    wire MEM_RtRd_NZ = (MEM_RtRd != 5'b00000);
    wire WB_RtRd_NZ  = (WB_RtRd  != 5'b00000);

    // ID Dependencies
    wire Rs_IDEX_Match  = (ID_Rs == EX_RtRd)  & EX_RtRd_NZ  & (WantRsByID | NeedRsByID) & EX_RegWrite;
    wire Rt_IDEX_Match  = (ID_Rt == EX_RtRd)  & EX_RtRd_NZ  & (WantRtByID | NeedRtByID) & EX_RegWrite;
    wire Rs_IDMEM_Match = (ID_Rs == MEM_RtRd) & MEM_RtRd_NZ & (WantRsByID | NeedRsByID) & MEM_RegWrite;
    wire Rt_IDMEM_Match = (ID_Rt == MEM_RtRd) & MEM_RtRd_NZ & (WantRtByID | NeedRtByID) & MEM_RegWrite;
    wire Rs_IDWB_Match  = (ID_Rs == WB_RtRd)  & WB_RtRd_NZ  & (WantRsByID | NeedRsByID) & WB_RegWrite;
    wire Rt_IDWB_Match  = (ID_Rt == WB_RtRd)  & WB_RtRd_NZ  & (WantRtByID | NeedRtByID) & WB_RegWrite;
    // EX Dependencies
    wire Rs_EXMEM_Match = (EX_Rs == MEM_RtRd) & MEM_RtRd_NZ & (WantRsByEX | NeedRsByEX) & MEM_RegWrite;
    wire Rt_EXMEM_Match = (EX_Rt == MEM_RtRd) & MEM_RtRd_NZ & (WantRtByEX | NeedRtByEX) & MEM_RegWrite;
    wire Rs_EXWB_Match  = (EX_Rs == WB_RtRd)  & WB_RtRd_NZ  & (WantRsByEX | NeedRsByEX) & WB_RegWrite;
    wire Rt_EXWB_Match  = (EX_Rt == WB_RtRd)  & WB_RtRd_NZ  & (WantRtByEX | NeedRtByEX) & WB_RegWrite;
    // MEM Dependencies
    wire Rt_MEMWB_Match = (MEM_Rt == WB_RtRd) & WB_RtRd_NZ  & WB_RegWrite;


    // ID needs data from EX  : Stall
    wire ID_Stall_1 = (Rs_IDEX_Match  &  NeedRsByID);
    wire ID_Stall_2 = (Rt_IDEX_Match  &  NeedRtByID);
    // ID needs data from MEM : Stall if mem access
    wire ID_Stall_3 = (Rs_IDMEM_Match &  (MEM_MemRead | MEM_MemWrite) & NeedRsByID);
    wire ID_Stall_4 = (Rt_IDMEM_Match &  (MEM_MemRead | MEM_MemWrite) & NeedRtByID);
    // ID wants data from MEM : Forward if not mem access
    wire ID_Fwd_1   = (Rs_IDMEM_Match & ~(MEM_MemRead | MEM_MemWrite));
    wire ID_Fwd_2   = (Rt_IDMEM_Match & ~(MEM_MemRead | MEM_MemWrite));
    // ID wants/needs data from WB  : Forward
    wire ID_Fwd_3   = (Rs_IDWB_Match);
    wire ID_Fwd_4   = (Rt_IDWB_Match);
    // EX needs data from MEM : Stall if mem access
    wire EX_Stall_1 = (Rs_EXMEM_Match &  (MEM_MemRead | MEM_MemWrite) & NeedRsByEX);
    wire EX_Stall_2 = (Rt_EXMEM_Match &  (MEM_MemRead | MEM_MemWrite) & NeedRtByEX);
    // EX wants data from MEM : Forward if not mem access
    wire EX_Fwd_1   = (Rs_EXMEM_Match & ~(MEM_MemRead | MEM_MemWrite));
    wire EX_Fwd_2   = (Rt_EXMEM_Match & ~(MEM_MemRead | MEM_MemWrite));
    // EX wants/needs data from WB  : Forward
    wire EX_Fwd_3   = (Rs_EXWB_Match);
    wire EX_Fwd_4   = (Rt_EXWB_Match);
    // MEM needs data from WB : Forward
    wire MEM_Fwd_1  = (Rt_MEMWB_Match);


    // Stalls and Control Flow Final Assignments
    assign WB_Stall = M_Stall;
    assign  M_Stall = IF_Stall | M_Stall_Controller;
    assign EX_Stall = (EX_Stall_1 | EX_Stall_2 | EX_Exception_Stall) | EX_ALU_Stall | M_Stall;
    assign ID_Stall = (ID_Stall_1 | ID_Stall_2 | ID_Stall_3 | ID_Stall_4 | ID_Exception_Stall) | EX_Stall;
    assign IF_Stall = InstMem_Read | InstMem_Ready | IF_Exception_Stall;

    // Forwarding Control Final Assignments
    assign ID_RsFwdSel = (ID_Fwd_1) ? 2'b01 : ((ID_Fwd_3) ? 2'b10 : 2'b00);
    assign ID_RtFwdSel = (Mfc0) ? 2'b11 : ((ID_Fwd_2) ? 2'b01 : ((ID_Fwd_4) ? 2'b10 : 2'b00));
    assign EX_RsFwdSel = (EX_Link) ? 2'b11 : ((EX_Fwd_1) ? 2'b01 : ((EX_Fwd_3) ? 2'b10 : 2'b00));
    assign EX_RtFwdSel = (EX_Link)  ? 2'b11 : ((EX_Fwd_2) ? 2'b01 : ((EX_Fwd_4) ? 2'b10 : 2'b00));
    assign M_WriteDataFwdSel = MEM_Fwd_1;

endmodule

/*
 * File         : IDEX_Stage.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   9-Jun-2011   GEA       Initial design.
 *   2.0   26-Jul-2012  GEA       Many updates have been made.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The Pipeline Register to bridge the Instruction Decode
 *   and Execute stages.
 */
module IDEX_Stage(
    input wire  clock,
    input wire  reset,
    input wire  ID_Flush,
    input wire  ID_Stall,
    input wire  EX_Stall,
    // Control Signals
    input wire  ID_Link,
    input wire  ID_RegDst,
    input wire  ID_ALUSrcImm,
    input wire  [4:0] ID_ALUOp,
    input wire  ID_Movn,
    input wire  ID_Movz,
    input wire  ID_LLSC,
    input wire  ID_MemRead,
    input wire  ID_MemWrite,
    input wire  ID_MemByte,
    input wire  ID_MemHalf,
    input wire  ID_MemSignExtend,
    input wire  ID_Left,
    input wire  ID_Right,
    input wire  ID_RegWrite,
    input wire  ID_MemtoReg,
    input wire  ID_ReverseEndian,
    // Hazard & Forwarding
    input wire  [4:0] ID_Rs,
    input wire  [4:0] ID_Rt,
    input wire  ID_WantRsByEX,
    input wire  ID_NeedRsByEX,
    input wire  ID_WantRtByEX,
    input wire  ID_NeedRtByEX,
    // Exception Control/Info
    input wire  ID_KernelMode,
    input wire  [31:0] ID_RestartPC,
    input wire  ID_IsBDS,
    input wire  ID_Trap,
    input wire  ID_TrapCond,
    input wire  ID_EX_CanErr,
    input wire  ID_M_CanErr,
    // Data Signals
    input wire  [31:0] ID_ReadData1,
    input wire  [31:0] ID_ReadData2,
    input wire  [16:0] ID_SignExtImm, // ID_Rd, ID_Shamt included here
    // ----------------
    output reg EX_Link,
    output wire [1:0] EX_LinkRegDst,
    output reg EX_ALUSrcImm,
    output reg [4:0] EX_ALUOp,
    output reg EX_Movn,
    output reg EX_Movz,
    output reg EX_LLSC,
    output reg EX_MemRead,
    output reg EX_MemWrite,
    output reg EX_MemByte,
    output reg EX_MemHalf,
    output reg EX_MemSignExtend,
    output reg EX_Left,
    output reg EX_Right,
    output reg EX_RegWrite,
    output reg EX_MemtoReg,
    output reg EX_ReverseEndian,
    output reg [4:0]  EX_Rs,
    output reg [4:0]  EX_Rt,
    output reg EX_WantRsByEX,
    output reg EX_NeedRsByEX,
    output reg EX_WantRtByEX,
    output reg EX_NeedRtByEX,
    output reg EX_KernelMode,
    output reg [31:0] EX_RestartPC,
    output reg EX_IsBDS,
    output reg EX_Trap,
    output reg EX_TrapCond,
    output reg EX_EX_CanErr,
    output reg EX_M_CanErr,
    output reg [31:0] EX_ReadData1,
    output reg [31:0] EX_ReadData2,
    output wire [31:0] EX_SignExtImm,
    output wire [4:0]      EX_Rd,
    output wire [4:0]      EX_Shamt
    );

    /***
     The purpose of a pipeline register is to capture data from one pipeline stage
     and provide it to the next pipeline stage. This creates at least one clock cycle
     of delay, but reduces the combinatorial path length of signals which allows for
     higher clock speeds.

     All pipeline registers update unless the forward stage is stalled. When this occurs
     or when the current stage is being flushed, the forward stage will receive data that
     is effectively a NOP and causes nothing to happen throughout the remaining pipeline
     traversal. In other words:

     A stall masks all control signals to forward stages. A flush permanently clears
     control signals to forward stages (but not certain data for exception purposes).
    ***/

    reg [16:0] EX_SignExtImm_pre;
    reg EX_RegDst;
    assign EX_LinkRegDst = (EX_Link) ? 2'b10 : ((EX_RegDst) ? 2'b01 : 2'b00);
    assign EX_Rd = EX_SignExtImm[15:11];
    assign EX_Shamt = EX_SignExtImm[10:6];
    assign EX_SignExtImm = (EX_SignExtImm_pre[16]) ? {15'h7fff, EX_SignExtImm_pre[16:0]} : {15'h0000, EX_SignExtImm_pre[16:0]};

    always @(posedge clock) begin
        EX_Link           <= (reset) ? 1'b0  : ((EX_Stall) ? EX_Link                                          : ID_Link);
        EX_RegDst         <= (reset) ? 1'b0  : ((EX_Stall) ? EX_RegDst                                        : ID_RegDst);
        EX_ALUSrcImm      <= (reset) ? 1'b0  : ((EX_Stall) ? EX_ALUSrcImm                                     : ID_ALUSrcImm);
        EX_ALUOp          <= (reset) ? 5'b0  : ((EX_Stall) ? EX_ALUOp         : ((ID_Stall | ID_Flush) ? 5'b0 : ID_ALUOp));
        EX_Movn           <= (reset) ? 1'b0  : ((EX_Stall) ? EX_Movn                                          : ID_Movn);
        EX_Movz           <= (reset) ? 1'b0  : ((EX_Stall) ? EX_Movz                                          : ID_Movz);
        EX_LLSC           <= (reset) ? 1'b0  : ((EX_Stall) ? EX_LLSC                                          : ID_LLSC);
        EX_MemRead        <= (reset) ? 1'b0  : ((EX_Stall) ? EX_MemRead       : ((ID_Stall | ID_Flush) ? 1'b0 : ID_MemRead));
        EX_MemWrite       <= (reset) ? 1'b0  : ((EX_Stall) ? EX_MemWrite      : ((ID_Stall | ID_Flush) ? 1'b0 : ID_MemWrite));
        EX_MemByte        <= (reset) ? 1'b0  : ((EX_Stall) ? EX_MemByte                                       : ID_MemByte);
        EX_MemHalf        <= (reset) ? 1'b0  : ((EX_Stall) ? EX_MemHalf                                       : ID_MemHalf);
        EX_MemSignExtend  <= (reset) ? 1'b0  : ((EX_Stall) ? EX_MemSignExtend                                 : ID_MemSignExtend);
        EX_Left           <= (reset) ? 1'b0  : ((EX_Stall) ? EX_Left                                          : ID_Left);
        EX_Right          <= (reset) ? 1'b0  : ((EX_Stall) ? EX_Right                                         : ID_Right);
        EX_RegWrite       <= (reset) ? 1'b0  : ((EX_Stall) ? EX_RegWrite      : ((ID_Stall | ID_Flush) ? 1'b0 : ID_RegWrite));
        EX_MemtoReg       <= (reset) ? 1'b0  : ((EX_Stall) ? EX_MemtoReg                                      : ID_MemtoReg);
        EX_ReverseEndian  <= (reset) ? 1'b0  : ((EX_Stall) ? EX_ReverseEndian                                 : ID_ReverseEndian);
        EX_RestartPC      <= (reset) ? 32'b0 : ((EX_Stall) ? EX_RestartPC                                     : ID_RestartPC);
        EX_IsBDS          <= (reset) ? 1'b0  : ((EX_Stall) ? EX_IsBDS                                         : ID_IsBDS);
        EX_Trap           <= (reset) ? 1'b0  : ((EX_Stall) ? EX_Trap          : ((ID_Stall | ID_Flush) ? 1'b0 : ID_Trap));
        EX_TrapCond       <= (reset) ? 1'b0  : ((EX_Stall) ? EX_TrapCond                                      : ID_TrapCond);
        EX_EX_CanErr      <= (reset) ? 1'b0  : ((EX_Stall) ? EX_EX_CanErr     : ((ID_Stall | ID_Flush) ? 1'b0 : ID_EX_CanErr));
        EX_M_CanErr       <= (reset) ? 1'b0  : ((EX_Stall) ? EX_M_CanErr      : ((ID_Stall | ID_Flush) ? 1'b0 : ID_M_CanErr));
        EX_ReadData1      <= (reset) ? 32'b0 : ((EX_Stall) ? EX_ReadData1                                     : ID_ReadData1);
        EX_ReadData2      <= (reset) ? 32'b0 : ((EX_Stall) ? EX_ReadData2                                     : ID_ReadData2);
        EX_SignExtImm_pre <= (reset) ? 17'b0 : ((EX_Stall) ? EX_SignExtImm_pre                                : ID_SignExtImm);
        EX_Rs             <= (reset) ? 5'b0  : ((EX_Stall) ? EX_Rs                                            : ID_Rs);
        EX_Rt             <= (reset) ? 5'b0  : ((EX_Stall) ? EX_Rt                                            : ID_Rt);
        EX_WantRsByEX     <= (reset) ? 1'b0  : ((EX_Stall) ? EX_WantRsByEX    : ((ID_Stall | ID_Flush) ? 1'b0 : ID_WantRsByEX));
        EX_NeedRsByEX     <= (reset) ? 1'b0  : ((EX_Stall) ? EX_NeedRsByEX    : ((ID_Stall | ID_Flush) ? 1'b0 : ID_NeedRsByEX));
        EX_WantRtByEX     <= (reset) ? 1'b0  : ((EX_Stall) ? EX_WantRtByEX    : ((ID_Stall | ID_Flush) ? 1'b0 : ID_WantRtByEX));
        EX_NeedRtByEX     <= (reset) ? 1'b0  : ((EX_Stall) ? EX_NeedRtByEX    : ((ID_Stall | ID_Flush) ? 1'b0 : ID_NeedRtByEX));
        EX_KernelMode     <= (reset) ? 1'b0  : ((EX_Stall) ? EX_KernelMode                                    : ID_KernelMode);
    end

endmodule
/*
 * File         : IFID_Stage.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   9-Jun-2011   GEA       Initial design.
 *   2.0   26-Jul-2012  GEA       Many updates have been made.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The Pipeline Register to bridge the Instruction Fetch
 *   and Instruction Decode stages.
 */
module IFID_Stage(
    input wire  clock,
    input wire  reset,
    input wire  IF_Flush,
    input wire  IF_Stall,
    input wire  ID_Stall,
    // Control Signals
    input wire  [31:0] IF_Instruction,
    // Data Signals
    input wire  [31:0] IF_PCAdd4,
    input wire  [31:0] IF_PC,
    input wire  IF_IsBDS,
    // ------------------
    output reg [31:0] ID_Instruction,
    output reg [31:0] ID_PCAdd4,
    output reg [31:0] ID_RestartPC,
    output reg ID_IsBDS,
    output reg ID_IsFlushed
    );

    /***
     The purpose of a pipeline register is to capture data from one pipeline stage
     and provide it to the next pipeline stage. This creates at least one clock cycle
     of delay, but reduces the combinatorial path length of signals which allows for
     higher clock speeds.

     All pipeline registers update unless the forward stage is stalled. When this occurs
     or when the current stage is being flushed, the forward stage will receive data that
     is effectively a NOP and causes nothing to happen throughout the remaining pipeline
     traversal. In other words:

     A stall masks all control signals to forward stages. A flush permanently clears
     control signals to forward stages (but not certain data for exception purposes).
    ***/


    /***
     The signal 'ID_IsFlushed' is needed because of interrupts. Normally, a flushed instruction
     is a NOP which will never cause an exception and thus its restart PC will never be needed
     or used. However, interrupts are detected in ID and may occur when any instruction, flushed
     or not, is in the ID stage. It is an error to save the restart PC of a flushed instruction
     since it was never supposed to execute (such as the "delay slot" after ERET or the branch
     delay slot after a canceled Branch Likely instruction). A simple way to prevent this is to
     pass a signal to ID indicating that its instruction was flushed. Interrupt detection is then
     masked when this signal is high, and the interrupt will trigger on the next instruction load to ID.
    ***/

    always @(posedge clock) begin
        ID_Instruction <= (reset) ? 32'b0 : ((ID_Stall) ? ID_Instruction : ((IF_Stall | IF_Flush) ? 32'b0 : IF_Instruction));
        ID_PCAdd4      <= (reset) ? 32'b0 : ((ID_Stall) ? ID_PCAdd4                                       : IF_PCAdd4);
        ID_IsBDS       <= (reset) ? 1'b0  : ((ID_Stall) ? ID_IsBDS                                        : IF_IsBDS);
        ID_RestartPC   <= (reset) ? 32'b0 : ((ID_Stall | IF_IsBDS) ? ID_RestartPC                         : IF_PC);
        ID_IsFlushed   <= (reset) ? 1'b0  : ((ID_Stall) ? ID_IsFlushed                                    : IF_Flush);
    end

endmodule

/*
 * File         : MemControl.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   24-Jun-2011  GEA       Initial design.
 *   2.0   28-Jun-2012  GEA       Expanded from a simple byte/half/word unit to
 *                                An advanced data memory controller capable of
 *                                handling big/little endian, atomic and unaligned
 *                                memory accesses.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A Data Memory Controller which handles all read and write requests from the
 *   processor to data memory. All data accesses--whether big endian, little endian,
 *   byte, half, word, or unaligned transfers--are transformed into a simple read
 *   and write command to data memory over a 32-bit data bus, where the read command
 *   is one bit and the write command is 4 bits, one for each byte in the 32-bit word.
 */
module MemControl(
    input wire  clock,
    input wire  reset,
    input wire  [31:0] DataIn,           // Data from CPU
    input wire  [31:0] Address,          // From CPU
    input wire  [31:0] MReadData,        // Data from Memory
    input wire  MemRead,                 // Memory Read command from CPU
    input wire  MemWrite,                // Memory Write command from CPU
    input wire  DataMem_Ready,           // Ready signal from Memory
    input wire  Byte,                    // Load/Store is Byte (8-bit)
    input wire  Half,                    // Load/Store is Half (16-bit)
    input wire  SignExtend,              // Sub-word load should be sign extended
    input wire  KernelMode,              // (Exception logic)
    input wire  ReverseEndian,           // Reverse Endian Memory for User Mode
    input wire  LLSC,                    // (LLSC logic)
    input wire  ERET,                    // (LLSC logic)
    input wire  Left,                    // Unaligned Load/Store Word Left
    input wire  Right,                   // Unaligned Load/Store Word Right
    input wire  M_Exception_Stall,
    input wire  IF_Stall,                // XXX Clean this up between this module and HAZ/FWD
    output reg [31:0] DataOut,      // Data to CPU
    output wire [31:0] MWriteData,       // Data to Memory
    output reg [3:0] WriteEnable,   // Write Enable to Memory for each of 4 bytes of Memory
    output wire ReadEnable,              // Read Enable to Memory
    output wire M_Stall,
    output wire EXC_AdEL,                // Load Exception
    output wire EXC_AdES                 // Store Exception
);


    /*** Reverse Endian Mode
         Normal memory accesses in the processor are Big Endian. The endianness can be reversed
         to Little Endian in User Mode only.
    */
    wire BE = KernelMode | ~ReverseEndian;

    /*** Indicator that the current memory reference must be word-aligned ***/
    wire Word = ~(Half | Byte | Left | Right);

    // Exception Detection
    wire EXC_KernelMem = ~KernelMode & (Address < `UMem_Lower);
    wire EXC_Word = Word & (Address[1] | Address[0]);
    wire EXC_Half = Half & Address[0];
    assign EXC_AdEL = MemRead  & (EXC_KernelMem | EXC_Word | EXC_Half);
    assign EXC_AdES = MemWrite & (EXC_KernelMem | EXC_Word | EXC_Half);

    /*** Load Linked and Store Conditional logic ***

         A 32-bit register keeps track of the address for atomic Load Linked / Store Conditional
         operations. This register can be updated during stalls since it is not visible to
         forward stages. It does not need to be flushed during exceptions, since ERET destroys
         the atomicity condition and there are no detrimental effects in an exception handler.

         The atomic condition is set with a Load Linked instruction, and cleared on an ERET
         instruction or when any store instruction writes to one or more bytes covered by
         the word address register. It does not update on a stall condition.

         The MIPS32 spec states that an ERET instruction between LL and SC will cause the
         atomicity condition to fail. This implementation uses the ERET signal from the ID
         stage, which means instruction sequences such as "LL SC" could appear to have an
         ERET instruction between them even though they don't. One way to fix this is to pass
         the ERET signal through the pipeline to the MEM stage. However, because of the nature
         of LL/SC operations (they occur in a loop which checks the result at each iteration),
         an ERET will normally never be inserted into the pipeline programmatically until the
         LL/SC sequence has completed (exceptions such as interrupts can still cause ERET, but
         they can still cause them in the LL SC sequence as well). In other words, by not passing
         ERET through the pipeline, the only possible effect is a performance penalty. Also this
         may be irrelevant since currently ERET stalls for forward stages which can cause exceptions,
         which includes LL and SC.
    */
    reg  [29:0] LLSC_Address;
    reg  LLSC_Atomic;
    wire LLSC_MemWrite_Mask;

    always @(posedge clock) begin
        LLSC_Address <= (reset) ? 30'b0 : (MemRead & LLSC) ? Address[31:2] : LLSC_Address;
    end

    always @(posedge clock) begin
        if (reset) begin
            LLSC_Atomic <= 1'b0;
        end
        else if (MemRead) begin
            LLSC_Atomic <= (LLSC) ? 1'b1 : LLSC_Atomic;
        end
        else if (ERET | (~M_Stall & ~IF_Stall & MemWrite & (Address[31:2] == LLSC_Address))) begin
            LLSC_Atomic <= 1'b0;
        end
        else begin
            LLSC_Atomic <= LLSC_Atomic;
        end
    end
    assign LLSC_MemWrite_Mask = (LLSC & MemWrite & (~LLSC_Atomic | (Address[31:2] != LLSC_Address)));

    wire WriteCondition = MemWrite & ~(EXC_KernelMem | EXC_Word | EXC_Half) & ~LLSC_MemWrite_Mask;
    wire ReadCondition  = MemRead  & ~(EXC_KernelMem | EXC_Word | EXC_Half);

    reg RW_Mask;
    always @(posedge clock) begin
        RW_Mask <= (reset) ? 1'b0 : (((MemWrite | MemRead) & DataMem_Ready) ? 1'b1 : ((~M_Stall & ~IF_Stall) ? 1'b0 : RW_Mask));
    end
    assign M_Stall = ReadEnable | (WriteEnable != 4'b0000) | DataMem_Ready    | M_Exception_Stall;
    assign ReadEnable  = ReadCondition  & ~RW_Mask;

    wire Half_Access_L  = (Address[1] ^  BE);
    wire Half_Access_R  = (Address[1] ~^ BE);
    wire Byte_Access_LL = Half_Access_L & (Address[1] ~^ Address[0]);
    wire Byte_Access_LM = Half_Access_L & (Address[0] ~^ BE);
    wire Byte_Access_RM = Half_Access_R & (Address[0] ^  BE);
    wire Byte_Access_RR = Half_Access_R & (Address[1] ~^ Address[0]);

    // Write-Enable Signals to Memory
    always @(*) begin
        if (WriteCondition & ~RW_Mask) begin
            if (Byte) begin
                WriteEnable[3] <= Byte_Access_LL;
                WriteEnable[2] <= Byte_Access_LM;
                WriteEnable[1] <= Byte_Access_RM;
                WriteEnable[0] <= Byte_Access_RR;
            end
            else if (Half) begin
                WriteEnable[3] <= Half_Access_L;
                WriteEnable[2] <= Half_Access_L;
                WriteEnable[1] <= Half_Access_R;
                WriteEnable[0] <= Half_Access_R;
            end
            else if (Left) begin
                case (Address[1:0])
                    2'b00 : WriteEnable <= (BE) ? 4'b1111 : 4'b0001;
                    2'b01 : WriteEnable <= (BE) ? 4'b0111 : 4'b0011;
                    2'b10 : WriteEnable <= (BE) ? 4'b0011 : 4'b0111;
                    2'b11 : WriteEnable <= (BE) ? 4'b0001 : 4'b1111;
                endcase
            end
            else if (Right) begin
                case (Address[1:0])
                    2'b00 : WriteEnable <= (BE) ? 4'b1000 : 4'b1111;
                    2'b01 : WriteEnable <= (BE) ? 4'b1100 : 4'b1110;
                    2'b10 : WriteEnable <= (BE) ? 4'b1110 : 4'b1100;
                    2'b11 : WriteEnable <= (BE) ? 4'b1111 : 4'b1000;
                endcase
            end
            else begin
                WriteEnable <= 4'b1111;
            end
        end
        else begin
            WriteEnable <= 4'b0000;
        end
    end

    // Data Going to Memory
    assign MWriteData[31:24] = (Byte) ? DataIn[7:0] : ((Half) ? DataIn[15:8] : DataIn[31:24]);
    assign MWriteData[23:16] = (Byte | Half) ? DataIn[7:0] : DataIn[23:16];
    assign MWriteData[15:8]  = (Byte) ? DataIn[7:0] : DataIn[15:8];
    assign MWriteData[7:0]   = DataIn[7:0];

    // Data Read from Memory
    always @(*) begin
        if (Byte) begin
            if (Byte_Access_LL) begin
                DataOut <= (SignExtend & MReadData[31]) ? {24'hFFFFFF, MReadData[31:24]} : {24'h000000, MReadData[31:24]};
            end
            else if (Byte_Access_LM) begin
                DataOut <= (SignExtend & MReadData[23]) ? {24'hFFFFFF, MReadData[23:16]} : {24'h000000, MReadData[23:16]};
            end
            else if (Byte_Access_RM) begin
                DataOut <= (SignExtend & MReadData[15]) ? {24'hFFFFFF, MReadData[15:8]}  : {24'h000000, MReadData[15:8]};
            end
            else begin
                DataOut <= (SignExtend & MReadData[7])  ? {24'hFFFFFF, MReadData[7:0]}   : {24'h000000, MReadData[7:0]};
            end
        end
        else if (Half) begin
            if (Half_Access_L) begin
                DataOut <= (SignExtend & MReadData[31]) ? {16'hFFFF, MReadData[31:16]} : {16'h0000, MReadData[31:16]};
            end
            else begin
                DataOut <= (SignExtend & MReadData[15]) ? {16'hFFFF, MReadData[15:0]}  : {16'h0000, MReadData[15:0]};
            end
        end
        else if (LLSC & MemWrite) begin
            DataOut <= (LLSC_Atomic & (Address[31:2] == LLSC_Address)) ? 32'h0000_0001 : 32'h0000_0000;
        end
        else if (Left) begin
            case (Address[1:0])
                2'b00 : DataOut <= (BE) ?  MReadData                      : {MReadData[7:0],  DataIn[23:0]};
                2'b01 : DataOut <= (BE) ? {MReadData[23:0], DataIn[7:0]}  : {MReadData[15:0], DataIn[15:0]};
                2'b10 : DataOut <= (BE) ? {MReadData[15:0], DataIn[15:0]} : {MReadData[23:0], DataIn[7:0]};
                2'b11 : DataOut <= (BE) ? {MReadData[7:0],  DataIn[23:0]} :  MReadData;
            endcase
        end
        else if (Right) begin
            case (Address[1:0])
                2'b00 : DataOut <= (BE) ? {DataIn[31:8],  MReadData[31:24]} : MReadData;
                2'b01 : DataOut <= (BE) ? {DataIn[31:16], MReadData[31:16]} : {DataIn[31:24], MReadData[31:8]};
                2'b10 : DataOut <= (BE) ? {DataIn[31:24], MReadData[31:8]}  : {DataIn[31:16], MReadData[31:16]};
                2'b11 : DataOut <= (BE) ? MReadData                         : {DataIn[31:8],  MReadData[31:24]};
            endcase
        end
        else begin
            DataOut <= MReadData;
        end
    end

endmodule

/*
 * File         : MEMWB_Stage.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   9-Jun-2011   GEA       Initial design.
 *   2.0   26-Jul-2012  GEA       Many updates have been made.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The Pipeline Register to bridge the Memory and Writeback stages.
 */
module MEMWB_Stage(
    input wire  clock,
    input wire  reset,
    input wire  M_Flush,
    input wire  M_Stall,
    input wire  WB_Stall,
    // Control Signals
    input wire  M_RegWrite,
    input wire  M_MemtoReg,
    // Data Signals
    input wire  [31:0] M_ReadData,
    input wire  [31:0] M_ALU_Result,
    input wire  [4:0]  M_RtRd,
    // ----------------
    output reg WB_RegWrite,
    output reg WB_MemtoReg,
    output reg [31:0] WB_ReadData,
    output reg [31:0] WB_ALU_Result,
    output reg [4:0]  WB_RtRd
    );


    /***
     The purpose of a pipeline register is to capture data from one pipeline stage
     and provide it to the next pipeline stage. This creates at least one clock cycle
     of delay, but reduces the combinatorial path length of signals which allows for
     higher clock speeds.

     All pipeline registers update unless the forward stage is stalled. When this occurs
     or when the current stage is being flushed, the forward stage will receive data that
     is effectively a NOP and causes nothing to happen throughout the remaining pipeline
     traversal. In other words:

     A stall masks all control signals to forward stages. A flush permanently clears
     control signals to forward stages (but not certain data for exception purposes).

     Since WB is the final stage in the pipeline, it would normally never stall.
     However, because the MEM stage may be using data forwarded from WB, WB must stall
     when MEM is stalled. If it didn't, the forward data would not be preserved. If
     the processor didn't forward any data, a stall would not be needed.

     In practice, the only time WB stalls is when forwarding for a Lw->Sw sequence, since
     MEM doesn't need the data until its stage, but it does not latch the forwarded data.
     This means WB_Stall is probably identical to M_Stall. There is no speed difference by
     allowing WB to stall.
    ***/

    always @(posedge clock) begin
        WB_RegWrite   <= (reset) ? 1'b0  : ((WB_Stall) ? WB_RegWrite   : ((M_Stall | M_Flush) ? 1'b0 : M_RegWrite));
        WB_MemtoReg   <= (reset) ? 1'b0  : ((WB_Stall) ? WB_MemtoReg                                 : M_MemtoReg);
        WB_ReadData   <= (reset) ? 32'b0 : ((WB_Stall) ? WB_ReadData                                 : M_ReadData);
        WB_ALU_Result <= (reset) ? 32'b0 : ((WB_Stall) ? WB_ALU_Result                               : M_ALU_Result);
        WB_RtRd       <= (reset) ? 5'b0  : ((WB_Stall) ? WB_RtRd                                     : M_RtRd);
    end

endmodule

// Movc:10->11, Trap:9->10, TrapCond:8->9, RegDst:7->8

/*** Datapath ***

     All Signals are Active High. Branching and Jump signals (determined by "PCSrc"),
     as well as ALU operation signals ("ALUOp") are handled by the controller and are not found here.

     Bit  Name          Description
     ------------------------------
     15:  PCSrc         (Instruction Type)
     14:                   11: Instruction is Jump to Register
                           10: Instruction is Branch
                           01: Instruction is Jump to Immediate
                           00: Instruction does not branch nor jump
     13:  Link          (Link on Branch/Jump)
     ------------------------------
     12:  ALUSrc        (ALU Source) [0=ALU input wire B is 2nd register file output wire; 1=Immediate value]
     11:  Movc          (Conditional Move)
     10:  Trap          (Trap Instruction)
     9 :  TrapCond      (Trap Condition) [0=ALU result is 0; 1=ALU result is not 0]
     8 :  RegDst        (Register File Target) [0=Rt field; 1=Rd field]
     ------------------------------
     7 :  LLSC          (Load Linked or Store Conditional)
     6 :  MemRead       (Data Memory Read)
     5 :  MemWrite      (Data Memory Write)
     4 :  MemHalf       (Half Word Memory Access)
     3 :  MemByte       (Byte size Memory Access)
     2 :  MemSignExtend (Sign Extend Read Memory) [0=Zero Extend; 1=Sign Extend]
     ------------------------------
     1 :  RegWrite      (Register File Write)
     0 :  MemtoReg      (Memory to Register) [0=Register File write data is ALU output wire; 1=Is Data Memory]
     ------------------------------
*/
`define DP_None        16'b000_00000_000000_00      // Instructions which require nothing of the main datapath.
`define DP_RType       16'b000_00001_000000_10      // Standard R-Type
`define DP_IType       16'b000_10000_000000_10      // Standard I-Type
`define DP_Branch      16'b100_00000_000000_00      // Standard Branch
`define DP_BranchLink  16'b101_00000_000000_10      // Branch and Link
`define DP_HiLoWr      16'b000_00000_000000_00      // Write to Hi/Lo ALU register (Div,Divu,Mult,Multu,Mthi,Mtlo). Currently 'DP_None'.
`define DP_Jump        16'b010_00000_000000_00      // Standard Jump
`define DP_JumpLink    16'b011_00000_000000_10      // Jump and Link
`define DP_JumpLinkReg 16'b111_00000_000000_10      // Jump and Link Register
`define DP_JumpReg     16'b110_00000_000000_00      // Jump Register
`define DP_LoadByteS   16'b000_10000_010011_11      // Load Byte Signed
`define DP_LoadByteU   16'b000_10000_010010_11      // Load Byte Unsigned
`define DP_LoadHalfS   16'b000_10000_010101_11      // Load Half Signed
`define DP_LoadHalfU   16'b000_10000_010100_11      // Load Half Unsigned
`define DP_LoadWord    16'b000_10000_010000_11      // Load Word
`define DP_ExtWrRt     16'b000_00000_000000_10      // A DP-external write to Rt
`define DP_ExtWrRd     16'b000_00001_000000_10      // A DP-external write to Rd
`define DP_Movc        16'b000_01001_000000_10      // Conditional Move
`define DP_LoadLinked  16'b000_10000_110000_11      // Load Linked
`define DP_StoreCond   16'b000_10000_101000_11      // Store Conditional
`define DP_StoreByte   16'b000_10000_001010_00      // Store Byte
`define DP_StoreHalf   16'b000_10000_001100_00      // Store Half
`define DP_StoreWord   16'b000_10000_001000_00      // Store Word
`define DP_TrapRegCNZ  16'b000_00110_000000_00      // Trap using Rs and Rt,  non-zero ALU (Tlt,  Tltu,  Tne)
`define DP_TrapRegCZ   16'b000_00100_000000_00      // Trap using RS and Rt,  zero ALU     (Teq,  Tge,   Tgeu)
`define DP_TrapImmCNZ  16'b000_10110_000000_00      // Trap using Rs and Imm, non-zero ALU (Tlti, Tltiu, Tnei)
`define DP_TrapImmCZ   16'b000_10100_000000_00      // Trap using Rs and Imm, zero ALU     (Teqi, Tgei,  Tgeiu)
//--------------------------------------------------------
`define DP_Add     `DP_RType
`define DP_Addi    `DP_IType
`define DP_Addiu   `DP_IType
`define DP_Addu    `DP_RType
`define DP_And     `DP_RType
`define DP_Andi    `DP_IType
`define DP_Beq     `DP_Branch
`define DP_Bgez    `DP_Branch
`define DP_Bgezal  `DP_BranchLink
`define DP_Bgtz    `DP_Branch
`define DP_Blez    `DP_Branch
`define DP_Bltz    `DP_Branch
`define DP_Bltzal  `DP_BranchLink
`define DP_Bne     `DP_Branch
`define DP_Break   `DP_None
`define DP_Clo     `DP_RType
`define DP_Clz     `DP_RType
`define DP_Div     `DP_HiLoWr
`define DP_Divu    `DP_HiLoWr
`define DP_Eret    `DP_None
`define DP_J       `DP_Jump
`define DP_Jal     `DP_JumpLink
`define DP_Jalr    `DP_JumpLinkReg
`define DP_Jr      `DP_JumpReg
`define DP_Lb      `DP_LoadByteS
`define DP_Lbu     `DP_LoadByteU
`define DP_Lh      `DP_LoadHalfS
`define DP_Lhu     `DP_LoadHalfU
`define DP_Ll      `DP_LoadLinked
`define DP_Lui     `DP_IType
`define DP_Lw      `DP_LoadWord
`define DP_Lwl     `DP_LoadWord
`define DP_Lwr     `DP_LoadWord
`define DP_Madd    `DP_HiLoWr
`define DP_Maddu   `DP_HiLoWr
`define DP_Mfc0    `DP_ExtWrRt
`define DP_Mfhi    `DP_ExtWrRd
`define DP_Mflo    `DP_ExtWrRd
`define DP_Movn    `DP_Movc
`define DP_Movz    `DP_Movc
`define DP_Msub    `DP_HiLoWr
`define DP_Msubu   `DP_HiLoWr
`define DP_Mtc0    `DP_None
`define DP_Mthi    `DP_HiLoWr
`define DP_Mtlo    `DP_HiLoWr
`define DP_Mul     `DP_RType
`define DP_Mult    `DP_HiLoWr
`define DP_Multu   `DP_HiLoWr
`define DP_Nor     `DP_RType
`define DP_Or      `DP_RType
`define DP_Ori     `DP_IType
`define DP_Pref    `DP_None  // Not Implemented
`define DP_Sb      `DP_StoreByte
`define DP_Sc      `DP_StoreCond
`define DP_Sh      `DP_StoreHalf
`define DP_Sll     `DP_RType
`define DP_Sllv    `DP_RType
`define DP_Slt     `DP_RType
`define DP_Slti    `DP_IType
`define DP_Sltiu   `DP_IType
`define DP_Sltu    `DP_RType
`define DP_Sra     `DP_RType
`define DP_Srav    `DP_RType
`define DP_Srl     `DP_RType
`define DP_Srlv    `DP_RType
`define DP_Sub     `DP_RType
`define DP_Subu    `DP_RType
`define DP_Sw      `DP_StoreWord
`define DP_Swl     `DP_StoreWord
`define DP_Swr     `DP_StoreWord
`define DP_Syscall `DP_None
`define DP_Teq     `DP_TrapRegCZ
`define DP_Teqi    `DP_TrapImmCZ
`define DP_Tge     `DP_TrapRegCZ
`define DP_Tgei    `DP_TrapImmCZ
`define DP_Tgeiu   `DP_TrapImmCZ
`define DP_Tgeu    `DP_TrapRegCZ
`define DP_Tlt     `DP_TrapRegCNZ
`define DP_Tlti    `DP_TrapImmCNZ
`define DP_Tltiu   `DP_TrapImmCNZ
`define DP_Tltu    `DP_TrapRegCNZ
`define DP_Tne     `DP_TrapRegCNZ
`define DP_Tnei    `DP_TrapImmCNZ
`define DP_Xor     `DP_RType
`define DP_Xori    `DP_IType




/*** Exception Information ***

     All signals are Active High.

     Bit  Meaning
     ------------
     2:   Instruction can cause exceptions in ID
     1:   Instruction can cause exceptions in EX
     0:   Instruction can cause exceptions in MEM
*/
`define EXC_None 3'b000
`define EXC_ID   3'b100
`define EXC_EX   3'b010
`define EXC_MEM  3'b001
//--------------------------------
`define EXC_Add     `EXC_EX
`define EXC_Addi    `EXC_EX
`define EXC_Addiu   `EXC_None
`define EXC_Addu    `EXC_None
`define EXC_And     `EXC_None
`define EXC_Andi    `EXC_None
`define EXC_Beq     `EXC_None
`define EXC_Bgez    `EXC_None
`define EXC_Bgezal  `EXC_None
`define EXC_Bgtz    `EXC_None
`define EXC_Blez    `EXC_None
`define EXC_Bltz    `EXC_None
`define EXC_Bltzal  `EXC_None
`define EXC_Bne     `EXC_None
`define EXC_Break   `EXC_ID
`define EXC_Clo     `EXC_None
`define EXC_Clz     `EXC_None
`define EXC_Div     `EXC_None
`define EXC_Divu    `EXC_None
`define EXC_Eret    `EXC_ID
`define EXC_J       `EXC_None
`define EXC_Jal     `EXC_None
`define EXC_Jalr    `EXC_None
`define EXC_Jr      `EXC_None
`define EXC_Lb      `EXC_MEM
`define EXC_Lbu     `EXC_MEM
`define EXC_Lh      `EXC_MEM
`define EXC_Lhu     `EXC_MEM
`define EXC_Ll      `EXC_MEM
`define EXC_Lui     `EXC_None
`define EXC_Lw      `EXC_MEM
`define EXC_Lwl     `EXC_MEM
`define EXC_Lwr     `EXC_MEM
`define EXC_Madd    `EXC_None
`define EXC_Maddu   `EXC_None
`define EXC_Mfc0    `EXC_ID
`define EXC_Mfhi    `EXC_None
`define EXC_Mflo    `EXC_None
`define EXC_Movn    `EXC_None
`define EXC_Movz    `EXC_None
`define EXC_Msub    `EXC_None
`define EXC_Msubu   `EXC_None
`define EXC_Mtc0    `EXC_ID
`define EXC_Mthi    `EXC_None
`define EXC_Mtlo    `EXC_None
`define EXC_Mul     `EXC_None
`define EXC_Mult    `EXC_None
`define EXC_Multu   `EXC_None
`define EXC_Nor     `EXC_None
`define EXC_Or      `EXC_None
`define EXC_Ori     `EXC_None
`define EXC_Pref    `EXC_None    // XXX
`define EXC_Sb      `EXC_MEM
`define EXC_Sc      `EXC_MEM
`define EXC_Sh      `EXC_MEM
`define EXC_Sll     `EXC_None
`define EXC_Sllv    `EXC_None
`define EXC_Slt     `EXC_None
`define EXC_Slti    `EXC_None
`define EXC_Sltiu   `EXC_None
`define EXC_Sltu    `EXC_None
`define EXC_Sra     `EXC_None
`define EXC_Srav    `EXC_None
`define EXC_Srl     `EXC_None
`define EXC_Srlv    `EXC_None
`define EXC_Sub     `EXC_EX
`define EXC_Subu    `EXC_None
`define EXC_Sw      `EXC_MEM
`define EXC_Swl     `EXC_MEM
`define EXC_Swr     `EXC_MEM
`define EXC_Syscall `EXC_ID
`define EXC_Teq     `EXC_MEM
`define EXC_Teqi    `EXC_MEM
`define EXC_Tge     `EXC_MEM
`define EXC_Tgei    `EXC_MEM
`define EXC_Tgeiu   `EXC_MEM
`define EXC_Tgeu    `EXC_MEM
`define EXC_Tlt     `EXC_MEM
`define EXC_Tlti    `EXC_MEM
`define EXC_Tltiu   `EXC_MEM
`define EXC_Tltu    `EXC_MEM
`define EXC_Tne     `EXC_MEM
`define EXC_Tnei    `EXC_MEM
`define EXC_Xor     `EXC_None
`define EXC_Xori    `EXC_None




/*** Hazard & Forwarding Datapath ***

     All signals are Active High.

     Bit  Meaning
     ------------
     7:   Wants Rs by ID
     6:   Needs Rs by ID
     5:   Wants Rt by ID
     4:   Needs Rt by ID
     3:   Wants Rs by EX
     2:   Needs Rs by EX
     1:   Wants Rt by EX
     0:   Needs Rt by EX
*/
`define HAZ_Nothing  8'b00000000    // Jumps, Lui, Mfhi/lo, special, etc.
`define HAZ_IDRsIDRt 8'b11110000    // Beq, Bne, Traps
`define HAZ_IDRs     8'b11000000    // Most branches, Jumps to registers
`define HAZ_IDRt     8'b00110000    // Mtc0
`define HAZ_IDRtEXRs 8'b10111100    // Movn, Movz
`define HAZ_EXRsEXRt 8'b10101111    // Many R-Type ops
`define HAZ_EXRs     8'b10001100    // Immediates: Loads, Clo/z, Mthi/lo, etc.
`define HAZ_EXRsWRt  8'b10101110    // Stores
`define HAZ_EXRt     8'b00100011    // Shifts using Shamt field
//-----------------------------------------
`define HAZ_Add     `HAZ_EXRsEXRt
`define HAZ_Addi    `HAZ_EXRs
`define HAZ_Addiu   `HAZ_EXRs
`define HAZ_Addu    `HAZ_EXRsEXRt
`define HAZ_And     `HAZ_EXRsEXRt
`define HAZ_Andi    `HAZ_EXRs
`define HAZ_Beq     `HAZ_IDRsIDRt
`define HAZ_Bgez    `HAZ_IDRs
`define HAZ_Bgezal  `HAZ_IDRs
`define HAZ_Bgtz    `HAZ_IDRs
`define HAZ_Blez    `HAZ_IDRs
`define HAZ_Bltz    `HAZ_IDRs
`define HAZ_Bltzal  `HAZ_IDRs
`define HAZ_Bne     `HAZ_IDRsIDRt
`define HAZ_Break   `HAZ_Nothing
`define HAZ_Clo     `HAZ_EXRs
`define HAZ_Clz     `HAZ_EXRs
`define HAZ_Div     `HAZ_EXRsEXRt
`define HAZ_Divu    `HAZ_EXRsEXRt
`define HAZ_Eret    `HAZ_Nothing
`define HAZ_J       `HAZ_Nothing
`define HAZ_Jal     `HAZ_Nothing
`define HAZ_Jalr    `HAZ_IDRs
`define HAZ_Jr      `HAZ_IDRs
`define HAZ_Lb      `HAZ_EXRs
`define HAZ_Lbu     `HAZ_EXRs
`define HAZ_Lh      `HAZ_EXRs
`define HAZ_Lhu     `HAZ_EXRs
`define HAZ_Ll      `HAZ_EXRs
`define HAZ_Lui     `HAZ_Nothing
`define HAZ_Lw      `HAZ_EXRs
`define HAZ_Lwl     `HAZ_EXRsEXRt
`define HAZ_Lwr     `HAZ_EXRsEXRt
`define HAZ_Madd    `HAZ_EXRsEXRt
`define HAZ_Maddu   `HAZ_EXRsEXRt
`define HAZ_Mfc0    `HAZ_Nothing
`define HAZ_Mfhi    `HAZ_Nothing
`define HAZ_Mflo    `HAZ_Nothing
`define HAZ_Movn    `HAZ_IDRtEXRs
`define HAZ_Movz    `HAZ_IDRtEXRs
`define HAZ_Msub    `HAZ_EXRsEXRt
`define HAZ_Msubu   `HAZ_EXRsEXRt
`define HAZ_Mtc0    `HAZ_IDRt
`define HAZ_Mthi    `HAZ_EXRs
`define HAZ_Mtlo    `HAZ_EXRs
`define HAZ_Mul     `HAZ_EXRsEXRt
`define HAZ_Mult    `HAZ_EXRsEXRt
`define HAZ_Multu   `HAZ_EXRsEXRt
`define HAZ_Nor     `HAZ_EXRsEXRt
`define HAZ_Or      `HAZ_EXRsEXRt
`define HAZ_Ori     `HAZ_EXRs
`define HAZ_Pref    `HAZ_Nothing // XXX
`define HAZ_Sb      `HAZ_EXRsWRt
`define HAZ_Sc      `HAZ_EXRsWRt
`define HAZ_Sh      `HAZ_EXRsWRt
`define HAZ_Sll     `HAZ_EXRt
`define HAZ_Sllv    `HAZ_EXRsEXRt
`define HAZ_Slt     `HAZ_EXRsEXRt
`define HAZ_Slti    `HAZ_EXRs
`define HAZ_Sltiu   `HAZ_EXRs
`define HAZ_Sltu    `HAZ_EXRsEXRt
`define HAZ_Sra     `HAZ_EXRt
`define HAZ_Srav    `HAZ_EXRsEXRt
`define HAZ_Srl     `HAZ_EXRt
`define HAZ_Srlv    `HAZ_EXRsEXRt
`define HAZ_Sub     `HAZ_EXRsEXRt
`define HAZ_Subu    `HAZ_EXRsEXRt
`define HAZ_Sw      `HAZ_EXRsWRt
`define HAZ_Swl     `HAZ_EXRsWRt
`define HAZ_Swr     `HAZ_EXRsWRt
`define HAZ_Syscall `HAZ_Nothing
`define HAZ_Teq     `HAZ_EXRsEXRt
`define HAZ_Teqi    `HAZ_EXRs
`define HAZ_Tge     `HAZ_EXRsEXRt
`define HAZ_Tgei    `HAZ_EXRs
`define HAZ_Tgeiu   `HAZ_EXRs
`define HAZ_Tgeu    `HAZ_EXRsEXRt
`define HAZ_Tlt     `HAZ_EXRsEXRt
`define HAZ_Tlti    `HAZ_EXRs
`define HAZ_Tltiu   `HAZ_EXRs
`define HAZ_Tltu    `HAZ_EXRsEXRt
`define HAZ_Tne     `HAZ_EXRsEXRt
`define HAZ_Tnei    `HAZ_EXRs
`define HAZ_Xor     `HAZ_EXRsEXRt
`define HAZ_Xori    `HAZ_EXRs

/*
 * File         : Mux2.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A 2-input wire Mux of variable width, defaulting to 32-bit width.
 */
module Mux2 #(parameter WIDTH = 32)(
    input wire  sel,
    input wire  [(WIDTH-1):0] in0, in1,
    output wire [(WIDTH-1):0] out
    );

    assign out = (sel) ? in1 : in0;

endmodule

/*
 * File         : Mux4.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A 4-input wire Mux of variable width, defaulting to 32-bit width.
 */
module Mux4 #(parameter WIDTH = 32)(
    input wire  [1:0] sel,
    input wire  [(WIDTH-1):0] in0, in1, in2, in3,
    output reg [(WIDTH-1):0] out
    );

    always @(*) begin
        case (sel)
            2'b00 : out <= in0;
            2'b01 : out <= in1;
            2'b10 : out <= in2;
            2'b11 : out <= in3;
        endcase
    end

endmodule

/*
 * File         : Processor.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   23-Jul-2011  GEA       Initial design.
 *   2.0   26-May-2012  GEA       Release version with CP0.
 *   2.01   1-Nov-2012  GEA       Fixed issue with Jal.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   The top-level MIPS32 Processor. This file is mostly the instantiation
 *   and wiring of the building blocks of the processor according to the
 *   hardware design diagram. It contains very little logic itself.
 */
module Processor(
    input wire  clock,
    input wire  reset,
    input wire  [4:0] Interrupts,            // 5 general-purpose hardware interrupts
    input wire  NMI,                         // Non-maskable interrupt
    // Data Memory Interface
    input wire  DataMem_Ready,
    output wire DataMem_Read,
    output wire [3:0]  DataMem_Write,        // 4-bit Write, one for each byte in word.
    output wire [29:0] DataMem_Address,      // Addresses are words, not bytes.
    inout wire [31:0] DataMem_Bus,
    // Instruction Memory Interface
    input wire  [31:0] InstMem_In,
    output wire [29:0] InstMem_Address,      // Addresses are words, not bytes.
    input wire  InstMem_Ready,
    output wire InstMem_Read,
    output wire [7:0] IP                     // Pending interrupts (diagnostic)
);

    wire [31:0] DataMem_In;
    wire [31:0] DataMem_Out;

    assign DataMem_Bus = ((DataMem_Write[3] | DataMem_Write[2] | DataMem_Write[1] | DataMem_Write[0]) && !DataMem_Read) ? DataMem_Out : 32'bZ;
    assign DataMem_In  = (!(DataMem_Write[3] | DataMem_Write[2] | DataMem_Write[1] | DataMem_Write[0]) && DataMem_Read) ? DataMem_Bus : 32'b0;

    /*** MIPS Instruction and Components (ID Stage) ***/
    wire [31:0] Instruction;
    wire [5:0]  OpCode = Instruction[31:26];
    wire [4:0]  Rs = Instruction[25:21];
    wire [4:0]  Rt = Instruction[20:16];
    wire [4:0]  Rd = Instruction[15:11];
    wire [5:0]  Funct = Instruction[5:0];
    wire [15:0] Immediate = Instruction[15:0];
    wire [25:0] JumpAddress = Instruction[25:0];
    wire [2:0]  Cp0_Sel = Instruction[2:0];

    /*** IF (Instruction Fetch) Signals ***/
    wire IF_Stall, IF_Flush;
    wire IF_EXC_AdIF;
    wire IF_Exception_Stall;
    wire IF_Exception_Flush;
    wire IF_IsBDS;
    wire [31:0] IF_PCAdd4, IF_PC_PreExc, IF_PCIn, IF_PCOut, IF_Instruction;

    /*** ID (Instruction Decode) Signals ***/
    wire ID_Stall;
    wire [1:0] ID_PCSrc;
    wire [1:0] ID_RsFwdSel, ID_RtFwdSel;
    wire ID_Link, ID_Movn, ID_Movz;
    wire ID_SignExtend;
    wire ID_LLSC;
    wire ID_RegDst, ID_ALUSrcImm, ID_MemWrite, ID_MemRead, ID_MemByte, ID_MemHalf, ID_MemSignExtend, ID_RegWrite, ID_MemtoReg;
    wire [4:0] ID_ALUOp;
    wire ID_Mfc0, ID_Mtc0, ID_Eret;
    wire ID_NextIsDelay;
    wire ID_CanErr, ID_ID_CanErr, ID_EX_CanErr, ID_M_CanErr;
    wire ID_KernelMode;
    wire ID_ReverseEndian;
    wire ID_Trap, ID_TrapCond;
    wire ID_EXC_Sys, ID_EXC_Bp, ID_EXC_RI;
    wire ID_Exception_Stall;
    wire ID_Exception_Flush;
    wire ID_PCSrc_Exc;
    wire [31:0] ID_ExceptionPC;
    wire ID_CP1, ID_CP2, ID_CP3;
    wire [31:0] ID_PCAdd4;
    wire [31:0] ID_ReadData1_RF, ID_ReadData1_End;
    wire [31:0] ID_ReadData2_RF, ID_ReadData2_End;
    wire [31:0] CP0_RegOut;
    wire ID_CmpEQ, ID_CmpGZ, ID_CmpLZ, ID_CmpGEZ, ID_CmpLEZ;
    wire [29:0] ID_SignExtImm = (ID_SignExtend & Immediate[15]) ? {14'h3FFF, Immediate} : {14'h0000, Immediate};
    wire [31:0] ID_ImmLeftShift2 = {ID_SignExtImm[29:0], 2'b00};
    wire [31:0] ID_JumpAddress = {ID_PCAdd4[31:28], JumpAddress[25:0], 2'b00};
    wire [31:0] ID_BranchAddress;
    wire [31:0] ID_RestartPC;
    wire ID_IsBDS;
    wire ID_Left, ID_Right;
    wire ID_IsFlushed;

    /*** EX (Execute) Signals ***/
    wire EX_ALU_Stall, EX_Stall;
    wire [1:0] EX_RsFwdSel, EX_RtFwdSel;
    wire EX_Link;
    wire [1:0] EX_LinkRegDst;
    wire EX_ALUSrcImm;
    wire [4:0] EX_ALUOp;
    wire EX_Movn, EX_Movz;
    wire EX_LLSC;
    wire EX_MemRead, EX_MemWrite, EX_MemByte, EX_MemHalf, EX_MemSignExtend, EX_RegWrite, EX_MemtoReg;
    wire [4:0] EX_Rs, EX_Rt;
    wire EX_WantRsByEX, EX_NeedRsByEX, EX_WantRtByEX, EX_NeedRtByEX;
    wire EX_Trap, EX_TrapCond;
    wire EX_CanErr, EX_EX_CanErr, EX_M_CanErr;
    wire EX_KernelMode;
    wire EX_ReverseEndian;
    wire EX_Exception_Stall;
    wire EX_Exception_Flush;
    wire [31:0] EX_ReadData1_PR, EX_ReadData1_Fwd, EX_ReadData2_PR, EX_ReadData2_Fwd, EX_ReadData2_Imm;
    wire [31:0] EX_SignExtImm;
    wire [4:0] EX_Rd, EX_RtRd, EX_Shamt;
    wire [31:0] EX_ALUResult;
    wire EX_BZero;
    wire EX_EXC_Ov;
    wire [31:0] EX_RestartPC;
    wire EX_IsBDS;
    wire EX_Left, EX_Right;

    /*** MEM (Memory) Signals ***/
    wire M_Stall, M_Stall_Controller;
    wire M_LLSC;
    wire M_MemRead, M_MemWrite, M_MemByte, M_MemHalf, M_MemSignExtend;
    wire M_RegWrite, M_MemtoReg;
    wire M_WriteDataFwdSel;
    wire M_EXC_AdEL, M_EXC_AdES;
    wire M_M_CanErr;
    wire M_KernelMode;
    wire M_ReverseEndian;
    wire M_Trap, M_TrapCond;
    wire M_EXC_Tr;
    wire M_Exception_Flush;
    wire [31:0] M_ALUResult, M_ReadData2_PR;
    wire [4:0] M_RtRd;
    wire [31:0] M_MemReadData;
    wire [31:0] M_RestartPC;
    wire M_IsBDS;
    wire [31:0] M_WriteData_Pre;
    wire M_Left, M_Right;
    wire M_Exception_Stall;

    /*** WB (Writeback) Signals ***/
    wire WB_Stall, WB_RegWrite, WB_MemtoReg;
    wire [31:0] WB_ReadData, WB_ALUResult;
    wire [4:0]  WB_RtRd;
    wire [31:0] WB_WriteData;

    /*** Other Signals ***/
    wire [7:0] ID_DP_Hazards, HAZ_DP_Hazards;

    /*** Assignments ***/
    assign IF_Instruction = (IF_Stall) ? 32'h00000000 : InstMem_In;
    assign IF_IsBDS = ID_NextIsDelay;
    assign HAZ_DP_Hazards = {ID_DP_Hazards[7:4], EX_WantRsByEX, EX_NeedRsByEX, EX_WantRtByEX, EX_NeedRtByEX};
    assign IF_EXC_AdIF = IF_PCOut[1] | IF_PCOut[0];
    assign ID_CanErr = ID_ID_CanErr | ID_EX_CanErr | ID_M_CanErr;
    assign EX_CanErr = EX_EX_CanErr | EX_M_CanErr;

    // External Memory Interface
    reg IRead, IReadMask;
    assign InstMem_Address = IF_PCOut[31:2];
    assign DataMem_Address = M_ALUResult[31:2];
    always @(posedge clock) begin
        IRead <= (reset) ? 1'b1 : ~InstMem_Ready;
        IReadMask <= (reset) ? 1'b0 : ((IRead & InstMem_Ready) ? 1'b1 : ((~IF_Stall) ? 1'b0 : IReadMask));
    end
    assign InstMem_Read = IRead & ~IReadMask;


    /*** Datapath Controller ***/
    Control Controller (
        .ID_Stall       (ID_Stall),
        .OpCode         (OpCode),
        .Funct          (Funct),
        .Rs             (Rs),
        .Rt             (Rt),
        .Cmp_EQ         (ID_CmpEQ),
        .Cmp_GZ         (ID_CmpGZ),
        .Cmp_GEZ        (ID_CmpGEZ),
        .Cmp_LZ         (ID_CmpLZ),
        .Cmp_LEZ        (ID_CmpLEZ),
        .IF_Flush       (IF_Flush),
        .DP_Hazards     (ID_DP_Hazards),
        .PCSrc          (ID_PCSrc),
        .SignExtend     (ID_SignExtend),
        .Link           (ID_Link),
        .Movn           (ID_Movn),
        .Movz           (ID_Movz),
        .Mfc0           (ID_Mfc0),
        .Mtc0           (ID_Mtc0),
        .CP1            (ID_CP1),
        .CP2            (ID_CP2),
        .CP3            (ID_CP3),
        .Eret           (ID_Eret),
        .Trap           (ID_Trap),
        .TrapCond       (ID_TrapCond),
        .EXC_Sys        (ID_EXC_Sys),
        .EXC_Bp         (ID_EXC_Bp),
        .EXC_RI         (ID_EXC_RI),
        .ID_CanErr      (ID_ID_CanErr),
        .EX_CanErr      (ID_EX_CanErr),
        .M_CanErr       (ID_M_CanErr),
        .NextIsDelay    (ID_NextIsDelay),
        .RegDst         (ID_RegDst),
        .ALUSrcImm      (ID_ALUSrcImm),
        .ALUOp          (ID_ALUOp),
        .LLSC           (ID_LLSC),
        .MemWrite       (ID_MemWrite),
        .MemRead        (ID_MemRead),
        .MemByte        (ID_MemByte),
        .MemHalf        (ID_MemHalf),
        .MemSignExtend  (ID_MemSignExtend),
        .Left           (ID_Left),
        .Right          (ID_Right),
        .RegWrite       (ID_RegWrite),
        .MemtoReg       (ID_MemtoReg)
);

    /*** Hazard and Forward Control Unit ***/
    Hazard_Detection HazardControl (
        .DP_Hazards          (HAZ_DP_Hazards),
        .ID_Rs               (Rs),
        .ID_Rt               (Rt),
        .EX_Rs               (EX_Rs),
        .EX_Rt               (EX_Rt),
        .EX_RtRd             (EX_RtRd),
        .MEM_RtRd            (M_RtRd),
        .WB_RtRd             (WB_RtRd),
        .EX_Link             (EX_Link),
        .EX_RegWrite         (EX_RegWrite),
        .MEM_RegWrite        (M_RegWrite),
        .WB_RegWrite         (WB_RegWrite),
        .MEM_MemRead         (M_MemRead),
        .MEM_MemWrite        (M_MemWrite),
        .InstMem_Read        (InstMem_Read),
        .InstMem_Ready       (InstMem_Ready),
        .Mfc0                (ID_Mfc0),
        .IF_Exception_Stall  (IF_Exception_Stall),
        .ID_Exception_Stall  (ID_Exception_Stall),
        .EX_Exception_Stall  (EX_Exception_Stall),
        .EX_ALU_Stall        (EX_ALU_Stall),
        .M_Stall_Controller  (M_Stall_Controller),
        .IF_Stall            (IF_Stall),
        .ID_Stall            (ID_Stall),
        .EX_Stall            (EX_Stall),
        .M_Stall             (M_Stall),
        .WB_Stall            (WB_Stall),
        .ID_RsFwdSel         (ID_RsFwdSel),
        .ID_RtFwdSel         (ID_RtFwdSel),
        .EX_RsFwdSel         (EX_RsFwdSel),
        .EX_RtFwdSel         (EX_RtFwdSel),
        .M_WriteDataFwdSel   (M_WriteDataFwdSel)
);

    /*** Coprocessor 0: Exceptions and Interrupts ***/
    CPZero CP0 (
        .clock               (clock),
        .Mfc0                (ID_Mfc0),
        .Mtc0                (ID_Mtc0),
        .IF_Stall            (IF_Stall),
        .ID_Stall            (ID_Stall),
        .COP1                (ID_CP1),
        .COP2                (ID_CP2),
        .COP3                (ID_CP3),
        .ERET                (ID_Eret),
        .Rd                  (Rd),
        .Sel                 (Cp0_Sel),
        .Reg_In              (ID_ReadData2_End),
        .Reg_Out             (CP0_RegOut),
        .KernelMode          (ID_KernelMode),
        .ReverseEndian       (ID_ReverseEndian),
        .Int                 (Interrupts),
        .reset               (reset),
        .EXC_NMI             (NMI),
        .EXC_AdIF            (IF_EXC_AdIF),
        .EXC_AdEL            (M_EXC_AdEL),
        .EXC_AdES            (M_EXC_AdES),
        .EXC_Ov              (EX_EXC_Ov),
        .EXC_Tr              (M_EXC_Tr),
        .EXC_Sys             (ID_EXC_Sys),
        .EXC_Bp              (ID_EXC_Bp),
        .EXC_RI              (ID_EXC_RI),
        .ID_RestartPC        (ID_RestartPC),
        .EX_RestartPC        (EX_RestartPC),
        .M_RestartPC         (M_RestartPC),
        .ID_IsFlushed        (ID_IsFlushed),
        .IF_IsBD             (IF_IsBDS),
        .ID_IsBD             (ID_IsBDS),
        .EX_IsBD             (EX_IsBDS),
        .M_IsBD              (M_IsBDS),
        .BadAddr_M           (M_ALUResult),
        .BadAddr_IF          (IF_PCOut),
        .ID_CanErr           (ID_CanErr),
        .EX_CanErr           (EX_CanErr),
        .M_CanErr            (M_M_CanErr),
        .IF_Exception_Stall  (IF_Exception_Stall),
        .ID_Exception_Stall  (ID_Exception_Stall),
        .EX_Exception_Stall  (EX_Exception_Stall),
        .M_Exception_Stall   (M_Exception_Stall),
        .IF_Exception_Flush  (IF_Exception_Flush),
        .ID_Exception_Flush  (ID_Exception_Flush),
        .EX_Exception_Flush  (EX_Exception_Flush),
        .M_Exception_Flush   (M_Exception_Flush),
        .Exc_PC_Sel          (ID_PCSrc_Exc),
        .Exc_PC_Out          (ID_ExceptionPC),
        .IP                  (IP)
);

    /*** PC Source Non-Exception Mux ***/
    Mux4 #(.WIDTH(32)) PCSrcStd_Mux (
        .sel  (ID_PCSrc),
        .in0  (IF_PCAdd4),
        .in1  (ID_JumpAddress),
        .in2  (ID_BranchAddress),
        .in3  (ID_ReadData1_End),
        .out  (IF_PC_PreExc)
);

    /*** PC Source Exception Mux ***/
    Mux2 #(.WIDTH(32)) PCSrcExc_Mux (
        .sel  (ID_PCSrc_Exc),
        .in0  (IF_PC_PreExc),
        .in1  (ID_ExceptionPC),
        .out  (IF_PCIn)
);

    /*** Program Counter (MIPS spec is 0xBFC00000 starting address) ***/
    Register #(.WIDTH(32), .INIT(`EXC_Vector_Base_Reset)) PC (
        .clock   (clock),
        .reset   (reset),
        //.enable  (~IF_Stall),   // XXX verify. HERE. Was 1 but on stall latches PC+4, ad nauseum.
        .enable (~(IF_Stall | ID_Stall)),
        .D       (IF_PCIn),
        .Q       (IF_PCOut)
);

    /*** PC +4 Adder ***/
    Add PC_Add4 (
        .A  (IF_PCOut),
        .B  (32'h00000004),
        .C  (IF_PCAdd4)
);

    /*** Instruction Fetch -> Instruction Decode Stage Register ***/
    IFID_Stage IFID (
        .clock           (clock),
        .reset           (reset),
        .IF_Flush        (IF_Exception_Flush | IF_Flush),
        .IF_Stall        (IF_Stall),
        .ID_Stall        (ID_Stall),
        .IF_Instruction  (IF_Instruction),
        .IF_PCAdd4       (IF_PCAdd4),
        .IF_PC           (IF_PCOut),
        .IF_IsBDS        (IF_IsBDS),
        .ID_Instruction  (Instruction),
        .ID_PCAdd4       (ID_PCAdd4),
        .ID_RestartPC    (ID_RestartPC),
        .ID_IsBDS        (ID_IsBDS),
        .ID_IsFlushed    (ID_IsFlushed)
);

    /*** Register File ***/
    RegisterFile RegisterFile (
        .clock      (clock),
        .reset      (reset),
        .ReadReg1   (Rs),
        .ReadReg2   (Rt),
        .WriteReg   (WB_RtRd),
        .WriteData  (WB_WriteData),
        .RegWrite   (WB_RegWrite),
        .ReadData1  (ID_ReadData1_RF),
        .ReadData2  (ID_ReadData2_RF)
);

    /*** ID Rs Forwarding/Link Mux ***/
    Mux4 #(.WIDTH(32)) IDRsFwd_Mux (
        .sel  (ID_RsFwdSel),
        .in0  (ID_ReadData1_RF),
        .in1  (M_ALUResult),
        .in2  (WB_WriteData),
        .in3  (32'hxxxxxxxx),
        .out  (ID_ReadData1_End)
);

    /*** ID Rt Forwarding/CP0 Mfc0 Mux ***/
    Mux4 #(.WIDTH(32)) IDRtFwd_Mux (
        .sel  (ID_RtFwdSel),
        .in0  (ID_ReadData2_RF),
        .in1  (M_ALUResult),
        .in2  (WB_WriteData),
        .in3  (CP0_RegOut),
        .out  (ID_ReadData2_End)
);

    /*** Condition Compare Unit ***/
    Compare Compare (
        .A    (ID_ReadData1_End),
        .B    (ID_ReadData2_End),
        .EQ   (ID_CmpEQ),
        .GZ   (ID_CmpGZ),
        .LZ   (ID_CmpLZ),
        .GEZ  (ID_CmpGEZ),
        .LEZ  (ID_CmpLEZ)
);

    /*** Branch Address Adder ***/
    Add BranchAddress_Add (
        .A  (ID_PCAdd4),
        .B  (ID_ImmLeftShift2),
        .C  (ID_BranchAddress)
);

    /*** Instruction Decode -> Execute Pipeline Stage ***/
    IDEX_Stage IDEX (
        .clock             (clock),
        .reset             (reset),
        .ID_Flush          (ID_Exception_Flush),
        .ID_Stall          (ID_Stall),
        .EX_Stall          (EX_Stall),
        .ID_Link           (ID_Link),
        .ID_RegDst         (ID_RegDst),
        .ID_ALUSrcImm      (ID_ALUSrcImm),
        .ID_ALUOp          (ID_ALUOp),
        .ID_Movn           (ID_Movn),
        .ID_Movz           (ID_Movz),
        .ID_LLSC           (ID_LLSC),
        .ID_MemRead        (ID_MemRead),
        .ID_MemWrite       (ID_MemWrite),
        .ID_MemByte        (ID_MemByte),
        .ID_MemHalf        (ID_MemHalf),
        .ID_MemSignExtend  (ID_MemSignExtend),
        .ID_Left           (ID_Left),
        .ID_Right          (ID_Right),
        .ID_RegWrite       (ID_RegWrite),
        .ID_MemtoReg       (ID_MemtoReg),
        .ID_ReverseEndian  (ID_ReverseEndian),
        .ID_Rs             (Rs),
        .ID_Rt             (Rt),
        .ID_WantRsByEX     (ID_DP_Hazards[3]),
        .ID_NeedRsByEX     (ID_DP_Hazards[2]),
        .ID_WantRtByEX     (ID_DP_Hazards[1]),
        .ID_NeedRtByEX     (ID_DP_Hazards[0]),
        .ID_KernelMode     (ID_KernelMode),
        .ID_RestartPC      (ID_RestartPC),
        .ID_IsBDS          (ID_IsBDS),
        .ID_Trap           (ID_Trap),
        .ID_TrapCond       (ID_TrapCond),
        .ID_EX_CanErr      (ID_EX_CanErr),
        .ID_M_CanErr       (ID_M_CanErr),
        .ID_ReadData1      (ID_ReadData1_End),
        .ID_ReadData2      (ID_ReadData2_End),
        .ID_SignExtImm     (ID_SignExtImm[16:0]),
        .EX_Link           (EX_Link),
        .EX_LinkRegDst     (EX_LinkRegDst),
        .EX_ALUSrcImm      (EX_ALUSrcImm),
        .EX_ALUOp          (EX_ALUOp),
        .EX_Movn           (EX_Movn),
        .EX_Movz           (EX_Movz),
        .EX_LLSC           (EX_LLSC),
        .EX_MemRead        (EX_MemRead),
        .EX_MemWrite       (EX_MemWrite),
        .EX_MemByte        (EX_MemByte),
        .EX_MemHalf        (EX_MemHalf),
        .EX_MemSignExtend  (EX_MemSignExtend),
        .EX_Left           (EX_Left),
        .EX_Right          (EX_Right),
        .EX_RegWrite       (EX_RegWrite),
        .EX_MemtoReg       (EX_MemtoReg),
        .EX_ReverseEndian  (EX_ReverseEndian),
        .EX_Rs             (EX_Rs),
        .EX_Rt             (EX_Rt),
        .EX_WantRsByEX     (EX_WantRsByEX),
        .EX_NeedRsByEX     (EX_NeedRsByEX),
        .EX_WantRtByEX     (EX_WantRtByEX),
        .EX_NeedRtByEX     (EX_NeedRtByEX),
        .EX_KernelMode     (EX_KernelMode),
        .EX_RestartPC      (EX_RestartPC),
        .EX_IsBDS          (EX_IsBDS),
        .EX_Trap           (EX_Trap),
        .EX_TrapCond       (EX_TrapCond),
        .EX_EX_CanErr      (EX_EX_CanErr),
        .EX_M_CanErr       (EX_M_CanErr),
        .EX_ReadData1      (EX_ReadData1_PR),
        .EX_ReadData2      (EX_ReadData2_PR),
        .EX_SignExtImm     (EX_SignExtImm),
        .EX_Rd             (EX_Rd),
        .EX_Shamt          (EX_Shamt)
);

    /*** EX Rs Forwarding Mux ***/
    Mux4 #(.WIDTH(32)) EXRsFwd_Mux (
        .sel  (EX_RsFwdSel),
        .in0  (EX_ReadData1_PR),
        .in1  (M_ALUResult),
        .in2  (WB_WriteData),
        .in3  (EX_RestartPC),
        .out  (EX_ReadData1_Fwd)
);

    /*** EX Rt Forwarding / Link Mux ***/
    Mux4 #(.WIDTH(32)) EXRtFwdLnk_Mux (
        .sel  (EX_RtFwdSel),
        .in0  (EX_ReadData2_PR),
        .in1  (M_ALUResult),
        .in2  (WB_WriteData),
        .in3  (32'h00000008),
        .out  (EX_ReadData2_Fwd)
);

    /*** EX ALU Immediate Mux ***/
    Mux2 #(.WIDTH(32)) EXALUImm_Mux (
        .sel  (EX_ALUSrcImm),
        .in0  (EX_ReadData2_Fwd),
        .in1  (EX_SignExtImm),
        .out  (EX_ReadData2_Imm)
);

    /*** EX RtRd / Link Mux ***/
    Mux4 #(.WIDTH(5)) EXRtRdLnk_Mux (
        .sel  (EX_LinkRegDst),
        .in0  (EX_Rt),
        .in1  (EX_Rd),
        .in2  (5'b11111),
        .in3  (5'bxxxxx),
        .out  (EX_RtRd)
);

    /*** Arithmetic Logic Unit ***/
    ALU ALU (
        .clock      (clock),
        .reset      (reset),
        .EX_Stall   (EX_Stall),
        .EX_Flush   (EX_Exception_Flush),
        .A          (EX_ReadData1_Fwd),
        .B          (EX_ReadData2_Imm),
        .Operation  (EX_ALUOp),
        .Shamt      (EX_Shamt),
        .Result     (EX_ALUResult),
        .BZero      (EX_BZero),
        .EXC_Ov     (EX_EXC_Ov),
        .ALU_Stall  (EX_ALU_Stall)
);

    /*** Execute -> Memory Pipeline Stage ***/
    EXMEM_Stage EXMEM (
        .clock             (clock),
        .reset             (reset),
        .EX_Flush          (EX_Exception_Flush),
        .EX_Stall          (EX_Stall),
        .M_Stall           (M_Stall),
        .EX_Movn           (EX_Movn),
        .EX_Movz           (EX_Movz),
        .EX_BZero          (EX_BZero),
        .EX_RegWrite       (EX_RegWrite),
        .EX_MemtoReg       (EX_MemtoReg),
        .EX_ReverseEndian  (EX_ReverseEndian),
        .EX_LLSC           (EX_LLSC),
        .EX_MemRead        (EX_MemRead),
        .EX_MemWrite       (EX_MemWrite),
        .EX_MemByte        (EX_MemByte),
        .EX_MemHalf        (EX_MemHalf),
        .EX_MemSignExtend  (EX_MemSignExtend),
        .EX_Left           (EX_Left),
        .EX_Right          (EX_Right),
        .EX_KernelMode     (EX_KernelMode),
        .EX_RestartPC      (EX_RestartPC),
        .EX_IsBDS          (EX_IsBDS),
        .EX_Trap           (EX_Trap),
        .EX_TrapCond       (EX_TrapCond),
        .EX_M_CanErr       (EX_M_CanErr),
        .EX_ALU_Result     (EX_ALUResult),
        .EX_ReadData2      (EX_ReadData2_Fwd),
        .EX_RtRd           (EX_RtRd),
        .M_RegWrite        (M_RegWrite),
        .M_MemtoReg        (M_MemtoReg),
        .M_ReverseEndian   (M_ReverseEndian),
        .M_LLSC            (M_LLSC),
        .M_MemRead         (M_MemRead),
        .M_MemWrite        (M_MemWrite),
        .M_MemByte         (M_MemByte),
        .M_MemHalf         (M_MemHalf),
        .M_MemSignExtend   (M_MemSignExtend),
        .M_Left            (M_Left),
        .M_Right           (M_Right),
        .M_KernelMode      (M_KernelMode),
        .M_RestartPC       (M_RestartPC),
        .M_IsBDS           (M_IsBDS),
        .M_Trap            (M_Trap),
        .M_TrapCond        (M_TrapCond),
        .M_M_CanErr        (M_M_CanErr),
        .M_ALU_Result      (M_ALUResult),
        .M_ReadData2       (M_ReadData2_PR),
        .M_RtRd            (M_RtRd)
);

    /*** Trap Detection Unit ***/
    TrapDetect TrapDetect (
        .Trap       (M_Trap),
        .TrapCond   (M_TrapCond),
        .ALUResult  (M_ALUResult),
        .EXC_Tr     (M_EXC_Tr)
);

    /*** MEM Write Data Mux ***/
    Mux2 #(.WIDTH(32)) MWriteData_Mux (
        .sel  (M_WriteDataFwdSel),
        .in0  (M_ReadData2_PR),
        .in1  (WB_WriteData),
        .out  (M_WriteData_Pre)
);

    /*** Data Memory Controller ***/
    MemControl DataMem_Controller (
        .clock         (clock),
        .reset         (reset),
        .DataIn        (M_WriteData_Pre),
        .Address       (M_ALUResult),
        .MReadData     (DataMem_In),
        .MemRead       (M_MemRead),
        .MemWrite      (M_MemWrite),
        .DataMem_Ready (DataMem_Ready),
        .Byte          (M_MemByte),
        .Half          (M_MemHalf),
        .SignExtend    (M_MemSignExtend),
        .KernelMode    (M_KernelMode),
        .ReverseEndian (M_ReverseEndian),
        .LLSC          (M_LLSC),
        .ERET          (ID_Eret),
        .Left          (M_Left),
        .Right         (M_Right),
        .M_Exception_Stall (M_Exception_Stall),
        .IF_Stall      (IF_Stall),
        .DataOut       (M_MemReadData),
        .MWriteData    (DataMem_Out),
        .WriteEnable   (DataMem_Write),
        .ReadEnable    (DataMem_Read),
        .M_Stall       (M_Stall_Controller),
        .EXC_AdEL      (M_EXC_AdEL),
        .EXC_AdES      (M_EXC_AdES)
);

    /*** Memory -> Writeback Pipeline Stage ***/
    MEMWB_Stage MEMWB (
        .clock          (clock),
        .reset          (reset),
        .M_Flush        (M_Exception_Flush),
        .M_Stall        (M_Stall),
        .WB_Stall       (WB_Stall),
        .M_RegWrite     (M_RegWrite),
        .M_MemtoReg     (M_MemtoReg),
        .M_ReadData     (M_MemReadData),
        .M_ALU_Result   (M_ALUResult),
        .M_RtRd         (M_RtRd),
        .WB_RegWrite    (WB_RegWrite),
        .WB_MemtoReg    (WB_MemtoReg),
        .WB_ReadData    (WB_ReadData),
        .WB_ALU_Result  (WB_ALUResult),
        .WB_RtRd        (WB_RtRd)
);

    /*** WB MemtoReg Mux ***/
    Mux2 #(.WIDTH(32)) WBMemtoReg_Mux (
        .sel  (WB_MemtoReg),
        .in0  (WB_ALUResult),
        .in1  (WB_ReadData),
        .out  (WB_WriteData)
);

endmodule

/*
 * File         : RegisterFile.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A Register File for a MIPS processor. Contains 32 general-purpose
 *   32-bit wide registers and two read ports. Register 0 always reads
 *   as zero.
 */
module RegisterFile(
    input wire  clock,
    input wire  reset,
    input wire  [4:0]  ReadReg1, ReadReg2, WriteReg,
    input wire  [31:0] WriteData,
    input wire  RegWrite,
    output wire [31:0] ReadData1, ReadData2
);

    // Register file of 32 32-bit registers. Register 0 is hardwired to 0s
    reg [31:0] registers [1:31];

    // Initialize all to zero
    integer i;
    initial begin
        for (i=1; i<32; i=i+1) begin
            registers[i] <= 0;
        end
    end

    // Sequential (clocked) write.
    // 'WriteReg' is the register index to write. 'RegWrite' is the command.
    always @(posedge clock) begin
        if (reset) begin
            for (i=1; i<32; i=i+1) begin
                registers[i] <= 0;
            end
        end
        else begin
            if (WriteReg != 0)
                registers[WriteReg] <= (RegWrite) ? WriteData : registers[WriteReg];
        end
    end

    // Combinatorial Read. Register 0 is all 0s.
    assign ReadData1 = (ReadReg1 == 0) ? 32'h00000000 : registers[ReadReg1];
    assign ReadData2 = (ReadReg2 == 0) ? 32'h00000000 : registers[ReadReg2];

endmodule

/*
 * File         : Register.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A variable-width register (d flip-flop) with configurable initial
 *   value. Default is 32-bit width and 0s for initial value.
 */
module Register #(parameter WIDTH = 32, INIT = 0)(
    input wire  clock,
    input wire  reset,
    input wire  enable,
    input wire  [(WIDTH-1):0] D,
    output reg [(WIDTH-1):0] Q
);

    initial
        Q = INIT;

    always @(posedge clock) begin
        Q <= (reset) ? INIT : ((enable) ? D : Q);
    end

endmodule

/*
 * File         : TrapDetect.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   15-May-2012  GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   Detects a Trap Exception in the pipeline.
 */
module TrapDetect(
    input wire  Trap,
    input wire  TrapCond,
    input wire  [31:0] ALUResult,
    output wire EXC_Tr
);

    wire ALUZero = (ALUResult == 32'h00000000);
    assign EXC_Tr = Trap & (TrapCond ^ ALUZero);

endmodule

