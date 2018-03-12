//==================================================================================================
//  Filename      : antares_defines.v
//  Created On    : Mon Aug 31 19:32:04 2015
//  Last Modified : Sun Sep 06 11:09:43 2015
//  Revision      : 0.1
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Opcodes and processor configuration
//==================================================================================================

//------------------------------------------------------------------------------
// Virtual/Physical Space
// No MMU, so VA = PA.
// First 0.5 GB: Kernel space
// Last  3.5 GB: User space
//
// WARNING: The address space is different to Standard MIPS
//------------------------------------------------------------------------------
`define ANTARES_SEG_0_SPACE_LOW    32'h0000_0000      // 256 MB: Internal Memory
`define ANTARES_SEG_0_SPACE_HIGH   32'h0FFF_FFFF
`define ANTARES_SEG_1_SPACE_LOW    32'h1000_0000      // 256 MB: I/O
`define ANTARES_SEG_1_SPACE_HIGH   32'h1FFF_FFFF
`define ANTARES_SEG_2_SPACE_LOW    32'h2000_0000      // 3.5 GB: External Memory
`define ANTARES_SEG_3_SPACE_HIGH   32'hFFFF_FFFF

//------------------------------------------------------------------------------
// Endianess
// 0 -> little-endian. 1 -> big-endian
//------------------------------------------------------------------------------
`define ANTARES_LITTLE_ENDIAN      0                   //
`define ANTARES_BIG_ENDIAN         1                   //

//------------------------------------------------------------------------------
// Exception Vector
//------------------------------------------------------------------------------
`define ANTARES_VECTOR_BASE_RESET      32'h0000_0010   // MIPS Standard is 0xBFC0_0000. Reset, soft-reset, NMI
`define ANTARES_VECTOR_BASE_BOOT       32'h0000_0000   // MIPS Standard is 0xBFC0_0200. Bootstrap (Status_BEV = 1)
`define ANTARES_VECTOR_BASE_NO_BOOT    32'h0000_0000   // MIPS Standard is 0x8000_0000. Normal (Status_BEV = 0)
`define ANTARES_VECTOR_OFFSET_GENERAL  32'h0000_0000   // MIPS Standard is 0x0000_0180. General exception, but TBL
`define ANTARES_VECTOR_OFFSET_SPECIAL  32'h0000_0008   // MIPS Standard is 0x0000_0200. Interrupts (Cause_IV = 1)

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
        - I : Opcode(6) + Imm(26)
*/
//------------------------------------------------------------------------------
// Opcode field for special instructions
//------------------------------------------------------------------------------
`define OP_TYPE_R               6'b00_0000          // Special
`define OP_TYPE_R2              6'b01_1100          // Special 2
`define OP_TYPE_REGIMM          6'b00_0001          // Branch/Trap
`define OP_TYPE_CP0             6'b01_0000          // Coprocessor 0
`define OP_TYPE_CP1             6'b01_0001          // Coprocessor 1
`define OP_TYPE_CP2             6'b01_0010          // Coprocessor 2
`define OP_TYPE_CP3             6'b01_0011          // Coprocessor 3

//------------------------------------------------------------------------------
// Instructions fields
//------------------------------------------------------------------------------
`define ANTARES_INSTR_OPCODE       31:26
`define ANTARES_INSTR_RS           25:21
`define ANTARES_INSTR_RT           20:16
`define ANTARES_INSTR_RD           15:11
`define ANTARES_INSTR_SHAMT        10:6
`define ANTARES_INSTR_FUNCT        5:0
`define ANTARES_INSTR_CP0_SEL      2:0
`define ANTARES_INSTR_IMM16        15:0
`define ANTARES_INSTR_IMM26        25:0

//------------------------------------------------------------------------------
// Opcode list
//------------------------------------------------------------------------------
`define OP_ADD                  `OP_TYPE_R
`define OP_ADDI                 6'b00_1000
`define OP_ADDIU                6'b00_1001
`define OP_ADDU                 `OP_TYPE_R
`define OP_AND                  `OP_TYPE_R
`define OP_ANDI                 6'b00_1100
`define OP_BEQ                  6'b00_0100
`define OP_BGEZ                 `OP_TYPE_REGIMM
`define OP_BGEZAL               `OP_TYPE_REGIMM
`define OP_BGTZ                 6'b00_0111
`define OP_BLEZ                 6'b00_0110
`define OP_BLTZ                 `OP_TYPE_REGIMM
`define OP_BLTZAL               `OP_TYPE_REGIMM
`define OP_BNE                  6'b00_0101
`define OP_BREAK                `OP_TYPE_R
`define OP_CLO                  `OP_TYPE_R2
`define OP_CLZ                  `OP_TYPE_R2
`define OP_DIV                  `OP_TYPE_R
`define OP_DIVU                 `OP_TYPE_R
`define OP_ERET                 `OP_TYPE_CP0
`define OP_J                    6'b00_0010
`define OP_JAL                  6'b00_0011
`define OP_JALR                 `OP_TYPE_R
`define OP_JR                   `OP_TYPE_R
`define OP_LB                   6'b10_0000
`define OP_LBU                  6'b10_0100
`define OP_LH                   6'b10_0001
`define OP_LHU                  6'b10_0101
`define OP_LL                   6'b11_0000
`define OP_LUI                  6'b00_1111
`define OP_LW                   6'b10_0011
`define OP_MADD                 `OP_TYPE_R2
`define OP_MADDU                `OP_TYPE_R2
`define OP_MFC0                 `OP_TYPE_CP0
`define OP_MFHI                 `OP_TYPE_R
`define OP_MFLO                 `OP_TYPE_R
`define OP_MOVN                 `OP_TYPE_R
`define OP_MOVZ                 `OP_TYPE_R
`define OP_MSUB                 `OP_TYPE_R2
`define OP_MSUBU                `OP_TYPE_R2
`define OP_MTC0                 `OP_TYPE_CP0
`define OP_MTHI                 `OP_TYPE_R
`define OP_MTLO                 `OP_TYPE_R
`define OP_MULT                 `OP_TYPE_R
`define OP_MULTU                `OP_TYPE_R
`define OP_NOR                  `OP_TYPE_R
`define OP_OR                   `OP_TYPE_R
`define OP_ORI                  6'b00_1101
`define OP_SB                   6'b10_1000
`define OP_SC                   6'b11_1000
`define OP_SH                   6'b10_1001
`define OP_SLL                  `OP_TYPE_R
`define OP_SLLV                 `OP_TYPE_R
`define OP_SLT                  `OP_TYPE_R
`define OP_SLTI                 6'b00_1010
`define OP_SLTIU                6'b00_1011
`define OP_SLTU                 `OP_TYPE_R
`define OP_SRA                  `OP_TYPE_R
`define OP_SRAV                 `OP_TYPE_R
`define OP_SRL                  `OP_TYPE_R
`define OP_SRLV                 `OP_TYPE_R
`define OP_SUB                  `OP_TYPE_R
`define OP_SUBU                 `OP_TYPE_R
`define OP_SW                   6'b10_1011
`define OP_SYSCALL              `OP_TYPE_R
`define OP_TEQ                  `OP_TYPE_R
`define OP_TEQI                 `OP_TYPE_REGIMM
`define OP_TGE                  `OP_TYPE_R
`define OP_TGEI                 `OP_TYPE_REGIMM
`define OP_TGEIU                `OP_TYPE_REGIMM
`define OP_TGEU                 `OP_TYPE_R
`define OP_TLT                  `OP_TYPE_R
`define OP_TLTI                 `OP_TYPE_REGIMM
`define OP_TLTIU                `OP_TYPE_REGIMM
`define OP_TLTU                 `OP_TYPE_R
`define OP_TNE                  `OP_TYPE_R
`define OP_TNEI                 `OP_TYPE_REGIMM
`define OP_XOR                  `OP_TYPE_R
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
`define FUNCTION_OP_MADD        6'b00_0000
`define FUNCTION_OP_MADDU       6'b00_0001
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
`define FUNCTION_OP_TEQ         6'b11_0100
`define FUNCTION_OP_TGE         6'b11_0000
`define FUNCTION_OP_TGEU        6'b11_0001
`define FUNCTION_OP_TLT         6'b11_0010
`define FUNCTION_OP_TLTU        6'b11_0011
`define FUNCTION_OP_TNE         6'b11_0110
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

//------------------------------------------------------------------------------
// ALU Operations
//------------------------------------------------------------------------------
`define ALU_OP_ADDU             5'd0
`define ALU_OP_ADD              5'd1
`define ALU_OP_SUB              5'd2
`define ALU_OP_SUBU             5'd3
`define ALU_OP_AND              5'd4
`define ALU_OP_MULS             5'd5
`define ALU_OP_MULU             5'd6
`define ALU_OP_NOR              5'd7
`define ALU_OP_OR               5'd8
`define ALU_OP_SLL              5'd9
`define ALU_OP_SRA              5'd10
`define ALU_OP_SRL              5'd11
`define ALU_OP_XOR              5'd12
`define ALU_OP_MFHI             5'd13
`define ALU_OP_MFLO             5'd14
`define ALU_OP_MTHI             5'd15
`define ALU_OP_MTLO             5'd16
`define ALU_OP_SLT              5'd17
`define ALU_OP_SLTU             5'd18
`define ALU_OP_DIV              5'd19
`define ALU_OP_DIVU             5'd20
`define ALU_OP_CLO              5'd21
`define ALU_OP_CLZ              5'd22
`define ALU_OP_MADD             5'd23
`define ALU_OP_MADDU            5'd24
`define ALU_OP_MSUB             5'd25
`define ALU_OP_MSUBU            5'd26
`define ALU_OP_A                5'd27
`define ALU_OP_B                5'd28

//------------------------------------------------------------------------------
/*
    Exception.

    All signals are active High.

    ----------------------------------------------------------------------------
        Bit     Meaning
    ----------------------------------------------------------------------------
        2  :    Instruction can cause exception @ ID
        1  :    Instruction can cause exception @ EX
        0  :    Instruction can cause exception @ MEM
    ----------------------------------------------------------------------------
*/
//------------------------------------------------------------------------------
`define EXCEPTION_NONE  3'b000
`define EXCEPTION_ID    3'b100
`define EXCEPTION_EX    3'b010
`define EXCEPTION_MEM   3'b001
//
`define EXC_ADD         `EXCEPTION_EX
`define EXC_ADDI        `EXCEPTION_EX
`define EXC_ADDIU       `EXCEPTION_NONE
`define EXC_ADDU        `EXCEPTION_NONE
`define EXC_AND         `EXCEPTION_NONE
`define EXC_ANDI        `EXCEPTION_NONE
`define EXC_BEQ         `EXCEPTION_NONE
`define EXC_BGEZ        `EXCEPTION_NONE
`define EXC_BGEZAL      `EXCEPTION_NONE
`define EXC_BGTZ        `EXCEPTION_NONE
`define EXC_BLEZ        `EXCEPTION_NONE
`define EXC_BLTZ        `EXCEPTION_NONE
`define EXC_BLTZAL      `EXCEPTION_NONE
`define EXC_BNE         `EXCEPTION_NONE
`define EXC_BREAK       `EXCEPTION_ID
`define EXC_CLO         `EXCEPTION_NONE
`define EXC_CLZ         `EXCEPTION_NONE
`define EXC_DIV         `EXCEPTION_NONE
`define EXC_DIVU        `EXCEPTION_NONE
`define EXC_ERET        `EXCEPTION_ID
`define EXC_J           `EXCEPTION_NONE
`define EXC_JAL         `EXCEPTION_NONE
`define EXC_JALR        `EXCEPTION_NONE
`define EXC_JR          `EXCEPTION_NONE
`define EXC_LB          `EXCEPTION_MEM
`define EXC_LBU         `EXCEPTION_MEM
`define EXC_LH          `EXCEPTION_MEM
`define EXC_LHU         `EXCEPTION_MEM
`define EXC_LL          `EXCEPTION_MEM
`define EXC_LUI         `EXCEPTION_NONE
`define EXC_LW          `EXCEPTION_MEM
`define EXC_MADD        `EXCEPTION_NONE
`define EXC_MADDU       `EXCEPTION_NONE
`define EXC_MFC0        `EXCEPTION_ID
`define EXC_MFHI        `EXCEPTION_NONE
`define EXC_MFLO        `EXCEPTION_NONE
`define EXC_MOVN        `EXCEPTION_NONE
`define EXC_MOVZ        `EXCEPTION_NONE
`define EXC_MSUB        `EXCEPTION_NONE
`define EXC_MSUBU       `EXCEPTION_NONE
`define EXC_MTC0        `EXCEPTION_ID
`define EXC_MTHI        `EXCEPTION_NONE
`define EXC_MTLO        `EXCEPTION_NONE
`define EXC_MULT        `EXCEPTION_NONE
`define EXC_MULTU       `EXCEPTION_NONE
`define EXC_NOR         `EXCEPTION_NONE
`define EXC_OR          `EXCEPTION_NONE
`define EXC_ORI         `EXCEPTION_NONE
`define EXC_SB          `EXCEPTION_MEM
`define EXC_SC          `EXCEPTION_MEM
`define EXC_SH          `EXCEPTION_MEM
`define EXC_SLL         `EXCEPTION_NONE
`define EXC_SLLV        `EXCEPTION_NONE
`define EXC_SLT         `EXCEPTION_NONE
`define EXC_SLTI        `EXCEPTION_NONE
`define EXC_SLTIU       `EXCEPTION_NONE
`define EXC_SLTU        `EXCEPTION_NONE
`define EXC_SRA         `EXCEPTION_NONE
`define EXC_SRAV        `EXCEPTION_NONE
`define EXC_SRL         `EXCEPTION_NONE
`define EXC_SRLV        `EXCEPTION_NONE
`define EXC_SUB         `EXCEPTION_EX
`define EXC_SUBU        `EXCEPTION_EX
`define EXC_SW          `EXCEPTION_MEM
`define EXC_SYSCALL     `EXCEPTION_ID
`define EXC_TEQ         `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TEQI        `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TGE         `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TGEI        `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TGEIU       `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TGEU        `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TLT         `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TLTI        `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TLTIU       `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TLTU        `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TNE         `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_TNEI        `EXCEPTION_MEM          // Requieres result from EX, so it triggers in the MEM stage
`define EXC_XOR         `EXCEPTION_NONE
`define EXC_XORI        `EXCEPTION_NONE

//------------------------------------------------------------------------------
/*
     Hazard and forwarding signals.

     All signals are Active High.

     ------------
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
//------------------------------------------------------------------------------
`define HAZ_NOTHING     8'b00000000    // Jumps, Lui, Mfhi/lo, special, etc.
`define HAZ_ID_RS_ID_RT 8'b11110000    // Beq, Bne, Traps
`define HAZ_ID_RS       8'b11000000    // Most branches, Jumps to registers
`define HAZ_ID_RT       8'b00110000    // Mtc0
`define HAZ_ID_RT_EX_RS 8'b10111100    // Movn, Movz
`define HAZ_EX_RS_EX_RT 8'b10101111    // Many R-Type ops
`define HAZ_EX_RS       8'b10001100    // Immediates: Loads, Clo/z, Mthi/lo, etc.
`define HAZ_EX_RS_W_RT  8'b10101110    // Stores
`define HAZ_EX_RT       8'b00100011    // Shifts using Shamt field
//-----------------------------------------
`define HAZ_ADD     `HAZ_EX_RS_EX_RT
`define HAZ_ADDI    `HAZ_EX_RS
`define HAZ_ADDIU   `HAZ_EX_RS
`define HAZ_ADDU    `HAZ_EX_RS_EX_RT
`define HAZ_AND     `HAZ_EX_RS_EX_RT
`define HAZ_ANDI    `HAZ_EX_RS
`define HAZ_BEQ     `HAZ_ID_RS_ID_RT
`define HAZ_BGEZ    `HAZ_ID_RS
`define HAZ_BGEZAL  `HAZ_ID_RS
`define HAZ_BGTZ    `HAZ_ID_RS
`define HAZ_BLEZ    `HAZ_ID_RS
`define HAZ_BLTZ    `HAZ_ID_RS
`define HAZ_BLTZAL  `HAZ_ID_RS
`define HAZ_BNE     `HAZ_ID_RS_ID_RT
`define HAZ_BREAK   `HAZ_NOTHING
`define HAZ_CLO     `HAZ_EX_RS
`define HAZ_CLZ     `HAZ_EX_RS
`define HAZ_DIV     `HAZ_EX_RS_EX_RT
`define HAZ_DIVU    `HAZ_EX_RS_EX_RT
`define HAZ_ERET    `HAZ_NOTHING
`define HAZ_J       `HAZ_NOTHING
`define HAZ_JAL     `HAZ_NOTHING
`define HAZ_JALR    `HAZ_ID_RS
`define HAZ_JR      `HAZ_ID_RS
`define HAZ_LB      `HAZ_EX_RS
`define HAZ_LBU     `HAZ_EX_RS
`define HAZ_LH      `HAZ_EX_RS
`define HAZ_LHU     `HAZ_EX_RS
`define HAZ_LL      `HAZ_EX_RS
`define HAZ_LUI     `HAZ_NOTHING
`define HAZ_LW      `HAZ_EX_RS
`define HAZ_MADD    `HAZ_EX_RS_EX_RT
`define HAZ_MADDU   `HAZ_EX_RS_EX_RT
`define HAZ_MFC0    `HAZ_NOTHING
`define HAZ_MFHI    `HAZ_NOTHING
`define HAZ_MFLO    `HAZ_NOTHING
`define HAZ_MOVN    `HAZ_ID_RT_EX_RS
`define HAZ_MOVZ    `HAZ_ID_RT_EX_RS
`define HAZ_MSUB    `HAZ_EX_RS_EX_RT
`define HAZ_MSUBU   `HAZ_EX_RS_EX_RT
`define HAZ_MTC0    `HAZ_ID_RT
`define HAZ_MTHI    `HAZ_EX_RS
`define HAZ_MTLO    `HAZ_EX_RS
`define HAZ_MULT    `HAZ_EX_RS_EX_RT
`define HAZ_MULTU   `HAZ_EX_RS_EX_RT
`define HAZ_NOR     `HAZ_EX_RS_EX_RT
`define HAZ_OR      `HAZ_EX_RS_EX_RT
`define HAZ_ORI     `HAZ_EX_RS
`define HAZ_SB      `HAZ_EX_RS_W_RT
`define HAZ_SC      `HAZ_EX_RS_W_RT
`define HAZ_SH      `HAZ_EX_RS_W_RT
`define HAZ_SLL     `HAZ_EX_RT
`define HAZ_SLLV    `HAZ_EX_RS_EX_RT
`define HAZ_SLT     `HAZ_EX_RS_EX_RT
`define HAZ_SLTI    `HAZ_EX_RS
`define HAZ_SLTIU   `HAZ_EX_RS
`define HAZ_SLTU    `HAZ_EX_RS_EX_RT
`define HAZ_SRA     `HAZ_EX_RT
`define HAZ_SRAV    `HAZ_EX_RS_EX_RT
`define HAZ_SRL     `HAZ_EX_RT
`define HAZ_SRLV    `HAZ_EX_RS_EX_RT
`define HAZ_SUB     `HAZ_EX_RS_EX_RT
`define HAZ_SUBU    `HAZ_EX_RS_EX_RT
`define HAZ_SW      `HAZ_EX_RS_W_RT
`define HAZ_SYSCALL `HAZ_NOTHING
`define HAZ_TEQ     `HAZ_EX_RS_EX_RT
`define HAZ_TEQI    `HAZ_EX_RS
`define HAZ_TGE     `HAZ_EX_RS_EX_RT
`define HAZ_TGEI    `HAZ_EX_RS
`define HAZ_TGEIU   `HAZ_EX_RS
`define HAZ_TGEU    `HAZ_EX_RS_EX_RT
`define HAZ_TLT     `HAZ_EX_RS_EX_RT
`define HAZ_TLTI    `HAZ_EX_RS
`define HAZ_TLTIU   `HAZ_EX_RS
`define HAZ_TLTU    `HAZ_EX_RS_EX_RT
`define HAZ_TNE     `HAZ_EX_RS_EX_RT
`define HAZ_TNEI    `HAZ_EX_RS
`define HAZ_XOR     `HAZ_EX_RS_EX_RT
`define HAZ_XORI    `HAZ_EX_RS

//------------------------------------------------------------------------------
/*
    Datapath controls.
    All signals are active High.
    ----------------------------------------------------------------------------
        Bit     Name                Description
    ----------------------------------------------------------------------------
        31 :                            Wants Rs by ID
        30 :                            Needs Rs by ID
        29 :                            Wants Rt by ID
        28 :                            Needs Rt by ID
        27 :                            Wants Rs by EX
        26 :                            Needs Rs by EX
        25 :                            Wants Rt by EX
        24 :                            Needs Rt by EX
        -------------------------------
        23 :    id_id_exception_source  Instruction can cause exception @ ID
        22 :    id_ex_exception_source  Instruction can cause exception @ EX
        21 :    id_mem_exception_source Instruction can cause exception @ MEM
        -------------------------------
        20 :    id_alu_operation        Operation to execute.
        19 :    .
        18 :    .
        17 :    .
        16 :    .
        -------------------------------
        15:     id_trap                 Trap instruction
        14:     id_trap_condition       Condition: ALU result = 0 (0), ALU result != 0 (1)
        -------------------------------
        13 :    id_gpr_we               Write enable (GPR)
        12 :    id_mem_to_gpr_select    Select data: ALU(0), MEM(1)
        -------------------------------
        11 :    id_alu_port_a_select    Select: Rs(0), shamt(1), 0x04(2), 0x10(3)
        10 :    .
        9  :    id_alu_port_b_select    Select: Rt(0), S/ZImm16(1), PCAdd4(2), CP0(3)
        8  :    .
        7  :    id_gpr_wa_select        Select register: Rd(0), Rt(1), 31(2)
        6  :    .
        -------------------------------
        5  :    id_jump                 Jump instruction
        4  :    id_branch               Branch instruction
        -------------------------------
        3  :    id_mem_write            Write to data memory
        2  :    id_mem_byte             Enable read/write one byte
        1  :    id_mem_halfword         Enable read/write 2 bytes (16 bits data)
        0  :    id_mem_data_sign_ext    Zero extend data (0) or Sign extend data (1)
    ----------------------------------------------------------------------------
*/
//------------------------------------------------------------------------------
`define DP_NONE        {`HAZ_NOTHING    , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_00_0000}
`define DP_ADD         {`HAZ_EX_RS_EX_RT, `EXCEPTION_EX  , `ALU_OP_ADD,   16'b00_10_000000_00_0000}
`define DP_ADDI        {`HAZ_EX_RS      , `EXCEPTION_EX  , `ALU_OP_ADD,   16'b00_10_000101_00_0000}
`define DP_ADDIU       {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_ADDU,  16'b00_10_000101_00_0000}
`define DP_ADDU        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_ADDU,  16'b00_10_000000_00_0000}
`define DP_AND         {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_10_000000_00_0000}
`define DP_ANDI        {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_10_000101_00_0000}
`define DP_BEQ         {`HAZ_ID_RS_ID_RT, `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_BGEZ        {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_BGEZAL      {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_ADD,   16'b00_10_101010_01_0000}
`define DP_BGTZ        {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_BLEZ        {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_BLTZ        {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_BLTZAL      {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_ADD,   16'b00_10_101010_01_0000}
`define DP_BNE         {`HAZ_ID_RS_ID_RT, `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_BREAK       {`HAZ_NOTHING    , `EXCEPTION_ID  , `ALU_OP_AND,   16'b00_00_000000_00_0000}
`define DP_CLO         {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_CLO,   16'b00_10_000000_00_0000}
`define DP_CLZ         {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_CLZ,   16'b00_10_000000_00_0000}
`define DP_DIV         {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_DIV,   16'b00_00_000000_00_0000}
`define DP_DIVU        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_DIVU,  16'b00_00_000000_00_0000}
`define DP_ERET        {`HAZ_NOTHING    , `EXCEPTION_ID  , `ALU_OP_AND,   16'b00_00_000000_00_0000}
`define DP_J           {`HAZ_NOTHING    , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_10_0000}
`define DP_JAL         {`HAZ_NOTHING    , `EXCEPTION_NONE, `ALU_OP_ADD,   16'b00_10_101010_10_0000}
`define DP_JALR        {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_ADD,   16'b00_10_101010_01_0000}
`define DP_JR          {`HAZ_ID_RS      , `EXCEPTION_NONE, `ALU_OP_AND,   16'b00_00_000000_01_0000}
`define DP_LB          {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_0101}
`define DP_LBU         {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_0100}
`define DP_LH          {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_0011}
`define DP_LHU         {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_0010}
`define DP_LL          {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_0000}
`define DP_LUI         {`HAZ_NOTHING    , `EXCEPTION_NONE, `ALU_OP_SLL,   16'b00_10_110101_00_0000}
`define DP_LW          {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_0000}
`define DP_MADD        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_MADD,  16'b00_00_000000_00_0000}
`define DP_MADDU       {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_MADDU, 16'b00_00_000000_00_0000}
`define DP_MFC0        {`HAZ_NOTHING    , `EXCEPTION_ID  , `ALU_OP_B,     16'b00_10_001101_00_0000}
`define DP_MFHI        {`HAZ_NOTHING    , `EXCEPTION_NONE, `ALU_OP_MFHI,  16'b00_10_000000_00_0000}
`define DP_MFLO        {`HAZ_NOTHING    , `EXCEPTION_NONE, `ALU_OP_MFLO,  16'b00_10_000000_00_0000}
`define DP_MOVN        {`HAZ_ID_RT_EX_RS, `EXCEPTION_NONE, `ALU_OP_A,     16'b00_00_000000_00_0000}
`define DP_MOVZ        {`HAZ_ID_RT_EX_RS, `EXCEPTION_NONE, `ALU_OP_A,     16'b00_00_000000_00_0000}
`define DP_MSUB        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_MSUB,  16'b00_00_000000_00_0000}
`define DP_MSUBU       {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_MSUBU, 16'b00_00_000000_00_0000}
`define DP_MTC0        {`HAZ_ID_RT      , `EXCEPTION_ID  , `ALU_OP_AND,   16'b00_00_000000_00_0000}
`define DP_MTHI        {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_MTHI,  16'b00_00_000000_00_0000}
`define DP_MTLO        {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_MTLO,  16'b00_00_000000_00_0000}
`define DP_MULT        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_MULS,  16'b00_00_000000_00_0000}
`define DP_MULTU       {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_MULU,  16'b00_00_000000_00_0000}
`define DP_NOR         {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_NOR,   16'b00_10_000000_00_0000}
`define DP_OR          {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_OR,    16'b00_10_000000_00_0000}
`define DP_ORI         {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_OR,    16'b00_10_000101_00_0000}
`define DP_SB          {`HAZ_EX_RS_W_RT , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_00_000100_00_1100}
`define DP_SC          {`HAZ_EX_RS_W_RT , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_11_000101_00_1000}
`define DP_SH          {`HAZ_EX_RS_W_RT , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_00_000100_00_1010}
`define DP_SLL         {`HAZ_EX_RT      , `EXCEPTION_NONE, `ALU_OP_SLL,   16'b00_10_010000_00_0000}
`define DP_SLLV        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_SLL,   16'b00_10_000000_00_0000}
`define DP_SLT         {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_SLT,   16'b00_10_000000_00_0000}
`define DP_SLTI        {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_SLT,   16'b00_10_000101_00_0000}
`define DP_SLTIU       {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_SLTU,  16'b00_10_000101_00_0000}
`define DP_SLTU        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_SLTU,  16'b00_10_000000_00_0000}
`define DP_SRA         {`HAZ_EX_RT      , `EXCEPTION_NONE, `ALU_OP_SRA,   16'b00_10_010000_00_0000}
`define DP_SRAV        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_SRA,   16'b00_10_000000_00_0000}
`define DP_SRL         {`HAZ_EX_RT      , `EXCEPTION_NONE, `ALU_OP_SRL,   16'b00_10_010000_00_0000}
`define DP_SRLV        {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_SRL,   16'b00_10_000000_00_0000}
`define DP_SUB         {`HAZ_EX_RS_EX_RT, `EXCEPTION_EX  , `ALU_OP_SUB,   16'b00_10_000000_00_0000}
`define DP_SUBU        {`HAZ_EX_RS_EX_RT, `EXCEPTION_EX  , `ALU_OP_SUBU,  16'b00_10_000000_00_0000}
`define DP_SW          {`HAZ_EX_RS_W_RT , `EXCEPTION_MEM , `ALU_OP_ADDU,  16'b00_00_000100_00_1000}
`define DP_SYSCALL     {`HAZ_NOTHING    , `EXCEPTION_ID  , `ALU_OP_ADDU,  16'b00_00_000000_00_0000}
`define DP_TEQ         {`HAZ_EX_RS_EX_RT, `EXCEPTION_MEM , `ALU_OP_SUBU,  16'b10_00_000000_00_0000}
`define DP_TEQI        {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_SUBU,  16'b10_00_000000_00_0000}
`define DP_TGE         {`HAZ_EX_RS_EX_RT, `EXCEPTION_MEM , `ALU_OP_SLT,   16'b10_00_000000_00_0000}
`define DP_TGEI        {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_SLT,   16'b10_00_000000_00_0000}
`define DP_TGEIU       {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_SLTU,  16'b10_00_000000_00_0000}
`define DP_TGEU        {`HAZ_EX_RS_EX_RT, `EXCEPTION_MEM , `ALU_OP_SLTU,  16'b10_00_000000_00_0000}
`define DP_TLT         {`HAZ_EX_RS_EX_RT, `EXCEPTION_MEM , `ALU_OP_SLT,   16'b11_00_000000_00_0000}
`define DP_TLTI        {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_SLT,   16'b11_00_000000_00_0000}
`define DP_TLTIU       {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_SLTU,  16'b11_00_000000_00_0000}
`define DP_TLTU        {`HAZ_EX_RS_EX_RT, `EXCEPTION_MEM , `ALU_OP_SLTU,  16'b11_00_000000_00_0000}
`define DP_TNE         {`HAZ_EX_RS_EX_RT, `EXCEPTION_MEM , `ALU_OP_SUBU,  16'b11_00_000000_00_0000}
`define DP_TNEI        {`HAZ_EX_RS      , `EXCEPTION_MEM , `ALU_OP_SUBU,  16'b11_00_000000_00_0000}
`define DP_XOR         {`HAZ_EX_RS_EX_RT, `EXCEPTION_NONE, `ALU_OP_XOR,   16'b00_10_000000_00_0000}
`define DP_XORI        {`HAZ_EX_RS      , `EXCEPTION_NONE, `ALU_OP_XOR,   16'b00_10_000101_00_0000}

// EOF

//==================================================================================================
//  Filename      : antares_add.v
//  Created On    : Tue Sep  1 10:15:22 2015
//  Last Modified : Sat Nov 07 11:45:24 2015
//  Revision      : 0.1
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : A simple 32-bits adder
//==================================================================================================

module antares_add (
                    input wire [31:0]  a,
                    input wire [31:0]  b,
                    output wire [31:0] c
                    );


    assign c = a + b;

endmodule // antares_add
//==================================================================================================
//  Filename      : antares_alu.v
//  Created On    : Thu Sep  3 09:14:03 2015
//  Last Modified : Sat Nov 07 11:45:51 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : The Execution unit.
//                  Performs the following operations:
//                  - Arithmetic
//                  - Logical
//                  - Shift
//                  - Comparison
//==================================================================================================


module antares_alu #(parameter ENABLE_HW_MULT = 1,
                     parameter ENABLE_HW_DIV  = 1,
                     parameter ENABLE_HW_CLOZ = 1
                     )(
                       input wire             clk,
                       input wire             rst,
                       input wire [31:0]      ex_alu_port_a,
                       input wire [31:0]      ex_alu_port_b,
                       input wire [4:0]       ex_alu_operation,
                       input wire             ex_stall,
                       input wire             ex_flush,
                       output wire            ex_request_stall,
                       output reg [31:0] ex_alu_result,
                       output wire            ex_b_is_zero,
                       output reg        exc_overflow
                       );

    //--------------------------------------------------------------------------
    // Signal Declaration: reg
    //--------------------------------------------------------------------------
    reg [63:0]         hilo; // hold the result from MULT instruction
    reg                div_active; // 1 if the divider is currently active.
    reg                hilo_access;        // check access

    ///-------------------------------------------------------------------------
    // Signal Declaration: wire
    //--------------------------------------------------------------------------
    wire [31:0]        A;                // Port A (unsigned)
    wire [31:0]        B;                // Port B (unsigned)
    wire signed [31:0] add_sub_result;   // A+B or A - B
    wire [4:0]         ex_alu_operation; // Operation
    wire [63:0]        mult_result;      // Multiplication result
    wire [31:0]        hi;               // HILO[63:32]
    wire [31:0]        lo;               // HILO[31:0]
    wire [31:0]        shift_result;     // Shift result
    wire [31:0]        quotient;         // Division
    wire [31:0]        remainder;        // Division
    wire               op_divs;          // Signed division
    wire               op_divu;          // Unsigned division
    wire               div_stall;        // Stall
    wire [31:0]        dividend;
    wire [31:0]        divisor;
    wire               enable_ex;        // Enable operations

    wire               op_mults;         // Signed multiplication
    wire               op_multu;         // Unsigned multiplication
    wire               mult_active;      // Mult ex_alu_operation active inside the pipeline
    wire               mult_ready;       // Mult result ready
    wire               mult_stall;
    wire [31:0]        mult_input_a;
    wire [31:0]        mult_input_b;
    wire               mult_signed_op;
    wire               mult_enable_op;

    wire               _op_divs;
    wire               _op_divu;
    wire               _op_mults;
    wire               _op_multu;

    wire [31:0]        shift_input_data;
    wire [4:0]         shift_shamnt;
    wire               shift_direction;
    wire               shift_sign_extend;

    wire [5:0]         clo_result;
    wire [5:0]         clz_result;

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign A                  = ex_alu_port_a;                                                  // unsigned
    assign B                  = ex_alu_port_b;                                                  // unsigned
    assign ex_b_is_zero       = (B == 32'b0);
    assign add_sub_result     = ((ex_alu_operation == `ALU_OP_ADD) | (ex_alu_operation == `ALU_OP_ADDU)) ? (A + B) : (A - B);
    assign hi                 = hilo[63:32];
    assign lo                 = hilo[31:0];
    assign enable_ex          = ~((ex_stall ^ ex_request_stall) | ex_flush);
    assign _op_divs           = (B != 32'd0) & (div_active == 1'b0) & (ex_alu_operation == `ALU_OP_DIV);
    assign _op_divu           = (B != 32'd0) & (div_active == 1'b0) & (ex_alu_operation == `ALU_OP_DIVU);
    assign _op_mults          = (mult_active == 1'b0) & (ex_alu_operation == `ALU_OP_MULS);
    assign _op_multu          = (mult_active == 1'b0) & (ex_alu_operation == `ALU_OP_MULU);
    assign op_divs            =  _op_divs & enable_ex;
    assign op_divu            =  _op_divu & enable_ex;
    assign op_mults           =  _op_mults & enable_ex;
    assign op_multu           =  _op_multu & enable_ex;
    // request stall if: division, multiplication, and device unit is busy.
    // Do not use the op_XXX signal: combinatorial loop. So, this will stall the unit waiting for the instruction.
    assign ex_request_stall   = (_op_divu | _op_divs | div_stall | _op_mults | _op_multu | (mult_active ^ mult_ready)) & hilo_access;
    assign mult_stall         = ex_stall ^ex_request_stall;
    assign mult_input_a       = ex_alu_port_a[31:0];
    assign mult_input_b       = ex_alu_port_b[31:0];
    assign mult_signed_op     = ex_alu_operation == `ALU_OP_MULS;
    assign mult_enable_op     = op_mults | op_multu;
    assign shift_input_data   = ex_alu_port_b;
    assign shift_shamnt       = ex_alu_port_a[4:0];
    assign shift_direction    = ex_alu_operation == `ALU_OP_SLL;
    assign shift_sign_extend  = ex_alu_operation == `ALU_OP_SRA;
    assign dividend           = ex_alu_port_a[31:0];
    assign divisor            = ex_alu_port_b[31:0];

    //--------------------------------------------------------------------------
    // the BIG multiplexer
    //--------------------------------------------------------------------------
    always @(*) begin
        case(ex_alu_operation)
            `ALU_OP_ADD  : ex_alu_result = add_sub_result;
            `ALU_OP_ADDU : ex_alu_result = add_sub_result;
            `ALU_OP_SUB  : ex_alu_result = add_sub_result;
            `ALU_OP_SUBU : ex_alu_result = add_sub_result;
            `ALU_OP_AND  : ex_alu_result = ex_alu_port_a & ex_alu_port_b;
            `ALU_OP_CLO  : ex_alu_result = {26'b0, clo_result};
            `ALU_OP_CLZ  : ex_alu_result = {26'b0, clz_result};
            `ALU_OP_NOR  : ex_alu_result = ~(ex_alu_port_a | ex_alu_port_b);
            `ALU_OP_OR   : ex_alu_result = ex_alu_port_a | ex_alu_port_b;
            `ALU_OP_SLL  : ex_alu_result = shift_result;
            `ALU_OP_SRA  : ex_alu_result = shift_result;
            `ALU_OP_SRL  : ex_alu_result = shift_result;
            `ALU_OP_XOR  : ex_alu_result = ex_alu_port_a ^ ex_alu_port_b;
            `ALU_OP_MFHI : ex_alu_result = hi;
            `ALU_OP_MFLO : ex_alu_result = lo;
            `ALU_OP_SLT  : ex_alu_result = {31'b0, $signed(ex_alu_port_a) < $signed(ex_alu_port_b)};
            `ALU_OP_SLTU : ex_alu_result = {31'b0, ex_alu_port_a < ex_alu_port_b};
            `ALU_OP_A    : ex_alu_result = ex_alu_port_a;
            `ALU_OP_B    : ex_alu_result = ex_alu_port_b;
            default      : ex_alu_result = 32'bx;
        endcase // case (ex_alu_operation)
    end // always @ (*)

    //--------------------------------------------------------------------------
    // Detect Overflow
    //--------------------------------------------------------------------------
    always @(*) begin
        case (ex_alu_operation)
            `ALU_OP_ADD : exc_overflow = ((A[31] ~^ B[31]) & (A[31] ^ add_sub_result[31]));
            `ALU_OP_SUB : exc_overflow = ((A[31]  ^ B[31]) & (A[31] ^ add_sub_result[31]));
            default     : exc_overflow = 1'b0;
        endcase // case (ex_alu_operation)
    end

    //--------------------------------------------------------------------------
    // Write to HILO register
    // Div has priority over mult
    //
    // WARNING: THIS HAVE A BUG: HILO + X can't be done, unless the multiplier
    // has finished.
    // TODO: CHECK
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            /*AUTORESET*/
            // Beginning of autoreset for uninitialized flops
            hilo <= 64'h0;
            // End of automatics
        end
        else if ((div_stall == 1'b0) & (div_active == 1'b1)) begin // Divider unit has finished.
            hilo <= {remainder, quotient};
        end
        else if(mult_ready) begin
            case (ex_alu_operation)
                `ALU_OP_MULS    : hilo <= mult_result;
                `ALU_OP_MULU    : hilo <= mult_result;
                `ALU_OP_MADD    : hilo <= hilo + mult_result;
                `ALU_OP_MADDU   : hilo <= hilo + mult_result;
                `ALU_OP_MSUB    : hilo <= hilo - mult_result;
                `ALU_OP_MSUBU   : hilo <= hilo - mult_result;
                default         : hilo <= hilo;
            endcase // case (ex_alu_operation)
        end // if (enable_ex & mult_ready)
        else if (enable_ex) begin
            case (ex_alu_operation)
                `ALU_OP_MTHI    : hilo <= {A, lo};
                `ALU_OP_MTLO    : hilo <= {hi, A};
                default         : hilo <= hilo;
            endcase // case (ex_alu_operation)
        end
    end // always @ (posedge clk)

    //--------------------------------------------------------------------------
    // Check if the div unit is currently active
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            /*AUTORESET*/
            // Beginning of autoreset for uninitialized flops
            div_active <= 1'h0;
            // End of automatics
        end
        else begin
            case(div_active)
                1'd0 : div_active <= (op_divs || op_divu) ? 1'b1 : 1'b0;
                1'd1 : div_active <= (~div_stall) ? 1'b0 : 1'b1;
            endcase // case (div_active)
        end // else: !if(rst)
    end // always @ (posedge clk)

    //--------------------------------------------------------------------------
    // Detect access to HILO register
    //--------------------------------------------------------------------------
    always @(*) begin
        case (ex_alu_operation)
            `ALU_OP_DIV   : hilo_access = 1'b1;
            `ALU_OP_DIVU  : hilo_access = 1'b1;
            `ALU_OP_MULS  : hilo_access = 1'b1;
            `ALU_OP_MULU  : hilo_access = 1'b1;
            `ALU_OP_MADD  : hilo_access = 1'b1;
            `ALU_OP_MADDU : hilo_access = 1'b1;
            `ALU_OP_MSUB  : hilo_access = 1'b1;
            `ALU_OP_MSUBU : hilo_access = 1'b1;
            `ALU_OP_MTHI  : hilo_access = 1'b1;
            `ALU_OP_MTLO  : hilo_access = 1'b1;
            `ALU_OP_MFHI  : hilo_access = 1'b1;
            `ALU_OP_MFLO  : hilo_access = 1'b1;
            default       : hilo_access = 1'b0;
        endcase
    end

    //--------------------------------------------------------------------------
    // Count Leading Ones/Zeros
    //--------------------------------------------------------------------------
    generate
        // Hardware CLO_CLZ
        if (ENABLE_HW_CLOZ) begin
            antares_cloz cloz(/*AUTOINST*/
                              // Outputs
                              .clo_result       (clo_result[5:0]),
                              .clz_result       (clz_result[5:0]),
                              // Inputs
                              .A                (A[31:0]));
        end
        // Disable
        else begin
            assign clo_result = 6'dx;
            assign clz_result = 6'bx;
        end // else: !if(ENABLE_HW_CLOZ)

    endgenerate

    //--------------------------------------------------------------------------
    // Shifter: instantiation
    //--------------------------------------------------------------------------
    antares_shifter shifter(/*AUTOINST*/
                            // Outputs
                            .shift_result       (shift_result[31:0]),
                            // Inputs
                            .shift_input_data   (shift_input_data[31:0]),
                            .shift_shamnt       (shift_shamnt[4:0]),
                            .shift_direction    (shift_direction),
                            .shift_sign_extend  (shift_sign_extend));

    //--------------------------------------------------------------------------
    // 32 x 32 bits multiplier: instantiation
    //--------------------------------------------------------------------------
    generate
        // Hardware multiplier
        if (ENABLE_HW_MULT) begin
            antares_multiplier mult(// Inputs
                                    .flush              (ex_flush),
                                    /*AUTOINST*/
                                    // Outputs
                                    .mult_result        (mult_result[63:0]),
                                    .mult_active        (mult_active),
                                    .mult_ready         (mult_ready),
                                    // Inputs
                                    .clk                (clk),
                                    .rst                (rst),
                                    .mult_input_a       (mult_input_a[31:0]),
                                    .mult_input_b       (mult_input_b[31:0]),
                                    .mult_signed_op     (mult_signed_op),
                                    .mult_enable_op     (mult_enable_op),
                                    .mult_stall         (mult_stall));
        end // if (ENABLE_HW_MULT)
        //  No hardware multiplier
        else begin
            assign mult_result  = 64'h0;    // disabled
            assign mult_active  = 1'b0;     // disabled
            assign mult_ready   = 1'b0;     // disabled
        end // else: !if(ENABLE_HW_MULT)
    endgenerate

    //--------------------------------------------------------------------------
    // instantiate the divider unit
    //--------------------------------------------------------------------------
    generate
        // Hardware divider
        if (ENABLE_HW_DIV) begin
            antares_divider divider(/*AUTOINST*/
                                    // Outputs
                                    .quotient           (quotient[31:0]),
                                    .remainder          (remainder[31:0]),
                                    .div_stall          (div_stall),
                                    // Inputs
                                    .clk                (clk),
                                    .rst                (rst),
                                    .op_divs            (op_divs),
                                    .op_divu            (op_divu),
                                    .dividend           (dividend[31:0]),
                                    .divisor            (divisor[31:0]));
        end // if (ENABLE_HW_DIV)
        // No hardware divider
        else begin
            assign quotient  = 32'h0;   // disabled
            assign remainder = 32'h0;   // disabled
            assign div_stall = 1'b0;    // disabled
        end // else: !if(ENABLE_HW_DIV)
    endgenerate

endmodule // antares_alu
//==================================================================================================
//  Filename      : antares_branch_unit.v
//  Created On    : Fri Sep  4 21:35:54 2015
//  Last Modified : Sat Nov 07 11:49:10 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Branch target calculation
//==================================================================================================


module antares_branch_unit (
                            input wire [5:0]       opcode,            // Instruction opcode
                            input wire [31:0]      id_pc_add4,        // Instruction address + 4
                            input wire [31:0]      id_data_rs,        // Data from R0
                            input wire [31:0]      id_data_rt,        // Data from R1
                            input wire [25:0]      op_imm26,          // imm21/Imm16
                            output reg [31:0] pc_branch_address, // Destination address
                            output reg        id_take_branch     // Valid branch
                            ) ;

    //--------------------------------------------------------------------------
    // Signal Declaration: wire
    //--------------------------------------------------------------------------
    wire              beq;
    wire              bne;
    wire              bgez;
    wire              bgtz;
    wire              blez;
    wire              bltz;
    wire [31:0]       long_jump;
    wire [31:0]       short_jump;
    wire [5:0]        inst_function;
    wire [4:0]        op_rt;

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign beq           = id_data_rs == id_data_rt;
    assign bne           = ~beq;
    assign bgez          = ~bltz;
    assign bgtz          = ~blez;
    assign blez          = bltz | ~(|id_data_rs);
    assign bltz          = id_data_rs[31];
    assign long_jump     = {id_pc_add4[31:28], op_imm26, 2'b00 };
    assign short_jump    = $signed(id_pc_add4) + $signed( { {14{op_imm26[15]}}, op_imm26[`ANTARES_INSTR_IMM16], 2'b00 } );
    assign inst_function = op_imm26[`ANTARES_INSTR_FUNCT];
    assign op_rt         = op_imm26[`ANTARES_INSTR_RT];

    //--------------------------------------------------------------------------
    // Get branch address
    //--------------------------------------------------------------------------
    always @(*) begin
        case (opcode)
            `OP_BEQ         : begin pc_branch_address = short_jump; id_take_branch = beq;  end
            `OP_BGTZ        : begin pc_branch_address = short_jump; id_take_branch = bgtz; end
            `OP_BLEZ        : begin pc_branch_address = short_jump; id_take_branch = blez; end
            `OP_BNE         : begin pc_branch_address = short_jump; id_take_branch = bne;  end
            `OP_J           : begin pc_branch_address = long_jump;  id_take_branch = 1'b1; end
            `OP_JAL         : begin pc_branch_address = long_jump;  id_take_branch = 1'b1; end
            `OP_TYPE_REGIMM : begin
                case (op_rt)
                    `RT_OP_BGEZ   : begin pc_branch_address = short_jump; id_take_branch = bgez; end
                    `RT_OP_BGEZAL : begin pc_branch_address = short_jump; id_take_branch = bgez; end
                    `RT_OP_BLTZ   : begin pc_branch_address = short_jump; id_take_branch = bltz; end
                    `RT_OP_BLTZAL : begin pc_branch_address = short_jump; id_take_branch = bltz; end
                    default       : begin pc_branch_address = 32'bx;    id_take_branch = 1'b0; end
                endcase // case (op_rt)
            end
            `OP_TYPE_R      : begin
                case(inst_function)
                    `FUNCTION_OP_JALR : begin pc_branch_address = id_data_rs; id_take_branch = 1'b1; end
                    `FUNCTION_OP_JR   : begin pc_branch_address = id_data_rs; id_take_branch = 1'b1; end
                    default           : begin pc_branch_address = 32'bx; id_take_branch = 1'b0; end
                endcase // case (inst_function)
            end
            default         : begin pc_branch_address = 32'bx; id_take_branch = 1'b0;    end
        endcase // case (opcode)
    end // always @ (*)
endmodule // antares_branch_unit
//==================================================================================================
//  Filename      : antares_cloz.v
//  Created On    : Thu Sep  3 16:03:13 2015
//  Last Modified : Sat Nov 07 11:49:40 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Count leading ones/zeros unit.
//==================================================================================================

module antares_cloz (
                     input wire [31:0] A,
                     output wire [5:0] clo_result,
                     output wire [5:0] clz_result
                     );

    /*AUTOREG*/
    // Beginning of automatic regs (for this module's undeclared outputs)
    reg [5:0]           clo_result;
    reg [5:0]           clz_result;
    // End of automatics

    //--------------------------------------------------------------------------
    // Count Leading Ones
    //--------------------------------------------------------------------------
    always @(*) begin
        casez (A)
            32'b0zzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd0;
            32'b10zz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd1;
            32'b110z_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd2;
            32'b1110_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd3;
            32'b1111_0zzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd4;
            32'b1111_10zz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd5;
            32'b1111_110z_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd6;
            32'b1111_1110_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd7;
            32'b1111_1111_0zzz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd8;
            32'b1111_1111_10zz_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd9;
            32'b1111_1111_110z_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd10;
            32'b1111_1111_1110_zzzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd11;
            32'b1111_1111_1111_0zzz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd12;
            32'b1111_1111_1111_10zz_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd13;
            32'b1111_1111_1111_110z_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd14;
            32'b1111_1111_1111_1110_zzzz_zzzz_zzzz_zzzz : clo_result  = 6'd15;
            32'b1111_1111_1111_1111_0zzz_zzzz_zzzz_zzzz : clo_result  = 6'd16;
            32'b1111_1111_1111_1111_10zz_zzzz_zzzz_zzzz : clo_result  = 6'd17;
            32'b1111_1111_1111_1111_110z_zzzz_zzzz_zzzz : clo_result  = 6'd18;
            32'b1111_1111_1111_1111_1110_zzzz_zzzz_zzzz : clo_result  = 6'd19;
            32'b1111_1111_1111_1111_1111_0zzz_zzzz_zzzz : clo_result  = 6'd20;
            32'b1111_1111_1111_1111_1111_10zz_zzzz_zzzz : clo_result  = 6'd21;
            32'b1111_1111_1111_1111_1111_110z_zzzz_zzzz : clo_result  = 6'd22;
            32'b1111_1111_1111_1111_1111_1110_zzzz_zzzz : clo_result  = 6'd23;
            32'b1111_1111_1111_1111_1111_1111_0zzz_zzzz : clo_result  = 6'd24;
            32'b1111_1111_1111_1111_1111_1111_10zz_zzzz : clo_result  = 6'd25;
            32'b1111_1111_1111_1111_1111_1111_110z_zzzz : clo_result  = 6'd26;
            32'b1111_1111_1111_1111_1111_1111_1110_zzzz : clo_result  = 6'd27;
            32'b1111_1111_1111_1111_1111_1111_1111_0zzz : clo_result  = 6'd28;
            32'b1111_1111_1111_1111_1111_1111_1111_10zz : clo_result  = 6'd29;
            32'b1111_1111_1111_1111_1111_1111_1111_110z : clo_result  = 6'd30;
            32'b1111_1111_1111_1111_1111_1111_1111_1110 : clo_result  = 6'd31;
            32'b1111_1111_1111_1111_1111_1111_1111_1111 : clo_result  = 6'd32;
            default : clo_result                                      = 6'd0;
        endcase // casez (A)
    end // always @ (*)

    //--------------------------------------------------------------------------
    // Count Leading Zeros
    //--------------------------------------------------------------------------
    always @(*) begin
        casez (A)
            32'b1zzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd0;
            32'b01zz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd1;
            32'b001z_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd2;
            32'b0001_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd3;
            32'b0000_1zzz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd4;
            32'b0000_01zz_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd5;
            32'b0000_001z_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd6;
            32'b0000_0001_zzzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd7;
            32'b0000_0000_1zzz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd8;
            32'b0000_0000_01zz_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd9;
            32'b0000_0000_001z_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd10;
            32'b0000_0000_0001_zzzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd11;
            32'b0000_0000_0000_1zzz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd12;
            32'b0000_0000_0000_01zz_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd13;
            32'b0000_0000_0000_001z_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd14;
            32'b0000_0000_0000_0001_zzzz_zzzz_zzzz_zzzz : clz_result  = 6'd15;
            32'b0000_0000_0000_0000_1zzz_zzzz_zzzz_zzzz : clz_result  = 6'd16;
            32'b0000_0000_0000_0000_01zz_zzzz_zzzz_zzzz : clz_result  = 6'd17;
            32'b0000_0000_0000_0000_001z_zzzz_zzzz_zzzz : clz_result  = 6'd18;
            32'b0000_0000_0000_0000_0001_zzzz_zzzz_zzzz : clz_result  = 6'd19;
            32'b0000_0000_0000_0000_0000_1zzz_zzzz_zzzz : clz_result  = 6'd20;
            32'b0000_0000_0000_0000_0000_01zz_zzzz_zzzz : clz_result  = 6'd21;
            32'b0000_0000_0000_0000_0000_001z_zzzz_zzzz : clz_result  = 6'd22;
            32'b0000_0000_0000_0000_0000_0001_zzzz_zzzz : clz_result  = 6'd23;
            32'b0000_0000_0000_0000_0000_0000_1zzz_zzzz : clz_result  = 6'd24;
            32'b0000_0000_0000_0000_0000_0000_01zz_zzzz : clz_result  = 6'd25;
            32'b0000_0000_0000_0000_0000_0000_001z_zzzz : clz_result  = 6'd26;
            32'b0000_0000_0000_0000_0000_0000_0001_zzzz : clz_result  = 6'd27;
            32'b0000_0000_0000_0000_0000_0000_0000_1zzz : clz_result  = 6'd28;
            32'b0000_0000_0000_0000_0000_0000_0000_01zz : clz_result  = 6'd29;
            32'b0000_0000_0000_0000_0000_0000_0000_001z : clz_result  = 6'd30;
            32'b0000_0000_0000_0000_0000_0000_0000_0001 : clz_result  = 6'd31;
            32'b0000_0000_0000_0000_0000_0000_0000_0000 : clz_result  = 6'd32;
            default : clz_result                                      = 6'd0;
        endcase // casez (A)
    end // always @ (*)
endmodule // antares_cloz
//==================================================================================================
//  Filename      : antares_control_unit.v
//  Created On    : Fri Sep  4 11:55:21 2015
//  Last Modified : Sat Nov 07 11:53:12 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Instruction decode and control unit (no pipeline control)
//==================================================================================================


module antares_control_unit #(parameter ENABLE_HW_MULT = 1, // Enable multiply instructions
                              parameter ENABLE_HW_DIV = 1,  // Enable div instructions
                              parameter ENABLE_HW_CLOZ = 1  // Enable CL=/CLZ instructions
                              )(
                                input wire [5:0]  opcode,                  // The instruction opcode
                                input wire [5:0]  op_function,             // For RR-type instruction
                                input wire [4:0]  op_rs,                   // For mtc0 and mfc0 instructions
                                input wire [4:0]  op_rt,                   // For branch instructions
                                output wire [7:0] dp_hazard,
                                output wire       id_imm_sign_ext,         // sign extend the imm16
                                output wire       id_movn,                 // MOVN instruction
                                output wire       id_movz,                 // MOVZ instruction
                                output wire       id_llsc,                 // LL/SC instructions
                                output wire       id_syscall,              // Syscall exception
                                output wire       id_breakpoint,           // Breakpoint exception
                                output wire       id_reserved,             // Reserved instruction exception
                                output wire       id_mfc0,                 // Coprocessor 0 instruction
                                output wire       id_mtc0,                 // Coprocessor 0 instruction
                                output wire       id_eret,                 // Coprocessor 0 instruction
                                output wire       id_cp1_instruction,      // Coprocessor 1 instruction
                                output wire       id_cp2_instruction,      // Coprocessor 2 instruction
                                output wire       id_cp3_instruction,      // Coprocessor 3 instruction
                                output wire       id_id_exception_source,  // Instruction is a potential source of exception
                                output wire       id_ex_exception_source,  // Instruction is a potential source of exception
                                output wire       id_mem_exception_source, // Instruction is a potential source of exception
                                output wire       id_trap,                 // Trap instruction
                                output wire       id_trap_condition,       // Trap condition
                                output wire       id_gpr_we,               // write data from WB stage, to GPR
                                output wire       id_mem_to_gpr_select,    // Select GPR write data: MEM or ALU
                                output wire [4:0] id_alu_operation,        // ALU function
                                output wire [1:0] id_alu_port_a_select,    // Shift, jump and link
                                output wire [1:0] id_alu_port_b_select,    // R-instruction, I-instruction or jump
                                output wire [1:0] id_gpr_wa_select,        // Select GPR write address
                                output wire       id_jump,                 // Jump instruction
                                output wire       id_branch,               // Branch instruction
                                output wire       id_mem_write,            // Write to Memory: 0 = read, 1 = write.
                                output wire       id_mem_byte,             // Read/Write one byte
                                output wire       id_mem_halfword,         // Read/Write halfword (16 bits )
                                output wire       id_mem_data_sign_ext     // Sign extend for byte/halfword memory operations
                                );

    //--------------------------------------------------------------------------
    // Signal Declaration: reg
    //--------------------------------------------------------------------------
    reg     [31:0]  datapath;               // all control signals

    //--------------------------------------------------------------------------
    // Signal Declaration: wires
    //--------------------------------------------------------------------------
    wire    no_mult;
    wire    no_div;
    wire    no_clo_clz;

    //--------------------------------------------------------------------------
    // assigments
    //--------------------------------------------------------------------------
    assign id_imm_sign_ext     = (opcode != `OP_ANDI) & (opcode != `OP_ORI) & (opcode != `OP_XORI);
    assign id_movn             = (opcode == `OP_TYPE_R) & (op_function == `FUNCTION_OP_MOVN);
    assign id_movz             = (opcode == `OP_TYPE_R) & (op_function == `FUNCTION_OP_MOVZ);
    assign id_llsc             = (opcode == `OP_LL) | (opcode == `OP_SC);
    assign id_syscall          = (opcode == `OP_TYPE_R) & (op_function == `FUNCTION_OP_SYSCALL);
    assign id_breakpoint       = (opcode == `OP_TYPE_R) & (op_function == `FUNCTION_OP_BREAK);
    assign id_mfc0             = (opcode == `OP_TYPE_CP0) & (op_rs == `RS_OP_MFC);
    assign id_mtc0             = (opcode == `OP_TYPE_CP0) & (op_rs == `RS_OP_MTC);
    assign id_eret             = (opcode == `OP_TYPE_CP0) & (op_rs == `RS_OP_ERET) & (op_function == `FUNCTION_OP_ERET);
    assign id_cp1_instruction  = (opcode == `OP_TYPE_CP1);
    assign id_cp2_instruction  = (opcode == `OP_TYPE_CP2);
    assign id_cp3_instruction  = (opcode == `OP_TYPE_CP3);
    assign id_reserved         = no_mult | no_div | no_clo_clz;

    //--------------------------------------------------------------------------
    // Check for mult instructions
    //--------------------------------------------------------------------------
    generate
        if(ENABLE_HW_MULT) begin
            assign no_mult = 1'b0;
        end
        else begin
            assign no_mult = ((datapath[20:16] == `ALU_OP_MADD) | (datapath[20:16] == `ALU_OP_MADDU) |
                              (datapath[20:16] == `ALU_OP_MSUB) | (datapath[20:16] == `ALU_OP_MSUBU) |
                              (datapath[20:16] == `ALU_OP_MULS) | (datapath[20:16] == `ALU_OP_MULU));
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Check for div instructions
    //--------------------------------------------------------------------------
    generate
        if(ENABLE_HW_DIV) begin
            assign no_div = 1'b0;
        end
        else begin
            assign no_div = ((datapath[20:16] == `ALU_OP_DIV) | (datapath[20:16] == `ALU_OP_DIVU));
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Check for CL0/CLZ instructions
    //--------------------------------------------------------------------------
    generate
        if(ENABLE_HW_CLOZ) begin
            assign no_clo_clz = 1'b0;
        end
        else begin
            assign no_clo_clz = ((datapath[20:16] == `ALU_OP_CLO) | (datapath[20:16] == `ALU_OP_CLZ));
        end
    endgenerate

    /*
     Datapath controls.
     All signals are active High.
     ----------------------------------------------------------------------------
     Bit     Name                Description
     ----------------------------------------------------------------------------
        31 :                            Wants Rs by ID
        30 :                            Needs Rs by ID
        29 :                            Wants Rt by ID
        28 :                            Needs Rt by ID
        27 :                            Wants Rs by EX
        26 :                            Needs Rs by EX
        25 :                            Wants Rt by EX
        24 :                            Needs Rt by EX
        -------------------------------
        23 :    id_id_exception_source  Instruction can cause exception @ ID
        22 :    id_ex_exception_source  Instruction can cause exception @ EX
        21 :    id_mem_exception_source Instruction can cause exception @ MEM
        -------------------------------
        20 :    id_alu_operation        Operation to execute.
        19 :    .
        18 :    .
        17 :    .
        16 :    .
        -------------------------------
        15:     id_trap                 Trap instruction
        14:     id_trap_condition       Condition: ALU result = 0 (0), ALU result != 0 (1)
        -------------------------------
        13 :    id_gpr_we               Write enable (GPR)
        12 :    id_mem_to_gpr_select    Select data: ALU(0), MEM(1)
        -------------------------------
        11 :    id_alu_port_a_select    Select: Rs(0), shamt(1), 0x04(2), 0x10(3)
        10 :    .
        9  :    id_alu_port_b_select    Select: Rt(0), SImm16(1), PCAdd4(2), ZImm16(3)
        8  :    .
        7  :    id_gpr_wa_select        Select register: Rd(0), Rt(1), 31(2)
        6  :    .
        -------------------------------
        5  :    id_jump                 Jump instruction
        4  :    id_branch               Branch instruction
        -------------------------------
        3  :    id_mem_write            Write to data memory
        2  :    id_mem_byte             Enable read/write one byte
        1  :    id_mem_halfword         Enable read/write 2 bytes (16 bits data)
        0  :    id_mem_data_sign_ext    Zero extend data (0) or Sign extend data (1)
    ----------------------------------------------------------------------------
    */
    assign dp_hazard                = datapath[31:24];
    assign id_id_exception_source   = datapath[23];
    assign id_ex_exception_source   = datapath[22];
    assign id_mem_exception_source  = datapath[21];
    assign id_alu_operation         = datapath[20:16];
    assign id_trap                  = datapath[15];
    assign id_trap_condition        = datapath[14];
    assign id_gpr_we                = datapath[13];
    assign id_mem_to_gpr_select     = datapath[12];
    assign id_alu_port_a_select     = datapath[11:10];
    assign id_alu_port_b_select     = datapath[9:8];
    assign id_gpr_wa_select         = datapath[7:6];
    assign id_jump                  = datapath[5];
    assign id_branch                = datapath[4];
    assign id_mem_write             = datapath[3];
    assign id_mem_byte              = datapath[2];
    assign id_mem_halfword          = datapath[1];
    assign id_mem_data_sign_ext     = datapath[0];

    //--------------------------------------------------------------------------
    // set the control signals
    //--------------------------------------------------------------------------
    always @(*) begin
        case(opcode)
            `OP_TYPE_R      :   begin
                                    case (op_function)
                                        `FUNCTION_OP_ADD     : begin datapath = `DP_ADD;     end
                                        `FUNCTION_OP_ADDU    : begin datapath = `DP_ADDU;    end
                                        `FUNCTION_OP_AND     : begin datapath = `DP_AND;     end
                                        `FUNCTION_OP_BREAK   : begin datapath = `DP_BREAK;   end
                                        `FUNCTION_OP_DIV     : begin datapath = `DP_DIV;     end
                                        `FUNCTION_OP_DIVU    : begin datapath = `DP_DIVU;    end
                                        `FUNCTION_OP_JALR    : begin datapath = `DP_JALR;    end
                                        `FUNCTION_OP_JR      : begin datapath = `DP_JR;      end
                                        `FUNCTION_OP_MFHI    : begin datapath = `DP_MFHI;    end
                                        `FUNCTION_OP_MFLO    : begin datapath = `DP_MFLO;    end
                                        `FUNCTION_OP_MOVN    : begin datapath = `DP_MOVN;    end
                                        `FUNCTION_OP_MOVZ    : begin datapath = `DP_MOVZ;    end
                                        `FUNCTION_OP_MTHI    : begin datapath = `DP_MTHI;    end
                                        `FUNCTION_OP_MTLO    : begin datapath = `DP_MTLO;    end
                                        `FUNCTION_OP_MULT    : begin datapath = `DP_MULT;    end
                                        `FUNCTION_OP_MULTU   : begin datapath = `DP_MULTU;   end
                                        `FUNCTION_OP_NOR     : begin datapath = `DP_NOR;     end
                                        `FUNCTION_OP_OR      : begin datapath = `DP_OR;      end
                                        `FUNCTION_OP_SLL     : begin datapath = `DP_SLL;     end
                                        `FUNCTION_OP_SLLV    : begin datapath = `DP_SLLV;    end
                                        `FUNCTION_OP_SLT     : begin datapath = `DP_SLT;     end
                                        `FUNCTION_OP_SLTU    : begin datapath = `DP_SLTU;    end
                                        `FUNCTION_OP_SRA     : begin datapath = `DP_SRA;     end
                                        `FUNCTION_OP_SRAV    : begin datapath = `DP_SRAV;    end
                                        `FUNCTION_OP_SRL     : begin datapath = `DP_SRL;     end
                                        `FUNCTION_OP_SRLV    : begin datapath = `DP_SRLV;    end
                                        `FUNCTION_OP_SUB     : begin datapath = `DP_SUB;     end
                                        `FUNCTION_OP_SUBU    : begin datapath = `DP_SUBU;    end
                                        `FUNCTION_OP_SYSCALL : begin datapath = `DP_SYSCALL; end
                                        `FUNCTION_OP_TEQ     : begin datapath = `DP_TEQ;     end
                                        `FUNCTION_OP_TGE     : begin datapath = `DP_TGE;     end
                                        `FUNCTION_OP_TGEU    : begin datapath = `DP_TGEU;    end
                                        `FUNCTION_OP_TLT     : begin datapath = `DP_TLT;     end
                                        `FUNCTION_OP_TLTU    : begin datapath = `DP_TLTU;    end
                                        `FUNCTION_OP_TNE     : begin datapath = `DP_TNE;     end
                                        `FUNCTION_OP_XOR     : begin datapath = `DP_XOR;     end
                                        default              : begin datapath = `DP_NONE;    end
                                    endcase // case (op_function)
            end // case: `OP_TYPE_R
            `OP_TYPE_R2     :   begin
                                    case (op_function)
                                        `FUNCTION_OP_CLO   : begin datapath = `DP_CLO;   end
                                        `FUNCTION_OP_CLZ   : begin datapath = `DP_CLZ;   end
                                        `FUNCTION_OP_MADD  : begin datapath = `DP_MADD;  end
                                        `FUNCTION_OP_MADDU : begin datapath = `DP_MADDU; end
                                        `FUNCTION_OP_MSUB  : begin datapath = `DP_MSUB;  end
                                        `FUNCTION_OP_MSUBU : begin datapath = `DP_MSUBU; end
                                        default            : begin datapath = `DP_NONE;  end
                                    endcase // case (op_function)
            end // case: `OP_TYPE_R2
            `OP_TYPE_REGIMM :   begin
                                    case (op_rt)
                                        `RT_OP_BGEZ   : begin datapath = `DP_BGEZ;   end
                                        `RT_OP_BGEZAL : begin datapath = `DP_BGEZAL; end
                                        `RT_OP_BLTZ   : begin datapath = `DP_BLTZ;   end
                                        `RT_OP_BLTZAL : begin datapath = `DP_BLTZAL; end
                                        `RT_OP_TEQI   : begin datapath = `DP_TEQI;   end
                                        `RT_OP_TGEI   : begin datapath = `DP_TGEI;   end
                                        `RT_OP_TGEIU  : begin datapath = `DP_TGEIU;  end
                                        `RT_OP_TLTI   : begin datapath = `DP_TLTI;   end
                                        `RT_OP_TLTIU  : begin datapath = `DP_TLTIU;  end
                                        `RT_OP_TNEI   : begin datapath = `DP_TNEI;   end
                                        default       : begin datapath = `DP_NONE;   end
                                    endcase // case (op_rt)
            end // case: `OP_TYPE_REGIMM
            `OP_TYPE_CP0    :   begin
                                    case (op_rs)
                                        `RS_OP_MFC  : begin datapath = `DP_MFC0; end
                                        `RS_OP_MTC  : begin datapath = `DP_MTC0; end
                                        `RS_OP_ERET : begin datapath = `DP_ERET; end
                                        default     : begin datapath = `DP_NONE; end
                                    endcase // case (op_rs)
            end
            `OP_ADDI        :   begin datapath = `DP_ADDI;  end
            `OP_ADDIU       :   begin datapath = `DP_ADDIU; end
            `OP_ANDI        :   begin datapath = `DP_ANDI;  end
            `OP_BEQ         :   begin datapath = `DP_BEQ;   end
            `OP_BGTZ        :   begin datapath = `DP_BGTZ;  end
            `OP_BLEZ        :   begin datapath = `DP_BLEZ;  end
            `OP_BNE         :   begin datapath = `DP_BNE;   end
            `OP_J           :   begin datapath = `DP_J;     end
            `OP_JAL         :   begin datapath = `DP_JAL;   end
            `OP_LB          :   begin datapath = `DP_LB;    end
            `OP_LBU         :   begin datapath = `DP_LBU;   end
            `OP_LH          :   begin datapath = `DP_LH;    end
            `OP_LHU         :   begin datapath = `DP_LHU;   end
            `OP_LL          :   begin datapath = `DP_LL;    end
            `OP_LUI         :   begin datapath = `DP_LUI;   end
            `OP_LW          :   begin datapath = `DP_LW;    end
            `OP_ORI         :   begin datapath = `DP_ORI;   end
            `OP_SB          :   begin datapath = `DP_SB;    end
            `OP_SC          :   begin datapath = `DP_SC;    end
            `OP_SH          :   begin datapath = `DP_SH;    end
            `OP_SLTI        :   begin datapath = `DP_SLTI;  end
            `OP_SLTIU       :   begin datapath = `DP_SLTIU; end
            `OP_SW          :   begin datapath = `DP_SW;    end
            `OP_XORI        :   begin datapath = `DP_XORI;  end
            default         :   begin datapath = `DP_NONE;  end
        endcase // case (opcode)
    end // always @ (*)
endmodule // antares_control_unit
//==================================================================================================
//  Filename      : antares_core.v
//  Created On    : Sat Sep  5 21:45:33 2015
//  Last Modified : Sat Nov 07 11:56:09 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Antares core.
//==================================================================================================

module antares_core #(parameter ENABLE_HW_MULT = 1,
                      parameter ENABLE_HW_DIV = 1,
                      parameter ENABLE_HW_CLOZ = 1
                      )(
                        input wire         clk,
                        input wire         rst,
                        output wire        halted, // CP0 Status Register, bit 16
                        // Interrupts
                        input wire [4:0]   interrupts, // External interrupts
                        input wire         nmi, // Non-maskable interrupt
                        // External Instruction Memory/Instruction Cache
                        input wire [31:0]  iport_data_i, // Data from memory
                        input wire         iport_ready, // memory is ready
                        input wire         iport_error, // Bus error
                        output wire [31:0] iport_address, // data address
                        output wire [3:0]  iport_wr, // write = byte select, read = 0000,
                        output wire        iport_enable, // enable operation
                        // External Data Memory/Data Cache
                        input wire         dport_ready, // memory is ready
                        input wire         dport_error, // Bus error
                        output wire [31:0] dport_address, // data address
                        inout wire [31:0]  dport_data,
                        output wire [3:0]  dport_wr, // write = byte select, read = 0000,
                        output wire        dport_enable   // enable operation
                        );

    wire [31:0]  dport_data_i; // Data from memory
    wire [31:0] dport_data_o; // data to memory

    assign dport_data    = ((dport_wr[3] | dport_wr[2] | dport_wr[1] | dport_wr[0]) && dport_enable) ? dport_data_o : 32'bZ;
    assign dport_data_i  = (!(dport_wr[3] | dport_wr[2] | dport_wr[1] | dport_wr[0]) && dport_enable) ? dport_data : 32'b0;

    /*AUTOWIRE*/
    // Beginning of automatic wires (for undeclared instantiated-module outputs)
    wire [31:0]         cp0_data_output;        // From cpzero0 of antares_cpzero.v
    wire                dmem_request_stall;     // From load_store_unit0 of antares_load_store_unit.v
    wire [7:0]          dp_hazard;              // From control_unit0 of antares_control_unit.v
    wire [4:0]          ex_alu_operation;       // From IDEX_register of antares_idex_register.v
    wire [1:0]          ex_alu_port_a_select;   // From IDEX_register of antares_idex_register.v
    wire [1:0]          ex_alu_port_b_select;   // From IDEX_register of antares_idex_register.v
    wire [31:0]         ex_data_rs;             // From IDEX_register of antares_idex_register.v
    wire [31:0]         ex_data_rt;             // From IDEX_register of antares_idex_register.v
    wire [3:0]          ex_dp_hazard;           // From IDEX_register of antares_idex_register.v
    wire                ex_flush;               // From cpzero0 of antares_cpzero.v
    wire [1:0]          ex_gpr_wa_select;       // From IDEX_register of antares_idex_register.v
    wire                ex_gpr_we;              // From IDEX_register of antares_idex_register.v
    wire                ex_mem_byte;            // From IDEX_register of antares_idex_register.v
    wire                ex_mem_data_sign_ext;   // From IDEX_register of antares_idex_register.v
    wire                ex_mem_halfword;        // From IDEX_register of antares_idex_register.v
    wire                ex_mem_to_gpr_select;   // From IDEX_register of antares_idex_register.v
    wire                ex_mem_write;           // From IDEX_register of antares_idex_register.v
    wire [4:0]          ex_rs;                  // From IDEX_register of antares_idex_register.v
    wire [4:0]          ex_rt;                  // From IDEX_register of antares_idex_register.v
    wire [16:0]         ex_sign_imm16;          // From IDEX_register of antares_idex_register.v
    wire                ex_stall;               // From hazard_unit0 of antares_hazard_unit.v
    wire                exc_address_if;         // From load_store_unit0 of antares_load_store_unit.v
    wire                exc_address_l_mem;      // From load_store_unit0 of antares_load_store_unit.v
    wire                exc_address_s_mem;      // From load_store_unit0 of antares_load_store_unit.v
    wire                exc_syscall;            // From control_unit0 of antares_control_unit.v
    wire [1:0]          forward_ex_rs;          // From hazard_unit0 of antares_hazard_unit.v
    wire [1:0]          forward_ex_rt;          // From hazard_unit0 of antares_hazard_unit.v
    wire [1:0]          forward_id_rs;          // From hazard_unit0 of antares_hazard_unit.v
    wire [1:0]          forward_id_rt;          // From hazard_unit0 of antares_hazard_unit.v
    wire [4:0]          id_alu_operation;       // From control_unit0 of antares_control_unit.v
    wire [1:0]          id_alu_port_a_select;   // From control_unit0 of antares_control_unit.v
    wire [1:0]          id_alu_port_b_select;   // From control_unit0 of antares_control_unit.v
    wire                id_branch;              // From control_unit0 of antares_control_unit.v
    wire                id_flush;               // From cpzero0 of antares_cpzero.v
    wire [1:0]          id_gpr_wa_select;       // From control_unit0 of antares_control_unit.v
    wire                id_gpr_we;              // From control_unit0 of antares_control_unit.v
    wire [31:0]         id_instruction;         // From IFID_register of antares_ifid_register.v
    wire                id_jump;                // From control_unit0 of antares_control_unit.v
    wire                id_mem_byte;            // From control_unit0 of antares_control_unit.v
    wire                id_mem_data_sign_ext;   // From control_unit0 of antares_control_unit.v
    wire                id_mem_halfword;        // From control_unit0 of antares_control_unit.v
    wire                id_mem_to_gpr_select;   // From control_unit0 of antares_control_unit.v
    wire                id_mem_write;           // From control_unit0 of antares_control_unit.v
    wire [31:0]         id_pc_add4;             // From IFID_register of antares_ifid_register.v
    wire                id_stall;               // From hazard_unit0 of antares_hazard_unit.v
    wire                id_take_branch;         // From branch_unit0 of antares_branch_unit.v
    wire                if_flush;               // From cpzero0 of antares_cpzero.v
    wire [31:0]         if_new_pc;              // From pc_source_exception of antares_mux_2_1.v
    wire [31:0]         if_pc;                  // From pc_register of antares_pc_register.v
    wire [31:0]         if_pc_add4;             // From pc_add4 of antares_add.v
    wire                if_stall;               // From hazard_unit0 of antares_hazard_unit.v
    wire                imem_request_stall;     // From load_store_unit0 of antares_load_store_unit.v
    wire [31:0]         mem_alu_result;         // From EXMEM_register of antares_exmem_register.v
    wire                mem_flush;              // From cpzero0 of antares_cpzero.v
    wire [4:0]          mem_gpr_wa;             // From EXMEM_register of antares_exmem_register.v
    wire                mem_gpr_we;             // From EXMEM_register of antares_exmem_register.v
    wire                mem_mem_byte;           // From EXMEM_register of antares_exmem_register.v
    wire                mem_mem_data_sign_ext;  // From EXMEM_register of antares_exmem_register.v
    wire                mem_mem_halfword;       // From EXMEM_register of antares_exmem_register.v
    wire [31:0]         mem_mem_store_data;     // From EXMEM_register of antares_exmem_register.v
    wire                mem_mem_to_gpr_select;  // From EXMEM_register of antares_exmem_register.v
    wire                mem_mem_write;          // From EXMEM_register of antares_exmem_register.v
    wire                mem_stall;              // From hazard_unit0 of antares_hazard_unit.v
    wire [31:0]         pc_branch_address;      // From branch_unit0 of antares_branch_unit.v
    wire [31:0]         wb_alu_data;            // From MEMWB_register of antares_memwb_register.v
    wire [4:0]          wb_gpr_wa;              // From MEMWB_register of antares_memwb_register.v
    wire                wb_gpr_we;              // From MEMWB_register of antares_memwb_register.v
    wire                wb_mem_to_gpr_select;   // From MEMWB_register of antares_memwb_register.v
    wire [31:0]         wb_read_data;           // From MEMWB_register of antares_memwb_register.v
    wire                wb_stall;               // From hazard_unit0 of antares_hazard_unit.v
    // End of automatics

    // manual wires
    wire [5:0]          opcode;
    wire [4:0]          op_rs;
    wire [4:0]          op_rt;
    wire [4:0]          op_rd;
    wire [5:0]          op_function;
    wire [15:0]         op_imm16;
    wire [25:0]         op_imm26;
    wire [2:0]          op_cp0_select;

    wire    [31:0]  if_instruction;
    wire    [31:0]  id_gpr_rs;
    wire    [31:0]  id_gpr_rt;
    wire    [31:0]  wb_gpr_wd;
    wire    [31:0]  id_forward_rs;
    wire    [31:0]  id_forward_rt;
    wire    [31:0]  ex_forward_rs;
    wire    [31:0]  ex_forward_rt;
    wire    [31:0]  ex_alu_result;
    wire            ex_request_stall;
    wire    [31:0]  ex_alu_port_a;
    wire    [31:0]  ex_alu_port_b;
    wire    [4:0]   ex_gpr_wa;
    wire    [31:0]  mem_read_data;

    wire            halt_0;
    reg             halt_1;
    reg             halt_2;
    reg             halt_3;

    wire            id_mfc0;
    wire            id_mtc0;
    wire            id_eret;
    wire            id_cp1_instruction;
    wire            id_cp2_instruction;
    wire            id_cp3_instruction;
    wire            exc_overflow;
    wire            exc_trap;
    wire            exc_breakpoint;
    wire            exc_reserved;
    wire    [31:0]  id_exception_pc;
    wire    [31:0]  ex_exception_pc;
    wire    [31:0]  mem_exception_pc;
    wire            id_exception_source;
    wire            ex_exception_source;
    wire            mem_exception_source;
    wire            id_is_flushed;
    wire            if_is_bds;
    wire            id_is_bds;
    wire            ex_is_bds;
    wire            mem_is_bds;
    wire            id_kernel_mode;
    wire            ex_kernel_mode;
    wire            mem_kernel_mode;
    wire            if_exception_stall;
    wire            id_exception_stall;
    wire            ex_exception_stall;
    wire            mem_exception_stall;
    wire            exception_pc_select;
    wire    [31:0]  pc_exception;
    wire    [31:0]  pc_pre_exc_selection;
    wire            id_llsc;
    wire            ex_llsc;
    wire            mem_llsc;
    wire            id_movn;
    wire            id_movz;
    wire            ex_movn;
    wire            ex_movz;
    wire            ex_b_is_zero;
    wire            id_trap;
    wire            ex_trap;
    wire            id_trap_condition;
    wire            ex_trap_condition;
    wire            mem_trap;
    wire            mem_trap_condition;
    wire            id_id_exception_source;
    wire            id_ex_exception_source;
    wire            id_mem_exception_source;
    wire            ex_ex_exception_source;
    wire            ex_mem_exception_source;
    wire            mem_mem_exception_source;
    wire            id_imm_sign_ext;
    wire    [31:0]  ex_cp0_data;

    wire            exception_ready;
    wire            pc_source_select;
    wire            if_stall_pc_register;
    wire [7:0]      haz_dp_hazards;

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign opcode                = id_instruction[`ANTARES_INSTR_OPCODE];
    assign op_rs                 = id_instruction[`ANTARES_INSTR_RS];
    assign op_rt                 = id_instruction[`ANTARES_INSTR_RT];
    assign op_rd                 = id_instruction[`ANTARES_INSTR_RD];
    assign op_function           = id_instruction[`ANTARES_INSTR_FUNCT];
    assign op_imm16              = id_instruction[`ANTARES_INSTR_IMM16];
    assign op_imm26              = id_instruction[`ANTARES_INSTR_IMM26];
    assign op_cp0_select         = id_instruction[`ANTARES_INSTR_CP0_SEL];

    assign id_exception_source   = id_id_exception_source | id_ex_exception_source | id_mem_exception_source;
    assign ex_exception_source   = ex_ex_exception_source | ex_mem_exception_source;
    assign mem_exception_source  = mem_mem_exception_source;

    assign if_is_bds             = id_take_branch;
    assign exc_trap              = mem_trap & (mem_trap_condition ^ (mem_alu_result == 32'b0));
    assign pc_source_select      = (id_take_branch & id_branch) | id_jump;
    assign if_stall_pc_register  = if_stall | id_stall | halt_0;

    assign haz_dp_hazards = {dp_hazard[7:4], ex_dp_hazard};

    //------------------------------------------------------------------------------------------------------------------
    // UPDATE: Getting the halt signal from the CP0.
    always @(posedge clk) begin
        if (rst) begin
            halt_1 <= 1'b0;
            halt_2 <= 1'b0;
            halt_3 <= 1'b0;
        end
        else begin
            halt_1 <= halt_0;
            halt_2 <= halt_1;
            halt_3 <= halt_2;
        end
    end // always @ (posedge clk)
    assign halted  = halt_3;

    //--------------------------------------------------------------------------
    // IF stage (A)
    //--------------------------------------------------------------------------
    antares_mux_2_1 pc_source(// Outputs
                              .out    (pc_pre_exc_selection[31:0]),
                              // Inputs
                              .in0    (if_pc_add4[31:0]),
                              .in1    (pc_branch_address[31:0]),
                              .select (pc_source_select)
                              /*AUTOINST*/);

    antares_mux_2_1 pc_source_exception (// Outputs
                                         .out    (if_new_pc[31:0]),
                                         // Inputs
                                         .in0    (pc_pre_exc_selection[31:0]),
                                         .in1    (pc_exception[31:0]),
                                         .select (exception_pc_select)
                                         /*AUTOINST*/);

    antares_pc_register pc_register (// Inputs
                                     .if_stall          (if_stall_pc_register),
                                     /*AUTOINST*/
                                     // Outputs
                                     .if_pc             (if_pc[31:0]),
                                     // Inputs
                                     .clk               (clk),
                                     .rst               (rst),
                                     .if_new_pc         (if_new_pc[31:0]));
    //--------------------------------------------------------------------------
    // IF stage (B)
    //--------------------------------------------------------------------------
    antares_add pc_add4 (// Outputs
                         .c (if_pc_add4[31:0]),
                         // Inputs
                         .a (if_pc[31:0]),
                         .b (32'd4)
                         /*AUTOINST*/);

    antares_ifid_register IFID_register (// Inputs
                                         .if_exception_pc       (if_pc[31:0]),
                                         /*AUTOINST*/
                                         // Outputs
                                         .id_instruction        (id_instruction[31:0]),
                                         .id_pc_add4            (id_pc_add4[31:0]),
                                         .id_exception_pc       (id_exception_pc[31:0]),
                                         .id_is_bds             (id_is_bds),
                                         .id_is_flushed         (id_is_flushed),
                                         // Inputs
                                         .clk                   (clk),
                                         .rst                   (rst),
                                         .if_instruction        (if_instruction[31:0]),
                                         .if_pc_add4            (if_pc_add4[31:0]),
                                         .if_is_bds             (if_is_bds),
                                         .if_flush              (if_flush),
                                         .if_stall              (if_stall),
                                         .id_stall              (id_stall));
    //--------------------------------------------------------------------------
    // ID stage
    //--------------------------------------------------------------------------
    antares_reg_file GPR (// Outputs
                          .gpr_rd_a   (id_gpr_rs[31:0]),
                          .gpr_rd_b   (id_gpr_rt[31:0]),
                          // Inputs
                          .clk        (clk),
                          .gpr_ra_a   (op_rs[4:0]),
                          .gpr_ra_b   (op_rt[4:0]),
                          .gpr_wa     (wb_gpr_wa[4:0]),
                          .gpr_wd     (wb_gpr_wd[31:0]),
                          .gpr_we     (wb_gpr_we)
                          /*AUTOINST*/);

    antares_branch_unit branch_unit0 (// Inputs
                                      .id_data_rs       (id_forward_rs[31:0]),
                                      .id_data_rt       (id_forward_rt[31:0]),
                                      /*AUTOINST*/
                                      // Outputs
                                      .pc_branch_address(pc_branch_address[31:0]),
                                      .id_take_branch   (id_take_branch),
                                      // Inputs
                                      .opcode           (opcode[5:0]),
                                      .id_pc_add4       (id_pc_add4[31:0]),
                                      .op_imm26         (op_imm26[25:0]));

    antares_control_unit #(/*AUTOINSTPARAM*/
                           // Parameters
                           .ENABLE_HW_MULT      (ENABLE_HW_MULT),
                           .ENABLE_HW_DIV       (ENABLE_HW_DIV),
                           .ENABLE_HW_CLOZ      (ENABLE_HW_CLOZ))
                         control_unit0 (// Outputs
                                        .id_syscall              (exc_syscall),
                                        .id_breakpoint           (exc_breakpoint),
                                        .id_reserved             (exc_reserved),
                                        /*AUTOINST*/
                                        // Outputs
                                        .dp_hazard               (dp_hazard[7:0]),
                                        .id_imm_sign_ext         (id_imm_sign_ext),
                                        .id_movn                 (id_movn),
                                        .id_movz                 (id_movz),
                                        .id_llsc                 (id_llsc),
                                        .id_mfc0                 (id_mfc0),
                                        .id_mtc0                 (id_mtc0),
                                        .id_eret                 (id_eret),
                                        .id_cp1_instruction      (id_cp1_instruction),
                                        .id_cp2_instruction      (id_cp2_instruction),
                                        .id_cp3_instruction      (id_cp3_instruction),
                                        .id_id_exception_source  (id_id_exception_source),
                                        .id_ex_exception_source  (id_ex_exception_source),
                                        .id_mem_exception_source (id_mem_exception_source),
                                        .id_trap                 (id_trap),
                                        .id_trap_condition       (id_trap_condition),
                                        .id_gpr_we               (id_gpr_we),
                                        .id_mem_to_gpr_select    (id_mem_to_gpr_select),
                                        .id_alu_operation        (id_alu_operation[4:0]),
                                        .id_alu_port_a_select    (id_alu_port_a_select[1:0]),
                                        .id_alu_port_b_select    (id_alu_port_b_select[1:0]),
                                        .id_gpr_wa_select        (id_gpr_wa_select[1:0]),
                                        .id_jump                 (id_jump),
                                        .id_branch               (id_branch),
                                        .id_mem_write            (id_mem_write),
                                        .id_mem_byte             (id_mem_byte),
                                        .id_mem_halfword         (id_mem_halfword),
                                        .id_mem_data_sign_ext    (id_mem_data_sign_ext),
                                        // Inputs
                                        .opcode                  (opcode[5:0]),
                                        .op_function             (op_function[5:0]),
                                        .op_rs                   (op_rs[4:0]),
                                        .op_rt                   (op_rt[4:0]));

    antares_mux_4_1 ForwardRsID (// Outputs
                                   .out    (id_forward_rs[31:0]),
                                   // Inputs
                                   .in0    (id_gpr_rs[31:0]),
                                   .in1    (mem_alu_result[31:0]),
                                   .in2    (wb_gpr_wd[31:0]),
                                   .in3    (32'bx),
                                   .select (forward_id_rs[1:0])
                                   /*AUTOINST*/);

    antares_mux_4_1 ForwardRtID (// Outputs
                                   .out    (id_forward_rt[31:0]),
                                   // Inputs
                                   .in0    (id_gpr_rt[31:0]),
                                   .in1    (mem_alu_result[31:0]),
                                   .in2    (wb_gpr_wd[31:0]),
                                   .in3    (32'bx),
                                   .select (forward_id_rt[1:0])
                                   /*AUTOINST*/);

    antares_idex_register IDEX_register (// Inputs
                                         .id_data_rs              (id_forward_rs[31:0]),
                                         .id_data_rt              (id_forward_rt[31:0]),
                                         .id_sign_imm16           (op_imm16[15:0]),
                                         .id_cp0_data             (cp0_data_output[31:0]),
                                         .id_rs                   (op_rs[4:0]),
                                         .id_rt                   (op_rt[4:0]),
                                         .id_dp_hazard            (dp_hazard[3:0]),
                                         /*AUTOINST*/
                                         // Outputs
                                         .ex_alu_operation        (ex_alu_operation[4:0]),
                                         .ex_data_rs              (ex_data_rs[31:0]),
                                         .ex_data_rt              (ex_data_rt[31:0]),
                                         .ex_gpr_we               (ex_gpr_we),
                                         .ex_mem_to_gpr_select    (ex_mem_to_gpr_select),
                                         .ex_mem_write            (ex_mem_write),
                                         .ex_alu_port_a_select    (ex_alu_port_a_select[1:0]),
                                         .ex_alu_port_b_select    (ex_alu_port_b_select[1:0]),
                                         .ex_gpr_wa_select        (ex_gpr_wa_select[1:0]),
                                         .ex_mem_byte             (ex_mem_byte),
                                         .ex_mem_halfword         (ex_mem_halfword),
                                         .ex_mem_data_sign_ext    (ex_mem_data_sign_ext),
                                         .ex_rs                   (ex_rs[4:0]),
                                         .ex_rt                   (ex_rt[4:0]),
                                         .ex_dp_hazard            (ex_dp_hazard[3:0]),
                                         .ex_sign_imm16           (ex_sign_imm16[16:0]),
                                         .ex_cp0_data             (ex_cp0_data[31:0]),
                                         .ex_exception_pc         (ex_exception_pc[31:0]),
                                         .ex_movn                 (ex_movn),
                                         .ex_movz                 (ex_movz),
                                         .ex_llsc                 (ex_llsc),
                                         .ex_kernel_mode          (ex_kernel_mode),
                                         .ex_is_bds               (ex_is_bds),
                                         .ex_trap                 (ex_trap),
                                         .ex_trap_condition       (ex_trap_condition),
                                         .ex_ex_exception_source  (ex_ex_exception_source),
                                         .ex_mem_exception_source (ex_mem_exception_source),
                                         // Inputs
                                         .clk                     (clk),
                                         .rst                     (rst),
                                         .id_alu_operation        (id_alu_operation[4:0]),
                                         .id_gpr_we               (id_gpr_we),
                                         .id_mem_to_gpr_select    (id_mem_to_gpr_select),
                                         .id_mem_write            (id_mem_write),
                                         .id_alu_port_a_select    (id_alu_port_a_select[1:0]),
                                         .id_alu_port_b_select    (id_alu_port_b_select[1:0]),
                                         .id_gpr_wa_select        (id_gpr_wa_select[1:0]),
                                         .id_mem_byte             (id_mem_byte),
                                         .id_mem_halfword         (id_mem_halfword),
                                         .id_mem_data_sign_ext    (id_mem_data_sign_ext),
                                         .id_imm_sign_ext         (id_imm_sign_ext),
                                         .id_exception_pc         (id_exception_pc[31:0]),
                                         .id_movn                 (id_movn),
                                         .id_movz                 (id_movz),
                                         .id_llsc                 (id_llsc),
                                         .id_kernel_mode          (id_kernel_mode),
                                         .id_is_bds               (id_is_bds),
                                         .id_trap                 (id_trap),
                                         .id_trap_condition       (id_trap_condition),
                                         .id_ex_exception_source  (id_ex_exception_source),
                                         .id_mem_exception_source (id_mem_exception_source),
                                         .id_flush                (id_flush),
                                         .id_stall                (id_stall),
                                         .ex_stall                (ex_stall));
    //--------------------------------------------------------------------------
    // EX stage
    //--------------------------------------------------------------------------
    antares_alu #(/*AUTOINSTPARAM*/
                  // Parameters
                  .ENABLE_HW_MULT       (ENABLE_HW_MULT),
                  .ENABLE_HW_DIV        (ENABLE_HW_DIV),
                  .ENABLE_HW_CLOZ       (ENABLE_HW_CLOZ))
                execution_unit (/*AUTOINST*/
                                // Outputs
                                .ex_request_stall (ex_request_stall),
                                .ex_alu_result    (ex_alu_result[31:0]),
                                .ex_b_is_zero     (ex_b_is_zero),
                                .exc_overflow     (exc_overflow),
                                // Inputs
                                .clk              (clk),
                                .rst              (rst),
                                .ex_alu_port_a    (ex_alu_port_a[31:0]),
                                .ex_alu_port_b    (ex_alu_port_b[31:0]),
                                .ex_alu_operation (ex_alu_operation[4:0]),
                                .ex_stall         (ex_stall),
                                .ex_flush         (ex_flush));

    antares_mux_4_1 forward_rs_ex (// Outputs
                                   .out    (ex_forward_rs[31:0]),
                                   // Inputs
                                   .in0    (ex_data_rs[31:0]),
                                   .in1    (mem_alu_result[31:0]),
                                   .in2    (wb_gpr_wd[31:0]),
                                   .in3    (32'bx),
                                   .select (forward_ex_rs[1:0])
                                   /*AUTOINST*/);

    antares_mux_4_1 forward_rt_ex (// Outputs
                                   .out    (ex_forward_rt[31:0]),
                                   // Inputs
                                   .in0    (ex_data_rt[31:0]),
                                   .in1    (mem_alu_result[31:0]),
                                   .in2    (wb_gpr_wd[31:0]),
                                   .in3    (32'bx),
                                   .select (forward_ex_rt[1:0])
                                   /*AUTOINST*/);

    antares_mux_4_1 ALUPortA (// Outputs
                              .out    (ex_alu_port_a[31:0]),
                              // Inputs
                              .in0    (ex_forward_rs[31:0]),
                              .in1    ({27'b0, ex_sign_imm16[10:6]}), // shamnt
                              .in2    (32'd8), // PC + 8
                              .in3    (32'd16),
                              .select (ex_alu_port_a_select[1:0])
                              /*AUTOINST*/);

    antares_mux_4_1 ALUPortB (// Outputs
                              .out    (ex_alu_port_b[31:0]),
                              // Inputs
                              .in0    (ex_forward_rt[31:0]),
                              .in1    ({{15{ex_sign_imm16[16]}}, ex_sign_imm16[16:0]}),
                              .in2    (ex_exception_pc[31:0]),
                              .in3    (ex_cp0_data[31:0]),
                              .select (ex_alu_port_b_select[1:0])
                              /*AUTOINST*/);

    antares_mux_4_1 #(.WIDTH(5))
        mux_reg_wa(// Outputs
                   .out    (ex_gpr_wa[4:0]),
                   // Inputs
                   .in0    (ex_sign_imm16[15:11]), // Rd
                   .in1    (ex_rt[4:0]),
                   .in2    (5'b11111), // $31 = $Ra
                   .in3    (5'b00000), // NOP
                   .select (ex_gpr_wa_select[1:0])
                   /*AUTOINST*/);

    antares_exmem_register EXMEM_register (// Inputs
                                           .ex_mem_store_data        (ex_forward_rt[31:0]),
                                           /*AUTOINST*/
                                           // Outputs
                                           .mem_alu_result           (mem_alu_result[31:0]),
                                           .mem_mem_store_data       (mem_mem_store_data[31:0]),
                                           .mem_gpr_wa               (mem_gpr_wa[4:0]),
                                           .mem_gpr_we               (mem_gpr_we),
                                           .mem_mem_to_gpr_select    (mem_mem_to_gpr_select),
                                           .mem_mem_write            (mem_mem_write),
                                           .mem_mem_byte             (mem_mem_byte),
                                           .mem_mem_halfword         (mem_mem_halfword),
                                           .mem_mem_data_sign_ext    (mem_mem_data_sign_ext),
                                           .mem_exception_pc         (mem_exception_pc[31:0]),
                                           .mem_llsc                 (mem_llsc),
                                           .mem_kernel_mode          (mem_kernel_mode),
                                           .mem_is_bds               (mem_is_bds),
                                           .mem_trap                 (mem_trap),
                                           .mem_trap_condition       (mem_trap_condition),
                                           .mem_mem_exception_source (mem_mem_exception_source),
                                           // Inputs
                                           .clk                      (clk),
                                           .rst                      (rst),
                                           .ex_alu_result            (ex_alu_result[31:0]),
                                           .ex_gpr_wa                (ex_gpr_wa[4:0]),
                                           .ex_gpr_we                (ex_gpr_we),
                                           .ex_mem_to_gpr_select     (ex_mem_to_gpr_select),
                                           .ex_mem_write             (ex_mem_write),
                                           .ex_mem_byte              (ex_mem_byte),
                                           .ex_mem_halfword          (ex_mem_halfword),
                                           .ex_mem_data_sign_ext     (ex_mem_data_sign_ext),
                                           .ex_exception_pc          (ex_exception_pc[31:0]),
                                           .ex_movn                  (ex_movn),
                                           .ex_movz                  (ex_movz),
                                           .ex_b_is_zero             (ex_b_is_zero),
                                           .ex_llsc                  (ex_llsc),
                                           .ex_kernel_mode           (ex_kernel_mode),
                                           .ex_is_bds                (ex_is_bds),
                                           .ex_trap                  (ex_trap),
                                           .ex_trap_condition        (ex_trap_condition),
                                           .ex_mem_exception_source  (ex_mem_exception_source),
                                           .ex_flush                 (ex_flush),
                                           .ex_stall                 (ex_stall),
                                           .mem_stall                (mem_stall));
    //--------------------------------------------------------------------------
    // MEM stage
    //--------------------------------------------------------------------------
    antares_memwb_register MEMWB_register (// Inputs
                                           .mem_alu_data          (mem_alu_result[31:0]),
                                           /*AUTOINST*/
                                           // Outputs
                                           .wb_read_data          (wb_read_data[31:0]),
                                           .wb_alu_data           (wb_alu_data[31:0]),
                                           .wb_gpr_wa             (wb_gpr_wa[4:0]),
                                           .wb_mem_to_gpr_select  (wb_mem_to_gpr_select),
                                           .wb_gpr_we             (wb_gpr_we),
                                           // Inputs
                                           .clk                   (clk),
                                           .rst                   (rst),
                                           .mem_read_data         (mem_read_data[31:0]),
                                           .mem_gpr_wa            (mem_gpr_wa[4:0]),
                                           .mem_mem_to_gpr_select (mem_mem_to_gpr_select),
                                           .mem_gpr_we            (mem_gpr_we),
                                           .mem_flush             (mem_flush),
                                           .mem_stall             (mem_stall),
                                           .wb_stall              (wb_stall));
    //--------------------------------------------------------------------------
    // WB stage
    //--------------------------------------------------------------------------
    antares_mux_2_1 mux_mem_ex_result (// Outputs
                                       .out    (wb_gpr_wd[31:0]),
                                       // Inputs
                                       .in0    (wb_alu_data[31:0]),
                                       .in1    (wb_read_data[31:0]),
                                       .select (wb_mem_to_gpr_select)
                                       /*AUTOINST*/);
    //--------------------------------------------------------------------------
    // HDU, LSU and CP0
    //--------------------------------------------------------------------------
    antares_hazard_unit hazard_unit0 (// Inputs
                                      .id_rs               (op_rs[4:0]),
                                      .id_rt               (op_rt[4:0]),
                                      .mem_mem_read        (mem_mem_to_gpr_select),
                                      .DP_Hazards          (haz_dp_hazards[7:0]),
                                      /*AUTOINST*/
                                      // Outputs
                                      .forward_id_rs       (forward_id_rs[1:0]),
                                      .forward_id_rt       (forward_id_rt[1:0]),
                                      .forward_ex_rs       (forward_ex_rs[1:0]),
                                      .forward_ex_rt       (forward_ex_rt[1:0]),
                                      .if_stall            (if_stall),
                                      .id_stall            (id_stall),
                                      .ex_stall            (ex_stall),
                                      .mem_stall           (mem_stall),
                                      .wb_stall            (wb_stall),
                                      // Inputs
                                      .ex_rs               (ex_rs[4:0]),
                                      .ex_rt               (ex_rt[4:0]),
                                      .ex_gpr_wa           (ex_gpr_wa[4:0]),
                                      .mem_gpr_wa          (mem_gpr_wa[4:0]),
                                      .wb_gpr_wa           (wb_gpr_wa[4:0]),
                                      .ex_gpr_we           (ex_gpr_we),
                                      .mem_gpr_we          (mem_gpr_we),
                                      .wb_gpr_we           (wb_gpr_we),
                                      .mem_mem_write       (mem_mem_write),
                                      .ex_request_stall    (ex_request_stall),
                                      .dmem_request_stall  (dmem_request_stall),
                                      .imem_request_stall  (imem_request_stall),
                                      .if_exception_stall  (if_exception_stall),
                                      .id_exception_stall  (id_exception_stall),
                                      .ex_exception_stall  (ex_exception_stall),
                                      .mem_exception_stall (mem_exception_stall));

    antares_load_store_unit load_store_unit0 (// Outputs
                                              .imem_data          (if_instruction[31:0]),
                                              .dmem_data_o        (mem_read_data[31:0]),
                                              // Inputs
                                              .imem_address       (if_pc[31:0]),
                                              .dmem_address       (mem_alu_result[31:0]),
                                              .dmem_data_i        (mem_mem_store_data[31:0]),
                                              .dmem_halfword      (mem_mem_halfword),
                                              .dmem_byte          (mem_mem_byte),
                                              .dmem_read          (mem_mem_to_gpr_select),
                                              .dmem_write         (mem_mem_write),
                                              .dmem_sign_extend   (mem_mem_data_sign_ext),
                                              /*AUTOINST*/
                                              // Outputs
                                              .iport_address      (iport_address[31:0]),
                                              .iport_wr           (iport_wr[3:0]),
                                              .iport_enable       (iport_enable),
                                              .dport_address      (dport_address[31:0]),
                                              .dport_data_o       (dport_data_o[31:0]),
                                              .dport_wr           (dport_wr[3:0]),
                                              .dport_enable       (dport_enable),
                                              .exc_address_if     (exc_address_if),
                                              .exc_address_l_mem  (exc_address_l_mem),
                                              .exc_address_s_mem  (exc_address_s_mem),
                                              .imem_request_stall (imem_request_stall),
                                              .dmem_request_stall (dmem_request_stall),
                                              // Inputs
                                              .clk                (clk),
                                              .rst                (rst),
                                              .iport_data_i       (iport_data_i[31:0]),
                                              .iport_ready        (iport_ready),
                                              .iport_error        (iport_error),
                                              .dport_data_i       (dport_data_i[31:0]),
                                              .dport_ready        (dport_ready),
                                              .dport_error        (dport_error),
                                              .exception_ready    (exception_ready),
                                              .mem_kernel_mode    (mem_kernel_mode),
                                              .mem_llsc           (mem_llsc),
                                              .id_eret            (id_eret));

    antares_cpzero cpzero0 (// Outputs
                            .halt                 (halt_0),
                            // Inputs
                            .mfc0                 (id_mfc0),
                            .mtc0                 (id_mtc0),
                            .eret                 (id_eret),
                            .cp1_instruction      (id_cp1_instruction),
                            .cp2_instruction      (id_cp2_instruction),
                            .cp3_instruction      (id_cp3_instruction),
                            .register_address     (op_rd[4:0]),
                            .select               (op_cp0_select[2:0]),
                            .data_input           (id_forward_rt[31:0]),
                            .exc_nmi              (nmi),
                            .exc_ibus_error       (iport_error),
                            .exc_dbus_error       (dport_error),
                            .bad_address_if       (if_pc[31:0]),
                            .bad_address_mem      (mem_alu_result[31:0]),
                            /*AUTOINST*/
                            // Outputs
                            .cp0_data_output      (cp0_data_output[31:0]),
                            .id_kernel_mode       (id_kernel_mode),
                            .if_exception_stall   (if_exception_stall),
                            .id_exception_stall   (id_exception_stall),
                            .ex_exception_stall   (ex_exception_stall),
                            .mem_exception_stall  (mem_exception_stall),
                            .if_flush             (if_flush),
                            .id_flush             (id_flush),
                            .ex_flush             (ex_flush),
                            .mem_flush            (mem_flush),
                            .exception_ready      (exception_ready),
                            .exception_pc_select  (exception_pc_select),
                            .pc_exception         (pc_exception[31:0]),
                            // Inputs
                            .clk                  (clk),
                            .if_stall             (if_stall),
                            .id_stall             (id_stall),
                            .interrupts           (interrupts[4:0]),
                            .rst                  (rst),
                            .exc_address_if       (exc_address_if),
                            .exc_address_l_mem    (exc_address_l_mem),
                            .exc_address_s_mem    (exc_address_s_mem),
                            .exc_overflow         (exc_overflow),
                            .exc_trap             (exc_trap),
                            .exc_syscall          (exc_syscall),
                            .exc_breakpoint       (exc_breakpoint),
                            .exc_reserved         (exc_reserved),
                            .id_exception_pc      (id_exception_pc[31:0]),
                            .ex_exception_pc      (ex_exception_pc[31:0]),
                            .mem_exception_pc     (mem_exception_pc[31:0]),
                            .id_exception_source  (id_exception_source),
                            .ex_exception_source  (ex_exception_source),
                            .mem_exception_source (mem_exception_source),
                            .id_is_flushed        (id_is_flushed),
                            .if_is_bds            (if_is_bds),
                            .id_is_bds            (id_is_bds),
                            .ex_is_bds            (ex_is_bds),
                            .mem_is_bds           (mem_is_bds));
endmodule // antares_core
//==================================================================================================
//  Filename      : antares_cpzero.v
//  Created On    : Sat Sep  5 18:48:44 2015
//  Last Modified : Sat Nov 07 11:59:09 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : The Coprocessor 0 (CP0)
//                  This module allows interrupts; traps, system calls and other exceptions.
//                  No Virtual Memory management
//                  Only a subset of CP0 (MIPS32 compliant).
//==================================================================================================


module antares_cpzero (
                       input wire             clk,
                       // CP0
                       input wire             mfc0,                 // mfc0 instruction
                       input wire             mtc0,                 // mtc0 instruction
                       input wire             eret,                 // eret instruction
                       input wire             cp1_instruction,      // Instruction for co-processor 1 (invalid for now)
                       input wire             cp2_instruction,      // Instruction for co-processor 2 (invalid for now)
                       input wire             cp3_instruction,      // Instruction for co-processor 3 (invalid for now)
                       input wire [4:0]       register_address,     // CP0 Register
                       input wire [2:0]       select,               // Select register
                       input wire [31:0]      data_input,           // Input wire data (write)
                       input wire             if_stall,             // Can not write to CP0 if IF/ID is stalled
                       input wire             id_stall,             // Can not write to CP0 if IF/ID is stalled
                       output reg [31:0] cp0_data_output,      // Output data (read)
                       output wire            id_kernel_mode,       // Kernel mode: 0 Kernel, 1 User
                       // Hardware/External Interrupts
                       input wire [4:0]       interrupts,           // Up to 5 external interrupts
                       // exceptions
                       input wire             rst,                  // External reset
                       input wire             exc_nmi,              // Non-maskable interrupt
                       input wire             exc_address_if,       // Address error: IF stage
                       input wire             exc_address_l_mem,    // Address error: MEM stage, load instruction
                       input wire             exc_address_s_mem,    // Address error: MEM stage, store instruction
                       input wire             exc_ibus_error,       // Instruction Bus Error
                       input wire             exc_dbus_error,       // Data Bus Error
                       input wire             exc_overflow,         // Integer overflow: EX stage
                       input wire             exc_trap,             // Trap exception
                       input wire             exc_syscall,          // System call
                       input wire             exc_breakpoint,       // Breakpoint
                       input wire             exc_reserved,         // Reserved Instruction
                       // exception data
                       input wire [31:0]      id_exception_pc,      // Exception PC @ ID stage
                       input wire [31:0]      ex_exception_pc,      // Exception PC @ EX stage
                       input wire [31:0]      mem_exception_pc,     // Exception PC @ MEM stage
                       input wire [31:0]      bad_address_if,       // Bad address that caused the exception
                       input wire [31:0]      bad_address_mem,      // Bad address that caused the exception
                       input wire             id_exception_source,  // Instruction @ ID stage is a potential source of exception
                       input wire             ex_exception_source,  // Instruction @ EX stage is a potential source of exception
                       input wire             mem_exception_source, // Instruction @ MEM stage is a potential source of exception
                       input wire             id_is_flushed,        // BDS for ERET instruction
                       input wire             if_is_bds,            // Instruction at this stage is a Branch Delay Slot
                       input wire             id_is_bds,            // Instruction at this stage is a Branch Delay Slot
                       input wire             ex_is_bds,            // Instruction at this stage is a Branch Delay Slot
                       input wire             mem_is_bds,           // Instruction at this stage is a Branch Delay Slot
                       // pipeline control
                       output wire            halt,                 // Halt the processor.
                       output wire            if_exception_stall,   // Stall pipeline: exception and wait for a clean pipeline
                       output wire            id_exception_stall,   // Stall pipeline: exception and wait for a clean pipeline
                       output wire            ex_exception_stall,   // Stall pipeline: exception and wait for a clean pipeline
                       output wire            mem_exception_stall,  // Stall pipeline: exception and wait for a clean pipeline
                       output wire            if_flush,             // Flush the pipeline: exception.
                       output wire            id_flush,             // Flush the pipeline: exception.
                       output wire            ex_flush,             // Flush the pipeline: exception.
                       output wire            mem_flush,            // Flush the pipeline: exception.
                       output wire            exception_ready,
                       output wire            exception_pc_select,  // Select the PC from CP0
                       output reg [31:0] pc_exception          // Address for the new PC (exception/return from exception)
                       );

    //--------------------------------------------------------------------------
    // Internal wires/registers
    //--------------------------------------------------------------------------
    wire              exception_cp;            // Unusable co-processor
    wire              interrupt_5;             // Hardware interrupt #5: Count/Compare (timer)
    wire              interrupt_enabled;       // Interrupt?
    wire              exception_interrupt;     // The interrupt is OK to process.
    wire              cp0_enable_write;        // Write to CP0 is OK (no hazards)
    wire              exception_no_interrupts; // All exceptions, but Interrupts, Reset, Soft-Reset, NMI

    reg [4:0]         cause_ExcCode_aux;       // Hold the ExcCode (?)

    wire              if_exception;            // exceptions by stage
    wire              id_exception;            // exceptions by stage
    wire              ex_exception;            // exceptions by stage
    wire              mem_exception;           // exceptions by stage

    wire              if_exception_mask;       // enable exception at this stage
    wire              id_exception_mask;       // enable exception at this stage
    wire              ex_exception_mask;       // enable exception at this stage
    wire              mem_exception_mask;      // enable exception at this stage

    wire              if_exception_ready;      // ready to process
    wire              id_exception_ready;      // ready to process
    wire              ex_exception_ready;      // ready to process
    wire              mem_exception_ready;     // ready to process

    //--------------------------------------------------------------------------
    // CP0 Registers
    // Defined in "MIPS32 Architecture for Programmers Volume III:
    // The MIPS32 Privileged Resource Architecture" by Imagination Technologies, LTD.
    // Only a subset.
    //--------------------------------------------------------------------------
    // Status Register:
    wire    [2:0]   Status_CU_321 = 3'b000;                 // Access Control to CPs, [2]->Cp3, ... [0]->Cp1
    reg             Status_CU_0;                            // Access Control to CP0
    wire            Status_RP     = 0;
    wire            Status_FR     = 0;
    wire            Status_RE     = 0;                      // Reverse Endian Memory for User Mode
    wire            Status_MX     = 0;
    wire            Status_PX     = 0;
    reg             Status_BEV;                             // Exception vector locations (0->Norm, 1->Bootstrap)
    wire            Status_TS     = 0;
    wire            Status_SR     = 0;                      // Soft reset (Not implemented)
    reg             Status_NMI;                             // Non-Maskable Interrupt
    wire    [1:0]   Status_RES    = 0;                      // Reserved.
    reg             Status_HALT;                            // Stop processor
    reg     [7:0]   Status_IM;                              // Interrupt mask
    wire            Status_KX     = 0;                      // 64-bits mode. (Not implemented)
    wire            Status_SX     = 0;                      // 64-bits mode. (Not implemented)
    wire            Status_UX     = 0;                      // 64-bits mode. (Not implemented)
    reg     [1:0]   Status_KSU;                             // CPU privileged level: 0 -> kernel, 1 -> supervisor, 2 -> user
    reg             Status_ERL;                             // Error Level     (0->Normal, 1->Error (reset, NMI))
    reg             Status_EXL;                             // Exception level (0->Normal, 1->Exception)
    reg             Status_IE;                              // Interrupt Enable
    wire    [31:0]  Status;                                 // Status Register (Register 12, Select 0)

    // Cause Register:
    reg             Cause_BD;                               // Exception at BDS
    reg     [1:0]   Cause_CE;                               // Co-processor error: Unusable co-processor
    reg             Cause_IV;                               // Special exception entry point
    wire            Cause_WP = 0;                           // Enable watchpoint exception mode.
    reg     [7:0]   Cause_IP;                               // Pending hardware interrupts
    reg     [4:0]   Cause_ExcCode;                          // Exception code.
    wire    [31:0]  Cause;                                  // Cause Register (Register 13, Select 0)

    // Processor Identification:
    wire    [7:0]   ID_Options = 8'b0000_0000;              // Company Options -> to define
    wire    [7:0]   ID_CID     = 8'b0000_0000;              // Company ID -> to zero
    wire    [7:0]   ID_PID     = 8'b0000_0000;              // CPU ID
    wire    [7:0]   ID_Rev     = 8'b0000_0001;              // Revision
    wire    [31:0]  PRId;                                   // Processor ID (Register 15, Select 0)

    // Configuration Register:
    wire            Config_M    = 1;                        // Continuation bit. 1-> if another config register is available
    wire    [14:0]  Config_Impl = 15'b000_0000_0000_0000;   // Implementation-dependent configuration flags.
    wire            Config_BE   = `ANTARES_LITTLE_ENDIAN;   // Endiannes
    wire    [1:0]   Config_AT   = 2'b00;                    // MIPS32
    wire    [2:0]   Config_AR   = 3'b000;                   // MIPS32 Release 1
    wire    [2:0]   Config_MT   = 3'b000;                   // MMU -> none
    wire            Config_VI   = 1'b0;                     // L1 I-cache do not use virtual address
    wire    [2:0]   Config_K0   = 3'b000;                   // Fixed kseg0 region is cached or uncached? behavior?
    wire    [31:0]  Config;                                 // Config Register (Register 16, Select 0)

    // Configuration Register 1:
    wire            Config1_M   = 0;                        // Continuation bit
    wire    [5:0]   Config1_MMU = 6'b000000;                // MMU size
    wire    [2:0]   Config1_IS  = 3'b000;                   // Number of index positions: 64 x 2^S
    wire    [2:0]   Config1_IL  = 3'b000;                   // 0 -> no cache. Else: 2^(L + 1)
    wire    [2:0]   Config1_IA  = 3'b000;                   // Associativity -> (A + 1)
    wire    [2:0]   Config1_DS  = 3'b000;                   // Number of index positions: 64 x 2^S
    wire    [2:0]   Config1_DL  = 3'b000;                   // 0 -> no cache. Else: 2^(L + 1)
    wire    [2:0]   Config1_DA  = 3'b000;                   // Associativity -> (A + 1)
    wire            Config1_C2  = 0;                        // Co-processor 2?
    wire            Config1_MD  = 0;                        // MDMX ASE?
    wire            Config1_PC  = 0;                        // Performance Counters ?
    wire            Config1_WR  = 0;                        // Watch Registers ?
    wire            Config1_CA  = 0;                        // MIPS16?
    wire            Config1_EP  = 0;                        // EJTAG?
    wire            Config1_FP  = 0;                        // Floating-point?
    wire    [31:0]  Config1;                                // Config Register (Register 16, Select 1)

    reg     [31:0]  BadVAddr;                               // BadVAddr Register (Register 8, Select 0)
    reg     [31:0]  Count;                                  // Count Register (Register 9, Select 0)
    reg     [31:0]  Compare;                                // Compare Register (Register 11, Select 0)
    reg     [31:0]  EPC;                                    // Exception Program Counter (Register 14, Select 0)
    reg     [31:0]  ErrorEPC;                               // Error Register (Register 30, Select 0)

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign  Status  = {Status_CU_321, Status_CU_0, Status_RP, Status_FR, Status_RE, Status_MX,           // bits 31-24
                       Status_PX, Status_BEV, Status_TS, Status_SR, Status_NMI, Status_RES, Status_HALT, // bits 23-16
                       Status_IM,                                                                        // bits 15-8
                       Status_KX, Status_SX, Status_UX, Status_KSU, Status_ERL, Status_EXL, Status_IE};  // bits 7-0
    assign  Cause   = {Cause_BD, 1'b0, Cause_CE, 4'b0000,                                                // bits 31-24
                       Cause_IV, Cause_WP, 6'b000000,                                                    // bits 23-16
                       Cause_IP,                                                                         // bits 15-8
                       1'b0, Cause_ExcCode, 2'b0};                                                       // bits 7-0
    assign  PRId    = {ID_Options,                                                                       // bits 31-24
                       ID_CID,                                                                           // bits 23-16
                       ID_PID,                                                                           // bits 15-8
                       ID_Rev};                                                                          // bits 7-0
    assign  Config  = {Config_M, Config_Impl,                                                            // bits 31-16
                       Config_BE, Config_AT, Config_AR, Config_MT,                                       // bits 15-7
                       3'b000, Config_VI, Config_K0};                                                    // bits 6-0
    assign  Config1 = {Config1_M, Config1_MMU,
                       Config1_IS, Config1_IL, Config1_IA,
                       Config1_DS, Config1_DL, Config1_DA,
                       Config1_C2, Config1_MD, Config1_PC, Config1_WR, Config1_CA, Config1_EP, Config1_FP};

    assign exception_cp = cp1_instruction | cp2_instruction | cp3_instruction |                         // Check if the co-processor instruction is valid.
                          ( (mtc0 | mfc0 | eret) & ~(Status_CU_0 | id_kernel_mode) );                      // For CP0   : only if it has been enabled, or in kernel mode, it's ok to use these instructions.
                                                                                                        // For CP3-1 : Always trap.

    assign exception_no_interrupts = exc_address_if | exc_ibus_error | exc_syscall | exc_breakpoint | exc_reserved |    // All exceptions, but interrupts, reset, soft-reset and nmi
                                     exception_cp | exc_overflow | exc_address_l_mem | exc_address_s_mem |              //
                                     exc_dbus_error | exc_trap;                                                         //

    assign id_kernel_mode         = (Status_KSU != 2'b10) | Status_EXL | Status_ERL;                        // Kernel mode if mode != user, Exception level or Error level. To inhibit new exceptions/interrupts
    assign interrupt_5         = (Count == Compare) & Status_IM[7];                                      // Counter interrupt (#5)
    assign interrupt_enabled   = exc_nmi | ( Status_IE & ( (Cause_IP[7:0] & Status_IM[7:0]) != 8'b0 ) ); // Interrupt  if NMI, Interrupts are enabled (global) and the individual interrupt is enable.
    assign exception_interrupt = interrupt_enabled & ~Status_EXL & ~Status_ERL & ~id_is_flushed;         // Interrupt is OK to process if: no exception level and no error level, and the instruction is a forced NOP.
    assign cp0_enable_write    = mtc0 & ~id_stall & (Status_CU_0 | id_kernel_mode) &
                               (~mem_exception & ~ex_exception & ~id_exception & ~if_exception);         // Write to CP0 if ID is not stalled, CP0 is enabled or in kernel mode, and no exceptions
    assign halt                = Status_HALT;

    //--------------------------------------------------------------------------
    // Hazards
    // Rules:
    //  - In case of exception, the stage could be stalled if:
    //      - A forward stage is capable of causing an exception, AND
    //      - A forward stage is not causing an exception.
    //  - An exception is ready to process if not stalled.
    //
    // In case of exception: clear commits, convert to NOP (a.k.a. flush the stage).
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // Exceptions by stage
    //--------------------------------------------------------------------------
    assign mem_exception = exc_address_l_mem | exc_address_s_mem | exc_dbus_error | exc_trap;                // Error on load, store, data read, or trap
    assign ex_exception  = exc_overflow;                                                                     // overflow
    assign id_exception  = exc_syscall | exc_breakpoint | exc_reserved | exception_cp | exception_interrupt; // Syscall, breakpoint, reserved instruction, Co-processor or interrupt
    assign if_exception  = exc_address_if | exc_ibus_error;                                                  // Error on load or bus

    //--------------------------------------------------------------------------
    // Mask exception: assert  in case of possible exceptions in forward stages,
    //      or if being stalled.
    // Can not process the exception if IF is stalled (unable to commit the new PC)
    //
    // NOTE: Abort IF operation in case of exception
    //--------------------------------------------------------------------------
    assign mem_exception_mask = 0;
    assign ex_exception_mask  = mem_exception_source;
    assign id_exception_mask  = mem_exception_source | ex_exception_source;
    assign if_exception_mask  = mem_exception_source | ex_exception_source | id_exception_source | exception_interrupt; // In case of interrupt, abort this

    //--------------------------------------------------------------------------
    // Generate the stall signals
    // No writes to CP0 until a clean state (no stalls).
    //--------------------------------------------------------------------------
    assign mem_exception_stall = mem_exception & mem_exception_mask;
    assign ex_exception_stall  = ex_exception & ex_exception_mask & ~mem_exception;
    assign id_exception_stall  = (id_exception | eret | mtc0) & id_exception_mask & ~(mem_exception | ex_exception);
    assign if_exception_stall  = if_exception & if_exception_mask & ~(mem_exception | ex_exception | id_exception);

    //--------------------------------------------------------------------------
    // Signal the valid exception to process
    //--------------------------------------------------------------------------
    assign mem_exception_ready = mem_exception & ~mem_exception_mask;
    assign ex_exception_ready  = ex_exception & ~ex_exception_mask;
    assign id_exception_ready  = id_exception & ~id_exception_mask;
    assign if_exception_ready  = if_exception & ~if_exception_mask;

    //--------------------------------------------------------------------------
    // Flush the stages in case of exception
    //--------------------------------------------------------------------------
    assign mem_flush = mem_exception;
    assign ex_flush  = mem_exception | ex_exception;
    assign id_flush  = mem_exception | ex_exception | id_exception;
    assign if_flush  = mem_exception | ex_exception | id_exception | if_exception | (eret & ~id_stall);       // ERET doest not execute the next instruction!!

    //--------------------------------------------------------------------------
    // Read CP0 registers
    //--------------------------------------------------------------------------
    always @(*) begin
        if(mfc0 & (Status_CU_0 | id_kernel_mode)) begin
            case (register_address)
                5'd8   : cp0_data_output = BadVAddr;
                5'd9   : cp0_data_output = Count;
                5'd11  : cp0_data_output = Compare;
                5'd12  : cp0_data_output = Status;
                5'd13  : cp0_data_output = Cause;
                5'd14  : cp0_data_output = EPC;
                5'd15  : cp0_data_output = PRId;
                5'd16  : cp0_data_output = (select == 3'b000) ? Config : Config1;
                5'd30  : cp0_data_output = ErrorEPC;
                default: cp0_data_output = 32'h0000_0000;
            endcase
        end
        else begin
            cp0_data_output = 32'h0000_0000;
        end
    end

    //--------------------------------------------------------------------------
    // Write CP0 registers.
    // Reset, soft-reset, NMI.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            Status_BEV <= 1'b1;
            Status_NMI <= 1'b0;
            Status_ERL <= 1'b1;
            ErrorEPC   <= 32'b0;
        end
        else if (id_exception_ready & exc_nmi) begin
            Status_BEV <= 1'b1;
            Status_NMI <= 1'b1;
            Status_ERL <= 1'b1;
            ErrorEPC   <= id_exception_pc;
        end
        else begin
            Status_BEV <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[22] : Status_BEV;
            Status_NMI <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[19] : Status_NMI;
            Status_ERL <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[2]  : ((Status_ERL & eret & ~id_stall) ? 1'b0 : Status_ERL);
            ErrorEPC   <= (cp0_enable_write & (register_address == 5'd30) & (select == 3'b000)) ? data_input : ErrorEPC;
        end
    end

    //--------------------------------------------------------------------------
    // Write CP0 registers.
    // Other registers
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            Count         <= 32'b0;
            Compare       <= 32'b0;
            Status_HALT   <= 1'b0;
            Status_CU_0   <= 1'b0;
            //Status_RE     <= 1'b0;
            Status_IM     <= 8'b0;
            Status_KSU    <= 2'b0;
            Status_IE     <= 1'b0;
            Cause_IV      <= 1'b0;
            Cause_IP      <= 8'b0;
        end
        else begin
            Count         <= (cp0_enable_write & (register_address == 5'd9 ) & (select == 3'b000)) ? data_input       : Count + 1'b1;
            Compare       <= (cp0_enable_write & (register_address == 5'd11) & (select == 3'b000)) ? data_input       : Compare;
            Status_HALT   <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[16]   : Status_HALT;
            Status_CU_0   <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[28]   : Status_CU_0;
            //Status_RE     <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[25]   : Status_RE;
            Status_IM     <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[15:8] : Status_IM;
            Status_KSU    <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[4:3]  : Status_KSU;
            Status_IE     <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[0]    : Status_IE;
            Cause_IV      <= (cp0_enable_write & (register_address == 5'd13) & (select == 3'b000)) ? data_input[23]   : Cause_IV;
            /* Cause_IP indicates 8 interrupts:
               [7]   is set by the timer comparison, and cleared by writing to "Compare".
               [6:2] are set and cleared by external hardware.
               [1:0] are set and cleared by software.
             */
            Cause_IP[7]   <= (cp0_enable_write & (register_address == 5'd11) & (select == 3'b000)) ? 1'b0 : ((Cause_IP[7] == 0) ? interrupt_5 : Cause_IP[7]);    // If reading -> 0, Otherwise if 0 -> interrupt_5.
            Cause_IP[6:2] <= interrupts[4:0];
            Cause_IP[1:0] <= (cp0_enable_write & (register_address == 5'd13) & (select == 3'b000)) ? data_input[9:8] : Cause_IP[1:0];
        end
    end

    //--------------------------------------------------------------------------
    // Write CP0 registers.
    // Exception and Interrupt Processing
    // Ignore if EXL or ERL is asserted
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            Cause_BD      <= 1'b0;
            Cause_CE      <= 2'b00;
            Cause_ExcCode <= 5'b0;
            Status_EXL    <= 1'b0;
            EPC           <= 32'h0;
            BadVAddr      <= 32'h0;
        end
        else begin
            // MEM stage
            if (mem_exception_ready) begin
                Cause_BD      <= (Status_EXL) ? Cause_BD : mem_is_bds;
                Cause_CE      <= (cp3_instruction) ? 2'b11 : ((cp2_instruction) ? 2'b10 : ((cp1_instruction) ? 2'b01 : 2'b00));
                Cause_ExcCode <= cause_ExcCode_aux;
                Status_EXL    <= 1'b1;
                EPC           <= (Status_EXL) ? EPC : mem_exception_pc;
                BadVAddr      <= bad_address_mem;
            end
            // EX stage
            else if (ex_exception_ready) begin
                Cause_BD      <= (Status_EXL) ? Cause_BD : ex_is_bds;
                Cause_CE      <= (cp3_instruction) ? 2'b11 : ((cp2_instruction) ? 2'b10 : ((cp1_instruction) ? 2'b01 : 2'b00));
                Cause_ExcCode <= cause_ExcCode_aux;
                Status_EXL    <= 1'b1;
                EPC           <= (Status_EXL) ? EPC : ex_exception_pc;
                BadVAddr      <= BadVAddr;
            end
            // ID stage
            else if (id_exception_ready) begin
                Cause_BD      <= (Status_EXL) ? Cause_BD : id_is_bds;
                Cause_CE      <= (cp3_instruction) ? 2'b11 : ((cp2_instruction) ? 2'b10 : ((cp1_instruction) ? 2'b01 : 2'b00));
                Cause_ExcCode <= cause_ExcCode_aux;
                Status_EXL    <= 1'b1;
                EPC           <= (Status_EXL) ? EPC : id_exception_pc;
                BadVAddr      <= BadVAddr;
            end
            // IF stage
            else if (if_exception_ready) begin
                Cause_BD      <= (Status_EXL) ? Cause_BD : if_is_bds;
                Cause_CE      <= (cp3_instruction) ? 2'b11 : ((cp2_instruction) ? 2'b10 : ((cp1_instruction) ? 2'b01 : 2'b00));
                Cause_ExcCode <= cause_ExcCode_aux;
                Status_EXL    <= 1'b1;
                EPC           <= (Status_EXL) ? EPC : bad_address_if;
                BadVAddr      <= bad_address_if;
            end
            // No exceptions this cycle
            else begin
                Cause_BD      <= 1'b0;
                Cause_CE      <= Cause_CE;
                Cause_ExcCode <= Cause_ExcCode;
                // Without new exceptions, 'Status_EXL' is set by software or cleared by ERET.
                Status_EXL    <= (cp0_enable_write & (register_address == 5'd12) & (select == 3'b000)) ? data_input[1] : ((Status_EXL & eret & ~id_stall) ? 1'b0 : Status_EXL);
                // The EPC is also writable by software
                EPC           <= (cp0_enable_write & (register_address == 5'd14) & (select == 3'b000)) ? data_input : EPC;
                BadVAddr      <= BadVAddr;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Set the program counter
    // The PC register handles the reset scenario.
    //--------------------------------------------------------------------------
    always @(*) begin
        if (rst) begin
            pc_exception = `ANTARES_VECTOR_BASE_RESET;
        end
        if (eret & ~id_stall) begin
            pc_exception = (Status_ERL) ? ErrorEPC : EPC;
        end
        else if (exception_no_interrupts) begin
            pc_exception = (Status_BEV) ? (`ANTARES_VECTOR_BASE_BOOT + `ANTARES_VECTOR_OFFSET_GENERAL) : (`ANTARES_VECTOR_BASE_NO_BOOT + `ANTARES_VECTOR_OFFSET_GENERAL);
        end
        else if (exc_nmi) begin
            pc_exception = `ANTARES_VECTOR_BASE_RESET;
        end
        else if (exception_interrupt & Cause_IV) begin
            pc_exception = (Status_BEV) ? (`ANTARES_VECTOR_BASE_BOOT + `ANTARES_VECTOR_OFFSET_SPECIAL) : (`ANTARES_VECTOR_BASE_NO_BOOT + `ANTARES_VECTOR_OFFSET_SPECIAL);
        end
        else begin
            pc_exception = (Status_BEV) ? (`ANTARES_VECTOR_BASE_BOOT + `ANTARES_VECTOR_OFFSET_GENERAL) : (`ANTARES_VECTOR_BASE_NO_BOOT + `ANTARES_VECTOR_OFFSET_GENERAL);
        end
    end

    assign exception_ready     = if_exception_ready | id_exception_ready | ex_exception_ready | mem_exception_ready;
    assign exception_pc_select = rst | (eret & ~id_stall) | exception_ready;

    //--------------------------------------------------------------------------
    // Set the Cause register
    // Ordered by Pipeline Stage with Interrupts last
    //--------------------------------------------------------------------------
    always @(*) begin
        if      (exc_address_l_mem)   cause_ExcCode_aux = 5'h4;     // 00100 (EXC_AdEL)
        else if (exc_address_s_mem)   cause_ExcCode_aux = 5'h5;     // 00101 (EXC_AdES)
        else if (exc_dbus_error)      cause_ExcCode_aux = 5'h7;     // 00111 (EXC_DBE)
        else if (exc_trap)            cause_ExcCode_aux = 5'hd;     // 01101 (EXC_Tr)
        else if (exc_overflow)        cause_ExcCode_aux = 5'hc;     // 01100 (EXC_Ov)
        else if (exc_syscall)         cause_ExcCode_aux = 5'h8;     // 01000 (EXC_Sys)
        else if (exc_breakpoint)      cause_ExcCode_aux = 5'h9;     // 01001 (EXC_Bp)
        else if (exc_reserved)        cause_ExcCode_aux = 5'ha;     // 01010 (EXC_RI)
        else if (exception_cp)        cause_ExcCode_aux = 5'hb;     // 01011 (EXC_CpU)
        else if (exc_address_if)      cause_ExcCode_aux = 5'h4;     // 00100 (EXC_AdIF)
        else if (exc_ibus_error)      cause_ExcCode_aux = 5'h6;     // 00110 (EXC_IBE)
        else if (exception_interrupt) cause_ExcCode_aux = 5'h0;     // 00000 (EXC_Int)
        else                          cause_ExcCode_aux = 5'bxxxx;  // What the hell?
    end
endmodule // antares_cpzero

//==================================================================================================
//  Filename      : antares_divider.v
//  Created On    : Thu Sep  3 08:41:07 2015
//  Last Modified : Sat Nov 07 12:01:42 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : A multi-cycle divider unit.
//                  op_div and op_divu MUST BE dis-asserted after the setup
//                  cycle for normal operation, or the operation will be restarted.
//                  WARNING: no exception if divisor == 0.
//==================================================================================================

module antares_divider (
                        input wire         clk,
                        input wire         rst,
                        input wire         op_divs,
                        input wire         op_divu,
                        input wire [31:0]  dividend,
                        input wire [31:0]  divisor,
                        output wire [31:0] quotient,
                        output wire [31:0] remainder,
                        output wire        div_stall
                        );

    //--------------------------------------------------------------------------
    // Signal Declaration: reg
    //--------------------------------------------------------------------------
    reg           active;          // 1 while running
    reg           neg_result;      // 1 if the result must be negative
    reg           neg_remainder;   // 1 if the remainder must be negative
    reg [4:0]     cycle;           // number of cycles needed.
    reg [31:0]    result;          // Store the result.
    reg [31:0]    denominator;     // divisor
    reg [31:0]    residual;        // current remainder

    //--------------------------------------------------------------------------
    // Signal Declaration: wire
    //--------------------------------------------------------------------------
    wire [32:0]   partial_sub;        // temp

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign quotient    = !neg_result ? result : -result;
    assign remainder   = !neg_remainder ? residual : -residual;
    assign div_stall   = active;
    assign partial_sub = {residual[30:0], result[31]} - denominator;            // calculate partial result

    //--------------------------------------------------------------------------
    // State Machine. This needs 32 cycles to calculate the result.
    // The result is loaded after 34 cycles
    // The first cycle is setup.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            /*AUTORESET*/
            // Beginning of autoreset for uninitialized flops
            active <= 1'h0;
            cycle <= 5'h0;
            denominator <= 32'h0;
            neg_result <= 1'h0;
            neg_remainder <= 1'h0;
            residual <= 32'h0;
            result <= 32'h0;
            // End of automatics
        end
        else begin
            if(op_divs) begin
                // Signed division.
                cycle         <= 5'd31;
                result        <= (dividend[31] == 1'b0) ? dividend : -dividend;
                denominator   <= (divisor[31] == 1'b0) ? divisor : -divisor;
                residual      <= 32'b0;
                neg_result    <= dividend[31] ^ divisor[31];
                neg_remainder <= dividend[31];
                active        <= 1'b1;
            end
            else if (op_divu) begin
                // Unsigned division.
                cycle         <= 5'd31;
                result        <= dividend;
                denominator   <= divisor;
                residual      <= 32'b0;
                neg_result    <= 1'b0;
                neg_remainder <= 1'h0;
                active        <= 1'b1;
            end
            else if (active) begin
                // run a iteration
                if(partial_sub[32] == 1'b0) begin
                    residual <= partial_sub[31:0];
                    result   <= {result[30:0], 1'b1};
                end
                else begin
                    residual <= {residual[30:0], result[31]};
                    result   <= {result[30:0], 1'b0};
                end

                if (cycle == 5'b0) begin
                    active <= 1'b0;
                end

                cycle <= cycle - 5'd1;
            end
        end
    end

endmodule // antares_divider
//==================================================================================================
//  Filename      : antares_exmem_register.v
//  Created On    : Sat Sep  5 21:23:28 2015
//  Last Modified : Sat Nov 07 12:04:18 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Pipeline register: EX -> MEM
//==================================================================================================

module antares_exmem_register (
                               input wire             clk,                      // main clock
                               input wire             rst,                      // main reset
                               input wire [31:0]      ex_alu_result,            // ALU result
                               input wire [31:0]      ex_mem_store_data,        // data to memory
                               input wire [4:0]       ex_gpr_wa,                // GPR write address
                               input wire             ex_gpr_we,                // GPR write enable
                               input wire             ex_mem_to_gpr_select,     // Select MEM/ALU to GPR
                               input wire             ex_mem_write,             // Mem write operation
                               input wire             ex_mem_byte,              // byte access
                               input wire             ex_mem_halfword,          // halfword access
                               input wire             ex_mem_data_sign_ext,     // Sign/Zero extend data from memory
                               input wire [31:0]      ex_exception_pc,
                               input wire             ex_movn,
                               input wire             ex_movz,
                               input wire             ex_b_is_zero,
                               input wire             ex_llsc,
                               input wire             ex_kernel_mode,
                               input wire             ex_is_bds,
                               input wire             ex_trap,
                               input wire             ex_trap_condition,
                               input wire             ex_mem_exception_source,  //
                               input wire             ex_flush,                 // clean
                               input wire             ex_stall,                 // stall EX stage
                               input wire             mem_stall,                // stall MEM stage
                               output reg [31:0] mem_alu_result,           // Same signals, but on mem stage
                               output reg [31:0] mem_mem_store_data,       //
                               output reg [4:0]  mem_gpr_wa,               //
                               output reg        mem_gpr_we,               //
                               output reg        mem_mem_to_gpr_select,    //
                               output reg        mem_mem_write,            //
                               output reg        mem_mem_byte,             //
                               output reg        mem_mem_halfword,         //
                               output reg        mem_mem_data_sign_ext,    //
                               output reg [31:0] mem_exception_pc,
                               output reg        mem_llsc,
                               output reg        mem_kernel_mode,
                               output reg        mem_is_bds,
                               output reg        mem_trap,
                               output reg        mem_trap_condition,
                               output reg        mem_mem_exception_source
                               );

    // Check for MOVN or MOVZ instruction
    wire    mov_reg_write = (ex_movn &  ~ex_b_is_zero) | (ex_movz &  ex_b_is_zero);

    //--------------------------------------------------------------------------
    // Propagate signals
    // Clear WE and Write signals only, on EX stall.
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        mem_alu_result           <= (rst) ? 32'b0 : ((mem_stall) ? mem_alu_result                                           : ex_alu_result);
        mem_mem_store_data       <= (rst) ? 32'b0 : ((mem_stall) ? mem_mem_store_data                                       : ex_mem_store_data);
        mem_gpr_wa               <= (rst) ? 5'b0  : ((mem_stall) ? mem_gpr_wa                                               : ex_gpr_wa);
        mem_gpr_we               <= (rst) ? 1'b0  : ((mem_stall) ? mem_gpr_we               : ((ex_stall | ex_flush) ? 1'b0 : ((ex_movz | ex_movn) ? mov_reg_write : ex_gpr_we)));
        mem_mem_to_gpr_select    <= (rst) ? 1'b0  : ((mem_stall) ? mem_mem_to_gpr_select    : ((ex_stall | ex_flush) ? 1'b0 : ex_mem_to_gpr_select));     // test
        mem_mem_write            <= (rst) ? 1'b0  : ((mem_stall) ? mem_mem_write            : ((ex_stall | ex_flush) ? 1'b0 : ex_mem_write));
        mem_mem_byte             <= (rst) ? 1'b0  : ((mem_stall) ? mem_mem_byte                                             : ex_mem_byte);
        mem_mem_halfword         <= (rst) ? 1'b0  : ((mem_stall) ? mem_mem_halfword                                         : ex_mem_halfword);
        mem_mem_data_sign_ext    <= (rst) ? 1'b0  : ((mem_stall) ? mem_mem_data_sign_ext                                    : ex_mem_data_sign_ext);
        mem_exception_pc         <= (rst) ? 32'b0 : ((mem_stall) ? mem_exception_pc                                         : ex_exception_pc);
        mem_llsc                 <= (rst) ? 1'b0  : ((mem_stall) ? mem_llsc                                                 : ex_llsc);
        mem_kernel_mode          <= (rst) ? 1'b0  : ((mem_stall) ? mem_kernel_mode                                          : ex_kernel_mode);
        mem_is_bds               <= (rst) ? 1'b0  : ((mem_stall) ? mem_is_bds                                               : ex_is_bds);
        mem_trap                 <= (rst) ? 1'b0  : ((mem_stall) ? mem_trap                 : ((ex_stall | ex_flush) ? 1'b0 : ex_trap));
        mem_trap_condition       <= (rst) ? 1'b0  : ((mem_stall) ? mem_trap_condition                                       : ex_trap_condition);
        mem_mem_exception_source <= (rst) ? 1'b0  : ((mem_stall) ? mem_mem_exception_source : ((ex_stall | ex_flush) ? 1'b0 : ex_mem_exception_source));
    end // always @ (posedge clk)
endmodule // antares_exmem_register
//==================================================================================================
//  Filename      : antares_hazard_unit.v
//  Created On    : Fri Sep  4 22:32:20 2015
//  Last Modified : Sat Nov 07 12:03:58 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Hazard detection and pipeline control unit.
//==================================================================================================


module antares_hazard_unit (
                            input wire [7:0]  DP_Hazards,          //
                            input wire [4:0]  id_rs,               // Rs @ ID stage
                            input wire [4:0]  id_rt,               // Rt @ ID stage
                            input wire [4:0]  ex_rs,               // Rs @ EX stage
                            input wire [4:0]  ex_rt,               // Rt @ EX stage
                            input wire [4:0]  ex_gpr_wa,           // Write Address @ EX stage
                            input wire [4:0]  mem_gpr_wa,          // Write Address @ MEM stage
                            input wire [4:0]  wb_gpr_wa,           // Write Address @ WB stage
                            input wire        ex_gpr_we,           // GPR write enable @ EX
                            input wire        mem_gpr_we,          // GPR write enable @ MEM
                            input wire        wb_gpr_we,           // GPR write enable @ WB
                            input wire        mem_mem_write,       //
                            input wire        mem_mem_read,        //
                            input wire        ex_request_stall,    // Ex unit request a stall
                            input wire        dmem_request_stall,  // LSU: stall for Data access
                            input wire        imem_request_stall,  // LSU: stall for Instruction Fetch
                            input wire        if_exception_stall,  // Stall waiting for possible exception
                            input wire        id_exception_stall,  // Stall waiting for possible exception
                            input wire        ex_exception_stall,  // Stall waiting for possible exception
                            input wire        mem_exception_stall, //
                            output wire [1:0] forward_id_rs,       // Forwarding Rs multiplexer: Selector @ ID
                            output wire [1:0] forward_id_rt,       // Forwarding Rt multiplexer: Selector @ ID
                            output wire [1:0] forward_ex_rs,       // Forwarding Rs multiplexer: Selector @ EX
                            output wire [1:0] forward_ex_rt,       // Forwarding Rt multiplexer: Selector @ EX
                            output wire       if_stall,            // Stall pipeline register
                            output wire       id_stall,            // Stall pipeline register
                            output wire       ex_stall,            // Stall pipeline register
                            //output wire       ex_stall_unit;     // Stall the EX unit.
                            output wire       mem_stall,           // Stall pipeline register
                            output wire       wb_stall             // Stall pipeline register
                            );

    //--------------------------------------------------------------------------
    // Signal Declaration: wire
    //--------------------------------------------------------------------------
    // no forwarding if reading register zero
    wire         ex_wa_nz;
    wire         mem_wa_nz;
    wire         wb_wa_nz;
    // Need/Want signals
    wire         WantRsID;
    wire         WantRtID;
    wire         WantRsEX;
    wire         WantRtEX;
    wire         NeedRsID;
    wire         NeedRtID;
    wire         NeedRsEX;
    wire         NeedRtEX;
    // verify match: register address and write address (EX, MEM & WB)
    wire         id_ex_rs_match;
    wire         id_ex_rt_match;
    wire         id_mem_rs_match;
    wire         id_mem_rt_match;
    wire         id_wb_rs_match;
    wire         id_wb_rt_match;
    wire         ex_mem_rs_match;
    wire         ex_mem_rt_match;
    wire         ex_wb_rs_match;
    wire         ex_wb_rt_match;
    // stall signals
    wire         stall_id_1;
    wire         stall_id_2;
    wire         stall_id_3;
    wire         stall_id_4;
    wire         stall_ex_1;
    wire         stall_ex_2;

    // forward signals
    wire         forward_mem_id_rs;
    wire         forward_mem_id_rt;
    wire         forward_wb_id_rs;
    wire         forward_wb_id_rt;
    wire         forward_mem_ex_rs;
    wire         forward_mem_ex_rt;
    wire         forward_wb_ex_rs;
    wire         forward_wb_ex_rt;

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign WantRsID = DP_Hazards[7];
    assign NeedRsID = DP_Hazards[6];
    assign WantRtID = DP_Hazards[5];
    assign NeedRtID = DP_Hazards[4];
    assign WantRsEX = DP_Hazards[3];
    assign NeedRsEX = DP_Hazards[2];
    assign WantRtEX = DP_Hazards[1];
    assign NeedRtEX = DP_Hazards[0];

    // Check if the register to use is $zero
    assign ex_wa_nz  = |(ex_gpr_wa);
    assign mem_wa_nz = |(mem_gpr_wa);
    assign wb_wa_nz  = |(wb_gpr_wa);

    // ID dependencies
    assign id_ex_rs_match  = (ex_wa_nz)  & (id_rs == ex_gpr_wa)  & (WantRsID | NeedRsID) & ex_gpr_we;
    assign id_ex_rt_match  = (ex_wa_nz)  & (id_rt == ex_gpr_wa)  & (WantRtID | NeedRtID) & ex_gpr_we;
    assign id_mem_rs_match = (mem_wa_nz) & (id_rs == mem_gpr_wa) & (WantRsID | NeedRsID) & mem_gpr_we;
    assign id_mem_rt_match = (mem_wa_nz) & (id_rt == mem_gpr_wa) & (WantRtID | NeedRtID) & mem_gpr_we;
    assign id_wb_rs_match  = (wb_wa_nz)  & (id_rs == wb_gpr_wa)  & (WantRsID | NeedRsID) & wb_gpr_we;
    assign id_wb_rt_match  = (wb_wa_nz)  & (id_rt == wb_gpr_wa)  & (WantRtID | NeedRtID) & wb_gpr_we;
    // EX dependencies
    assign ex_mem_rs_match = (mem_wa_nz) & (ex_rs == mem_gpr_wa) & (WantRsEX | NeedRsEX) & mem_gpr_we;
    assign ex_mem_rt_match = (mem_wa_nz) & (ex_rt == mem_gpr_wa) & (WantRtEX | NeedRtEX) & mem_gpr_we;
    assign ex_wb_rs_match  = (wb_wa_nz)  & (ex_rs == wb_gpr_wa)  & (WantRsEX | NeedRsEX) & wb_gpr_we;
    assign ex_wb_rt_match  = (wb_wa_nz)  & (ex_rt == wb_gpr_wa)  & (WantRtEX | NeedRtEX) & wb_gpr_we;

    // stall signals
    assign stall_id_1 = id_ex_rs_match & NeedRsID; // Needs data from EX (Rs)
    assign stall_id_2 = id_ex_rt_match & NeedRtID; // Needs data from EX (Rt)
    assign stall_id_3 = id_mem_rs_match & NeedRsID & (mem_mem_read | mem_mem_write); // Needs data from MEM (Rs)
    assign stall_id_4 = id_mem_rt_match & NeedRtID & (mem_mem_read | mem_mem_write); // Needs data from MEM (Rt)
    assign stall_ex_1 = ex_mem_rs_match & NeedRsEX & (mem_mem_read | mem_mem_write); // Needs data from MEM (Rs)
    assign stall_ex_2 = ex_mem_rt_match & NeedRtEX & (mem_mem_read | mem_mem_write); // Needs data from MEM (Rt)

    // forwarding signals
    assign forward_mem_id_rs = id_mem_rs_match & ~(mem_mem_read | mem_mem_write); // forward if not mem access
    assign forward_mem_id_rt = id_mem_rt_match & ~(mem_mem_read | mem_mem_write); // forward if not mem access;
    assign forward_wb_id_rs  = id_wb_rs_match;
    assign forward_wb_id_rt  = id_wb_rt_match;
    assign forward_mem_ex_rs = ex_mem_rs_match & ~(mem_mem_read | mem_mem_write);
    assign forward_mem_ex_rt = ex_mem_rt_match & ~(mem_mem_read | mem_mem_write);
    assign forward_wb_ex_rs  = ex_wb_rs_match;
    assign forward_wb_ex_rt  = ex_wb_rt_match;

    //--------------------------------------------------------------------------
    // Assign stall signals
    //--------------------------------------------------------------------------
    assign wb_stall  = mem_stall;
    assign mem_stall = dmem_request_stall | mem_exception_stall | if_stall; // check the if_stall
    assign ex_stall  = stall_ex_1 | stall_ex_2 | ex_exception_stall | ex_request_stall | mem_stall;
    assign id_stall  = stall_id_1 | stall_id_2 | stall_id_3 | stall_id_4 | id_exception_stall | ex_stall;
    assign if_stall  = imem_request_stall | if_exception_stall;

    //--------------------------------------------------------------------------
    // forwarding control signals
    //--------------------------------------------------------------------------
    // sel | ID stage           | EX stage
    //--------------------------------------------------------------------------
    // 00 -> ID (no forwarding) | EX (no forwarding)
    // 01 -> MEM                | MEM
    // 10 -> WB                 | WB
    // 11 -> don't care         | don't care
    //--------------------------------------------------------------------------
    assign forward_id_rs = (forward_mem_id_rs) ? 2'b01 : ((forward_wb_id_rs) ? 2'b10 : 2'b00);
    assign forward_id_rt = (forward_mem_id_rt) ? 2'b01 : ((forward_wb_id_rt) ? 2'b10 : 2'b00);
    assign forward_ex_rs = (forward_mem_ex_rs) ? 2'b01 : ((forward_wb_ex_rs) ? 2'b10 : 2'b00);
    assign forward_ex_rt = (forward_mem_ex_rt) ? 2'b01 : ((forward_wb_ex_rt) ? 2'b10 : 2'b00);
endmodule // antares_hazard_unit
//==================================================================================================
//  Filename      : antares_idex_register.v
//  Created On    : Sat Sep  5 21:08:59 2015
//  Last Modified : Sat Nov 07 12:09:34 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Pipeline register: ID -> EX
//==================================================================================================

module antares_idex_register (
                              input wire             clk,                     // Main clock
                              input wire             rst,                     // Main reset
                              input wire [4:0]       id_alu_operation,        // ALU operation from ID stage
                              input wire [31:0]      id_data_rs,              // Data Rs (forwarded)
                              input wire [31:0]      id_data_rt,              // Data Rt (forwarded)
                              input wire             id_gpr_we,               // GPR write enable
                              input wire             id_mem_to_gpr_select,    // Select MEM/ALU to GPR
                              input wire             id_mem_write,            // write to memory
                              input wire [1:0]       id_alu_port_a_select,    // Select: GPR, shamt, 0x00000004
                              input wire [1:0]       id_alu_port_b_select,    // Select: GPR, Imm16, PCAdd4
                              input wire [1:0]       id_gpr_wa_select,        // Select: direccion: Rt, Rd, $31
                              input wire             id_mem_byte,             // byte access
                              input wire             id_mem_halfword,         // halfword access
                              input wire             id_mem_data_sign_ext,    // Zero/Sign extend
                              input wire [4:0]       id_rs,                   // Rs
                              input wire [4:0]       id_rt,                   // Rt
                              input wire [3:0]       id_dp_hazard,
                              input wire             id_imm_sign_ext,         // extend the imm16
                              input wire [15:0]      id_sign_imm16,           // sign_ext(imm16)
                              input wire [31:0]      id_cp0_data,             //
                              input wire [31:0]      id_exception_pc,         // Current PC
                              input wire             id_movn,
                              input wire             id_movz,
                              input wire             id_llsc,
                              input wire             id_kernel_mode,
                              input wire             id_is_bds,
                              input wire             id_trap,
                              input wire             id_trap_condition,
                              input wire             id_ex_exception_source,
                              input wire             id_mem_exception_source,
                              input wire             id_flush,                // clean
                              input wire             id_stall,                // Stall ID stage
                              input wire             ex_stall,                // Stall EX stage
                              output reg [4:0]  ex_alu_operation,        // Same signals, but on EX stage
                              output reg [31:0] ex_data_rs,              //
                              output reg [31:0] ex_data_rt,              //
                              output reg        ex_gpr_we,               //
                              output reg        ex_mem_to_gpr_select,    //
                              output reg        ex_mem_write,            //
                              output reg [1:0]  ex_alu_port_a_select,    //
                              output reg [1:0]  ex_alu_port_b_select,    //
                              output reg [1:0]  ex_gpr_wa_select,        //
                              output reg        ex_mem_byte,             //
                              output reg        ex_mem_halfword,         //
                              output reg        ex_mem_data_sign_ext,    //
                              output reg [4:0]  ex_rs,                   //
                              output reg [4:0]  ex_rt,                   //
                              output reg [3:0]  ex_dp_hazard,
                              output reg [16:0] ex_sign_imm16,           //
                              output reg [31:0] ex_cp0_data,
                              output reg [31:0] ex_exception_pc,
                              output reg        ex_movn,
                              output reg        ex_movz,
                              output reg        ex_llsc,
                              output reg        ex_kernel_mode,
                              output reg        ex_is_bds,
                              output reg        ex_trap,
                              output reg        ex_trap_condition,
                              output reg        ex_ex_exception_source,
                              output reg        ex_mem_exception_source
                              );

    // sign extend the imm16
    wire [16:0] id_imm_extended = (id_imm_sign_ext) ? {id_sign_imm16[15], id_sign_imm16[15:0]} : {1'b0, id_sign_imm16};

    //--------------------------------------------------------------------------
    // Propagate signals
    // Clear only critical signals: op, WE, MEM write and Next PC
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        ex_alu_operation        <= (rst) ? 5'b0  : ((ex_stall & ~id_flush) ? ex_alu_operation        : ((id_stall | id_flush) ? 5'b0 : id_alu_operation));
        ex_data_rs              <= (rst) ? 32'b0 : ((ex_stall & ~id_flush) ? ex_data_rs                                              : id_data_rs);
        ex_data_rt              <= (rst) ? 32'b0 : ((ex_stall & ~id_flush) ? ex_data_rt                                              : id_data_rt);
        ex_gpr_we               <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_gpr_we               : ((id_stall | id_flush) ? 1'b0 : id_gpr_we));
        ex_mem_to_gpr_select    <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_mem_to_gpr_select    : ((id_stall | id_flush) ? 1'b0 : id_mem_to_gpr_select));
        ex_mem_write            <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_mem_write            : ((id_stall | id_flush) ? 1'b0 : id_mem_write));
        ex_alu_port_a_select    <= (rst) ? 2'b0  : ((ex_stall & ~id_flush) ? ex_alu_port_a_select                                    : id_alu_port_a_select);
        ex_alu_port_b_select    <= (rst) ? 2'b0  : ((ex_stall & ~id_flush) ? ex_alu_port_b_select                                    : id_alu_port_b_select);
        ex_gpr_wa_select        <= (rst) ? 2'b0  : ((ex_stall & ~id_flush) ? ex_gpr_wa_select                                        : id_gpr_wa_select);
        ex_mem_byte             <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_mem_byte                                             : id_mem_byte);
        ex_mem_halfword         <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_mem_halfword                                         : id_mem_halfword);
        ex_mem_data_sign_ext    <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_mem_data_sign_ext                                    : id_mem_data_sign_ext);
        ex_rs                   <= (rst) ? 5'b0  : ((ex_stall & ~id_flush) ? ex_rs                                                   : id_rs);
        ex_rt                   <= (rst) ? 5'b0  : ((ex_stall & ~id_flush) ? ex_rt                                                   : id_rt);
        ex_dp_hazard            <= (rst) ? 4'b0  : ((ex_stall & ~id_flush) ? ex_dp_hazard            : ((id_stall | id_flush) ? 4'b0 : id_dp_hazard));
        ex_sign_imm16           <= (rst) ? 17'b0 : ((ex_stall & ~id_flush) ? ex_sign_imm16                                           : id_imm_extended);
        ex_cp0_data             <= (rst) ? 32'b0 : ((ex_stall & ~id_flush) ? ex_cp0_data                                             : id_cp0_data);
        ex_exception_pc         <= (rst) ? 32'b0 : ((ex_stall & ~id_flush) ? ex_exception_pc                                         : id_exception_pc);
        ex_movn                 <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_movn                 : ((id_stall | id_flush) ? 1'b0 : id_movn));
        ex_movz                 <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_movz                 : ((id_stall | id_flush) ? 1'b0 : id_movz));
        ex_llsc                 <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_llsc                                                 : id_llsc);
        ex_kernel_mode          <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_kernel_mode                                          : id_kernel_mode);
        ex_is_bds               <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_is_bds                                               : id_is_bds);
        ex_trap                 <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_trap                 : ((id_stall | id_flush) ? 1'b0 : id_trap));
        ex_trap_condition       <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_trap_condition                                       : id_trap_condition);
        ex_ex_exception_source  <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_ex_exception_source  : ((id_stall | id_flush) ? 1'b0 : id_ex_exception_source));
        ex_mem_exception_source <= (rst) ? 1'b0  : ((ex_stall & ~id_flush) ? ex_mem_exception_source : ((id_stall | id_flush) ? 1'b0 : id_mem_exception_source));
    end // always @ (posedge clk)
endmodule // antares_idex_register
//==================================================================================================
//  Filename      : antares_ifid_register.v
//  Created On    : Sat Sep  5 21:00:32 2015
//  Last Modified : Sat Nov 07 12:08:03 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Pipeline register: IF -> ID
//==================================================================================================

module antares_ifid_register (
                              input wire             clk,             // main clock
                              input wire             rst,             // main reset
                              input wire [31:0]      if_instruction,  // Instruction from IF
                              input wire [31:0]      if_pc_add4,      // PC + 1 from IF
                              input wire [31:0]      if_exception_pc, // PC     from IF
                              input wire             if_is_bds,       // This instruction is a BDS.
                              input wire             if_flush,        // clean
                              input wire             if_stall,        // Stall IF
                              input wire             id_stall,        // Stall ID
                              output reg [31:0] id_instruction,  // ID instruction
                              output reg [31:0] id_pc_add4,      // PC + 1 to ID
                              output reg [31:0] id_exception_pc, // PC to ID
                              output reg        id_is_bds,       // Instruction is a BDS
                              output reg        id_is_flushed    // This instruction must be ignored
                              );

    always @(posedge clk) begin
        id_instruction  <= (rst) ? 32'b0 : ((id_stall) ? id_instruction  : ((if_stall | if_flush) ? 32'b0 : if_instruction));
        id_pc_add4      <= (rst) ? 32'b0 : ((id_stall) ? id_pc_add4                                       : if_pc_add4);
        id_exception_pc <= (rst) ? 32'b0 : ((id_stall) ? id_exception_pc                                  : if_exception_pc);
        id_is_bds       <= (rst) ? 1'b0  : ((id_stall) ? id_is_bds                                        : if_is_bds);
        id_is_flushed   <= (rst) ? 1'b0  : ((id_stall) ? id_is_flushed                                    : if_flush);
    end
endmodule // antares_ifid_register
//==================================================================================================
//  Filename      : antares_load_store_unit.v
//  Created On    : Sat Sep  5 10:38:09 2015
//  Last Modified : Sat Nov 07 12:09:07 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Handle memory access; using a 4-way handshaking protocol:
//                  1.- Assert enable signal.
//                  2.- Ready goes high when data is available.
//                  3.- If Ready is high; enable signal goes low.
//                  4.- Next cycle; if enable is low, clear Ready signal.
//
//                  Time diagram:
//
//                  Clock Tick:   |  |  |  |  |  |  |  |  |  |  |
//                                 ______         ___
//                  Enable:     __|      |_______|   |______
//                                       __          __
//                  Ready:         _____|  |________|  |____
//==================================================================================================


module antares_load_store_unit (
                                input wire             clk,                // Clock
                                input wire             rst,                // Reset
                                // Instruction interface: LSU <-> CPU
                                input wire [31:0]      imem_address,       // Instruction address
                                output reg [31:0] imem_data,          // Instruction data
                                // MEM interface: LSU <-> CPU
                                input wire [31:0]      dmem_address,       // Data address
                                input wire [31:0]      dmem_data_i,        // Data to memory
                                input wire             dmem_halfword,      // halfword access
                                input wire             dmem_byte,          // byte access
                                input wire             dmem_read,          // read data memory
                                input wire             dmem_write,         // write data memory
                                input wire             dmem_sign_extend,   // read data (byte/half) with sign extended
                                output reg [31:0] dmem_data_o,        // data from memory
                                // Instruction Port: LSU <-> MEM[instruction]
                                input wire [31:0]      iport_data_i,       // Data from memory
                                input wire             iport_ready,        // memory is ready
                                input wire             iport_error,        // Bus error
                                output wire [31:0]     iport_address,      // data address
                                output wire [3:0]      iport_wr,           // write = byte select, read = 0000,
                                output wire            iport_enable,       // enable operation
                                // Data Port : LSU <-> (MEM[data],    I/O)
                                input wire [31:0]      dport_data_i,       // Data from memory
                                input wire             dport_ready,        // memory is ready
                                input wire             dport_error,        // Bus error
                                output wire [31:0]     dport_address,      // data address
                                output wire [31:0]     dport_data_o,       // data to memory
                                output reg [3:0]  dport_wr,           // write = byte select, read = 0000,
                                output wire            dport_enable,       // enable operation
                                // pipeline signals
                                input wire             exception_ready,
                                input wire             mem_kernel_mode,    // For exception logic
                                input wire             mem_llsc,           // Atomic operation
                                input wire             id_eret,            // for llsc1
                                output wire            exc_address_if,     // panic
                                output wire            exc_address_l_mem,  // panic
                                output wire            exc_address_s_mem,  // panic
                                output wire            imem_request_stall, // long operation
                                output wire            dmem_request_stall  // long operation
                                );

    //--------------------------------------------------------------------------
    // wire and registers
    //--------------------------------------------------------------------------
    wire              exc_invalid_word_iaddress;      // Not word-aligned instructions address
    wire              exc_invalid_space_iaddress;     // try to access I/O space
    wire              exc_invalid_word_maddress;      // Not word-aligned data address
    wire              exc_invalid_half_maddress;      // Not halfword-aligned data address
    wire              exc_invalid_space_maddress;     // try to access kernel space
    wire              dmem_operation;                 // Read or Write?
    wire              data_word;                      // LW/SW operation

    wire              exc_invalid_maddress;
    wire              write_enable;
    wire              read_enable;

    reg [29:0]        llsc_address;
    reg               llsc_atomic;
    wire              llsc_mem_write_mask;

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    // Check for invalid access from instruction port.
    assign exc_invalid_word_iaddress  = imem_address[1] | imem_address[0];
    assign exc_invalid_space_iaddress = 0; // TODO: check for invalid IM access.
    // Check for invalid access from data port.
    assign exc_invalid_word_maddress  = (dmem_address[1] | dmem_address[0]) & data_word;
    assign exc_invalid_half_maddress  = dmem_address[0] & dmem_halfword;
    assign exc_invalid_space_maddress = ~mem_kernel_mode & (dmem_address < `ANTARES_SEG_2_SPACE_LOW);
    assign exc_invalid_maddress       = exc_invalid_space_maddress | exc_invalid_word_maddress | exc_invalid_half_maddress;
    // Exception signals.
    assign exc_address_if             = exc_invalid_word_iaddress | exc_invalid_space_iaddress;
    assign exc_address_l_mem          = dmem_read  & exc_invalid_maddress;
    assign exc_address_s_mem          = dmem_write & exc_invalid_maddress;

    assign write_enable               = dmem_write & ~exc_invalid_maddress & ~llsc_mem_write_mask;
    assign read_enable                = dmem_read  & ~exc_invalid_maddress;

    assign dmem_operation             = (write_enable ^ read_enable) | mem_llsc;
    assign data_word                  = ~(dmem_halfword | dmem_byte);

    assign imem_request_stall         = iport_enable;
    assign dmem_request_stall         = dport_enable;

    assign iport_enable               = (~rst & ~iport_ready & ~exception_ready & ~iport_error);
    assign dport_enable               = ~dport_ready & dmem_operation & ~dport_error;

    //--------------------------------------------------------------------------
    // Load Linked and Store Conditional logic
    //--------------------------------------------------------------------------
    /*
        From XUM project:

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

    always @(posedge clk) begin
        llsc_address <= (rst) ? 30'b0 : ( (dmem_read & mem_llsc) ? dmem_address[31:2] : llsc_address );
    end

    always @(posedge clk) begin
        if (rst) begin
            /*AUTORESET*/
            // Beginning of autoreset for uninitialized flops
            llsc_atomic <= 1'h0;
            // End of automatics
        end
        else if (dmem_read) begin
            llsc_atomic <= (mem_llsc) ? 1'b1 : llsc_atomic;
        end
        else if (id_eret | (~dmem_request_stall & dmem_write & (dmem_address[31:2] == llsc_address))) begin
            llsc_atomic <= 1'b0;
        end
        else begin
            llsc_atomic <= llsc_atomic;
        end
    end // always @ (posedge clk)
    // If atomic and using the same address: enable the write. Else, ignore.
    assign llsc_mem_write_mask = (mem_llsc & dmem_write & (~llsc_atomic | (dmem_address[31:2] != llsc_address)));

    //--------------------------------------------------------------------------
    // Map address and I/O ports
    //--------------------------------------------------------------------------
    assign iport_address = imem_address[31:0]; // full assign
    assign dport_address = dmem_address[31:0]; // full assign

    //--------------------------------------------------------------------------
    // Read instruction memory
    //--------------------------------------------------------------------------
    assign iport_wr = 4'b0000;  // DO NOT WRITE
    always @(*) begin
        imem_data = iport_data_i;  // simple
    end

    //--------------------------------------------------------------------------
    // Read from data data port.
    //--------------------------------------------------------------------------
    always @(*) begin
        if (dmem_byte) begin
            case (dmem_address[1:0])
                2'b00   : dmem_data_o = (dmem_sign_extend) ? { {24{dport_data_i[7]} },  dport_data_i[7:0] }   : {24'b0, dport_data_i[7:0]};
                2'b01   : dmem_data_o = (dmem_sign_extend) ? { {24{dport_data_i[15]} }, dport_data_i[15:8] }  : {24'b0, dport_data_i[15:8]};
                2'b10   : dmem_data_o = (dmem_sign_extend) ? { {24{dport_data_i[23]} }, dport_data_i[23:16] } : {24'b0, dport_data_i[23:16]};
                2'b11   : dmem_data_o = (dmem_sign_extend) ? { {24{dport_data_i[31]} }, dport_data_i[31:24] } : {24'b0, dport_data_i[31:24]};
                default : dmem_data_o = 32'hx;
            endcase // case (dmem_address[1:0])
        end
        else if (dmem_halfword) begin
            case (dmem_address[1])
                1'b0    : dmem_data_o = (dmem_sign_extend) ? { {16{dport_data_i[15]} }, dport_data_i[15:0] }   : {16'b0, dport_data_i[15:0]};
                1'b1    : dmem_data_o = (dmem_sign_extend) ? { {16{dport_data_i[31]} }, dport_data_i[31:16] }  : {16'b0, dport_data_i[31:16]};
                default : dmem_data_o = 32'hx;
            endcase // case (dmem_address[1])
        end
        else if (mem_llsc & dmem_write) begin
            dmem_data_o = (llsc_atomic & (dmem_address[31:2] == llsc_address)) ? 32'h0000_0001 : 32'h0000_0000;
        end
        else begin
            dmem_data_o = dport_data_i;
        end
    end // always @ (*)

    //--------------------------------------------------------------------------
    // Write to data port
    // Format data:
    // byte : {b, b, b, b}
    // half : {h, h}
    // word : {w}
    //
    // Modify to implement Reverse Endian
    //--------------------------------------------------------------------------
    always @(*) begin
        dport_wr = 4'b0000;
        if (write_enable) begin
             dport_wr[3] = (dmem_byte & (dmem_address[1:0] == 2'b11)) | (dmem_halfword & dmem_address[1])  | data_word;
             dport_wr[2] = (dmem_byte & (dmem_address[1:0] == 2'b10)) | (dmem_halfword & dmem_address[1])  | data_word;
             dport_wr[1] = (dmem_byte & (dmem_address[1:0] == 2'b01)) | (dmem_halfword & ~dmem_address[1]) | data_word;
             dport_wr[0] = (dmem_byte & (dmem_address[1:0] == 2'b00)) | (dmem_halfword & ~dmem_address[1]) | data_word;
        end
    end

    assign dport_data_o[31:24] = (dmem_byte) ? dmem_data_i[7:0] : ((dmem_halfword) ? dmem_data_i[15:8] : dmem_data_i[31:24]);
    assign dport_data_o[23:16] = (dmem_byte | dmem_halfword) ? dmem_data_i[7:0] : dmem_data_i[23:16];
    assign dport_data_o[15:8]  = (dmem_byte) ? dmem_data_i[7:0]: dmem_data_i[15:8];
    assign dport_data_o[7:0]   = dmem_data_i[7:0];
endmodule // antares_load_store_unit
//==================================================================================================
//  Filename      : antares_memwb_register.v
//  Created On    : Sat Sep  5 21:41:57 2015
//  Last Modified : Sat Nov 07 12:10:59 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Pipeline register: MEM -> WB
//==================================================================================================

module antares_memwb_register (
                               input wire             clk,                   // main clock
                               input wire             rst,                   // main reset
                               input wire [31:0]      mem_read_data,         // data from Memory
                               input wire [31:0]      mem_alu_data,          // data from ALU
                               input wire [4:0]       mem_gpr_wa,            // GPR write enable
                               input wire             mem_mem_to_gpr_select, // select MEM/ALU to GPR
                               input wire             mem_gpr_we,            // GPR write enable
                               input wire             mem_flush,
                               input wire             mem_stall,             // stall MEM stage
                               input wire             wb_stall,              // stall WB stage
                               output reg [31:0] wb_read_data,          // data from Memory
                               output reg [31:0] wb_alu_data,           // data from ALU
                               output reg [4:0]  wb_gpr_wa,             // GPR write address
                               output reg        wb_mem_to_gpr_select,  // select MEM/ALU to GPR
                               output reg        wb_gpr_we              // GPR write enable
                               );

    //--------------------------------------------------------------------------
    // Propagate signals
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        wb_read_data         <= (rst) ? 32'b0 : ((wb_stall) ? wb_read_data                                           : mem_read_data);
        wb_alu_data          <= (rst) ? 32'b0 : ((wb_stall) ? wb_alu_data                                            : mem_alu_data);
        wb_gpr_wa            <= (rst) ? 5'b0  : ((wb_stall) ? wb_gpr_wa                                              : mem_gpr_wa);
        wb_mem_to_gpr_select <= (rst) ? 1'b0  : ((wb_stall) ? wb_mem_to_gpr_select                                   : mem_mem_to_gpr_select);
        wb_gpr_we            <= (rst) ? 1'b0  : ((wb_stall) ? wb_gpr_we            : ((mem_stall | mem_flush) ? 1'b0 : mem_gpr_we));
    end
endmodule // antares_memwb_register
//==================================================================================================
//  Filename      : antares_multiplier.v
//  Created On    : Wed Sep  2 22:05:36 2015
//  Last Modified : Sat Nov 07 12:11:51 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : 32 x 32 pipelined multiplier.
//                  For signed operations: invert, perform unsigned mult, set result sign.
//==================================================================================================

module antares_multiplier(
                          input wire         clk,            // clock
                          input wire         rst,            // reset
                          input wire [31:0]  mult_input_a,   // Data
                          input wire [31:0]  mult_input_b,   // Data
                          input wire         mult_signed_op, // Unsigned (0) or signed operation (1)
                          input wire         mult_enable_op, // Signal a valid operation
                          input wire         mult_stall,     // Freeze the pipeline
                          input wire         flush,          // Flush the pipeline
                          output wire [63:0] mult_result,    // Result
                          output wire        mult_active,    // Active operations @ pipeline
                          output wire        mult_ready      // Valid data on output port (result)
                          );

    //--------------------------------------------------------------------------
    // Signal Declaration: reg
    //--------------------------------------------------------------------------
    reg [32:0]    A;
    reg [32:0]    B;
    reg [31:0]    result_ll_0;
    reg [31:0]    result_lh_0;
    reg [31:0]    result_hl_0;
    reg [31:0]    result_hh_0; // keep only 32 bits (ISE Warning)
    reg [31:0]    result_ll_1;
    reg [31:0]    result_hh_1; // keep only 32 bits (ISE Warning)
    reg [32:0]    result_mid_1;
    reg [63:0]    result_mult;

    reg           active0;     // Pipeline the enable signal, so HDU can know if a valid operation is in the pipeline
    reg           active1;
    reg           active2;
    reg           active3;
    reg           sign_result0;
    reg           sign_result1;
    reg           sign_result2;
    reg           sign_result3;

    ///-------------------------------------------------------------------------
    // Signal Declaration: wire
    //--------------------------------------------------------------------------
    wire          sign_a;
    wire          sign_b;
    wire [47:0]   partial_sum;
    wire [32:0]   a_sign_ext;
    wire [32:0]   b_sign_ext;

    //--------------------------------------------------------------------------
    // assignments
    //--------------------------------------------------------------------------
    assign sign_a      = (mult_signed_op) ? mult_input_a[31] : 1'b0;
    assign sign_b      = (mult_signed_op) ? mult_input_b[31] : 1'b0;
    assign a_sign_ext  = {sign_a, mult_input_a};
    assign b_sign_ext  = {sign_b, mult_input_b};
    assign partial_sum = {15'b0, result_mid_1} + {result_hh_1[31:0], result_ll_1[31:16]};
    assign mult_result = (sign_result3) ? -result_mult : result_mult;                         // Set true sign.
    assign mult_ready  = active3;
    assign mult_active = active0 | active1 | active2 | active3;                             // 4th stage holds the result

    //--------------------------------------------------------------------------
    // Implement the pipeline
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst | flush) begin
            /*AUTORESET*/
            // Beginning of autoreset for uninitialized flops
            A <= 33'h0;
            B <= 33'h0;
            active0 <= 1'h0;
            active1 <= 1'h0;
            active2 <= 1'h0;
            active3 <= 1'h0;
            result_hh_0 <= 32'h0;
            result_hh_1 <= 32'h0;
            result_hl_0 <= 32'h0;
            result_lh_0 <= 32'h0;
            result_ll_0 <= 32'h0;
            result_ll_1 <= 32'h0;
            result_mid_1 <= 33'h0;
            result_mult <= 64'h0;
            sign_result0 <= 1'h0;
            sign_result1 <= 1'h0;
            sign_result2 <= 1'h0;
            sign_result3 <= 1'h0;
            // End of automatics
        end
        else if(~mult_stall) begin
            // --- first stage
            // Change sign. Perform unsigned multiplication. Save the result sign.
            A            <= sign_a ? -a_sign_ext : a_sign_ext;
            B            <= sign_b ? -b_sign_ext : b_sign_ext;
            sign_result0 <= sign_a ^ sign_b;
            active0      <= mult_enable_op;
            // --- second stage
            result_ll_0  <= A[15:0]  *  B[15:0];       // 16 x 16
            result_lh_0  <= A[15:0]  *  B[32:16];      // 16 x 17
            result_hl_0  <= A[32:16] *  B[15:0];       // 17 x 16
            result_hh_0  <= A[31:16] *  B[31:16];      // 16 x 16
            sign_result1 <= sign_result0;
            active1      <= active0;
            // --- third stage
            result_ll_1  <= result_ll_0;
            result_hh_1  <= result_hh_0;
            result_mid_1 <= result_lh_0 + result_hl_0;      // sum mid
            sign_result2 <= sign_result1;
            active2      <= active1;
            // -- fourth stage
            result_mult  <= {partial_sum, result_ll_1[15:0]};
            sign_result3 <= sign_result2;
            active3      <= active2;
        end
    end

endmodule
//==================================================================================================
//  Filename      : antares_mux_2_1.v
//  Created On    : Mon Aug 31 21:12:26 2015
//  Last Modified : Sat Nov 07 12:13:45 2015
//  Revision      : 0.1
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : A 2-input wire multiplexer, with parameterizable width
//==================================================================================================

module antares_mux_2_1 #(parameter WIDTH = 32)
    (
     input wire [WIDTH-1:0]      in0,
     input wire [WIDTH-1:0]      in1,
     input wire                  select,
     output reg [WIDTH-1:0] out
    );

    always @(/*AUTOSENSE*/in0 or in1 or select) begin
        case (select)
            1'b0: out = in0;
            1'b1: out = in1;
        endcase // case (select)
    end // always @ (...

endmodule // antares_mux_2_1
//==================================================================================================
//  Filename      : antares_mux_4_1.v
//  Created On    : Mon Aug 31 23:14:22 2015
//  Last Modified : Sat Nov 07 12:14:18 2015
//  Revision      : 0.1
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : A 4-input wire multiplexer, with parameterizable width.
//==================================================================================================

module antares_mux_4_1 #(parameter WIDTH = 32)
    (
     input wire [1:0]            select,
     input wire [WIDTH-1:0]      in0,
     input wire [WIDTH-1:0]      in1,
     input wire [WIDTH-1:0]      in2,
     input wire [WIDTH-1:0]      in3,
     output reg [WIDTH-1:0] out
     );

    always @ ( /*AUTOSENSE*/in0 or in1 or in2 or in3 or select) begin
        case (select)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
        endcase // case (select)
    end // always @ (...

endmodule // antares_mux_4_1
//==================================================================================================
//  Filename      : antares_pc_register.v
//  Created On    : Tue Sep  1 10:21:01 2015
//  Last Modified : Sat Nov 07 12:14:50 2015
//  Revision      : 0.1
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Program Counter (PC)
//==================================================================================================


module antares_pc_register (
                            input wire             clk,
                            input wire             rst,
                            input wire [31:0]      if_new_pc,
                            input wire             if_stall,
                            output reg [31:0] if_pc
                            );

    always @ ( posedge clk ) begin
        if_pc <= (rst) ? `ANTARES_VECTOR_BASE_RESET : ((if_stall) ? if_pc : if_new_pc);
    end

endmodule // antares_pc_register
//==================================================================================================
//  Filename      : antares_reg_file.v
//  Created On    : Tue Sep  1 10:29:48 2015
//  Last Modified : Sat Nov 07 12:15:25 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : 32 General Purpose Registers (GPR)
//                  WARNING: This register file DO NOT HAVE A RESET.
//==================================================================================================

module antares_reg_file (
                         input wire         clk,
                         input wire [4:0]   gpr_ra_a,
                         input wire [4:0]   gpr_ra_b,
                         input wire [4:0]   gpr_wa,
                         input wire [31:0]  gpr_wd,
                         input wire         gpr_we,
                         output wire [31:0] gpr_rd_a,
                         output wire [31:0] gpr_rd_b
                         );

    // Register file of 32 32-bit registers. Register 0 is always 0
    reg [31:0]    registers [0:31];

    // Initialize all to zero
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) begin
            registers[i] <= 0;
        end
    end

    // Clocked write
    always @ ( posedge clk ) begin
        if(gpr_wa != 5'b0)
          registers[gpr_wa] <= (gpr_we) ? gpr_wd :  registers[gpr_wa];
    end

    // Combinatorial read (no delay). Register 0 is read as 0 always.
    assign gpr_rd_a = (gpr_ra_a == 5'b0) ? 32'b0 : registers[gpr_ra_a];
    assign gpr_rd_b = (gpr_ra_b == 5'b0) ? 32'b0 : registers[gpr_ra_b];

endmodule // antares_reg_file
//==================================================================================================
//  Filename      : antares_shifter.v
//  Created On    : Wed Sep  2 09:04:04 2015
//  Last Modified : Sat Nov 07 12:16:18 2015
//  Revision      : 1.0
//  Author        : Angel Terrones
//  Company       : Universidad Simón Bolívar
//  Email         : aterrones@usb.ve
//
//  Description   : Arithmetic/Loogic shifter.
//                  WARNING: shift_shamnt range is 0 -> 31
//==================================================================================================

module antares_shifter (
                        input wire [31:0]  shift_input_data,  // Input wire data
                        input wire [4:0]   shift_shamnt,      // Shift amount
                        input wire         shift_direction,   // 0: right, 1: left
                        input wire         shift_sign_extend, // 1: Signed operation
                        output wire [31:0] shift_result       // Result
                        );

    //-------------------------------------------------------------------------
    // Signal Declaration: reg
    //--------------------------------------------------------------------------
    reg [31:0]    input_inv;          // invert input wire for shift left
    reg [31:0]    result_shift_temp;  // shift result
    reg [31:0]    result_inv;         // invert output for shift left

    //-------------------------------------------------------------------------
    // Signal Declaration: wire
    //--------------------------------------------------------------------------
    wire          sign;
    wire [31:0]   operand;

    //-------------------------------------------------------------------------
    // assignments
    //-------------------------------------------------------------------------
    assign sign         = (shift_sign_extend) ? shift_input_data[31] : 1'b0;
    assign operand      = (shift_direction) ? input_inv : shift_input_data;
    assign shift_result = (shift_direction) ? result_inv : result_shift_temp;

    //-------------------------------------------------------------------------
    // invert data if the operation is SLL
    //-------------------------------------------------------------------------
    integer       index0;
    integer       index1;
    // first inversion: input wire data
    always @ (*) begin
        for (index0 = 0; index0 < 32; index0 = index0 + 1) begin
            input_inv[31 - index0] = shift_input_data[index0];
        end
    end
    // second inversion : output
    always @(*) begin
        for (index1 = 0; index1 < 32; index1 = index1 + 1)
          result_inv[31 - index1] = result_shift_temp[index1];
    end

    //--------------------------------------------------------------------------
    // the BIG multiplexer
    // Perform SRA. Sign depends if operation is SRA or SRL (shift_sign_extend)
    //--------------------------------------------------------------------------
    always @(*) begin
        case(shift_shamnt)
            5'd0    : result_shift_temp =               operand[31:0];
            5'd1    : result_shift_temp = { {1 {sign}}, operand[31:1] };
            5'd2    : result_shift_temp = { {2 {sign}}, operand[31:2] };
            5'd3    : result_shift_temp = { {3 {sign}}, operand[31:3] };
            5'd4    : result_shift_temp = { {4 {sign}}, operand[31:4] };
            5'd5    : result_shift_temp = { {5 {sign}}, operand[31:5] };
            5'd6    : result_shift_temp = { {6 {sign}}, operand[31:6] };
            5'd7    : result_shift_temp = { {7 {sign}}, operand[31:7] };
            5'd8    : result_shift_temp = { {8 {sign}}, operand[31:8] };
            5'd9    : result_shift_temp = { {9 {sign}}, operand[31:9] };
            5'd10   : result_shift_temp = { {10{sign}}, operand[31:10] };
            5'd11   : result_shift_temp = { {11{sign}}, operand[31:11] };
            5'd12   : result_shift_temp = { {12{sign}}, operand[31:12] };
            5'd13   : result_shift_temp = { {13{sign}}, operand[31:13] };
            5'd14   : result_shift_temp = { {14{sign}}, operand[31:14] };
            5'd15   : result_shift_temp = { {15{sign}}, operand[31:15] };
            5'd16   : result_shift_temp = { {16{sign}}, operand[31:16] };
            5'd17   : result_shift_temp = { {17{sign}}, operand[31:17] };
            5'd18   : result_shift_temp = { {18{sign}}, operand[31:18] };
            5'd19   : result_shift_temp = { {19{sign}}, operand[31:19] };
            5'd20   : result_shift_temp = { {20{sign}}, operand[31:20] };
            5'd21   : result_shift_temp = { {21{sign}}, operand[31:21] };
            5'd22   : result_shift_temp = { {22{sign}}, operand[31:22] };
            5'd23   : result_shift_temp = { {23{sign}}, operand[31:23] };
            5'd24   : result_shift_temp = { {24{sign}}, operand[31:24] };
            5'd25   : result_shift_temp = { {25{sign}}, operand[31:25] };
            5'd26   : result_shift_temp = { {26{sign}}, operand[31:26] };
            5'd27   : result_shift_temp = { {27{sign}}, operand[31:27] };
            5'd28   : result_shift_temp = { {28{sign}}, operand[31:28] };
            5'd29   : result_shift_temp = { {29{sign}}, operand[31:29] };
            5'd30   : result_shift_temp = { {30{sign}}, operand[31:30] };
            5'd31   : result_shift_temp = { {31{sign}}, operand[31:31] };
            default : result_shift_temp = 32'bx;
        endcase
    end

endmodule // antares_shifter
