######################################
#     Author: Khalil A. Estell       #
#   Last Modified: March 8th 2018    #
######################################
.text

.macro PUSH(%register)
    subi $sp, $sp, 4
    sw  %register, ($sp)
.end_macro

.macro POP(%register)
    lw  %register ($sp)
    addi $sp, $sp, 4
.end_macro

main:
    nop
    ### Initializing START
    li $sp, 0x2ffc
    li $v0, 0
    sw $v0, vga_position
    ### Initialization ENDING
    # Writing Prompt to screen
    la $a0, prompt
    jal PUTS
    # Read from Keyboard and print to screen function
    la $a0, name
    li $a1, 32
    jal FGETS
    # Write first section
    la $a0, reply_start
    jal PUTS
    # Write name to screen
    la $a0, name
    jal PUTS
    # Write end of reply
    la $a0, reply_end
    jal PUTS
    # Write final prompt
    la $a0, final_prompt
    jal PUTS
end:
    j end

#==========================
# Put String Function
#==========================
PUTS:
    .eqv  CURRENT_ASCII         $s0
    .eqv  STRING_ADDR           $s1

    # Push Return address and saved registers to stack
    PUSH($ra)
    PUSH($s0)
    PUSH($s1)
    add     STRING_ADDR, $zero, $a0
    puts_write_to_screen:
        # Load first byte from string
        lbu     CURRENT_ASCII, (STRING_ADDR)
        addi    STRING_ADDR, STRING_ADDR, 1
        beq     CURRENT_ASCII, $zero, puts_return
        add     $a0, $zero, CURRENT_ASCII
        jal     PUTCHAR
        j       puts_write_to_screen
    puts_return:
    # Pop Return address and saved registers to registers
        POP($s1)
        POP($s0)
        POP($ra)
        jr      $ra

#==========================
# Put Char Function
#==========================
PUTCHAR:
    .eqv  VGA_BASE              $t0
    .eqv  VGA_OFFSET            $t1
    .eqv  VGA_FINAL_ADDRESS     $t2
    .eqv  TEMP_VAR              $t3
    .eqv  CHAR_TO_PRINT         $a0
    # Function start
    la      VGA_BASE,   vga_address
    lw      VGA_OFFSET, vga_position
    beq     CHAR_TO_PRINT, $zero, putchar_return
    beq     CHAR_TO_PRINT, 0xF7, putchar_backspace
    beq     CHAR_TO_PRINT, 0xEB, putchar_left_key
    bne     CHAR_TO_PRINT, '\n', putchar_not_newline_char
    ### address = (address + 80) - (address % 80);
    li      TEMP_VAR, 80
    div     VGA_OFFSET, TEMP_VAR
    mfhi    TEMP_VAR
    addi    VGA_OFFSET, VGA_OFFSET, 80
    sub     VGA_OFFSET, VGA_OFFSET, TEMP_VAR

    putchar_not_newline_char:
        add     VGA_FINAL_ADDRESS, VGA_BASE, VGA_OFFSET
        sb      CHAR_TO_PRINT, (VGA_FINAL_ADDRESS)
        li      TEMP_VAR, ' '
        sb      TEMP_VAR, 1(VGA_FINAL_ADDRESS)
        j       putchar_return
    putchar_backspace:
    putchar_left_key:
        subi    VGA_OFFSET, VGA_OFFSET, 1
        add     VGA_FINAL_ADDRESS, VGA_BASE, VGA_OFFSET
        li      TEMP_VAR, ' '
        sb      TEMP_VAR, (VGA_FINAL_ADDRESS)
        sw      VGA_OFFSET, vga_position
        jr      $ra
    putchar_return:
        addi    VGA_OFFSET, VGA_OFFSET, 1
        sw      VGA_OFFSET, vga_position
        jr      $ra

FGETS:
    .eqv STRING_BUFFER_ARG      $a0
    .eqv LIMIT_ARG              $a1
    .eqv READY                  $t1
    .eqv STRING_BUFFER          $s0
    .eqv LIMIT                  $s1
    .eqv CHAR                   $s2
    .eqv ORIGINAL_STRING_ADDR   $s3
    # Push Return address and saved registers to stack
    PUSH    ($ra)
    PUSH    ($s0)
    PUSH    ($s1)
    PUSH    ($s2)
    PUSH    ($s3)

    li      CHAR, 0x0
    add     STRING_BUFFER, $zero, STRING_BUFFER_ARG
    add     ORIGINAL_STRING_ADDR, $zero, STRING_BUFFER_ARG
    # Making space for null character
    subi    LIMIT, LIMIT_ARG, 1

    keyboard_polling:
        lw      READY, keyboard_ready
        bne     READY, 1, keyboard_polling
        lbu     CHAR, keyboard_ascii
        # If at limit, bypass printing character
        beq     CHAR, 0xF7, fgets_backspace
        beq     CHAR, 0xEB, fgets_left_key
        beq     LIMIT, 0, fgets_decision
        add     $a0, $zero, CHAR
        jal     PUTCHAR
    fgets_decision:
        beq     CHAR, '\n', fgets_end
        # If this point is reached and the limit has
        # been met return to polling the keyboard,
        # do not store the next key
        beq     LIMIT, 0, keyboard_polling
        sb      CHAR, (STRING_BUFFER)
        addi    STRING_BUFFER, STRING_BUFFER, 1
        subi    LIMIT, LIMIT, 1
        j       keyboard_polling
    fgets_left_key:
    fgets_backspace:
        # check if you are already at the beginning of the line
        # if so, go back to polling the keyboard
        beq     ORIGINAL_STRING_ADDR, STRING_BUFFER, keyboard_polling
        # if not add back 1 to limit
        addi    LIMIT, LIMIT, 1
        # move the string pointer back 1
        subi    STRING_BUFFER, STRING_BUFFER, 1
        # send backspace characters
        add     $a0, $zero, CHAR
        jal     PUTCHAR
        j keyboard_polling
    fgets_end:
        # place a string terminator at the end of string
        li      CHAR, 0x0
        sb      CHAR, (STRING_BUFFER)
        # Restore saved variables
        POP     ($s3)
        POP     ($s2)
        POP     ($s1)
        POP     ($s0)
        POP     ($ra)
        jr      $ra

# =====================================================
# RAM location
# =====================================================
.data 0x2000
    prompt:         .asciiz "What is your name: "
    reply_start:    .asciiz "\nHello, "
    reply_end:      .asciiz ". Nice to meet you.\n\n"
    final_prompt:   .asciiz "PROGRAM ENDING..."
    vga_position:   .word   0
    name:           .space  32
    # name:           .asciiz "<INSERT NAME HERE>"

# VGA RAM memory location
.data 0x1000
    vga_address: .space 1920
# Keyboard hardware location
.data 0x1784
    keyboard_ascii: .word 0xF7
.data 0x1788
    keyboard_ready: .word 1
