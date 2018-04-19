.macro load_byte (%register, %address)
    add $t7, $zero, %address
    srl $t6, $t7, 2
    sll $t6, $t6, 2
    add $s0, $zero, 0x3
    and $t7, $t7, $s0
    lw %register, ($t6)
    sll $t7, $t7, 3
    srlv %register, %register, $t7
    li $t7, 0xFF
    and %register, %register, $t7
.end_macro

.macro store_byte (%register, %address)
    add $t7, $zero, %address
    and $t7, $t7, 0x3
    sll $t7, $t7, 3
    sllv %register, %register, $t7
    li $t6, 0xFF
    sllv $t6, $t6, $t7
    add $t7, $zero, %address
    srl $t7, $t7, 2
    sll $t7, $t7, 2
    lw $t5, ($t7)
    nor $t6, $t6, $t6
    and $t6, $t6, $t5 
    or %register, $t6, %register
    sw %register, ($t7)
.end_macro

.text

main:
    nop
    la $a0, prompt
    jal puts
    la $a0, reply_0
    jal puts
    la $a0, reply_1
    jal puts
    la $a0, newline
    jal puts
    la $a0, final_prompt
    jal puts
end:
    j end
 
puts:
    .eqv  CURRENT_ASCII         $t0
    .eqv  VGA_BASE              $t1
    .eqv  TEMP_VAR              $t2
    .eqv  VGA_OFFSET            $t3
    .eqv  VGA_FINAL_ADDRESS     $t4
    .eqv  STRING_ADDR           $a0

    la      VGA_BASE, vga_address
    lw      VGA_OFFSET, vga_position
puts_write_to_screen:
    #load_byte(CURRENT_ASCII, STRING_ADDR)
    lb   CURRENT_ASCII, (STRING_ADDR)
    addi    STRING_ADDR, STRING_ADDR, 1
    beq     CURRENT_ASCII, '\n', puts_newline_detected
    beq     CURRENT_ASCII, $zero, puts_return
    # Calculate the final VGA address = VGA_BASE + OFFSET
    add     VGA_FINAL_ADDRESS, VGA_BASE, VGA_OFFSET
    # Store CURRENT_ASCII at VGA ram address.
    #store_byte(CURRENT_ASCII, VGA_FINAL_ADDRESS)
    sb CURRENT_ASCII, (VGA_FINAL_ADDRESS)
    # Increment VGA_OFFSET by 1
    addi    VGA_OFFSET, VGA_OFFSET, 1

    j       puts_write_to_screen
puts_newline_detected:
    #address = (address + 80) - (address % 80);
    li      TEMP_VAR, 80
    div     VGA_OFFSET, TEMP_VAR
    mfhi    TEMP_VAR
    addi    VGA_OFFSET, VGA_OFFSET, 80
    sub     VGA_OFFSET, VGA_OFFSET, TEMP_VAR
    j       puts_write_to_screen
puts_return:
    sw      VGA_OFFSET, vga_position
    jr      $ra

# RAM location
.data 0x2000
    prompt:         .asciiz "What is your name> "
    reply_0:        .asciiz "Hello, "
    reply_1:        .asciiz ". Nice to meet you.\n\n"
    final_prompt:   .asciiz "PROGRAM END..."
    newline:        .asciiz "\n"
    vga_position:   .word   0
    name:           .space  32 
# VGA RAM memory location
.data 0x1000
    vga_address: .space 900
# Keyboard hardware location
.data 0x1781
    keyboard_address: .space 1
