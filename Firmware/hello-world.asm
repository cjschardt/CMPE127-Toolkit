.text

main:
    nop
    li $sp, 0x2ffc
    li $v0, 0
    sw $v0, vga_position
    
    la $a0, prompt
    jal PUTS
end:
    j end
    nop
    
PUTS:
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
newline_return:
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
    j       newline_return
puts_return:
    sw      VGA_OFFSET, vga_position
    jr      $ra

# RAM location
.data 0x2000
    prompt:         .asciiz "Hello, World!\n"
    vga_position:   .word   0
# VGA RAM memory location
.data 0x1000
    vga_address: .space 900
# Keyboard hardware location
.data 0x1781
    keyboard_address: .space 1
