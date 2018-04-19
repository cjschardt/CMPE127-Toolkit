.text

main:
	li $v0, 7
	li $v1, 2
	add $t1, $v0, $v1
	sub $t2, $v0, $v1
	and $t3, $v0, $v1
	or  $t4, $v0, $v1
	xor $t5, $v0, $v1
	addi $t6, $v0, 100
	andi  $s0, $v0, 3
	ori   $s1, $v0, 0x1000
	xori  $s2, $v0, 0x1111
	li    $s3, 5
	li    $s4, 4
branch_here:
	addi $s4, $s4, 1
	beq $s3, $s4, branch_here
	li    $s3, 1
	li    $s4, 5
ble_branch:
	sub   $s4, $s4, $s3
	bgtz   $s4, ble_branch	
	sw    $v0, 0x1000
	lw    $s5, 0x1000
	sb    $v1, 0x1001
	
	li    $s5, 0xAABBCCDD
	sw    $s5, 0x1004
	lb    $a0, 0x1004
	lb    $a1, 0x1005
	lb    $a2, 0x1006
	lb    $a3, 0x1007
	
	jal  skip_addi
	nop
	nop
end:
	j end
	nop
	
skip_addi:
	li $k0, 0x500	
	jr $ra