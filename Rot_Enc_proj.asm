#################################################
# MIPS program for Rotary Encoder / LED Project #
# Engineer: Colin Schardt			#
# SJSU CMPE Summer Internship 2018		#
# Motherboard Division				#
#################################################

#################################################
# Variable Declarations				#
#################################################
# led_val_arr_ptr = $s0
# led_val = $s1

#################################################
# Data Declarations				#
#################################################
.data 0x2000
	led_val_arr:	.space	4320 		#1080 stored integers, or 360 RGB values stored (red, green, blue, repeat).
	
#################################################
# Main						#
#################################################
.text

main:
	jal load_colors
	j end
end:
	j end

#################################################
# Functions to load LED values			#
#################################################

load_colors:
	li $s0, 0			# initialize led_var_arr_ptr
	li $s1, 0			# initialize led_val
	li $t0, 720			# set exit parameter for the first loop
	
red_to_yellow:
	beq $t0, $s0, done0		# begin loop
	li $t1, 255
	sw $t1, led_val_arr($s0)	# stores 255 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4		# increments led_val_arr_ptr to the nextm free space
	sw $s1, led_val_arr($s0)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	li $t1, 0
	sw $t1, led_val_arr($s0) 	#stores 0 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, 4		# increments led_val by 4
	j red_to_yellow			# jump to begining of loop

done0:
	li $s1, 255			# set led_val to 255
	li $t0, 1440			# set parameter to leave second loop
	
yellow_to_green:
	beq $t0, $s0, done1		# begin loop
	sw $s1, led_val_arr($s0)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4		# increments led_val_arr_ptr to the nextm free space
	li $t1, 255
	sw $t1, led_val_arr($s0)	# stores 255 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	li $t1, 0
	sw $t1, led_val_arr($s0) 	#stores 0 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, -4		# increments led_val by -4
	j yellow_to_green		# jump to begining of loop

done1:
	li $s1, 0
	li $t0, 2160
	
green_to_cyan:
	beq $t0, $s0, done2		# begin loop
	li $t1, 0
	sw $t1, led_val_arr($s0)	# stores 0 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4		# increments led_val_arr_ptr to the nextm free space
	li $t1, 255
	sw $t1, led_val_arr($s0)	# stores 255 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	sw $s1, led_val_arr($s0) 	#stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, 4		# increments led_val by 4
	j green_to_cyan			# jump to begining of loop

done2:
	li $s1, 255
	li $t0, 2880
	
cyan_to_blue:
	beq $t0, $s0, done3		# begin loop
	li $t1, 0
	sw $t1, led_val_arr($s0)	# stores 0 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4		# increments led_val_arr_ptr to the nextm free space
	sw $s1, led_val_arr($s0)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	li $t1, 255
	sw $t1, led_val_arr($s0) 	#stores 255 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, -4		# increments led_val by -4
	j cyan_to_blue			# jump to begining of loop
	
done3:
	li $s1, 0
	li $t0, 3600

blue_to_magenta:
	beq $t0, $s0, done4		# begin loop
	sw $s1, led_val_arr($s0)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4		# increments led_val_arr_ptr to the nextm free space
	li $t1, 0
	sw $t1, led_val_arr($s0)	# stores 0 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	li $t1, 255
	sw $t1, led_val_arr($s0) 	#stores 255 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, 4		# increments led_val by 4
	j blue_to_magenta		# jump to begining of loop

done4:
	li $s1, 255
	li $t0, 4320

magenta_to_red:
	beq $t0, $s0, done5		# begin loop
	li $t1, 255
	sw $t1, led_val_arr($s0)	# stores 255 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4		# increments led_val_arr_ptr to the nextm free space
	li $t1, 0
	sw $t1, led_val_arr($s0)	# stores 0 into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	sw $s1, led_val_arr($s0) 	#stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, -4		# increments led_val by 4
	j magenta_to_red		# jump to begining of loop
	
done5:
	jr $ra
