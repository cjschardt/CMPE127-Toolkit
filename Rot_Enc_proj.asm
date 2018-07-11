#################################################
# MIPS program for Rotary Encoder / LED Project #
# Engineer: Colin Schardt			#
# SJSU CMPE Summer Internship 2018		#
# Motherboard Division				#
# Last Modified: July 10th, 2018		#
#################################################

#################################################
# Variable Declarations				#
#################################################
# led_val_arr_ptr = $s0
# led_val = $s1

#################################################
# Data Declarations				#
#################################################
# RAM Location of LED values
.data 0x2000
	led_val_arr:	.space	4320 	# 1080 stored integers, or 360 RGB values stored (red, green, blue, repeat).
.data 0x30e0
	curr_led_val:	.space	12	# 3 stored integers, or 1 RGB value
.data 0x30ec
	saved_led_val: .space	12	# 3 stored integers, or 1 RGB value

# Rotary Encoder Hardware Location
.data 0x1000
	rotary_encoder:	.space	4	# 1 stored integer, stores input value from rotary encoder
.data 0x1004
	encoder_button:	.space	1	# button input from rotary encoder. Will likely have to change since I dont want button signal stored in memory.
.data 0x1008
	encoder_switch:	.space	1	# switch inout from rotary encoder. Will likely change since I dont want switch signal stored in memory.
	
#################################################
# Main						#
#################################################
.text
start:
	jal load_colors
main:
#	li $t0, 330			#temp code for testing
#	sw $t0, rotary_encoder($0)	#temp code for testing
#	li $t0, 1			#temp code for testing
#	sw $t0, encoder_button($0)	#temp code for testing
#	sw $t0, encoder_switch($0)	#temp code for testing
	jal ld_out_color
	li $t0, 1
	lw $t1, encoder_button($0)
	bne $t1, $t0, end		# if button is pressed, save the current color 
	jal save_color
end:
	j main

#################################################
# Functions to load LED values to RAM		#
# Loads 1080 8-bit RGB values to be used by the #
# RGB LEDs.					# 
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
	sw $t1, led_val_arr($s0) 	# stores 255 into led_val_arr(led_val_arr_ptr)
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
	sw $t1, led_val_arr($s0) 	# stores 255 into led_val_arr(led_val_arr_ptr)
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
	sw $s1, led_val_arr($s0) 	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi $s0, $s0, 4
	addi $s1, $s1, -4		# increments led_val by 4
	j magenta_to_red		# jump to begining of loop
	
done5:
	jr $ra
	
#################################################
# Functions to select color			#
#################################################

get_color_rot:
	lw $s0, rotary_encoder		# load value from the rotary encoder to $s0
	li $t0, 12
	mul $s0, $s0, $t0		# multiply the value of $s0 by 12
	lw $s1, led_val_arr($s0)	# load red value into $s1 $ $s4
	lw $s4, led_val_arr($s0)
	addi $s0, $s0, 4
	lw $s2, led_val_arr($s0)	# load green value into $s2 & $s5
	lw $s5, led_val_arr($s0)
	addi $s0, $s0, 4
	lw $s3, led_val_arr($s0)	# load blue value into $s3 & $s6
	lw $s6, led_val_arr($s0)
	jr $ra
	
get_color_saved:
	li $t0, 0
	lw $s4, saved_led_val($t0)
	li $t0, 4
	lw $s5, saved_led_val($t0)
	li $t0, 8
	lw $s6, saved_led_val($t0)
	jr $ra
	
#################################################
# Function to save/select LED values		#
#################################################

save_color:
	lw $s0, rotary_encoder
	li $t0, 12
	mul $s0, $s0, $t0
	lw $t1, led_val_arr($s0)
	li $t0, 0
	sw $t1, saved_led_val($t0)
	addi $s0, $s0, 4
	lw $t1, led_val_arr($s0)
	li $t0, 4
	sw $t1, saved_led_val($t0)
	addi $s0, $s0, 4
	lw $t1, led_val_arr($s0)
	li $t0, 8
	sw $t1, saved_led_val($t0)
	jr $ra
	
ld_out_color:
	lw $s0, rotary_encoder
	li $t0, 12
	mul $s0, $s0, $t0
	lw $t1, led_val_arr($s0)
	li $t0, 0
	sw $t1, curr_led_val($t0)
	addi $s0, $s0, 4
	lw $t1, led_val_arr($s0)
	li $t0, 4
	sw $t1, curr_led_val($t0)
	addi $s0, $s0, 4
	lw $t1, led_val_arr($s0)
	li $t0, 8
	sw $t1, curr_led_val($t0)
	jr $ra

