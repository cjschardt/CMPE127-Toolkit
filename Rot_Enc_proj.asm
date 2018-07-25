#################################################
# MIPS program for Rotary Encoder / LED Project #
# Engineer: Colin Schardt			 #
# SJSU CMPE Summer Internship 2018		 #
# Motherboard Division				 #
# Last Modified: July 10th, 2018		 #
#################################################

#################################################
# Variable Declarations				 #
#################################################
.eqv led_val_arr_ptr	$s0
.eqv led_val		$s1

#################################################
# Data Declarations				 #
#################################################
# RAM Location of LED values
.data 0x2000
	led_val_arr:	.space	4320 	# 1080 stored integers, or 360 RGB values stored (red, green, blue, repeat).
.data 0x30e0
	curr_led_val:	.space	12	# 3 stored integers, or 1 RGB value
.data 0x30ec
	saved_led_val:	.space	12	# 3 stored integers, or 1 RGB value

# Rotary Encoder Hardware Location
.data 0x1000
	rotary_encoder:	.space	4	# 1 stored integer, stores input value from rotary encoder
.data 0x1004
	encoder_button:	.space	1	# button input from rotary encoder.
.data 0x1008
	encoder_switch:	.space	1	# switch input from rotary encoder.
	
#################################################
# Macros					 #
#################################################
.text
.macro update_led_val (%mem_location)
	lw led_val_arr_ptr, rotary_encoder		# load value from rotary encoder into led_val_array_ptr	
	li $t0, 12
	mul led_val_arr_ptr, led_val_arr_ptr, $t0	# multiply value from the rotary encoder by 12 to determine correct address
	lw led_val, led_val_arr(led_val_arr_ptr)	# load the value from that address of led_val_arr into led_val
	li $t0, 0					
	sw led_val, %mem_location($t0)			# store value into first address of saved_led_val
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increment led_val_arr_ptr to next address
	lw led_val, led_val_arr(led_val_arr_ptr)	# load data to led_val and transfer it to next address of saved_led_val
	li $t0, 4
	sw led_val, %mem_location($t0)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	lw led_val, led_val_arr(led_val_arr_ptr)	# transfer data from 3rd address of led_val_arr to saved_led_val
	li $t0, 8
	sw led_val, %mem_location($t0)
.end_macro
	
#################################################
# Main						 #
#################################################

start:
	jal load_colors
main:
#	li $t0, 330					#temp code for testing
#	sw $t0, rotary_encoder($0)			#temp code for testing
#	li $t0, 1					#temp code for testing
#	sw $t0, encoder_button($0)			#temp code for testing
#	sw $t0, encoder_switch($0)			#temp code for testing
	update_led_val (curr_led_val)
	li $t0, 1
	lw $t1, encoder_button($0)
	bne $t1, $t0, end				# if button is pressed, save the current color 
	update_led_val (saved_led_val)
end:
	j main

#################################################
# Functions to load LED values to RAM		 #
# Loads 1080 8-bit RGB values to be used by	 #
# the RGB LEDs.					 # 
#################################################

load_colors:
	li led_val_arr_ptr, 0				# initialize led_var_arr_ptr
	li led_val, 0					# initialize led_val
	li $t0, 720					# set exit parameter for the first loop
	
red_to_yellow:
	beq $t0, led_val_arr_ptr, done0			# begin loop
	li $t1, 255
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 255 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increments led_val_arr_ptr to the nextm free space
	sw led_val, led_val_arr(led_val_arr_ptr)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr 4
	li $t1, 0
	sw $t1, led_val_arr(led_val_arr_ptr) 		# stores 0 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	addi led_val, led_val, 4			# increments led_val by 4
	j red_to_yellow					# jump to begining of loop

done0:
	li led_val, 255					# set led_val to 255
	li $t0, 1440					# set parameter to leave second loop
	
yellow_to_green:
	beq $t0, led_val_arr_ptr, done1			# begin loop
	sw led_val, led_val_arr(led_val_arr_ptr)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increments led_val_arr_ptr to the nextm free space
	li $t1, 255
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 255 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	li $t1, 0
	sw $t1, led_val_arr(led_val_arr_ptr) 		#stores 0 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	addi led_val, led_val, -4			# increments led_val by -4
	j yellow_to_green				# jump to begining of loop

done1:
	li led_val, 0
	li $t0, 2160
	
green_to_cyan:
	beq $t0, led_val_arr_ptr, done2			# begin loop
	li $t1, 0
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 0 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increments led_val_arr_ptr to the nextm free space
	li $t1, 255
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 255 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	sw led_val, led_val_arr(led_val_arr_ptr) 	#stores led_val into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	addi led_val, led_val, 4			# increments led_val by 4
	j green_to_cyan					# jump to begining of loop

done2:
	li $s1, 255
	li $t0, 2880
	
cyan_to_blue:
	beq $t0, led_val_arr_ptr, done3			# begin loop
	li $t1, 0
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 0 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increments led_val_arr_ptr to the nextm free space
	sw led_val, led_val_arr(led_val_arr_ptr)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	li $t1, 255
	sw $t1, led_val_arr(led_val_arr_ptr) 		# stores 255 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	addi led_val, led_val, -4			# increments led_val by -4
	j cyan_to_blue					# jump to begining of loop
	
done3:
	li $s1, 0
	li $t0, 3600

blue_to_magenta:
	beq $t0, led_val_arr_ptr, done4			# begin loop
	sw led_val, led_val_arr(led_val_arr_ptr)	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increments led_val_arr_ptr to the nextm free space
	li $t1, 0
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 0 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	li $t1, 255
	sw $t1, led_val_arr(led_val_arr_ptr)	 	# stores 255 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	addi led_val, led_val, 4			# increments led_val by 4
	j blue_to_magenta				# jump to begining of loop

done4:
	li $s1, 255
	li $t0, 4320

magenta_to_red:
	beq $t0, led_val_arr_ptr, done5			# begin loop
	li $t1, 255
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 255 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4	# increments led_val_arr_ptr to the nextm free space
	li $t1, 0
	sw $t1, led_val_arr(led_val_arr_ptr)		# stores 0 into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	sw led_val, led_val_arr(led_val_arr_ptr) 	# stores led_val into led_val_arr(led_val_arr_ptr)
	addi led_val_arr_ptr, led_val_arr_ptr, 4
	addi led_val, led_val, -4			# increments led_val by 4
	j magenta_to_red				# jump to begining of loop
	
done5:
	jr $ra

