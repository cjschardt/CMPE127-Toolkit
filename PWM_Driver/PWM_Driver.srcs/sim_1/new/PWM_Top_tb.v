`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/01/2018 03:15:28 PM
// Design Name: 
// Module Name: PWM_Top_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module PWM_Top_tb;

parameter MAX_CYCLE = 'd1_000_000;

reg clk_tb, rst_tb, ld_tb;
reg [7:0] duty0_tb, period0_tb, duty1_tb, period1_tb, duty2_tb, period2_tb, duty3_tb, period3_tb;
wire pwm0_tb, pwm1_tb, pwm2_tb, pwm3_tb, pwm4_tb, pwm5_tb, pwm6_tb, pwm7_tb, pwm8_tb, pwm9_tb, pwmA_tb, pwmB_tb, pwmC_tb, pwmD_tb, pwmE_tb, pwmF_tb; //red_tb, blue_tb, green_tb;
wire [15:0] LED_tb;

PWM_Top DUT0 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty0_tb), .period(period0_tb), .signal(pwm0_tb));//, .LED16_R(red_tb), .LED16_G(green_tb), .LED16_B(blue_tb), .LED(LED_tb));
PWM_Top DUT1 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty1_tb), .period(period0_tb), .signal(pwm1_tb));
PWM_Top DUT2 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty2_tb), .period(period0_tb), .signal(pwm2_tb));
PWM_Top DUT3 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty3_tb), .period(period0_tb), .signal(pwm3_tb));
PWM_Top DUT4 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty0_tb), .period(period1_tb), .signal(pwm4_tb));
PWM_Top DUT5 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty1_tb), .period(period1_tb), .signal(pwm5_tb));
PWM_Top DUT6 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty2_tb), .period(period1_tb), .signal(pwm6_tb));
PWM_Top DUT7 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty3_tb), .period(period1_tb), .signal(pwm7_tb));
PWM_Top DUT8 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty0_tb), .period(period2_tb), .signal(pwm8_tb));
PWM_Top DUT9 (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty1_tb), .period(period2_tb), .signal(pwm9_tb));
PWM_Top DUTA (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty2_tb), .period(period2_tb), .signal(pwmA_tb));
PWM_Top DUTB (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty3_tb), .period(period2_tb), .signal(pwmB_tb));
PWM_Top DUTC (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty0_tb), .period(period3_tb), .signal(pwmC_tb));
PWM_Top DUTD (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty1_tb), .period(period3_tb), .signal(pwmD_tb));
PWM_Top DUTE (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty2_tb), .period(period3_tb), .signal(pwmE_tb));
PWM_Top DUTF (.clk_100MHz(clk_tb), .reset_button(rst_tb), .load_button(ld_tb), .duty_cycle(duty3_tb), .period(period3_tb), .signal(pwmF_tb));

integer i;
always begin
    clk_tb = 0;
    #1;
    clk_tb = ~clk_tb; 
    #1;
end
                 
initial begin
    duty0_tb = 64;
    duty1_tb = 128;
    duty2_tb = 191;
    duty3_tb = 255;
    period0_tb = 0;
    period1_tb = 64;
    period2_tb = 128;
    period3_tb = 191; 
    rst_tb = 0;
    ld_tb = 0;
    #5;
    rst_tb = 1;
    #5;
    rst_tb = 0;
    #5;
    ld_tb = 1;
    #5;
    ld_tb = 0;
    #5;
    for (i=0; i<MAX_CYCLE; i=i+1) begin
        #1;
    end
    /*
    duty_tb = 8'd191;   
    period_tb = 8'd255;
    rst_tb = 1;
    #5;
    rst_tb = 0;
    #5;
    ld_tb = 1;
    #5;
    ld_tb = 0;
    #5;
    for (i=0; i<MAX_CYCLE; i=i+1) begin
        #1;
    end
    duty_tb = 8'd127;   
    period_tb = 8'd255;
    rst_tb = 1;
    #5;
    rst_tb = 0;
    #5;
    ld_tb = 1;
    #5;
    ld_tb = 0;
    #5;
    for (i=0; i<MAX_CYCLE; i=i+1) begin
        #1;
    end
    duty_tb = 8'd63;   
    period_tb = 8'd255;
    rst_tb = 1;
    #5;
    rst_tb = 0;
    #5;
    ld_tb = 1;
    #5;
    ld_tb = 0;
    #5;
    for (i=0; i<MAX_CYCLE; i=i+1) begin
        #1;
    end*/
$finish; 
end
endmodule
