`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: SJSU
// Engineer: Colin Schardt
// 
// Create Date: 07/17/2018 06:06:04 PM
// Design Name: 
// Module Name: PWM_Top
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


module PWM_Top(
    input wire clk_100MHz, reset_button, load_button,
    input wire [7:0] duty_cycle, period,
    output wire signal, LED16_R, LED16_G, LED16_B, LED17_R, LED17_G, LED17_B,
    output wire [15:0] LED
    );

    wire pwm_clk, pwm_reset, pwm_load;
    //wire [7:0] pwm_duty, pwm_period;
    //wire pwm_signal;
    
    assign pwm_clk = clk_100MHz;
    assign pwm_reset = reset_button;
    assign pwm_load = load_button;
    
    PWM_Driver PWMGEN (.sys_clk(pwm_clk), .reset(pwm_reset), .load(pwm_load), .data({duty_cycle, period}), .signal(signal));
    
    assign LED = {period, duty_cycle};
    //assign signal = pwm_signal;
    assign LED17_R = reset_button;
    assign LED17_G = 0;
    assign LED17_B = 0;
    assign LED16_R = 0;
    assign LED16_G = load_button;
    assign LED16_B = 0;
    
    
endmodule