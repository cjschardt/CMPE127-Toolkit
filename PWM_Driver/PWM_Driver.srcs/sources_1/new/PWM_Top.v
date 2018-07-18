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
    input clk_100MHz, rst, rst_button, ld_button,
    input [7:0] duty_cycle, frequency,
    output pwm_signal      
    );
    
    wire [31:0] freq_conc;
    wire signal, clk_5KHz, dummy, rst_button_db, ld_button_db;
    assign freq_conc = frequency*'d1000;
    
    button_debouncer RSTDB (.clk(clk_5KHz), .button(rst_button), .debounced_button(rst_button_db));
    
    button_debouncer LDDB (.clk(clk_5KHz), .button(ld_button), .debounced_button(ld_button_db));
    
    PWM_Driver PWMD (.clk(clk_100MHz), .rst(rst_button_db), .ld(ld_button_db), .duty(duty_cycle), .freq(freq_conc), .signal(signal));
    
    clk_gen     CLK     (clk_100MHz, rst, dummy, clk_5KHz);
    
    assign pwm_signal = signal;
endmodule
