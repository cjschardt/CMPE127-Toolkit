`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  SJSU
// Engineer: Colin Schardt
// 
// Create Date: 07/10/2018 07:06:16 PM
// Design Name: 
// Module Name: PWM_Driver
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


module PWM_Driver(
    input wire clk,
    input wire rst,
    input wire ld,
    input wire [7:0] duty,
    input wire [31:0] freq,
    output reg signal,
    output reg [31:0] counter
);
// ==================================
//// Internal Parameter Field
// ==================================
parameter CLK_FREQ = 'd50_000_000;                        // System clock is 50MHz
parameter MAX_FREQ = 'd100_000;
parameter COUNT_WIDTH = 32;
// ==================================
//// Registers
// ==================================
//reg [COUNT_WIDTH - 1:0] counter;
reg [COUNT_WIDTH - 1:0] freq_compare_value;
reg [COUNT_WIDTH - 1:0] duty_compare_value;
reg [COUNT_WIDTH - 1:0] time_on;
//reg [15:0] duty_shift;
// ==================================
//// Wires
// ==================================
// ==================================
//// Wire Assignments
// ==================================  
// ==================================
//// Modules
// ==================================
// ==================================
//// Behavioral Block
// ==================================
//integer duty_compare_value;
always @(posedge clk or posedge rst) 
    begin
        //duty_shift = {duty, 8'b0};
        if (rst) begin
            counter = 0;
            signal = 0;
            time_on = 0;
            freq_compare_value <= 0;
            //duty_compare_value <= 0;
        end
        else if (ld) begin
            if (freq > MAX_FREQ) begin
                freq_compare_value <= CLK_FREQ/MAX_FREQ;
            end
            else begin
                freq_compare_value <= CLK_FREQ/freq;
            end
            duty_compare_value <= 'hFF00/duty;
            if (duty == 'd0) begin
                time_on <= 0;
            end
            else begin
                time_on <= (freq_compare_value*'hFF)/duty_compare_value; 
            end
        end
        else begin
            if (counter >= freq_compare_value) begin
                counter = 0;
                signal = 1;
            end
            if (counter >= time_on) begin
                signal = 0;
            end
            counter = counter + 1;
        end    
    end   
endmodule
