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
);
// ==================================
//// Internal Parameter Field
// ==================================
parameter CLK_FREQ = 'd50_000_000;                        // System clock is 50MHz
parameter MAX_FREQ = 'd100_000;                           // Max PWM frequency is 100kHz
parameter COUNT_WIDTH = 32;
// ==================================
//// Registers
// ==================================
reg [COUNT_WIDTH - 1:0] counter;
reg [COUNT_WIDTH - 1:0] freq_compare_value;
reg [COUNT_WIDTH - 1:0] duty_compare_value;
reg [COUNT_WIDTH - 1:0] time_on;
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

always @(posedge clk or posedge rst) 
    begin
        if (rst) begin                                                      // Reset values
            counter = 0;
            signal = 0;
            time_on = 0;
            freq_compare_value <= 0;
        end
        else if (ld) begin                                                  // load values into internal registers
            if (freq > MAX_FREQ)
                freq_compare_value <= CLK_FREQ/MAX_FREQ;
            else
                freq_compare_value <= CLK_FREQ/freq;
            duty_compare_value <= 'hFF00/duty;                              // fixed-point arithmatic
            if (duty == 'd0)
                time_on <= 0;
            else
                time_on <= (freq_compare_value*'hFF)/duty_compare_value;    // fixed-point arithmatic
        end
        else begin
            if (counter >= freq_compare_value) begin
                counter = 0;
                signal = 1;
            end
            if (counter >= time_on)
                signal = 0;
            counter = counter + 1;
        end    
    end   
endmodule
