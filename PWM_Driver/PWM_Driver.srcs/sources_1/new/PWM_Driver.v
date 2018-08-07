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


module PWM_Driver #(
    parameter INPUT_WIDTH = 8,
    parameter DATA_WIDTH = 16
    )(
    input wire sys_clk,
    input wire reset,
    input wire load,
    input wire [DATA_WIDTH  - 1:0] data,
    output reg signal
);
// ==================================
//// Internal Parameter Field
// ==================================
// ==================================
//// Registers
// ==================================
reg [DATA_WIDTH - 1:0] duty_counter;
reg [INPUT_WIDTH - 1:0] period_counter;
reg [DATA_WIDTH - 1:0] duty_compare_value;

// ==================================
//// Wires
// ==================================
wire [INPUT_WIDTH - 1:0] duty;
wire [INPUT_WIDTH - 1:0] period;
wire pwm_clk1;
// ==================================
//// Wire Assignments
// ==================================
assign period = data[7:0];
assign duty = data[15:8];
// ==================================
//// Modules
// ==================================
PWM_CLKGEN CLOCK1 ( .sys_clk(sys_clk), .period(period_counter), .pwm_clk(pwm_clk1));
// ==================================
//// Behavioral Block
// ==================================
//initial period_counter <= 1;
    
always @(posedge pwm_clk1 or negedge reset) begin
// Reset values
    if (reset) begin
        period_counter <= 0;
        duty_counter <= 0;
        signal <= 0;
        duty_compare_value <=0;
    end
// load values into internal registers
    else if (load) begin                                 
        period_counter <= ~period;
        duty_compare_value <= duty*4;        
    end
    else begin
// turn the signal on when counter resets
        if (duty_counter >= 1020) begin
            signal <= 1;
            duty_counter <= 0;
        end
// turn the signal off when counter passes duty cycle value
        else if (duty_counter >= duty_compare_value)
            signal <= 0;
        duty_counter <= duty_counter + 1;
    end
end
endmodule

module PWM_CLKGEN (
    input wire sys_clk,
    input wire [7:0] period,
    output reg pwm_clk
    );
    
    reg [7:0] period_adj;

    always @(posedge sys_clk) begin
        if (period == 'h00)
            period_adj <= 1;
        else
            period_adj <= period;
        pwm_clk <= 1;
        #period_adj;
        pwm_clk <= 0;
        #period_adj;
    end
endmodule