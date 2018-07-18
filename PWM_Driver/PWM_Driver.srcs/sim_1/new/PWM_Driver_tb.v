`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/16/2018 06:07:47 PM
// Design Name: 
// Module Name: PWM_Driver_tb
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


module PWM_Driver_tb;
    
    parameter MAX_CYCLE = 32'd25_000;

    reg clk_tb, rst_tb, ld_tb;
    reg [7:0] duty_tb;
    reg [31:0] freq_tb;
    wire [31:0] counter_tb;
    wire signal_tb;
    
    PWM_Driver DUT( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty_tb), .freq(freq_tb), .signal(signal_tb), .counter(counter_tb));
        
    task CLOCK; 
        input [31:0] clk_cnt;
        output [31:0] count;
        integer i;
        begin
            count = 0;
            for (i=0; i<clk_cnt; i=i+1) begin
                #1 clk_tb = 1;
                #1 clk_tb = 0;
                count = count+1;
            end
        end
    endtask
    
    integer k;
    integer j;
    integer duty_cycle;
    integer total_clks;
    initial begin
        duty_tb = 8'd64;
        duty_cycle = 255/duty_tb;
        freq_tb = 75_000;
        $display("PWM_Driver test start.\n");
        $display("duty = %d, freq = %d Hz.\n", duty_cycle, freq_tb);
        #5 rst_tb = 1;
        CLOCK(1, total_clks);
        #5 rst_tb = 0;
        CLOCK(1, total_clks);
        #5 ld_tb = 1;
        CLOCK(2, total_clks);
        #5 ld_tb = 0;
        CLOCK(2, total_clks);
        total_clks = 0;
        for (k=0; k<MAX_CYCLE; k=k+1) begin
            #1 clk_tb = 1;
            #1 clk_tb = 0;
            total_clks = k;
        end
        $finish;
    end    
endmodule
