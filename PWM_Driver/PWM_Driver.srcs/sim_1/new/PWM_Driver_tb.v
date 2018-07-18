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
    reg [7:0] duty0_tb, duty1_tb, duty2_tb, duty3_tb;
    reg [31:0] freq0_tb, freq1_tb, freq2_tb, freq3_tb;
    wire signal0_tb, signal1_tb, signal2_tb, signal3_tb, signal4_tb, signal5_tb, signal6_tb, signal7_tb, signal8_tb, signal9_tb;
    wire signalA_tb, signalB_tb, signalC_tb, signalD_tb, signalE_tb, signalF_tb;
    
    PWM_Driver DUT0( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty0_tb), .freq(freq0_tb), .signal(signal0_tb));
    
    PWM_Driver DUT1( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty1_tb), .freq(freq0_tb), .signal(signal1_tb));
    
    PWM_Driver DUT2( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty2_tb), .freq(freq0_tb), .signal(signal2_tb));
    
    PWM_Driver DUT3( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty3_tb), .freq(freq0_tb), .signal(signal3_tb));
    
    PWM_Driver DUT4( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty0_tb), .freq(freq1_tb), .signal(signal4_tb));
    
    PWM_Driver DUT5( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty1_tb), .freq(freq1_tb), .signal(signal5_tb));
    
    PWM_Driver DUT6( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty2_tb), .freq(freq1_tb), .signal(signal6_tb));
    
    PWM_Driver DUT7( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty3_tb), .freq(freq1_tb), .signal(signal7_tb));
    
    PWM_Driver DUT8( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty0_tb), .freq(freq2_tb), .signal(signal8_tb));
    
    PWM_Driver DUT9( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty1_tb), .freq(freq2_tb), .signal(signal9_tb));
    
    PWM_Driver DUTA( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty2_tb), .freq(freq2_tb), .signal(signalA_tb));
    
    PWM_Driver DUTB( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty3_tb), .freq(freq2_tb), .signal(signalB_tb));
    
    PWM_Driver DUTC( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty0_tb), .freq(freq3_tb), .signal(signalC_tb));
    
    PWM_Driver DUTD( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty1_tb), .freq(freq3_tb), .signal(signalD_tb));
    
    PWM_Driver DUTE( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty2_tb), .freq(freq3_tb), .signal(signalE_tb));
    
    PWM_Driver DUTF( .clk(clk_tb), .rst(rst_tb), .ld(ld_tb), .duty(duty3_tb), .freq(freq3_tb), .signal(signalF_tb));    
        
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
    
    integer j;
    integer duty_cycle;
    integer total_clks;
    initial begin
        duty0_tb = 8'd64;
        duty1_tb = 8'd128;
        duty2_tb = 8'd191;
        duty3_tb = 8'd255;
        freq0_tb = 32'd25_000;
        freq1_tb = 32'd50_000;
        freq2_tb = 32'd75_000;
        freq3_tb = 32'd100_000;
        $display("PWM_Driver test start.\n");
        #5 rst_tb = 1;
        CLOCK(1, total_clks);
        #5 rst_tb = 0;
        CLOCK(1, total_clks);
        #5 ld_tb = 1;
        CLOCK(2, total_clks);
        #5 ld_tb = 0;
        CLOCK(2, total_clks);
        total_clks = 0;
        CLOCK(MAX_CYCLE, total_clks);
        $finish;
    end    
endmodule
