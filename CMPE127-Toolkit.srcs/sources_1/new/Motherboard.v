`timescale 1ns / 1ps
`default_nettype none

`define SCAN_CODE_LENGTH        11
`define SCAN_CODE_DATA_LENGTH   8
`define RGB_RESOLUTION  		4
`define FREQ_IN                 32'd100_000_000
`define BREAK_CODE              8'hF0
`define EXTEND_CODE             8'hE0
`define SHIFT_CODE_LEFT         8'h12        
`define SHIFT_CODE_RIGHT        8'h59

`define LEFT_ARROW              8'h6B
`define DOWN_ARROW              8'h72
`define RIGHT_ARROW             8'hF4
`define UP_ARROW                8'h75

`define LEFT_ARROW_ASCII        8'hEB
`define DOWN_ARROW_ASCII        8'hF2
`define RIGHT_ARROW_ASCII       8'hF4
`define UP_ARROW_ASCII          8'hF5

`define BACKSPACE               8'hF7

`define KEYBOARD_ADDRESS        1781
`define KEYBOARD_READY_ADDRESS  1782
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 11/25/2017 05:38:31 PM
// Design Name:
// Module Name: Motherboard
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


module Motherboard(
	//// input 100 MHz clock
    input wire clk,
    input wire rst,
    input wire ps2_clk,
    input wire ps2_data,
    //// Horizontal sync pulse for VGA controller
    output wire hsync,
    //// Vertical sync pulse for VGA controller
    output wire vsync,
    //// RGB 4-bit singal that go to a DAC (range 0V <-> 0.7V) to generate a color intensity
    output wire [`RGB_RESOLUTION-1:0] r,
    output wire [`RGB_RESOLUTION-1:0] g,
    output wire [`RGB_RESOLUTION-1:0] b
);

// ==================================
//// Internal Parameter Field
// ==================================
// ==================================
//// Wires
// ==================================
wire keyboard_ready;

wire vga_busy;

wire [31:0] program_counter;
wire [31:0] mips_instruction;
wire [31:0] address_bus;
wire [31:0] data_bus;
wire write_enable;
wire read_enable;
wire byte_access;

wire [7:0] scan_code;
wire [31:0] extended_count;
wire [2:0] text_color;
wire [2:0] background_color;

wire [7:0] vga_data_bus;
wire [11:0] vga_address_bus;
// ==================================
//// Wire Assignments
// ==================================
assign text_color = 3'b010;
assign background_color = 3'b000;
// assign full  = (status_count == (LENGTH));
// assign empty = (status_count == 0);
// ==================================
//// Modules
// ==================================
// MIPS mips(
//     .clk(!clk), 
//     .reset(rst),
// 	.pc(program_counter),
// 	.instruction(mips_instruction),
// 	.write_enable(write_enable),
//     .data(data_bus),
// 	.execout(address_bus), 
// 	.dispSel(4'b0),
// 	.dispDat() 
// );

wire instruction_read;

Processor mips(
    .clock(clk),
    .reset(rst),
    .Interrupts(5'b0),            // 5 general-purpose hardware interrupts
    .NMI(1'b0),                         // Non-maskable interrupt
    // Data Memory Interface
    .DataMem_In(DataMem_In),
    .DataMem_Ready(DataMem_Ready),
    .DataMem_Read(read_enable),
    .DataMem_Write(DataMem_Write),        // 4-bit Write, one for each byte in word.
    .DataMem_Address(DataMem_Address),      // Addresses are words, not bytes.
    .DataMem_Out(DataMem_Out),
    // Instruction Memory Interface
    .InstMem_In(mips_instruction),
    .InstMem_Address(program_counter),      // Addresses are words, not bytes.
    .InstMem_Ready(instruction_read),
    .InstMem_Read(instruction_read),
    .IP()                     // Pending interrupts (diagnostic)
);




AND #(.WIDTH(2)) ram_oe_and (
	.in({ ram_access, !write_enable}),
	.out(ram_oe)
);

RAM_B #(
    .LENGTH(32'h1000,
    .COLUMN(3),
    .USE_FILE(1),
    .FILE_NAME("ram.mem")
) ramb_3 (
    .clk(clk),
    .we(we),
    .cs(cs),
    .oe(oe),
    .address(address_bus),
    .data(data_bus[31:24])
);

RAM_B #(
    .LENGTH(32'h1000,
    .COLUMN(2),
    .USE_FILE(1),
    .FILE_NAME("ram.mem")
) ramb_2 (
    .clk(clk),
    .we(we),
    .cs(cs),
    .oe(oe),
    .address(address_bus),
    .data(data_bus[23:16])
);

 RAM_B #(
    .LENGTH(32'h1000,
    .COLUMN(1),
    .USE_FILE(1),
    .FILE_NAME("ram.mem")
) ramb_1 (
    .clk(clk),
    .we(we),
    .cs(cs),
    .oe(oe),
    .address(address_bus),
    .data(data_bus[15:8])
);

 RAM_B #(
    .LENGTH(32'h1000,
    .COLUMN(0),
    .USE_FILE(1),
    .FILE_NAME("ram.mem")
) ramb_0 (
    .clk(clk),
    .we(we),
    .cs(cs),
    .oe(oe),
    .address(address_bus),
    .data(data_bus[7:0])
);

ROM #(
    .LENGTH(32'h1000),
    .WIDTH(32),
    .FILE_NAME("rom.mem")
) rom (
	.a(program_counter[11:0]),
	.out(mips_instruction) 
);

wire text_access;
wire extern_access;
wire ram_access;

DECODER #(.INPUT_WIDTH(3)) address_decoder
(
	.enable(1'b1),
	.in(address_bus[14:12]),
	.out({ ram_access, extern_access, text_access })
);

wire key_cs;
wire key_ready_cs;
wire ram_oe;
wire vga_fifo_cs;

AND #(.WIDTH(2)) key_cs_and (
	.in({ extern_access, (address_bus == `KEYBOARD_ADDRESS)}),
	.out(key_cs)
);

AND #(.WIDTH(2)) key_ready_cs_and (
	.in({ extern_access, (address_bus == `KEYBOARD_READY_ADDRESS)}),
	.out(key_ready_cs)
);

AND #(.WIDTH(4)) vga_fifo_cs_and (
	.in({ !key_cs, !key_ready_cs, !ram_access, extern_access}),
	.out(vga_fifo_cs)
);

ASCII_Keyboard keyboard(
    .clk(clk),
    .rst(rst),
    .ps2_clk(ps2_clk),
    .ps2_data(ps2_data),
    .oe(key_cs),
    .next(key_cs),
    .ascii(data_bus),
    .ready(keyboard_ready),
    .scan_code_reg(scan_code),
    .extended_count(extended_count),
    .command()
);

TRIBUFFER #(.WIDTH(32))
key_ready_buffer
(
	.oe(key_ready_cs),
	.in({ 31'b0, keyboard_ready }),
	.out(data_bus)
);

/*
RAM #(
    .WIDTH(32),
	.LENGTH(1024),
    .USE_FILE(1),
    .FILE_NAME("ram.mem")
) ram (
    .clk(clk),
    .we(write_enable),
    .cs(ram_access),
    .oe(ram_oe),
    .address({ 1'b0, address_bus[12:2]}),
    .data(data_bus)
);
*/

wire ascii_fifo_empty, address_fifo_empty;

FIFO #(
    .LENGTH(32),
    .WIDTH(8)
) vga_ascii_fifo (
    .clk(clk),
    .rst(rst),
    .wr_cs(vga_fifo_cs),
    .wr_en(write_enable),
    .rd_cs(!vga_busy),
    .rd_en(!vga_busy),
    .full(),
    .empty(ascii_fifo_empty),
    .out(vga_data_bus),
    .in(data_bus[7:0])
);

FIFO #(
    .LENGTH(32),
    .WIDTH(12)
) vga_address_fifo (
    .clk(clk),
    .rst(rst),
    .wr_cs(vga_fifo_cs),
    .wr_en(write_enable),
    .rd_cs(!vga_busy),
    .rd_en(!vga_busy),
    .full(),
    .empty(address_fifo_empty),
    .out(vga_address_bus),
    .in({ 1'b0, address_bus[10:0] })
);

wire vga_cs;

OR #(.WIDTH(2)) fifo_to_vga_cs_and (
	.in({ !ascii_fifo_empty, !address_fifo_empty }),
	.out(vga_cs)
);

VGA_Terminal vga_term(
    .clk(!clk),
    .rst(rst),
    .hsync(hsync),
    .vsync(vsync),
    .r(r),
    .g(g),
    .b(b),
    .values({
        program_counter,
        32'h0,
        address_bus,
        data_bus,

        32'h0,
        32'h0,
        32'h0,
        32'h0,
        
        32'h0,
        32'h0,
        32'h0,
        32'h0,
        
        32'h0,
        32'h0,
        32'h0,
        32'h0,
        
        32'h0,
        32'h0,
        32'h0,
        32'h0
        //mips_instruction
    }),
    .address(vga_address_bus),
    .data(vga_data_bus),
    .cs(vga_cs),
    .busy(vga_busy),
    .text(text_color),
    .background(background_color)
);

// ==================================
//// Registers
// ==================================
// ==================================
//// Behavioral Block
// ==================================

endmodule