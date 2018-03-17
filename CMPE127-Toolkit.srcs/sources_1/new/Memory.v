`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/20/2018 05:22:16 PM
// Design Name: 
// Module Name: Memory
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

module RAM_B #(
    parameter LENGTH = 32'h1000,
    parameter COLUMN = 0,
    parameter USE_FILE = 0,
    parameter FILE_NAME = "ram.mem"
)
(
    input wire clk,
    input wire rst,
    input wire we,
    input wire cs,
    input wire oe,
    input wire [ADDRESS_WIDTH-1:0] address,
    inout wire [BYTE_WIDTH-1:0] data
);

// ==================================
//// Internal Parameter Field
// ==================================
parameter ADDRESS_WIDTH = $clog2(LENGTH);
parameter RAM_WIDTH = 32;
parameter BYTE_WIDTH = 8;
// ==================================
//// Registers
// ==================================
reg [RAM_WIDTH-1:0] ram [0:LENGTH];
reg [BYTE_WIDTH-1:0] data_out;
// ==================================
//// Wires
// ==================================
// ==================================
//// Wire Assignments
// ==================================
assign data = (cs && oe && !we) ? data_out : 8'bz;
// ==================================
//// Modules
// ==================================
// ==================================
//// Behavioral Block
// ==================================
initial
begin
    if(USE_FILE)
    begin
        $readmemh(FILE_NAME, ram);
    end
end

always @(posedge clk or posedge rst)
begin
    if(rst)
    begin
        $readmemh(FILE_NAME, ram);
    end
    else if (cs && we)
    begin
       ram[address] = data << (COLUMN*8);
    end
    else if (cs && oe && !we)
    begin
        data_out = (ram[address] >> (COLUMN*8)) & 8'hFF;
    end
end

endmodule

module RAM #(
    parameter LENGTH = 32'h1000,
    parameter WIDTH = 32,
    parameter USE_FILE = 0,
    parameter FILE_NAME = "ram.mem"
)
(
    input wire clk,
    input wire we,
    input wire cs,
    input wire oe,
    input wire [ADDRESS_WIDTH-1:0] address,
    inout wire [WIDTH-1:0] data
);

// ==================================
//// Internal Parameter Field
// ==================================
parameter ADDRESS_WIDTH = $clog2(LENGTH);
// ==================================
//// Registers
// ==================================
reg [WIDTH-1:0] ram [0:LENGTH];
reg [WIDTH-1:0] data_out;
// ==================================
//// Wires
// ==================================
// ==================================
//// Wire Assignments
// ==================================
assign data = (cs && oe && !we) ? data_out : {(WIDTH){1'bz}};
// ==================================
//// Modules
// ==================================
// ==================================
//// Behavioral Block
// ==================================
integer i;
initial
begin
    if(USE_FILE)
    begin
        $readmemh(FILE_NAME, ram);
    end
    else
    begin
        data_out = {(WIDTH){ 1'b0 }};
		for (i=0; i<32; i=i+1) 
        begin
			ram[i] = {(WIDTH){ 1'b0 }};
        end
    end
end

always @(posedge clk)
begin
    if (cs && we)
    begin
       ram[address] = data;
    end
    else if (cs && oe && !we)
    begin
        data_out = ram[address];
    end
end

endmodule

module ROM #(
    parameter LENGTH = 32'h1000,
    parameter WIDTH = 32,
    parameter FILE_NAME = "rom.mem"
)
(
	input wire [$clog2(LENGTH)-1:0] a,
	output wire [WIDTH-1:0] out 
);

// ==================================
//// Internal Parameter Field
// ==================================
// ==================================
//// Wires
// ==================================
// ==================================
//// Wire Assignments
// ==================================
assign out = rom[a];
// ==================================
//// Modules
// ==================================
// ==================================
//// Registers
// ==================================
reg [WIDTH:0] rom[0:LENGTH];
// ==================================
//// Behavioral Block
// ==================================
//initialize rom from memfile_s.dat
initial
begin
    $readmemh(FILE_NAME, rom);
end

endmodule

module FIFO #(
    parameter LENGTH = 16,
    parameter WIDTH = 32
)
(
    input wire clk,
    input wire rst,
    input wire wr_cs,
    input wire wr_en,
    input wire rd_cs,
    input wire rd_en,
    output reg full,
    output reg empty,
    output reg [WIDTH-1:0] out,
    input wire [WIDTH-1:0] in
);
// ==================================
//// Internal Parameter Field
// ==================================
parameter ADDRESS_WIDTH = $clog2(LENGTH);
// ==================================
//// Registers
// ==================================
reg [ADDRESS_WIDTH-1:0] write_position;
reg [ADDRESS_WIDTH-1:0] read_position;
reg [ADDRESS_WIDTH:0] status_count;
reg [WIDTH-1:0] mem [0:LENGTH];
reg previously_empty;
// ==================================
//// Wires
// ==================================
// ==================================
//// Wire Assignments
// ==================================
// assign full  = (status_count == (LENGTH));
// assign empty = (status_count == 0);
// ==================================
//// Modules
// ==================================
// ==================================
//// Behavioral Block
// ==================================
always @(posedge clk or posedge rst)
begin
    if(rst)
    begin
        write_position = 0;
        read_position = 0;
        status_count = 0;
        mem[write_position] = " ";
        write_position = 0;
        previously_empty = 0;
        out = 0;
        full = 0;
        empty = 1;
    end
    else 
    begin
        if(status_count == 0)
        begin
            empty = 1;
        end
        if(status_count == LENGTH)
        begin
            full = 1;
        end
        //// Enqueue data
        if (wr_cs && wr_en && !full)
        begin
            mem[write_position] = in;
            if(write_position == LENGTH-1)
            begin
                write_position = 0;
            end
            else
            begin
                write_position = write_position + 1;
            end
            status_count = status_count + 1;
            empty = 0;
        end
        //// Dequeue data
        if (rd_cs && rd_en && !empty)
        begin
            out = mem[read_position];
            if(read_position == LENGTH-1)
            begin
                read_position = 0;
            end
            else
            begin
                read_position = read_position + 1;
            end
            status_count = status_count - 1;
            full = 0;
        end
    end
end

endmodule

/* TODO: dis still a fifo!! */
module STACK #(
    parameter LENGTH = 16,
    parameter WIDTH = 8
)
(
    input wire clk,
    input wire rst,
    input wire wr_cs,
    input wire wr_en,
    input wire rd_cs,
    input wire rd_en,
    output reg full,
    output reg empty,
    output reg [WIDTH-1:0] out,
    input wire [WIDTH-1:0] in
);
// ==================================
//// Internal Parameter Field
// ==================================
parameter ADDRESS_WIDTH = $clog2(LENGTH);
// ==================================
//// Registers
// ==================================
reg [ADDRESS_WIDTH-1:0] write_position;
reg [ADDRESS_WIDTH-1:0] read_position;
reg [ADDRESS_WIDTH:0] status_count;
reg [WIDTH-1:0] mem [0:LENGTH];
reg previously_empty;
// ==================================
//// Wires
// ==================================
// ==================================
//// Wire Assignments
// ==================================
// assign full  = (status_count == (LENGTH));
// assign empty = (status_count == 0);
// ==================================
//// Modules
// ==================================
// ==================================
//// Behavioral Block
// ==================================
always @(posedge clk or posedge rst)
begin
    if(rst)
    begin
        write_position = 0;
        read_position = 0;
        status_count = 0;
        mem[write_position] = " ";
        write_position = 0;
        previously_empty = 0;
        full = 0;
        empty = 1;
    end
    else 
    begin
        if(status_count == 0)
        begin
            empty = 1;
        end
        if(status_count == LENGTH)
        begin
            full = 1;
        end
        //// Enqueue data
        if (wr_cs && wr_en && !full)
        begin
            mem[write_position] = in;
            if(write_position == LENGTH-1)
            begin
                write_position = 0;
            end
            else
            begin
                write_position = write_position + 1;
            end
            status_count = status_count + 1;
            empty = 0;
        end
        //// Dequeue data
        if (rd_cs && rd_en && !empty)
        begin
            out = mem[read_position];
            if(read_position == LENGTH-1)
            begin
                read_position = 0;
            end
            else
            begin
                read_position = read_position + 1;
            end
            status_count = status_count - 1;
            full = 0;
        end
    end
end

endmodule

//module AsynchronousFIFO #(
//    parameter DATA_WIDTH    = 8,
//    parameter ADDRESS_WIDTH = 4,
//    parameter FIFO_DEPTH    = (1 << ADDRESS_WIDTH)
//)
//(
//    // Read Port
//    output reg  [DATA_WIDTH-1:0] out, 
//    output reg                   empty,
//    input wire                   read_enable,
//    input wire                   rclk,        
//    // Write Port
//    input wire  [DATA_WIDTH-1:0] in,  
//    output reg                   full,
//    input wire                   write_enable,
//    input wire                   wclk,
//    /// clear
//    input wire                   clear
//);

//    reg   [DATA_WIDTH-1:0]              Mem [FIFO_DEPTH-1:0];
//    wire  [ADDRESS_WIDTH-1:0]           pNextWordToWrite, pNextWordToRead;
//    wire                                EqualAddresses;
//    wire                                NextWriteAddressEn, NextReadAddressEn;
//    wire                                Set_Status, Rst_Status;
//    reg                                 Status;
//    wire                                PresetFull, PresetEmpty;

//    //(Uses a dual-port RAM).
//    //'out' logic:
//    always @ (posedge rclk)
//    begin
//        if (read_enable &  ! empty)
//        begin
//            out <= Mem[pNextWordToRead];
//        end
//    end            
//    //'in' logic:
//    always @ (posedge wclk)
//    begin
//        if (write_enable &  ! full)
//        begin
//            Mem[pNextWordToWrite] <= in;
//        end
//    end
//    //Fifo addresses support logic: 
//    //'Next Addresses' enable logic:
//    assign NextWriteAddressEn = write_enable & ~full;
//    assign NextReadAddressEn  = read_enable  & ~empty;
            
//    //Addreses (Gray counters) logic:
//    GrayCounter GrayCounter_pWr (
//        .GrayCount_out(pNextWordToWrite),
//        .Enable_in(NextWriteAddressEn),
//        .clear(clear),
//        .Clk(wclk)
//    );
        
//    GrayCounter GrayCounter_pRd (
//        .out(pNextWordToRead),
//        .en(NextReadAddressEn),
//        .clr(clear),
//        .clk(rclk)
//    );

//    //'EqualAddresses' logic:
//    assign EqualAddresses = (pNextWordToWrite == pNextWordToRead);

//    //'Quadrant selectors' logic:
//    assign Set_Status = (pNextWordToWrite[ADDRESS_WIDTH-2] ~^ pNextWordToRead[ADDRESS_WIDTH-1]) & 
//                        (pNextWordToWrite[ADDRESS_WIDTH-1] ^  pNextWordToRead[ADDRESS_WIDTH-2]);                            
//    assign Rst_Status = (pNextWordToWrite[ADDRESS_WIDTH-2] ^  pNextWordToRead[ADDRESS_WIDTH-1]) & 
//                        (pNextWordToWrite[ADDRESS_WIDTH-1] ~^ pNextWordToRead[ADDRESS_WIDTH-2]);
                        
//    //'Status' latch logic:
//    always @ (Set_Status, Rst_Status, clear) //D Latch w/ Asynchronous Clear & Preset.
//    begin
//        if (Rst_Status | clear)
//        begin
//            Status = 0;  //Going 'Empty'.
//        end
//        else if (Set_Status)
//        begin
//            Status = 1;  //Going 'Full'.
//        end
//    end            
//    //'full' logic for the writing port:
//    assign PresetFull = Status & EqualAddresses;  //'Full' Fifo.
    
//    always @ (posedge wclk, posedge PresetFull) //D Flip-Flop w/ Asynchronous Preset.
//    begin
//        if (PresetFull)
//        begin
//            full <= 1;
//        end
//        else
//        begin
//            full <= 0;
//        end
//    end           
//    //'empty' logic for the reading port:
//    assign PresetEmpty = ~Status & EqualAddresses;  //'Empty' Fifo.
    
//    always @ (posedge rclk, posedge PresetEmpty)  //D Flip-Flop w/ Asynchronous Preset.
//    begin
//        if (PresetEmpty)
//        begin
//            empty <= 1;
//        end
//        else
//        begin
//            empty <= 0;
//        end
//    end           
//endmodule

////==========================================
//// Function : Code Gray counter.
//// Coder    : Alex Claros F.
//// Date     : 15/May/2005.
////=======================================

module GrayCounter #(parameter COUNTER_WIDTH = 4)
(
    input wire                         clk,
    input wire                         en, 
    input wire                         clr,
    output reg  [COUNTER_WIDTH-1:0]    gray_count
);

reg    [COUNTER_WIDTH-1:0]         BinaryCount;

always @ (posedge clk)
begin
    if(clr) 
    begin
        BinaryCount   <= {COUNTER_WIDTH{1'b 0}} + 1;  //Gray count begins @ '1' with
        gray_count <= {COUNTER_WIDTH{1'b 0}};      // first 'Enable_in'.
    end
    else if (en) 
    begin
        BinaryCount   <= BinaryCount + 1;
        gray_count <= {BinaryCount[COUNTER_WIDTH-1],
                        BinaryCount[COUNTER_WIDTH-2:0] ^ BinaryCount[COUNTER_WIDTH-1:1]
                    };
    end
end    
endmodule