module X4_Encoder ( input [1:0] rot_button,
                    output reg [7:0] count);

//// Set count to 0 to begin
    initial
    begin
        count = 1'b0;
    end
//// Conditions if button 0 is pressed                        
    always @(posedge rot_button[0])
    begin
        case (rot_button[1])
            0: count = count + 1;
            1: count = count - 1;
            default: count = count;
        endcase
    end
//// Conditions if button 1 is pressed    
    always @(posedge rot_button[1])
    begin
        case (rot_button[0])
            0: count = count - 1;
            1: count = count + 1;
            default: count = count;
        endcase
    end    
//// Conditions if button 0 is released    
    always @(negedge rot_button[0])
    begin
        case (rot_button[1])
            0: count = count - 1;
            1: count = count + 1;
            default: count = count;
        endcase
    end
//// Conditions if button 1 is released        
    always @(negedge rot_button[1])
    begin
        case (rot_button[0])
            0: count = count + 1;
            1: count = count - 1;
            default: count = count;
        endcase
    end
endmodule                   