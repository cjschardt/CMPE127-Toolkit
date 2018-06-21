`timescale 1ns / 1ps
module X4_Encoder_tb;
    
    reg [1:0] rot_button_tb;
    wire [7:0] count_tb;
    
    X4_Encoder DUT ( .rot_button(rot_button_tb), .count(count_tb));
    
    integer i = 0;
    integer j = 0;
    initial
    begin
        $display("Begin Test.\n");
        rot_button_tb = 0; #1;
        
//// Encoder turns to the right        
        for (i = 0; i < 128; i = i + 1)
        //$display("%d\n", count_tb);
        begin
            if (count_tb != i)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, i, count_tb);
                $stop;
            end        
            rot_button_tb[0] = 1; #1
            $display("%d (line 26)\n", count_tb);
            i = i + 1;
            if (count_tb != i)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, i, count_tb);
                $stop;
            end
            rot_button_tb[1] = 1; #1
            $display("%d (line 34)\n", count_tb);
            i = i + 1;
            if (count_tb != i)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, i, count_tb);
                $stop;
            end
            rot_button_tb[0] = 0; #1
            $display("%d (line 42)\n", count_tb);
            i = i + 1;
            if (count_tb != i)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, i, count_tb);
                $stop;
            end
            rot_button_tb[1] = 0; #1;
            $display("%d (line 50)\n", count_tb);
        end
        
//// Encoder turns to the left        
        for (j = 128; j > 0; j = j - 1)
        begin
            if (count_tb != j)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, j, count_tb);
                $stop;
            end        
            rot_button_tb[1] = 1; #1
            $display("%d (line 62)\n", count_tb);
            j = j - 1;
            if (count_tb != j)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, j, count_tb);
                $stop;
            end
            rot_button_tb[0] = 1; #1
            $display("%d (line 70)\n", count_tb);
            j = j - 1;
            if (count_tb != j)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, j, count_tb);
                $stop;
            end
            rot_button_tb[1] = 0; #1
            $display("%d (line 75)\n", count_tb);
            j = j - 1;
            if (count_tb != j)
            begin
                $display("ERROR at time %t: expected count = %d, found count = %d\n", $time, j, count_tb);
                $stop;
            end
            rot_button_tb[0] = 0; #1;
            $display("%d (line 86)\n", count_tb);
        end
    $display("Test Successful.\n");
    $finish;
    end                  
endmodule
