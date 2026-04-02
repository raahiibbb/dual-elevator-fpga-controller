// ============================================================================
// seven_seg_mux.v - 4-Digit 7-Segment Display Multiplexer
// Drives 4 of the 8 digits on Nexys A7 (uses AN[3:0], blanks AN[7:4])
// Includes custom segment LUT for elevator floor digits and status letters.
//
// Display layout (left to right, AN[3] is more left, AN[0] is rightmost):
//   AN[3] = Lift 2 floor number   (0-7)
//   AN[2] = Lift 2 status letter  (u/d/o/c)
//   AN[1] = Lift 1 floor number   (0-7)
//   AN[0] = Lift 1 status letter  (u/d/o/c)
//
// Segment encoding (active-low, seg = {a,b,c,d,e,f,g}):
//   '0' = 7'b0000001   '1' = 7'b1001111   '2' = 7'b0010010
//   '3' = 7'b0000110   '4' = 7'b1001100   '5' = 7'b0100100
//   '6' = 7'b0100000   '7' = 7'b0001111
//   'o' = 7'b1100010   'c' = 7'b1110010
//   'd' = 7'b1000010   'u' = 7'b1100011
//
// Status encoding (2-bit input):
//   00 = door closed ('c')   01 = door open   ('o')
//   10 = moving up   ('u')   11 = moving down ('d')
// ============================================================================
module seven_seg_mux (
    input  wire        clk,          // 100 MHz system clock
    input  wire [2:0]  lift1_floor,  // Lift 1 current floor (0-7)
    input  wire [1:0]  lift1_status, // Lift 1 status (00=c,01=o,10=u,11=d)
    input  wire [2:0]  lift2_floor,  // Lift 2 current floor (0-7)
    input  wire [1:0]  lift2_status, // Lift 2 status (00=c,01=o,10=u,11=d)
    output reg  [6:0]  seg,          // 7-segment cathodes (active low)
    output reg  [7:0]  an            // 8 anodes (active low)
);

    // Refresh counter: 100MHz / 2^18 = ~381 Hz per digit (~95 Hz per digit with 4 digits)
    reg [19:0] refresh_counter = 0;
    always @(posedge clk) begin
        refresh_counter <= refresh_counter + 1;
    end

    wire [1:0] digit_sel = refresh_counter[17:16]; // cycles through 0,1,2,3

    // Floor number to 7-segment LUT (0-7)
    function [6:0] floor_to_seg;
        input [2:0] floor;
        begin
            case (floor)
                3'd0: floor_to_seg = 7'b0000001; // '0'
                3'd1: floor_to_seg = 7'b1001111; // '1'
                3'd2: floor_to_seg = 7'b0010010; // '2'
                3'd3: floor_to_seg = 7'b0000110; // '3'
                3'd4: floor_to_seg = 7'b1001100; // '4'
                3'd5: floor_to_seg = 7'b0100100; // '5'
                3'd6: floor_to_seg = 7'b0100000; // '6'
                3'd7: floor_to_seg = 7'b0001111; // '7'
            endcase
        end
    endfunction

    // Status code to 7-segment LUT
    function [6:0] status_to_seg;
        input [1:0] status;
        begin
            case (status)
                2'b00: status_to_seg = 7'b1110010; // 'c' - door closed
                2'b01: status_to_seg = 7'b1100010; // 'o' - door open
                2'b10: status_to_seg = 7'b1100011; // 'u' - moving up
                2'b11: status_to_seg = 7'b1000010; // 'd' - moving down
            endcase
        end
    endfunction

    always @(*) begin
        // Default: blank all digits
        an  = 8'b11111111;
        seg = 7'b1111111;

        case (digit_sel)
            2'd0: begin // AN[0] = Lift 1 status (rightmost)
                an  = 8'b11111101;
                seg = status_to_seg(lift1_status);
            end
            2'd1: begin // AN[1] = Lift 1 floor
                an  = 8'b11111011;
                seg = floor_to_seg(lift1_floor);
            end
            2'd2: begin // AN[2] = Lift 2 status
                an  = 8'b11011111;
                seg = status_to_seg(lift2_status);
            end
            2'd3: begin // AN[3] = Lift 2 floor (leftmost of the 4)
                an  = 8'b10111111;
                seg = floor_to_seg(lift2_floor);
            end
        endcase
    end

endmodule
