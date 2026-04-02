// ============================================================================
// uart_rx.v - UART Receiver (Verilog-2001, Vivado synthesizable)
// 8-N-1 format: 1 start bit, 8 data bits, 1 stop bit, no parity
// Samples at mid-bit for reliability. Double-FF synchronizer on RX input.
// Default: 100 MHz clock, 9600 baud => CLOCKS_PER_PULSE = 10417
// ============================================================================
module uart_rx #(
    parameter CLOCKS_PER_PULSE = 10417
)(
    input  wire       clk,
    input  wire       rst_n,      // active-low reset
    input  wire       rx,         // serial input
    output reg  [7:0] data_out,   // received byte (valid when data_ready pulses)
    output reg        data_ready  // pulses high for 1 clk when byte is received
);

    // State encoding
    localparam ST_IDLE  = 2'd0;
    localparam ST_START = 2'd1;
    localparam ST_DATA  = 2'd2;
    localparam ST_STOP  = 2'd3;

    reg [1:0]  state;
    reg [7:0]  shift_reg;
    reg [2:0]  bit_idx;
    reg [$clog2(CLOCKS_PER_PULSE)-1:0] clk_cnt;

    // Double-FF synchronizer for metastability protection
    reg rx_sync1, rx_sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            shift_reg  <= 8'd0;
            data_out   <= 8'd0;
            data_ready <= 1'b0;
            bit_idx    <= 3'd0;
            clk_cnt    <= 0;
        end else begin
            data_ready <= 1'b0;  // default: clear pulse

            case (state)
                ST_IDLE: begin
                    if (rx_sync2 == 1'b0) begin  // falling edge = start bit
                        clk_cnt <= 0;
                        state   <= ST_START;
                    end
                end

                ST_START: begin
                    // Sample at mid-start-bit to verify it's real
                    if (clk_cnt == (CLOCKS_PER_PULSE / 2) - 1) begin
                        if (rx_sync2 == 1'b0) begin
                            // Valid start bit - proceed to data
                            clk_cnt <= 0;
                            bit_idx <= 3'd0;
                            state   <= ST_DATA;
                        end else begin
                            // False start - go back to idle
                            state <= ST_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                ST_DATA: begin
                    if (clk_cnt == CLOCKS_PER_PULSE - 1) begin
                        clk_cnt <= 0;
                        shift_reg[bit_idx] <= rx_sync2;
                        if (bit_idx == 3'd7) begin
                            state <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                ST_STOP: begin
                    if (clk_cnt == CLOCKS_PER_PULSE - 1) begin
                        clk_cnt    <= 0;
                        data_out   <= shift_reg;
                        data_ready <= 1'b1;
                        state      <= ST_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
