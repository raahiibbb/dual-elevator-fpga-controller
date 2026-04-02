// ============================================================================
// uart_tx.v - UART Transmitter (Verilog-2001, Vivado synthesizable)
// 8-N-1 format: 1 start bit, 8 data bits, 1 stop bit, no parity
// Default: 100 MHz clock, 9600 baud => CLOCKS_PER_PULSE = 10417
// ============================================================================
module uart_tx #(
    parameter CLOCKS_PER_PULSE = 10417
)(
    input  wire       clk,
    input  wire       rst_n,      // active-low reset
    input  wire [7:0] data_in,    // byte to transmit
    input  wire       data_valid, // pulse high for 1 clk to start TX
    output reg        tx,         // serial output (idle high)
    output wire       tx_busy     // high while transmitting
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

    assign tx_busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            tx        <= 1'b1;
            shift_reg <= 8'd0;
            bit_idx   <= 3'd0;
            clk_cnt   <= 0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx <= 1'b1;
                    if (data_valid) begin
                        shift_reg <= data_in;
                        bit_idx   <= 3'd0;
                        clk_cnt   <= 0;
                        state     <= ST_START;
                    end
                end

                ST_START: begin
                    tx <= 1'b0;  // start bit
                    if (clk_cnt == CLOCKS_PER_PULSE - 1) begin
                        clk_cnt <= 0;
                        state   <= ST_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                ST_DATA: begin
                    tx <= shift_reg[bit_idx];
                    if (clk_cnt == CLOCKS_PER_PULSE - 1) begin
                        clk_cnt <= 0;
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
                    tx <= 1'b1;  // stop bit
                    if (clk_cnt == CLOCKS_PER_PULSE - 1) begin
                        clk_cnt <= 0;
                        state   <= ST_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    tx    <= 1'b1;
                end
            endcase
        end
    end

endmodule
