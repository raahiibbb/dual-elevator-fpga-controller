// ============================================================================
// debouncer.v - Switch/Button Debouncer (Verilog-2001, Vivado synthesizable)
// Debounces a single input signal. For multiple signals, instantiate multiple.
// Uses a counter-based approach: input must be stable for DEBOUNCE_TICKS cycles.
// Default: 100 MHz clock, ~10ms debounce = 1_000_000 ticks
// ============================================================================
module debouncer #(
    parameter DEBOUNCE_TICKS = 1_000_000  // ~10ms at 100MHz
)(
    input  wire clk,
    input  wire rst_n,
    input  wire btn_in,       // raw input from switch/button
    output reg  btn_out,      // debounced level output
    output wire btn_rise,     // single-cycle pulse on rising edge
    output wire btn_fall      // single-cycle pulse on falling edge
);

    reg [$clog2(DEBOUNCE_TICKS)-1:0] counter;
    reg btn_prev;

    // Edge detection
    assign btn_rise = (btn_out & ~btn_prev);
    assign btn_fall = (~btn_out & btn_prev);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter  <= 0;
            btn_out  <= 1'b0;
            btn_prev <= 1'b0;
        end else begin
            btn_prev <= btn_out;

            if (btn_in != btn_out) begin
                if (counter == DEBOUNCE_TICKS - 1) begin
                    btn_out <= btn_in;
                    counter <= 0;
                end else begin
                    counter <= counter + 1;
                end
            end else begin
                counter <= 0;
            end
        end
    end

endmodule
