// ============================================================================
// hcp_top.v - Top Level for HCP Board (Hall Call Panel / Outside Controller)
//
// This board captures hall call button presses and sends them to COP via UART.
// Receives elevator state from COP and displays on 7-seg and LEDs.
// Pressing BTNC (reset) resets local state AND sends a reset command to COP.
//
// Switch assignment (interleaved down/up for 8 floors):
//   SW[0]  = Floor 0 DOWN (unused - floor 0 has no DOWN)
//   SW[1]  = Floor 0 UP
//   SW[2]  = Floor 1 DOWN
//   SW[3]  = Floor 1 UP
//   SW[4]  = Floor 2 DOWN
//   SW[5]  = Floor 2 UP
//   SW[6]  = Floor 3 DOWN
//   SW[7]  = Floor 3 UP
//   SW[8]  = Floor 4 DOWN
//   SW[9]  = Floor 4 UP
//   SW[10] = Floor 5 DOWN
//   SW[11] = Floor 5 UP
//   SW[12] = Floor 6 DOWN
//   SW[13] = Floor 6 UP
//   SW[14] = Floor 7 DOWN
//   SW[15] = Floor 7 UP   (unused - floor 7 has no UP)
//
// LED assignment (matching switch order):
//   LED[0]  = (unused - floor 0 has no DOWN)
//   LED[1]  = Floor 0 UP indicator
//   LED[2]  = Floor 1 DOWN indicator
//   LED[3]  = Floor 1 UP indicator
//   LED[4]  = Floor 2 DOWN indicator
//   LED[5]  = Floor 2 UP indicator
//   LED[6]  = Floor 3 DOWN indicator
//   LED[7]  = Floor 3 UP indicator
//   LED[8]  = Floor 4 DOWN indicator
//   LED[9]  = Floor 4 UP indicator
//   LED[10] = Floor 5 DOWN indicator
//   LED[11] = Floor 5 UP indicator
//   LED[12] = Floor 6 DOWN indicator
//   LED[13] = Floor 6 UP indicator
//   LED[14] = Floor 7 DOWN indicator
//   LED[15] = (unused - floor 7 has no UP)
//
// 7-Segment (AN[3:0]):
//   AN[3] = Lift 2 floor, AN[2] = Lift 2 status
//   AN[1] = Lift 1 floor, AN[0] = Lift 1 status
//
// Push button:
//   BTNC = Reset (resets HCP + sends reset command to COP)
//
// PMOD JA:
//   JA[1] (C17) = UART TX to COP
//   JA[2] (D18) = UART RX from COP
// ============================================================================
module hcp_top (
    input  wire        clk,          // 100 MHz system clock (E3)

    // Switches
    input  wire [15:0] sw,           // SW[15:0]

    // Push button
    input  wire        btnc,         // Reset

    // LEDs
    output reg  [15:0] led,

    // Seven segment display
    output wire [6:0]  seg,
    output wire [7:0]  an,

    // PMOD JA for UART
    output wire        ja_tx,        // JA[1] - TX out
    input  wire        ja_rx         // JA[2] - RX in
);

    // ---- Reset generation ----
    wire reset_rise;
    debouncer #(.DEBOUNCE_TICKS(500000)) db_reset (
        .clk(clk), .rst_n(1'b1), .btn_in(btnc),
        .btn_out(), .btn_rise(reset_rise), .btn_fall()
    );

    reg [3:0] rst_cnt = 4'hF;
    reg       sys_reset = 1'b1;
    wire      rst_n = ~sys_reset;

    always @(posedge clk) begin
        if (reset_rise) begin
            rst_cnt   <= 4'hF;
            sys_reset <= 1'b1;
        end else if (rst_cnt > 0) begin
            rst_cnt <= rst_cnt - 1;
        end else begin
            sys_reset <= 1'b0;
        end
    end

    // ---- Reset command to COP via UART ----
    // After local reset, hold reset_cmd high for ~50ms (5M cycles) to ensure
    // COP receives at least a few UART frames with the reset flag set.
    reg [22:0] reset_cmd_cnt = 23'd0;
    wire       reset_cmd = (reset_cmd_cnt != 0);

    always @(posedge clk) begin
        if (sys_reset) begin
            reset_cmd_cnt <= 23'd5_000_000; // ~50ms at 100MHz
        end else if (reset_cmd_cnt != 0) begin
            reset_cmd_cnt <= reset_cmd_cnt - 1;
        end
    end

    // ---- Post-reset lockout for switch debouncers ----
    // After reset, ignore switch rises for enough time to let debouncers settle
    // so that already-ON switches don't immediately re-register as new requests.
    // Lockout duration: 2x debounce ticks = 1M cycles = ~10ms
    reg [19:0] settle_cnt = 20'd0;
    wire       settle_done = (settle_cnt == 0);

    always @(posedge clk) begin
        if (sys_reset) begin
            settle_cnt <= 20'd1_000_000;
        end else if (settle_cnt != 0) begin
            settle_cnt <= settle_cnt - 1;
        end
    end

    // ---- Debounce hall call switches (capture rising edge) ----
    wire [15:0] sw_rise;
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : sw_deb
            debouncer #(.DEBOUNCE_TICKS(500000)) db_sw (
                .clk(clk), .rst_n(rst_n), .btn_in(sw[i]),
                .btn_out(), .btn_rise(sw_rise[i]), .btn_fall()
            );
        end
    endgenerate

    // ---- Accumulate hall call requests ----
    reg [7:0] up_req_hold   = 8'b0;
    reg [7:0] down_req_hold = 8'b0;

    // Received indicator state from COP
    wire [7:0] rx_up_ind, rx_down_ind;
    wire [2:0] lift1_floor, lift2_floor;
    wire [1:0] lift1_status, lift2_status;
    wire       rx_valid;

    always @(posedge clk) begin
        if (sys_reset) begin
            up_req_hold   <= 8'b0;
            down_req_hold <= 8'b0;
        end else if (settle_done) begin
            // Set on switch rising edge (interleaved mapping)
            // SW[1]  = Floor 0 UP,  SW[3]  = Floor 1 UP, ..., SW[13] = Floor 6 UP
            if (sw_rise[1])  up_req_hold[0] <= 1'b1;  // Floor 0 UP
            if (sw_rise[3])  up_req_hold[1] <= 1'b1;  // Floor 1 UP
            if (sw_rise[5])  up_req_hold[2] <= 1'b1;  // Floor 2 UP
            if (sw_rise[7])  up_req_hold[3] <= 1'b1;  // Floor 3 UP
            if (sw_rise[9])  up_req_hold[4] <= 1'b1;  // Floor 4 UP
            if (sw_rise[11]) up_req_hold[5] <= 1'b1;  // Floor 5 UP
            if (sw_rise[13]) up_req_hold[6] <= 1'b1;  // Floor 6 UP
            // Floor 7 UP does not exist (top floor)

            // SW[2]  = Floor 1 DOWN, SW[4]  = Floor 2 DOWN, ..., SW[14] = Floor 7 DOWN
            if (sw_rise[2])  down_req_hold[1] <= 1'b1; // Floor 1 DOWN
            if (sw_rise[4])  down_req_hold[2] <= 1'b1; // Floor 2 DOWN
            if (sw_rise[6])  down_req_hold[3] <= 1'b1; // Floor 3 DOWN
            if (sw_rise[8])  down_req_hold[4] <= 1'b1; // Floor 4 DOWN
            if (sw_rise[10]) down_req_hold[5] <= 1'b1; // Floor 5 DOWN
            if (sw_rise[12]) down_req_hold[6] <= 1'b1; // Floor 6 DOWN
            if (sw_rise[14]) down_req_hold[7] <= 1'b1; // Floor 7 DOWN
            // Floor 0 DOWN does not exist (ground floor)

            // Clear when COP acknowledges (indicator goes high = request is being served)
            if (rx_valid) begin
                if (rx_up_ind[0]) up_req_hold[0] <= 1'b0;
                if (rx_up_ind[1]) up_req_hold[1] <= 1'b0;
                if (rx_up_ind[2]) up_req_hold[2] <= 1'b0;
                if (rx_up_ind[3]) up_req_hold[3] <= 1'b0;
                if (rx_up_ind[4]) up_req_hold[4] <= 1'b0;
                if (rx_up_ind[5]) up_req_hold[5] <= 1'b0;
                if (rx_up_ind[6]) up_req_hold[6] <= 1'b0;
                if (rx_down_ind[1]) down_req_hold[1] <= 1'b0;
                if (rx_down_ind[2]) down_req_hold[2] <= 1'b0;
                if (rx_down_ind[3]) down_req_hold[3] <= 1'b0;
                if (rx_down_ind[4]) down_req_hold[4] <= 1'b0;
                if (rx_down_ind[5]) down_req_hold[5] <= 1'b0;
                if (rx_down_ind[6]) down_req_hold[6] <= 1'b0;
                if (rx_down_ind[7]) down_req_hold[7] <= 1'b0;
            end
        end
    end

    // ---- UART Handler ----
    hcp_uart_handler #(
        .CLOCKS_PER_PULSE(10417),
        .TX_INTERVAL(1000000)
    ) u_uart (
        .clk         (clk),
        .rst_n       (rst_n),
        .uart_tx     (ja_tx),
        .uart_rx     (ja_rx),
        // TX data (hall calls -> COP)
        .up_request  (up_req_hold),
        .down_request(down_req_hold),
        .reset_cmd   (reset_cmd),
        // RX data (elevator state from COP)
        .lift1_floor (lift1_floor),
        .lift1_status(lift1_status),
        .lift2_floor (lift2_floor),
        .lift2_status(lift2_status),
        .up_ind      (rx_up_ind),
        .down_ind    (rx_down_ind),
        .rx_valid    (rx_valid)
    );

    // ---- LED outputs (matching interleaved switch order) ----
    always @(posedge clk) begin
        if (sys_reset) begin
            led <= 16'b0;
        end else begin
            led[0]  <= 1'b0;            // unused (Floor 0 has no DOWN)
            led[1]  <= rx_up_ind[0];    // Floor 0 UP
            led[2]  <= rx_down_ind[1];  // Floor 1 DOWN
            led[3]  <= rx_up_ind[1];    // Floor 1 UP
            led[4]  <= rx_down_ind[2];  // Floor 2 DOWN
            led[5]  <= rx_up_ind[2];    // Floor 2 UP
            led[6]  <= rx_down_ind[3];  // Floor 3 DOWN
            led[7]  <= rx_up_ind[3];    // Floor 3 UP
            led[8]  <= rx_down_ind[4];  // Floor 4 DOWN
            led[9]  <= rx_up_ind[4];    // Floor 4 UP
            led[10] <= rx_down_ind[5];  // Floor 5 DOWN
            led[11] <= rx_up_ind[5];    // Floor 5 UP
            led[12] <= rx_down_ind[6];  // Floor 6 DOWN
            led[13] <= rx_up_ind[6];    // Floor 6 UP
            led[14] <= rx_down_ind[7];  // Floor 7 DOWN
            led[15] <= 1'b0;            // unused (Floor 7 has no UP)
        end
    end

    // ---- 7-Segment Display (mirrored from COP) ----
    seven_seg_mux u_ssd (
        .clk         (clk),
        .lift1_floor (lift1_floor),
        .lift1_status(lift1_status),
        .lift2_floor (lift2_floor),
        .lift2_status(lift2_status),
        .seg         (seg),
        .an          (an)
    );

endmodule
