// ============================================================================
// cop_top.v - Top Level for COP Board (Car Operating Panel / Inside Controller)
//
// This board runs the elevator controller logic.
// Local I/O: floor request switches, door buttons, floor request LEDs, 7-seg
// UART: receives hall calls from HCP, sends state to HCP
//
// Switch/Button assignment (8 floors):
//   SW[7:0]   = Lift 1 floor request (floors 0-7)
//   SW[15:8]  = Lift 2 floor request (floors 0-7)
//   BTNL      = Lift 1 door open
//   BTNR      = Lift 1 door close
//   BTNU      = Lift 2 door open
//   BTND      = Lift 2 door close
//   BTNC      = System reset
//
// LED assignment:
//   LED[7:0]  = Lift 1 floor request indicators
//   LED[15:8] = Lift 2 floor request indicators
//
// 7-Segment (AN[3:0]):
//   AN[3] = Lift 2 floor, AN[2] = Lift 2 status
//   AN[1] = Lift 1 floor, AN[0] = Lift 1 status
//
// PMOD JA:
//   JA[1] (C17) = UART TX to HCP
//   JA[2] (D18) = UART RX from HCP
// ============================================================================
module cop_top (
    input  wire        clk,          // 100 MHz system clock (E3)

    // Switches
    input  wire [15:0] sw,           // SW[15:0]

    // Push buttons
    input  wire        btnl,         // Lift 1 door open
    input  wire        btnr,         // Lift 1 door close
    input  wire        btnu,         // Lift 2 door open
    input  wire        btnd,         // Lift 2 door close
    input  wire        btnc,         // Reset

    // LEDs
    output wire [15:0] led,

    // Seven segment display
    output wire [6:0]  seg,
    output wire [7:0]  an,

    // PMOD JA for UART
    output wire        ja_tx,        // JA[1] - TX out
    input  wire        ja_rx         // JA[2] - RX in
);

    // ---- Reset generation (active-high for controller, active-low for UART) ----
    wire reset_rise;
    debouncer #(.DEBOUNCE_TICKS(500000)) db_reset (
        .clk(clk), .rst_n(1'b1), .btn_in(btnc),
        .btn_out(), .btn_rise(reset_rise), .btn_fall()
    );
    // Hold reset for a few cycles
    reg [3:0] rst_cnt = 4'hF;
    reg       sys_reset = 1'b1;
    wire      rst_n = ~sys_reset;

    // Remote reset from HCP via UART
    wire remote_reset;

    always @(posedge clk) begin
        if (reset_rise || remote_reset) begin
            rst_cnt   <= 4'hF;
            sys_reset <= 1'b1;
        end else if (rst_cnt > 0) begin
            rst_cnt <= rst_cnt - 1;
        end else begin
            sys_reset <= 1'b0;
        end
    end

    // ---- Debounce switches (rising edge = request pulse) ----
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

    // ---- Debounce push buttons ----
    wire btnl_rise, btnr_rise, btnu_rise, btnd_rise;
    debouncer #(.DEBOUNCE_TICKS(500000)) db_btnl (
        .clk(clk), .rst_n(rst_n), .btn_in(btnl),
        .btn_out(), .btn_rise(btnl_rise), .btn_fall()
    );
    debouncer #(.DEBOUNCE_TICKS(500000)) db_btnr (
        .clk(clk), .rst_n(rst_n), .btn_in(btnr),
        .btn_out(), .btn_rise(btnr_rise), .btn_fall()
    );
    debouncer #(.DEBOUNCE_TICKS(500000)) db_btnu (
        .clk(clk), .rst_n(rst_n), .btn_in(btnu),
        .btn_out(), .btn_rise(btnu_rise), .btn_fall()
    );
    debouncer #(.DEBOUNCE_TICKS(500000)) db_btnd (
        .clk(clk), .rst_n(rst_n), .btn_in(btnd),
        .btn_out(), .btn_rise(btnd_rise), .btn_fall()
    );

    // ---- UART Handler ----
    wire [7:0] uart_up_req, uart_down_req;
    wire [2:0] lift1_floor, lift2_floor;
    wire [1:0] lift1_status, lift2_status;
    wire [7:0] up_ind, down_ind;
    wire [7:0] lift1_floor_ind, lift2_floor_ind;

    cop_uart_handler #(
        .CLOCKS_PER_PULSE(10417),
        .TX_INTERVAL(1000000)
    ) u_uart (
        .clk          (clk),
        .rst_n        (rst_n),
        .uart_tx      (ja_tx),
        .uart_rx      (ja_rx),
        // TX data (elevator state -> HCP)
        .lift1_floor  (lift1_floor),
        .lift1_status (lift1_status),
        .lift2_floor  (lift2_floor),
        .lift2_status (lift2_status),
        .up_ind       (up_ind),
        .down_ind     (down_ind),
        // RX data (hall calls from HCP)
        .up_request   (uart_up_req),
        .down_request (uart_down_req),
        .remote_reset (remote_reset)
    );

    // ---- Elevator Controller ----
    elevator_controller u_elev (
        .clk            (clk),
        .reset          (sys_reset),
        // Hall calls from UART
        .up_request     (uart_up_req),
        .down_request   (uart_down_req),
        // Car calls from local switches
        .lift1_floor_sw (sw_rise[7:0]),
        .lift2_floor_sw (sw_rise[15:8]),
        .door_open_btn  ({btnu_rise, btnl_rise}),
        .door_close_btn ({btnd_rise, btnr_rise}),
        // Indicators
        .up_request_ind (up_ind),
        .down_request_ind(down_ind),
        .lift1_floor_ind(lift1_floor_ind),
        .lift2_floor_ind(lift2_floor_ind),
        // Display
        .lift1_cur_floor (lift1_floor),
        .lift1_cur_status(lift1_status),
        .lift2_cur_floor (lift2_floor),
        .lift2_cur_status(lift2_status)
    );

    // ---- LED outputs ----
    assign led[7:0]  = lift1_floor_ind;
    assign led[15:8] = lift2_floor_ind;

    // ---- 7-Segment Display ----
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
