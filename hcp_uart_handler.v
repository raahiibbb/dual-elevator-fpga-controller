// ============================================================================
// hcp_uart_handler.v - HCP Board UART Protocol Handler (8-floor version)
// Transmits hall call requests to COP board, receives elevator state back.
//
// TX Protocol (HCP -> COP): 4 bytes sent periodically (~every 10ms)
//   Byte 0: 0xFF (sync marker)
//   Byte 1: up_request[7:0]   (bit 7 always 0 -> max 0x7F, never 0xFF)
//   Byte 2: down_request[7:0] (bit 0 always 0 -> max 0xFE, never 0xFF)
//   Byte 3: command byte {0000000, reset_flag} (max 0x01, never 0xFF)
//
// RX Protocol (COP -> HCP): 5 bytes
//   Byte 0: 0xFF (sync marker)
//   Byte 1: {0, 0, 0, lift1_floor[2:0], lift1_status[1:0]} (max 0x1F)
//   Byte 2: {0, 0, 0, lift2_floor[2:0], lift2_status[1:0]} (max 0x1F)
//   Byte 3: up_ind[7:0]   (bit 7 always 0 -> max 0x7F, never 0xFF)
//   Byte 4: down_ind[7:0] (bit 0 always 0 -> max 0xFE, never 0xFF)
// ============================================================================
module hcp_uart_handler #(
    parameter CLOCKS_PER_PULSE = 10417,   // 100MHz / 9600 baud
    parameter TX_INTERVAL      = 1000000  // ~10ms at 100MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    // UART physical lines
    output wire        uart_tx,
    input  wire        uart_rx,

    // Data TO transmit (hall calls -> COP)
    input  wire [7:0]  up_request,
    input  wire [7:0]  down_request,
    input  wire        reset_cmd,      // assert high to send reset command to COP

    // Data RECEIVED (elevator state from COP)
    output reg  [2:0]  lift1_floor,
    output reg  [1:0]  lift1_status,
    output reg  [2:0]  lift2_floor,
    output reg  [1:0]  lift2_status,
    output reg  [7:0]  up_ind,
    output reg  [7:0]  down_ind,
    output reg         rx_valid       // pulses when new state fully received
);

    // ---- TX side ----
    reg  [7:0]  tx_data;
    reg         tx_valid;
    wire        tx_busy;

    uart_tx #(.CLOCKS_PER_PULSE(CLOCKS_PER_PULSE)) u_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .data_in   (tx_data),
        .data_valid(tx_valid),
        .tx        (uart_tx),
        .tx_busy   (tx_busy)
    );

    // TX state machine - sends 4 bytes periodically
    localparam [2:0] TX_IDLE  = 3'd0;
    localparam [2:0] TX_BYTE0 = 3'd1;  // sync byte being sent
    localparam [2:0] TX_BYTE1 = 3'd2;  // up_request byte being sent
    localparam [2:0] TX_BYTE2 = 3'd3;  // down_request byte being sent
    localparam [2:0] TX_BYTE3 = 3'd4;  // command byte being sent

    reg [2:0]  tx_state;
    reg [$clog2(TX_INTERVAL)-1:0] tx_timer;
    reg [7:0]  tx_byte1_lat, tx_byte2_lat, tx_byte3_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state     <= TX_IDLE;
            tx_timer     <= 0;
            tx_data      <= 8'hFF;
            tx_valid     <= 1'b0;
            tx_byte1_lat <= 8'd0;
            tx_byte2_lat <= 8'd0;
            tx_byte3_lat <= 8'd0;
        end else begin
            tx_valid <= 1'b0; // default

            case (tx_state)
                TX_IDLE: begin
                    if (tx_timer >= TX_INTERVAL - 1) begin
                        tx_timer <= 0;
                        tx_byte1_lat <= up_request;
                        tx_byte2_lat <= down_request;
                        tx_byte3_lat <= {7'b0, reset_cmd};
                        // Send sync byte
                        tx_data  <= 8'hFF;
                        tx_valid <= 1'b1;
                        tx_state <= TX_BYTE0;
                    end else begin
                        tx_timer <= tx_timer + 1;
                    end
                end

                TX_BYTE0: begin
                    if (!tx_busy && !tx_valid) begin
                        tx_data  <= tx_byte1_lat;
                        tx_valid <= 1'b1;
                        tx_state <= TX_BYTE1;
                    end
                end

                TX_BYTE1: begin
                    if (!tx_busy && !tx_valid) begin
                        tx_data  <= tx_byte2_lat;
                        tx_valid <= 1'b1;
                        tx_state <= TX_BYTE2;
                    end
                end

                TX_BYTE2: begin
                    if (!tx_busy && !tx_valid) begin
                        tx_data  <= tx_byte3_lat;
                        tx_valid <= 1'b1;
                        tx_state <= TX_BYTE3;
                    end
                end

                TX_BYTE3: begin
                    if (!tx_busy && !tx_valid) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // ---- RX side ----
    wire [7:0] rx_data;
    wire       rx_ready;

    uart_rx #(.CLOCKS_PER_PULSE(CLOCKS_PER_PULSE)) u_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx        (uart_rx),
        .data_out  (rx_data),
        .data_ready(rx_ready)
    );

    // RX state machine - expects sync 0xFF then 4 data bytes
    localparam [2:0] RX_WAIT_SYNC  = 3'd0;
    localparam [2:0] RX_WAIT_DATA1 = 3'd1;  // lift1 state
    localparam [2:0] RX_WAIT_DATA2 = 3'd2;  // lift2 state
    localparam [2:0] RX_WAIT_DATA3 = 3'd3;  // up_ind
    localparam [2:0] RX_WAIT_DATA4 = 3'd4;  // down_ind

    reg [2:0] rx_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state     <= RX_WAIT_SYNC;
            lift1_floor  <= 3'b000;
            lift1_status <= 2'b01; // door open (matches reset)
            lift2_floor  <= 3'b000;
            lift2_status <= 2'b01;
            up_ind       <= 8'b0;
            down_ind     <= 8'b0;
            rx_valid     <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            if (rx_ready) begin
                case (rx_state)
                    RX_WAIT_SYNC: begin
                        if (rx_data == 8'hFF)
                            rx_state <= RX_WAIT_DATA1;
                    end
                    RX_WAIT_DATA1: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // treat as new frame
                        end else begin
                            lift1_floor  <= rx_data[4:2];
                            lift1_status <= rx_data[1:0];
                            rx_state     <= RX_WAIT_DATA2;
                        end
                    end
                    RX_WAIT_DATA2: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // re-sync
                        end else begin
                            lift2_floor  <= rx_data[4:2];
                            lift2_status <= rx_data[1:0];
                            rx_state     <= RX_WAIT_DATA3;
                        end
                    end
                    RX_WAIT_DATA3: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // re-sync
                        end else begin
                            up_ind   <= rx_data;
                            rx_state <= RX_WAIT_DATA4;
                        end
                    end
                    RX_WAIT_DATA4: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // re-sync
                        end else begin
                            down_ind <= rx_data;
                            rx_valid <= 1'b1;
                            rx_state <= RX_WAIT_SYNC;
                        end
                    end
                    default: rx_state <= RX_WAIT_SYNC;
                endcase
            end
        end
    end

endmodule
