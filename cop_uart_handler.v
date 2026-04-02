// ============================================================================
// cop_uart_handler.v - COP Board UART Protocol Handler (8-floor version)
// Receives hall call requests from HCP board, transmits elevator state back.
//
// TX Protocol (COP -> HCP): 5 bytes sent periodically (~every 10ms)
//   Byte 0: 0xFF (sync marker)
//   Byte 1: {0, 0, 0, lift1_floor[2:0], lift1_status[1:0]} (max 0x1F)
//   Byte 2: {0, 0, 0, lift2_floor[2:0], lift2_status[1:0]} (max 0x1F)
//   Byte 3: up_ind[7:0]   (bit 7 always 0 -> max 0x7F, never 0xFF)
//   Byte 4: down_ind[7:0] (bit 0 always 0 -> max 0xFE, never 0xFF)
//
// RX Protocol (HCP -> COP): 4 bytes
//   Byte 0: 0xFF (sync marker)
//   Byte 1: up_request[7:0]   (bit 7 always 0 -> max 0x7F, never 0xFF)
//   Byte 2: down_request[7:0] (bit 0 always 0 -> max 0xFE, never 0xFF)
//   Byte 3: command byte {0000000, reset_flag} (max 0x01, never 0xFF)
// ============================================================================
module cop_uart_handler #(
    parameter CLOCKS_PER_PULSE = 10417,   // 100MHz / 9600 baud
    parameter TX_INTERVAL      = 1000000  // ~10ms at 100MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    // UART physical lines
    output wire        uart_tx,
    input  wire        uart_rx,

    // Data TO transmit (elevator state -> HCP)
    input  wire [2:0]  lift1_floor,
    input  wire [1:0]  lift1_status,
    input  wire [2:0]  lift2_floor,
    input  wire [1:0]  lift2_status,
    input  wire [7:0]  up_ind,
    input  wire [7:0]  down_ind,

    // Data RECEIVED (hall calls from HCP)
    output reg  [7:0]  up_request,
    output reg  [7:0]  down_request,
    output reg         remote_reset    // pulse when HCP sends reset command
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

    // TX state machine - sends 5 bytes periodically
    localparam [2:0] TX_IDLE  = 3'd0;
    localparam [2:0] TX_BYTE0 = 3'd1;  // waiting for sync byte to finish
    localparam [2:0] TX_BYTE1 = 3'd2;  // waiting for lift1 byte to finish
    localparam [2:0] TX_BYTE2 = 3'd3;  // waiting for lift2 byte to finish
    localparam [2:0] TX_BYTE3 = 3'd4;  // waiting for up_ind byte to finish
    localparam [2:0] TX_BYTE4 = 3'd5;  // waiting for down_ind byte to finish

    reg [2:0]  tx_state;
    reg [$clog2(TX_INTERVAL)-1:0] tx_timer;

    // Latch data at start of transmission to avoid mid-frame changes
    reg [7:0] tx_byte1_lat, tx_byte2_lat, tx_byte3_lat, tx_byte4_lat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state     <= TX_IDLE;
            tx_timer     <= 0;
            tx_data      <= 8'hFF;
            tx_valid     <= 1'b0;
            tx_byte1_lat <= 8'd0;
            tx_byte2_lat <= 8'd0;
            tx_byte3_lat <= 8'd0;
            tx_byte4_lat <= 8'd0;
        end else begin
            tx_valid <= 1'b0; // default

            case (tx_state)
                TX_IDLE: begin
                    if (tx_timer >= TX_INTERVAL - 1) begin
                        tx_timer <= 0;
                        // Latch current state
                        tx_byte1_lat <= {3'b000, lift1_floor, lift1_status};
                        tx_byte2_lat <= {3'b000, lift2_floor, lift2_status};
                        tx_byte3_lat <= up_ind;
                        tx_byte4_lat <= down_ind;
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
                        tx_data  <= tx_byte4_lat;
                        tx_valid <= 1'b1;
                        tx_state <= TX_BYTE4;
                    end
                end

                TX_BYTE4: begin
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

    // RX state machine - expects sync 0xFF then 3 data bytes
    // Staging registers hold intermediate bytes until full frame arrives,
    // then all outputs are pulsed simultaneously for one cycle.
    localparam [1:0] RX_WAIT_SYNC  = 2'd0;
    localparam [1:0] RX_WAIT_DATA1 = 2'd1;  // up_request
    localparam [1:0] RX_WAIT_DATA2 = 2'd2;  // down_request
    localparam [1:0] RX_WAIT_DATA3 = 2'd3;  // command byte

    reg [1:0] rx_state;
    reg [7:0] rx_up_lat;    // staging register for up_request
    reg [7:0] rx_down_lat;  // staging register for down_request

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state     <= RX_WAIT_SYNC;
            up_request   <= 8'b0;
            down_request <= 8'b0;
            remote_reset <= 1'b0;
            rx_up_lat    <= 8'b0;
            rx_down_lat  <= 8'b0;
        end else begin
            // Clear pulse outputs each cycle
            up_request   <= 8'b0;
            down_request <= 8'b0;
            remote_reset <= 1'b0;

            if (rx_ready) begin
                case (rx_state)
                    RX_WAIT_SYNC: begin
                        if (rx_data == 8'hFF)
                            rx_state <= RX_WAIT_DATA1;
                    end
                    RX_WAIT_DATA1: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // re-sync
                        end else begin
                            rx_up_lat <= rx_data;       // latch, don't output yet
                            rx_state  <= RX_WAIT_DATA2;
                        end
                    end
                    RX_WAIT_DATA2: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // re-sync
                        end else begin
                            rx_down_lat <= rx_data;     // latch, don't output yet
                            rx_state    <= RX_WAIT_DATA3;
                        end
                    end
                    RX_WAIT_DATA3: begin
                        if (rx_data == 8'hFF) begin
                            rx_state <= RX_WAIT_DATA1; // re-sync
                        end else begin
                            // Full frame received - output all at once
                            up_request   <= rx_up_lat;
                            down_request <= rx_down_lat;
                            remote_reset <= rx_data[0]; // bit 0 = reset flag
                            rx_state     <= RX_WAIT_SYNC;
                        end
                    end
                endcase
            end
        end
    end

endmodule
