// ============================================================================
// elevator_controller.v - Dual Elevator Controller (8 Floors: 0-7)
//
// Behaviour summary:
//   - A request (hall call UP/DOWN or car floor request) is captured and the
//     corresponding LED turns ON immediately.
//   - The assigned lift travels to the target floor (3 s per floor).
//   - After arriving, the door stays CLOSED for 1 s (ARRIVE_WAIT).
//   - Then the door OPENS for 3 s (DOOR_OPEN).
//   - When the door CLOSES again, the request is marked served and the LED
//     turns OFF.  (NOT when the door first opens.)
//
// Hall call at same floor:
//   - If a lift is already at the hall-call floor and is stoppable (idle,
//     arrive-wait, or door-open), the door opens IMMEDIATELY (no arrive-wait)
//     for that lift only.  If BOTH lifts are at the same floor, only ONE
//     serves the request (lift 1 has priority).
//   - DIRECTION FILTER for PH_DOOR_OPEN: a lift with its door open only
//     serves a hall call if its last travel direction matches (or is idle).
//     A lift that arrived going DOWN won't serve an UP call at the same
//     floor (the other lift gets it instead).  Timer resets to give a
//     full 3 s of open time from the moment the new call is captured.
//
// Door buttons from COP:
//   - OPEN pressed while IDLE:      manual door open (3 s).
//   - OPEN pressed while DOOR_OPEN: timer resets → 3 s more from now.
//   - CLOSE pressed while DOOR_OPEN: timer fast-forwards so the door
//     closes in at most 1 s.  Works for both manual and service opens.
//     If less than 1 s already remains, door closes on its own schedule.
//
// Hall call assignment when NO lift is at the floor:
//   Priority (for UP calls; DOWN is symmetric with PH_MOVING_DOWN):
//     1. Lift moving in the SAME direction AND target floor is AHEAD:
//        (UP call: lift_floor <= target; DOWN call: lift_floor >= target)
//        - Only one qualifies         -> that lift picks it up on the way.
//        - Both qualify               -> nearer lift picks it up.
//        - If a lift is moving same dir but target is BEHIND, it does NOT
//          qualify; the call falls through to lower-priority rules.
//     2. If no lift is moving in the same direction:
//        - One lift idle, other busy -> idle lift picks it up.
//        - Both idle                 -> nearer lift (tie: lift 1).
//        - Both busy (opposite dir)  -> nearer lift (tie: lift 1).
//
//   A hall call assigned to a same-direction lift is served either on the way
//   (if the floor is ahead) or after the lift finishes its current run and
//   reverses (if the floor is behind).
//
// Duplicate-assignment guard:
//   Once a hall call is assigned to one lift, it is NOT re-assigned on
//   subsequent UART pulses.  The HCP keeps sending until the indicator goes
//   high (one UART round-trip, ~20 ms).  The guard prevents both lifts from
//   being assigned the same call.
// ============================================================================
module elevator_controller (
    input  wire        clk,
    input  wire        reset,
    input  wire [7:0]  up_request,       // one-cycle pulses from COP UART RX
    input  wire [7:0]  down_request,
    input  wire [7:0]  lift1_floor_sw,   // one-cycle pulses from COP switches
    input  wire [7:0]  lift2_floor_sw,
    input  wire [1:0]  door_open_btn,    // {lift2, lift1}
    input  wire [1:0]  door_close_btn,   // {lift2, lift1}
    output reg  [7:0]  up_request_ind,
    output reg  [7:0]  down_request_ind,
    output reg  [7:0]  lift1_floor_ind,
    output reg  [7:0]  lift2_floor_ind,
    output wire [2:0]  lift1_cur_floor,
    output wire [1:0]  lift1_cur_status,
    output wire [2:0]  lift2_cur_floor,
    output wire [1:0]  lift2_cur_status
);

    // ---- Status encoding (matches seven_seg_mux) ----
    localparam [1:0] ST_CLOSED = 2'b00;
    localparam [1:0] ST_OPEN   = 2'b01;
    localparam [1:0] ST_UP     = 2'b10;
    localparam [1:0] ST_DOWN   = 2'b11;

    // ---- Phase encoding ----
    localparam [2:0] PH_IDLE        = 3'd0;
    localparam [2:0] PH_MOVING_UP   = 3'd1;
    localparam [2:0] PH_MOVING_DOWN = 3'd2;
    localparam [2:0] PH_ARRIVE_WAIT = 3'd3; // 1 s closed after reaching floor
    localparam [2:0] PH_DOOR_OPEN   = 3'd4; // 3 s open; requests cleared on close

    // ---- Timing constants (100 MHz clock) ----
    localparam [28:0] FLOOR_TRAVEL   = 29'd300_000_000; // 3 s per floor
    localparam [28:0] ARRIVE_WAIT_T  = 29'd100_000_000; // 1 s closed after arrival
    localparam [28:0] DOOR_OPEN_HOLD = 29'd300_000_000; // 3 s door open
    localparam [28:0] DOOR_CLOSE_FAST= 29'd200_000_000; // timer value that leaves 1 s remaining

    // ---- Per-lift state ----
    reg [2:0]  lift1_floor,    lift2_floor;
    reg [1:0]  lift1_status_r, lift2_status_r;
    reg [2:0]  lift1_phase,    lift2_phase;
    reg [28:0] timer1,         timer2;
    reg [1:0]  last_dir1,      last_dir2;      // 00=idle, 01=up, 10=down
    reg        service_active1, service_active2;
    reg        manual_open1,    manual_open2;

    // Door control flags (blocking assignments, used same cycle by FSM)
    // These are set in sections 1-4 and read by the FSM in section 5/6
    // to avoid non-blocking assignment conflicts on timer registers.
    reg        door_extend1, door_extend2;       // reset timer to 0 (extend open)
    reg        door_fast_close1, door_fast_close2; // jump timer to DOOR_CLOSE_FAST

    // Hall-call assignment tracking (one bit per floor per lift per direction)
    reg [7:0] lift1_up_assigned,   lift2_up_assigned;
    reg [7:0] lift1_down_assigned, lift2_down_assigned;

    // Combinational scratch
    reg [7:0] reqs1, reqs2;
    reg [2:0] next_floor1, next_floor2;
    integer   dist1, dist2;

    assign lift1_cur_floor  = lift1_floor;
    assign lift2_cur_floor  = lift2_floor;
    assign lift1_cur_status = lift1_status_r;
    assign lift2_cur_status = lift2_status_r;

    // ====================================================================
    // Helper functions (8-bit request vectors, 3-bit floor numbers)
    // ====================================================================

    function has_any;
        input [7:0] reqs;
        begin has_any = |reqs; end
    endfunction

    function has_at_floor;
        input [7:0] reqs;
        input [2:0] floor;
        begin has_at_floor = reqs[floor]; end
    endfunction

    function has_above;
        input [7:0] reqs;
        input [2:0] floor;
        begin
            case (floor)
                3'd0: has_above = |reqs[7:1];
                3'd1: has_above = |reqs[7:2];
                3'd2: has_above = |reqs[7:3];
                3'd3: has_above = |reqs[7:4];
                3'd4: has_above = |reqs[7:5];
                3'd5: has_above = |reqs[7:6];
                3'd6: has_above = reqs[7];
                default: has_above = 1'b0;
            endcase
        end
    endfunction

    function has_below;
        input [7:0] reqs;
        input [2:0] floor;
        begin
            case (floor)
                3'd0: has_below = 1'b0;
                3'd1: has_below = reqs[0];
                3'd2: has_below = |reqs[1:0];
                3'd3: has_below = |reqs[2:0];
                3'd4: has_below = |reqs[3:0];
                3'd5: has_below = |reqs[4:0];
                3'd6: has_below = |reqs[5:0];
                default: has_below = |reqs[6:0];
            endcase
        end
    endfunction

    function [2:0] nearest_above;
        input [7:0] reqs;
        input [2:0] floor;
        integer j;
        begin
            nearest_above = floor;
            for (j = 1; j <= 7; j = j + 1) begin
                if ((floor + j <= 7) && reqs[floor + j] && (nearest_above == floor))
                    nearest_above = floor + j;
            end
        end
    endfunction

    function [2:0] nearest_below;
        input [7:0] reqs;
        input [2:0] floor;
        integer j;
        begin
            nearest_below = floor;
            for (j = 1; j <= 7; j = j + 1) begin
                if ((floor >= j) && reqs[floor - j] && (nearest_below == floor))
                    nearest_below = floor - j;
            end
        end
    endfunction

    function [2:0] nearest_any;
        input [7:0] reqs;
        input [2:0] floor;
        integer j;
        reg [2:0] best;
        integer best_dist;
        integer d;
        begin
            best = floor;
            best_dist = 8;
            for (j = 0; j <= 7; j = j + 1) begin
                if (reqs[j]) begin
                    d = (j > floor) ? (j - floor) : (floor - j);
                    if (d < best_dist) begin
                        best_dist = d;
                        best = j[2:0];
                    end
                end
            end
            nearest_any = best;
        end
    endfunction

    // Check: lift at target floor, stoppable, direction-compatible for UP call.
    // PH_IDLE and PH_ARRIVE_WAIT always match (no direction conflict).
    // PH_DOOR_OPEN matches only if lift's last direction was NOT downward,
    // so a lift that arrived going DOWN won't hijack an UP call.
    function lift_stoppable_for_up;
        input [2:0] phase;
        input [2:0] lfloor;
        input [2:0] target;
        input [1:0] ldir;
        begin
            lift_stoppable_for_up = (lfloor == target) &&
                ((phase == PH_IDLE) || (phase == PH_ARRIVE_WAIT) ||
                 ((phase == PH_DOOR_OPEN) && (ldir != 2'b10)));
        end
    endfunction

    // Same as above but for DOWN calls: PH_DOOR_OPEN only if NOT last going UP.
    function lift_stoppable_for_down;
        input [2:0] phase;
        input [2:0] lfloor;
        input [2:0] target;
        input [1:0] ldir;
        begin
            lift_stoppable_for_down = (lfloor == target) &&
                ((phase == PH_IDLE) || (phase == PH_ARRIVE_WAIT) ||
                 ((phase == PH_DOOR_OPEN) && (ldir != 2'b01)));
        end
    endfunction

    integer fi; // loop variable for hall-call assignment

    // ====================================================================
    // Main sequential logic
    // ====================================================================
    always @(posedge clk) begin
        if (reset) begin
            lift1_floor         <= 3'd0;
            lift2_floor         <= 3'd0;
            lift1_status_r      <= ST_CLOSED;
            lift2_status_r      <= ST_CLOSED;
            lift1_phase         <= PH_IDLE;
            lift2_phase         <= PH_IDLE;
            timer1              <= 29'd0;
            timer2              <= 29'd0;
            last_dir1           <= 2'b00;
            last_dir2           <= 2'b00;
            service_active1     <= 1'b0;
            service_active2     <= 1'b0;
            manual_open1        <= 1'b0;
            manual_open2        <= 1'b0;
            lift1_floor_ind     <= 8'b0;
            lift2_floor_ind     <= 8'b0;
            lift1_up_assigned   <= 8'b0;
            lift2_up_assigned   <= 8'b0;
            lift1_down_assigned <= 8'b0;
            lift2_down_assigned <= 8'b0;
            up_request_ind      <= 8'b0;
            down_request_ind    <= 8'b0;
        end else begin

            // ---- Initialize door control flags (blocking) ----
            // These must be set before the FSM reads them later this cycle.
            door_extend1     = 1'b0;
            door_extend2     = 1'b0;
            door_fast_close1 = 1'b0;
            door_fast_close2 = 1'b0;

            // ============================================================
            // 1. Capture car (floor) requests from COP switches
            // ============================================================
            lift1_floor_ind <= lift1_floor_ind | lift1_floor_sw;
            lift2_floor_ind <= lift2_floor_ind | lift2_floor_sw;

            // ============================================================
            // 2. Hall-call UP assignment (floors 0-6; floor 7 has no UP)
            //
            //    Guard: skip if already assigned to either lift.
            //    Priority:
            //      a) Lift at same floor & stoppable  -> immediate door open
            //      b) Lift moving UP (same direction)  -> picks it up on the way or after
            //      c) Lift idle                        -> starts moving to hall-call floor
            //      d) Fallback (both opposite dir)     -> nearer lift
            //    Only ONE lift is assigned per hall call.
            // ============================================================
            for (fi = 0; fi <= 6; fi = fi + 1) begin
                if (up_request[fi] && !lift1_up_assigned[fi] && !lift2_up_assigned[fi]) begin
                    // --- (a) Same floor, stoppable, direction-compatible ---
                    // PH_DOOR_OPEN only matches if lift was NOT going DOWN
                    if (lift_stoppable_for_up(lift1_phase, lift1_floor, fi[2:0], last_dir1)) begin
                        lift1_up_assigned[fi] <= 1'b1;
                        service_active1       <= 1'b1;
                        manual_open1          <= 1'b0;
                        lift1_phase           <= PH_DOOR_OPEN;
                        lift1_status_r        <= ST_OPEN;
                        door_extend1           = 1'b1;  // flag: reset timer in FSM
                    end else if (lift_stoppable_for_up(lift2_phase, lift2_floor, fi[2:0], last_dir2)) begin
                        lift2_up_assigned[fi] <= 1'b1;
                        service_active2       <= 1'b1;
                        manual_open2          <= 1'b0;
                        lift2_phase           <= PH_DOOR_OPEN;
                        lift2_status_r        <= ST_OPEN;
                        door_extend2           = 1'b1;  // flag: reset timer in FSM
                    // --- (b) Same-direction preference (floor must be AHEAD) ---
                    // For UP calls: "ahead" means lift_floor <= target_floor
                    end else if ((lift1_phase == PH_MOVING_UP) && (lift1_floor <= fi[2:0])
                              && !((lift2_phase == PH_MOVING_UP) && (lift2_floor <= fi[2:0]))) begin
                        lift1_up_assigned[fi] <= 1'b1;
                    end else if ((lift2_phase == PH_MOVING_UP) && (lift2_floor <= fi[2:0])
                              && !((lift1_phase == PH_MOVING_UP) && (lift1_floor <= fi[2:0]))) begin
                        lift2_up_assigned[fi] <= 1'b1;
                    end else if ((lift1_phase == PH_MOVING_UP) && (lift1_floor <= fi[2:0])
                              && (lift2_phase == PH_MOVING_UP) && (lift2_floor <= fi[2:0])) begin
                        dist1 = (lift1_floor > fi[2:0]) ? (lift1_floor - fi[2:0]) : (fi[2:0] - lift1_floor);
                        dist2 = (lift2_floor > fi[2:0]) ? (lift2_floor - fi[2:0]) : (fi[2:0] - lift2_floor);
                        if (dist1 <= dist2) lift1_up_assigned[fi] <= 1'b1;
                        else                lift2_up_assigned[fi] <= 1'b1;
                    // --- (c) Idle preference ---
                    end else if ((lift1_phase == PH_IDLE) && (lift2_phase != PH_IDLE)) begin
                        lift1_up_assigned[fi] <= 1'b1;
                    end else if ((lift2_phase == PH_IDLE) && (lift1_phase != PH_IDLE)) begin
                        lift2_up_assigned[fi] <= 1'b1;
                    // --- (d) Both idle OR both opposite -> nearest ---
                    end else begin
                        dist1 = (lift1_floor > fi[2:0]) ? (lift1_floor - fi[2:0]) : (fi[2:0] - lift1_floor);
                        dist2 = (lift2_floor > fi[2:0]) ? (lift2_floor - fi[2:0]) : (fi[2:0] - lift2_floor);
                        if (dist1 <= dist2) lift1_up_assigned[fi] <= 1'b1;
                        else                lift2_up_assigned[fi] <= 1'b1;
                    end
                end
            end

            // ============================================================
            // 3. Hall-call DOWN assignment (floors 1-7; floor 0 has no DOWN)
            //    Same logic structure as UP, but checks PH_MOVING_DOWN.
            // ============================================================
            for (fi = 1; fi <= 7; fi = fi + 1) begin
                if (down_request[fi] && !lift1_down_assigned[fi] && !lift2_down_assigned[fi]) begin
                    // --- (a) Same floor, stoppable, direction-compatible ---
                    // PH_DOOR_OPEN only matches if lift was NOT going UP
                    if (lift_stoppable_for_down(lift1_phase, lift1_floor, fi[2:0], last_dir1)) begin
                        lift1_down_assigned[fi] <= 1'b1;
                        service_active1         <= 1'b1;
                        manual_open1            <= 1'b0;
                        lift1_phase             <= PH_DOOR_OPEN;
                        lift1_status_r          <= ST_OPEN;
                        door_extend1             = 1'b1;  // flag: reset timer in FSM
                    end else if (lift_stoppable_for_down(lift2_phase, lift2_floor, fi[2:0], last_dir2)) begin
                        lift2_down_assigned[fi] <= 1'b1;
                        service_active2         <= 1'b1;
                        manual_open2            <= 1'b0;
                        lift2_phase             <= PH_DOOR_OPEN;
                        lift2_status_r          <= ST_OPEN;
                        door_extend2             = 1'b1;  // flag: reset timer in FSM
                    // --- (b) Same-direction preference (floor must be AHEAD) ---
                    // For DOWN calls: "ahead" means lift_floor >= target_floor
                    end else if ((lift1_phase == PH_MOVING_DOWN) && (lift1_floor >= fi[2:0])
                              && !((lift2_phase == PH_MOVING_DOWN) && (lift2_floor >= fi[2:0]))) begin
                        lift1_down_assigned[fi] <= 1'b1;
                    end else if ((lift2_phase == PH_MOVING_DOWN) && (lift2_floor >= fi[2:0])
                              && !((lift1_phase == PH_MOVING_DOWN) && (lift1_floor >= fi[2:0]))) begin
                        lift2_down_assigned[fi] <= 1'b1;
                    end else if ((lift1_phase == PH_MOVING_DOWN) && (lift1_floor >= fi[2:0])
                              && (lift2_phase == PH_MOVING_DOWN) && (lift2_floor >= fi[2:0])) begin
                        dist1 = (lift1_floor > fi[2:0]) ? (lift1_floor - fi[2:0]) : (fi[2:0] - lift1_floor);
                        dist2 = (lift2_floor > fi[2:0]) ? (lift2_floor - fi[2:0]) : (fi[2:0] - lift2_floor);
                        if (dist1 <= dist2) lift1_down_assigned[fi] <= 1'b1;
                        else                lift2_down_assigned[fi] <= 1'b1;
                    // --- (c) Idle preference ---
                    end else if ((lift1_phase == PH_IDLE) && (lift2_phase != PH_IDLE)) begin
                        lift1_down_assigned[fi] <= 1'b1;
                    end else if ((lift2_phase == PH_IDLE) && (lift1_phase != PH_IDLE)) begin
                        lift2_down_assigned[fi] <= 1'b1;
                    // --- (d) Both idle OR both opposite -> nearest ---
                    end else begin
                        dist1 = (lift1_floor > fi[2:0]) ? (lift1_floor - fi[2:0]) : (fi[2:0] - lift1_floor);
                        dist2 = (lift2_floor > fi[2:0]) ? (lift2_floor - fi[2:0]) : (fi[2:0] - lift2_floor);
                        if (dist1 <= dist2) lift1_down_assigned[fi] <= 1'b1;
                        else                lift2_down_assigned[fi] <= 1'b1;
                    end
                end
            end

            // ============================================================
            // 4. Door buttons from COP
            //
            //    OPEN  + IDLE      → manual open (3 s, closeable by close btn)
            //    OPEN  + DOOR_OPEN → flag to reset timer (extend by full 3 s)
            //    CLOSE + DOOR_OPEN → flag to fast-forward timer (close in ≤1 s)
            //
            //    Flags are read by the FSM's PH_DOOR_OPEN case to avoid
            //    non-blocking assignment conflicts on the timer register.
            // ============================================================
            // -- Lift 1 door open --
            if (door_open_btn[0]) begin
                if (lift1_phase == PH_IDLE) begin
                    lift1_phase     <= PH_DOOR_OPEN;
                    lift1_status_r  <= ST_OPEN;
                    door_extend1     = 1'b1;  // FSM will reset timer
                    service_active1 <= 1'b0;
                    manual_open1    <= 1'b1;
                end else if (lift1_phase == PH_DOOR_OPEN) begin
                    door_extend1 = 1'b1;      // extend: full 3 s from now
                end
            end
            // -- Lift 2 door open --
            if (door_open_btn[1]) begin
                if (lift2_phase == PH_IDLE) begin
                    lift2_phase     <= PH_DOOR_OPEN;
                    lift2_status_r  <= ST_OPEN;
                    door_extend2     = 1'b1;  // FSM will reset timer
                    service_active2 <= 1'b0;
                    manual_open2    <= 1'b1;
                end else if (lift2_phase == PH_DOOR_OPEN) begin
                    door_extend2 = 1'b1;      // extend: full 3 s from now
                end
            end
            // -- Lift 1 door close --
            // Works in ANY PH_DOOR_OPEN (manual or service).
            // Flag tells FSM to jump timer so at most 1 s remains.
            if (door_close_btn[0] && (lift1_phase == PH_DOOR_OPEN)) begin
                door_fast_close1 = 1'b1;
                manual_open1    <= 1'b0;
            end
            // -- Lift 2 door close --
            if (door_close_btn[1] && (lift2_phase == PH_DOOR_OPEN)) begin
                door_fast_close2 = 1'b1;
                manual_open2    <= 1'b0;
            end

            // ============================================================
            // 5. Lift 1 FSM
            // ============================================================
            reqs1 = lift1_floor_ind | lift1_up_assigned | lift1_down_assigned;
            next_floor1 = nearest_any(reqs1, lift1_floor);

            case (lift1_phase)
                PH_IDLE: begin
                    lift1_status_r <= ST_CLOSED;
                    timer1 <= 29'd0;
                    if (has_at_floor(reqs1, lift1_floor)) begin
                        // Hall call at current floor -> skip arrive-wait, open immediately
                        if (has_at_floor(lift1_up_assigned | lift1_down_assigned, lift1_floor)) begin
                            lift1_phase     <= PH_DOOR_OPEN;
                            lift1_status_r  <= ST_OPEN;
                            service_active1 <= 1'b1;
                            manual_open1    <= 1'b0;
                        end else begin
                            // Car request at current floor -> normal arrive-wait first
                            lift1_phase     <= PH_ARRIVE_WAIT;
                            lift1_status_r  <= ST_CLOSED;
                            service_active1 <= 1'b1;
                            manual_open1    <= 1'b0;
                        end
                        timer1 <= 29'd0;
                    end else if (has_any(reqs1)) begin
                        if (next_floor1 > lift1_floor) begin
                            lift1_phase    <= PH_MOVING_UP;
                            lift1_status_r <= ST_UP;
                            last_dir1      <= 2'b01;
                            timer1         <= 29'd0;
                        end else if (next_floor1 < lift1_floor) begin
                            lift1_phase    <= PH_MOVING_DOWN;
                            lift1_status_r <= ST_DOWN;
                            last_dir1      <= 2'b10;
                            timer1         <= 29'd0;
                        end
                    end
                end

                PH_MOVING_UP: begin
                    lift1_status_r <= ST_UP;
                    if (timer1 < FLOOR_TRAVEL) begin
                        timer1 <= timer1 + 1'b1;
                    end else begin
                        timer1 <= 29'd0;
                        lift1_floor <= lift1_floor + 1'b1;
                        if (has_at_floor(reqs1, lift1_floor + 1'b1)) begin
                            lift1_phase     <= PH_ARRIVE_WAIT;
                            lift1_status_r  <= ST_CLOSED;
                            service_active1 <= 1'b1;
                            manual_open1    <= 1'b0;
                        end else if (!has_above(reqs1, lift1_floor + 1'b1)) begin
                            if (has_below(reqs1, lift1_floor + 1'b1)) begin
                                lift1_phase    <= PH_MOVING_DOWN;
                                lift1_status_r <= ST_DOWN;
                                last_dir1      <= 2'b10;
                            end else begin
                                lift1_phase    <= PH_IDLE;
                                lift1_status_r <= ST_CLOSED;
                            end
                        end
                    end
                end

                PH_MOVING_DOWN: begin
                    lift1_status_r <= ST_DOWN;
                    if (timer1 < FLOOR_TRAVEL) begin
                        timer1 <= timer1 + 1'b1;
                    end else begin
                        timer1 <= 29'd0;
                        lift1_floor <= lift1_floor - 1'b1;
                        if (has_at_floor(reqs1, lift1_floor - 1'b1)) begin
                            lift1_phase     <= PH_ARRIVE_WAIT;
                            lift1_status_r  <= ST_CLOSED;
                            service_active1 <= 1'b1;
                            manual_open1    <= 1'b0;
                        end else if (!has_below(reqs1, lift1_floor - 1'b1)) begin
                            if (has_above(reqs1, lift1_floor - 1'b1)) begin
                                lift1_phase    <= PH_MOVING_UP;
                                lift1_status_r <= ST_UP;
                                last_dir1      <= 2'b01;
                            end else begin
                                lift1_phase    <= PH_IDLE;
                                lift1_status_r <= ST_CLOSED;
                            end
                        end
                    end
                end

                PH_ARRIVE_WAIT: begin
                    lift1_status_r <= ST_CLOSED;
                    if (timer1 < ARRIVE_WAIT_T) begin
                        timer1 <= timer1 + 1'b1;
                    end else begin
                        timer1         <= 29'd0;
                        lift1_phase    <= PH_DOOR_OPEN;
                        lift1_status_r <= ST_OPEN;
                    end
                end

                // Door open for 3 s.  ALL requests at this floor are cleared
                // when the door CLOSES (timer expires), not when it opens.
                // Flags from sections 2-4 override normal timer counting:
                //   door_extend1     → reset timer to 0 (full 3 s extension)
                //   door_fast_close1 → jump timer to leave ≤1 s remaining
                //   Open beats close if both pressed simultaneously.
                PH_DOOR_OPEN: begin
                    lift1_status_r <= ST_OPEN;
                    if (door_extend1) begin
                        // Door open extension (button or new hall call at floor)
                        timer1 <= 29'd0;
                    end else if (door_fast_close1 && (timer1 < DOOR_CLOSE_FAST)) begin
                        // Door close fast-forward (only if >1 s remains)
                        timer1 <= DOOR_CLOSE_FAST;
                    end else if (timer1 < DOOR_OPEN_HOLD) begin
                        timer1 <= timer1 + 1'b1;
                    end else begin
                        timer1         <= 29'd0;
                        lift1_phase    <= PH_IDLE;
                        lift1_status_r <= ST_CLOSED;
                        manual_open1   <= 1'b0;
                        if (service_active1) begin
                            // Serve: clear car request + both hall-call directions
                            lift1_floor_ind[lift1_floor]     <= 1'b0;
                            lift1_up_assigned[lift1_floor]   <= 1'b0;
                            lift1_down_assigned[lift1_floor] <= 1'b0;
                            service_active1                  <= 1'b0;
                        end
                    end
                end

                default: begin
                    lift1_phase    <= PH_IDLE;
                    lift1_status_r <= ST_CLOSED;
                    timer1         <= 29'd0;
                end
            endcase

            // ============================================================
            // 6. Lift 2 FSM (identical structure to Lift 1)
            // ============================================================
            reqs2 = lift2_floor_ind | lift2_up_assigned | lift2_down_assigned;
            next_floor2 = nearest_any(reqs2, lift2_floor);

            case (lift2_phase)
                PH_IDLE: begin
                    lift2_status_r <= ST_CLOSED;
                    timer2 <= 29'd0;
                    if (has_at_floor(reqs2, lift2_floor)) begin
                        if (has_at_floor(lift2_up_assigned | lift2_down_assigned, lift2_floor)) begin
                            lift2_phase     <= PH_DOOR_OPEN;
                            lift2_status_r  <= ST_OPEN;
                            service_active2 <= 1'b1;
                            manual_open2    <= 1'b0;
                        end else begin
                            lift2_phase     <= PH_ARRIVE_WAIT;
                            lift2_status_r  <= ST_CLOSED;
                            service_active2 <= 1'b1;
                            manual_open2    <= 1'b0;
                        end
                        timer2 <= 29'd0;
                    end else if (has_any(reqs2)) begin
                        if (next_floor2 > lift2_floor) begin
                            lift2_phase    <= PH_MOVING_UP;
                            lift2_status_r <= ST_UP;
                            last_dir2      <= 2'b01;
                            timer2         <= 29'd0;
                        end else if (next_floor2 < lift2_floor) begin
                            lift2_phase    <= PH_MOVING_DOWN;
                            lift2_status_r <= ST_DOWN;
                            last_dir2      <= 2'b10;
                            timer2         <= 29'd0;
                        end
                    end
                end

                PH_MOVING_UP: begin
                    lift2_status_r <= ST_UP;
                    if (timer2 < FLOOR_TRAVEL) begin
                        timer2 <= timer2 + 1'b1;
                    end else begin
                        timer2 <= 29'd0;
                        lift2_floor <= lift2_floor + 1'b1;
                        if (has_at_floor(reqs2, lift2_floor + 1'b1)) begin
                            lift2_phase     <= PH_ARRIVE_WAIT;
                            lift2_status_r  <= ST_CLOSED;
                            service_active2 <= 1'b1;
                            manual_open2    <= 1'b0;
                        end else if (!has_above(reqs2, lift2_floor + 1'b1)) begin
                            if (has_below(reqs2, lift2_floor + 1'b1)) begin
                                lift2_phase    <= PH_MOVING_DOWN;
                                lift2_status_r <= ST_DOWN;
                                last_dir2      <= 2'b10;
                            end else begin
                                lift2_phase    <= PH_IDLE;
                                lift2_status_r <= ST_CLOSED;
                            end
                        end
                    end
                end

                PH_MOVING_DOWN: begin
                    lift2_status_r <= ST_DOWN;
                    if (timer2 < FLOOR_TRAVEL) begin
                        timer2 <= timer2 + 1'b1;
                    end else begin
                        timer2 <= 29'd0;
                        lift2_floor <= lift2_floor - 1'b1;
                        if (has_at_floor(reqs2, lift2_floor - 1'b1)) begin
                            lift2_phase     <= PH_ARRIVE_WAIT;
                            lift2_status_r  <= ST_CLOSED;
                            service_active2 <= 1'b1;
                            manual_open2    <= 1'b0;
                        end else if (!has_below(reqs2, lift2_floor - 1'b1)) begin
                            if (has_above(reqs2, lift2_floor - 1'b1)) begin
                                lift2_phase    <= PH_MOVING_UP;
                                lift2_status_r <= ST_UP;
                                last_dir2      <= 2'b01;
                            end else begin
                                lift2_phase    <= PH_IDLE;
                                lift2_status_r <= ST_CLOSED;
                            end
                        end
                    end
                end

                PH_ARRIVE_WAIT: begin
                    lift2_status_r <= ST_CLOSED;
                    if (timer2 < ARRIVE_WAIT_T) begin
                        timer2 <= timer2 + 1'b1;
                    end else begin
                        timer2         <= 29'd0;
                        lift2_phase    <= PH_DOOR_OPEN;
                        lift2_status_r <= ST_OPEN;
                    end
                end

                PH_DOOR_OPEN: begin
                    lift2_status_r <= ST_OPEN;
                    if (door_extend2) begin
                        timer2 <= 29'd0;
                    end else if (door_fast_close2 && (timer2 < DOOR_CLOSE_FAST)) begin
                        timer2 <= DOOR_CLOSE_FAST;
                    end else if (timer2 < DOOR_OPEN_HOLD) begin
                        timer2 <= timer2 + 1'b1;
                    end else begin
                        timer2         <= 29'd0;
                        lift2_phase    <= PH_IDLE;
                        lift2_status_r <= ST_CLOSED;
                        manual_open2   <= 1'b0;
                        if (service_active2) begin
                            lift2_floor_ind[lift2_floor]     <= 1'b0;
                            lift2_up_assigned[lift2_floor]   <= 1'b0;
                            lift2_down_assigned[lift2_floor] <= 1'b0;
                            service_active2                  <= 1'b0;
                        end
                    end
                end

                default: begin
                    lift2_phase    <= PH_IDLE;
                    lift2_status_r <= ST_CLOSED;
                    timer2         <= 29'd0;
                end
            endcase

            // ============================================================
            // 7. Indicator outputs (reflect ALL pending assigned hall calls)
            // ============================================================
            up_request_ind   <= lift1_up_assigned   | lift2_up_assigned;
            down_request_ind <= lift1_down_assigned | lift2_down_assigned;
        end
    end
endmodule
