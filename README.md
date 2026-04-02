# Dual-Elevator FPGA Controller

A digital elevator simulator controlling **two elevators across eight floors**, implemented entirely in synthesizable Verilog and deployed on **two Digilent Nexys A7 (Artix-7) FPGA boards** communicating over a custom UART protocol.

## What Makes This Different

In many countries, elevator systems use **independent hall call buttons per elevator** — passengers must blindly choose which elevator to call with no knowledge of car positions or directions. This leads to both elevators responding to the same floor while others go unserved.

Our system uses a **unified hall call panel** shared by both elevators. A single set of UP/DOWN buttons on each floor feeds into a centralized dispatcher that assigns the most suitable elevator based on direction, proximity, and current state. This is standard in modern high-rises globally but rare in local installations — and we've implemented it from scratch on FPGA hardware.

## System Architecture

```
┌─────────────────────────────┐         UART (9600 baud)         ┌─────────────────────────────┐
│      COP Board (Board 1)    │◄───────── TX/RX ────────────────►│      HCP Board (Board 2)    │
│                             │         via PMOD JA               │                             │
│  ┌───────────────────────┐  │                                   │  ┌───────────────────────┐  │
│  │  elevator_controller  │  │  COP→HCP: 5-byte frame           │  │   hcp_uart_handler    │  │
│  │  (dual FSM + dispatch)│  │  (floor, status, indicators)     │  │   (TX hall calls,     │  │
│  └───────────────────────┘  │                                   │  │    RX elevator state)  │  │
│  ┌───────────────────────┐  │  HCP→COP: 4-byte frame           │  └───────────────────────┘  │
│  │   cop_uart_handler    │  │  (up/down requests, reset cmd)   │                             │
│  └───────────────────────┘  │                                   │  SW[15:0] = Hall call btns  │
│                             │                                   │  LED[15:0] = Hall indicators│
│  SW[7:0]  = Lift 1 car call│                                   │  7-Seg = Mirrored status    │
│  SW[15:8] = Lift 2 car call│                                   │  BTNC = System reset        │
│  BTNL/R = Lift 1 door O/C  │                                   │                             │
│  BTNU/D = Lift 2 door O/C  │                                   │                             │
│  LED[15:0] = Floor req LEDs│                                   │                             │
│  7-Seg = Floor + Status     │                                   │                             │
└─────────────────────────────┘                                   └─────────────────────────────┘
```

## Elevator Controller Features

### Five-Phase FSM (per elevator)
| Phase | Duration | Description |
|-------|----------|-------------|
| `IDLE` | — | Waiting for requests |
| `MOVING_UP` | 3s/floor | Travelling upward |
| `MOVING_DOWN` | 3s/floor | Travelling downward |
| `ARRIVE_WAIT` | 1s | Doors closed after reaching target floor |
| `DOOR_OPEN` | 3s | Doors open, requests cleared on close |

### Hall Call Assignment Algorithm
Priority-based dispatching (evaluated per hall call):
1. **Same-floor stoppable** — elevator already at the floor and idle/waiting/door-open (direction-compatible) → immediate door open
2. **Same-direction ahead** — elevator moving in the call's direction with the floor still ahead → picks it up on the way
3. **Idle preference** — one idle, one busy → idle elevator takes it
4. **Nearest fallback** — both idle or both busy in opposite directions → nearest elevator (tie: Lift 1)

### Direction-Aware Filtering
A lift that arrived going **DOWN** won't serve an **UP** hall call at the same floor (and vice versa) — the other elevator is dispatched instead. This prevents confusing passengers who pressed UP but see an elevator that just came from above.

### Door Control
- **Open button while idle** → manual 3s door open
- **Open button while door is open** → timer resets (full 3s extension)
- **Close button while door is open** → fast-forward to close within 1s
- **Hall call at current floor** → immediate door open (no arrive-wait)

### Duplicate Assignment Guard
Once a hall call is assigned to one elevator, subsequent UART pulses carrying the same request are ignored, preventing both elevators from being dispatched to the same floor.

## UART Protocol

Both directions use **0xFF** as a sync marker. Data encoding guarantees no data byte equals 0xFF.

**COP → HCP (5 bytes, every ~10ms):**
| Byte | Content | Max Value |
|------|---------|-----------|
| 0 | `0xFF` sync | — |
| 1 | `{000, lift1_floor[2:0], lift1_status[1:0]}` | `0x1F` |
| 2 | `{000, lift2_floor[2:0], lift2_status[1:0]}` | `0x1F` |
| 3 | `up_ind[7:0]` (bit 7 = 0, no UP at floor 7) | `0x7F` |
| 4 | `down_ind[7:0]` (bit 0 = 0, no DOWN at floor 0) | `0xFE` |

**HCP → COP (4 bytes, every ~10ms):**
| Byte | Content | Max Value |
|------|---------|-----------|
| 0 | `0xFF` sync | — |
| 1 | `up_request[7:0]` (bit 7 = 0) | `0x7F` |
| 2 | `down_request[7:0]` (bit 0 = 0) | `0xFE` |
| 3 | `{0000000, reset_flag}` | `0x01` |

Auto re-sync: if 0xFF appears mid-frame, the receiver restarts frame detection.

## File Structure

| File | Description |
|------|-------------|
| `cop_top.v` | COP board top-level — instantiates controller, UART handler, debouncers, 7-seg mux |
| `hcp_top.v` | HCP board top-level — instantiates UART handler, debouncers, 7-seg mux, hall call logic |
| `elevator_controller.v` | Core logic — dual FSM, hall call assignment, car calls, door control |
| `cop_uart_handler.v` | COP UART protocol — TX elevator state, RX hall calls |
| `hcp_uart_handler.v` | HCP UART protocol — TX hall calls, RX elevator state |
| `uart_tx.v` | Generic UART transmitter (8-N-1, parameterized baud rate) |
| `uart_rx.v` | Generic UART receiver with double-FF synchronizer |
| `debouncer.v` | Counter-based switch/button debouncer with edge detection |
| `seven_seg_mux.v` | 4-digit 7-segment multiplexer (floor number + status letter) |

## Hardware Requirements

- 2× Digilent Nexys A7 (Xilinx Artix-7 XC7A100T or XC7A50T)
- 1× PMOD-to-PMOD jumper wire connection (2 wires: TX↔RX cross-connected on PMOD JA)
- Xilinx Vivado for synthesis and programming

## Getting Started

1. **Clone the repository**
   ```
   git clone https://github.com/<your-username>/dual-elevator-fpga-controller.git
   ```

2. **Create two Vivado projects** — one for each board:
   - **car_operating_panel** set `cop_top.v` as the top module, include `elevator_controller.v`, `cop_uart_handler.v`, `uart_tx.v`, `uart_rx.v`, `debouncer.v`, `seven_seg_mux.v`
   - **hall_call_panel** set `hcp_top.v` as the top module, include `hcp_uart_handler.v`, `uart_tx.v`, `uart_rx.v`, `debouncer.v`, `seven_seg_mux.v`

3. **Add constraints** — create XDC constraint files mapping the ports to Nexys A7 pins (clock E3, switches, buttons, LEDs, 7-seg, PMOD JA)

4. **Synthesize, implement, and generate bitstreams** for both projects

5. **Wire the PMOD JA connectors** between the two boards:
   - Board 1 JA[1] (TX) → Board 2 JA[2] (RX)
   - Board 2 JA[1] (TX) → Board 1 JA[2] (RX)
   - Connect GND between boards

6. **Program both boards** and operate:
   - Use HCP board switches for hall calls (interleaved UP/DOWN per floor)
   - Use COP board switches for car calls, buttons for door control
   - Press BTNC on either board for system reset

## Seven-Segment Display

Both boards show identical status on 4 digits:

```
 AN[3]   AN[2]   AN[1]   AN[0]
┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐
│ L2  │ │ L2  │ │ L1  │ │ L1  │
│floor│ │ stat│ │floor│ │ stat│
└─────┘ └─────┘ └─────┘ └─────┘
```

Status letters: **u** = moving up, **d** = moving down, **o** = door open, **c** = door closed

## Parameters

Key timing constants (easily adjustable in `elevator_controller.v`):

| Parameter | Default | Real Time |
|-----------|---------|-----------|
| `FLOOR_TRAVEL` | 300,000,000 cycles | 3 seconds |
| `ARRIVE_WAIT_T` | 100,000,000 cycles | 1 second |
| `DOOR_OPEN_HOLD` | 300,000,000 cycles | 3 seconds |
| `DOOR_CLOSE_FAST` | 200,000,000 cycles | leaves 1s remaining |

UART baud rate: 9600 bps (`CLOCKS_PER_PULSE = 10417` at 100 MHz)

## License

This project was developed as a course (Level-3, Term-II; EEE304: Digital Electronics Laboratory) project at the Department of Electrical and Electronic Engineering, Bangladesh University of Engineering and Technology (BUET).
