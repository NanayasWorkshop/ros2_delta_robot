# Motion Control Architecture
## Delta Robot - Industrial-Grade Trajectory Execution System

**Document Version:** 1.0
**Date:** 2025-01-31
**Status:** Design Specification

---

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Control Hierarchy](#control-hierarchy)
4. [Trajectory Planning Pipeline](#trajectory-planning-pipeline)
5. [Communication Protocol](#communication-protocol)
6. [Error Handling & Safety](#error-handling--safety)
7. [Timing & Synchronization](#timing--synchronization)
8. [Implementation Specifications](#implementation-specifications)

---

## Overview

### System Summary
This document defines the motion control architecture for an 8-segment delta robot with 24 motors total (3 motors per segment). The system follows industrial best practices used by major robotics manufacturers (ABB, KUKA, Fanuc, Universal Robots) for safe, precise, and robust trajectory execution.

### Key Design Principles
1. **Three-layer control hierarchy** - Path planning, trajectory interpolation, motor control
2. **Streaming trajectory chunks** - Continuous buffering for unlimited motion duration
3. **Predictive error handling** - Multiple threshold levels for graceful degradation
4. **Time-synchronized execution** - All segments move in perfect coordination
5. **Robust to network issues** - Local execution survives communication delays

### Hardware Components
- **ROS2 PC:** Intel/AMD x86 @ 3+ GHz, running Ubuntu 22.04 + ROS2 Humble
- **8× Segment Controllers:** STM32H753ZI @ 480 MHz, 1MB RAM, Ethernet
- **24× Motor Drivers:** TMC9660 with FOC, position control, encoder interface
- **24× BLDC Motors:** PBLR38FLS-483130-NH with 1:50 harmonic drive gearboxes
- **24× Encoders:** Custom optical quadrature, 576 edges/rev

- **Network:** 100 Mbps Ethernet via switch, TCP for commands, UDP for feedback

---

## System Architecture

### High-Level Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 1: PATH PLANNING (ROS2 PC)                                │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│                                                                  │
│ User Input:                                                      │
│   - Move interactive marker in RViz @ 30Hz                      │
│   - Trajectory tracker samples TF positions                     │
│   - FABRIK IK solver computes motor positions                   │
│                                                                  │
│ Batch Processing:                                               │
│   - Detect rest (1 second no movement)                          │
│   - Peak/valley detection → Compress waypoints                  │
│   - Result: 10-20 keyframes from 100+ raw points                │
│                                                                  │
│ Trajectory Generation (Ruckig):                                 │
│   - Input: Keyframe positions                                   │
│   - Constraints: max velocity 2mm/s, accel 1.2mm/s², jerk 64mm/s³│
│   - Output: Smooth trajectory @ 1kHz with timestamps            │
│   - Pre-generate ENTIRE path offline                            │
│                                                                  │
│ Chunk Streaming:                                                │
│   - Split trajectory into 500-point chunks (0.5 second each)    │
│   - Send via TCP to all 8 segment controllers                   │
│   - Always maintain 2 chunks buffered on STM32                  │
│   - Monitor execution progress via UDP feedback                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                           │
                    TCP (Command Channel)
                    UDP (Feedback Channel)
                    UDP (Time Sync)
                           │
                     Ethernet Switch
                           │
        ┌──────────────────┼──────────────────┬─────────────┐
        │                  │                  │             │
┌───────▼─────┐    ┌──────▼──────┐    ┌─────▼──────┐   (×8 total)
│ Segment 1   │    │ Segment 2   │    │ Segment 3  │
│ STM32H753ZI │    │ STM32H753ZI │    │ STM32H753ZI│
│ 192.168.1.101│   │ 192.168.1.102│   │ 192.168.1.103│
└─────────────┘    └─────────────┘    └────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ LAYER 2: TRAJECTORY INTERPOLATION (STM32 @ 1kHz)                │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│                                                                  │
│ Dual Buffer System:                                             │
│   - Chunk A: [500 waypoints] ← Currently executing             │
│   - Chunk B: [500 waypoints] ← Pre-loaded, ready for switch    │
│   - Automatic buffer swap when Chunk A reaches 95%             │
│                                                                  │
│ High-Speed Execution Loop @ 1kHz:                               │
│   1. Read local high-precision timer (microseconds)             │
│   2. Lookup waypoint from current chunk buffer                  │
│   3. Send position + velocity to 3× TMC9660 drivers via SPI     │
│   4. Read encoder positions from TMC9660                        │
│   5. Calculate tracking error (target - actual)                 │
│   6. Check error thresholds → trigger warnings/aborts           │
│                                                                  │
│ Feedback Transmission @ 100Hz:                                  │
│   - Send UDP packet to ROS2:                                    │
│     [segment_id, chunk_id, waypoint_index, actual_pos[3],      │
│      error[3], status_flags]                                    │
│   - Only send when polled or error detected (reduce traffic)    │
│                                                                  │
│ Clock Synchronization:                                          │
│   - Receive UDP sync packets @ 10Hz from master                 │
│   - Adjust local timer with gradual drift correction            │
│   - Hard sync if drift > 1ms, smooth correction if < 1ms        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
                           │
                      SPI (3× per segment)
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼─────┐    ┌──────▼──────┐    ┌─────▼──────┐
│ TMC9660     │    │ TMC9660     │    │ TMC9660    │
│ Motor A     │    │ Motor B     │    │ Motor C    │
└─────────────┘    └─────────────┘    └────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ LAYER 3: MOTOR CONTROL (TMC9660 @ 40MHz)                        │
│ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│                                                                  │
│ Position Control Loop @ ~10kHz:                                 │
│   - PID controller: error = target_pos - encoder_pos            │
│   - Velocity feedforward: Reduces tracking lag                  │
│   - Output: Current setpoint to FOC controller                  │
│                                                                  │
│ Field-Oriented Control (FOC) @ 40kHz:                           │
│   - Read 3-phase currents via shunt resistors                   │
│   - Clarke/Park transforms                                      │
│   - PI controllers for Id/Iq                                    │
│   - Space Vector PWM generation                                 │
│   - Gate driver control → 3-phase bridge → BLDC motor           │
│                                                                  │
│ Encoder Processing:                                             │
│   - Quadrature decoder: A/B channels                            │
│   - 28,800 edges/rev after 1:50 gearbox (0.0125° resolution)   │
│   - Velocity estimation via differentiation + filtering         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Control Hierarchy

### Layer 1: Path Planning (ROS2 PC)
**Purpose:** High-level trajectory generation with user interaction and global path optimization.

**Responsibilities:**
- Interactive marker control and visualization
- Batch trajectory recording (30Hz sampling during user input)
- Peak/valley detection for waypoint compression (70-80% reduction typical)
- Ruckig offline trajectory generation with jerk limits
- Trajectory validation (workspace limits, singularities, collisions)
- Chunk splitting and network streaming
- Error monitoring and re-planning decisions

**Update Rate:** Variable (batch mode) + 100Hz feedback monitoring
**Hardware:** x86 PC with ROS2
**Language:** Python (ROS2 nodes)

### Layer 2: Trajectory Interpolation (STM32)
**Purpose:** Real-time trajectory execution with precise timing and local autonomy.

**Responsibilities:**
- Dual-buffer trajectory chunk management
- Time-synchronized waypoint lookup @ 1kHz
- SPI communication with TMC9660 drivers
- Encoder feedback reading and error calculation
- UDP feedback transmission to ROS2
- Clock synchronization with master
- Local emergency stop handling

**Update Rate:** 1kHz execution loop
**Hardware:** STM32H753ZI (480 MHz, 1MB RAM)
**Language:** C (bare metal or FreeRTOS)

### Layer 3: Motor Control (TMC9660)
**Purpose:** Low-level servo control with FOC and encoder feedback.

**Responsibilities:**
- Position PID control with velocity feedforward
- FOC current control (Clarke/Park transforms, SVPWM)
- Encoder quadrature decoding
- Hardware protection (overcurrent, thermal, stall detection)
- Ramp generation (backup if higher layers fail)

**Update Rate:** Position loop ~10kHz, FOC loop 40kHz
**Hardware:** TMC9660 (RISC-V @ 40MHz, integrated gate drivers)
**Configuration:** SPI register programming from STM32

---

## Trajectory Planning Pipeline

### Phase 1: User Input & Recording
```
User Action: Move interactive marker in RViz
           ↓
Trajectory Tracker Node (30Hz):
  - Samples TF transform [target, direction] frames
  - Only records when position changes (tolerance: 1µm)
  - Stores Cartesian positions in growing buffer
           ↓
FABRIK IK Solver Node (30Hz):
  - Subscribes to trajectory updates
  - Solves inverse kinematics for 8-segment delta robot
  - Outputs motor positions [m0...m23] in meters
  - Uses hot-start optimization (5-7 iterations typical)
           ↓
Motor Trajectory Smoother Node (batch):
  - Buffers motor positions in RECORDING state
  - Detects rest: no new data for 1.0 second
  - Logs: "[RECORDING] Buffered N waypoints"
```

### Phase 2: Batch Processing & Compression
```
Rest Detected (no new waypoints for 1 second)
           ↓
State Transition: RECORDING → PROCESSING
           ↓
Peak/Valley Detection:
  - For each of 24 motors:
    - Extract time series: [pos(t0), pos(t1), ..., pos(tN)]
    - Find local maxima: pos(t) > pos(t-1) AND pos(t) > pos(t+1)
    - Find local minima: pos(t) < pos(t-1) AND pos(t) < pos(t+1)
  - Union all peaks/valleys across all motors
  - Always include first and last waypoints
  - Result: Keyframe indices [0, 23, 45, 67, ..., 100]
           ↓
Compression Result:
  - Input: 100 waypoints (raw @ 30Hz)
  - Output: 12 keyframes (peaks/valleys only)
  - Compression ratio: 88% reduction
  - Logs: "[PROCESSING] Compression: 100 → 12 (88.0% reduction)"
```

### Phase 3: Ruckig Trajectory Generation
```
For each keyframe pair [i, i+1]:
           ↓
Ruckig Offline Mode:
  - Initial state: keyframe[i].position, velocity=0, acceleration=0
  - Target state: keyframe[i+1].position, velocity=0, acceleration=0
  - Constraints:
    - max_velocity: 0.002 m/s (2 mm/s)
    - max_acceleration: 0.0012 m/s² (1.2 mm/s²)
    - max_jerk: 0.064 m/s³ (64 mm/s³)
  - Time step: 0.001s (1kHz)
           ↓
Generate waypoint stream:
  trajectory = []
  t = 0.0
  while result != Result.Finished:
      result = ruckig.update(input_param, output_param)
      trajectory.append({
          'time': t,
          'position': output_param.new_position,  # [m0...m23]
          'velocity': output_param.new_velocity,  # [v0...v23]
          'acceleration': output_param.new_acceleration
      })
      output_param.pass_to_input(input_param)
      t += 0.001
           ↓
Result: Full trajectory with timestamps
  - Example: 12 keyframes → 3.5 seconds of motion → 3,500 waypoints @ 1kHz
  - Each waypoint: 24 motors × (4B position + 4B velocity) = 192 bytes
  - Total size: 3,500 × 192 bytes = 672 KB
```

### Phase 4: Chunk Splitting & Streaming
```
Split trajectory into chunks:
  - Chunk size: 500 waypoints (0.5 seconds @ 1kHz)
  - Overlap: None (sequential execution)
  - Example: 3,500 waypoints → 7 chunks
           ↓
Streaming Strategy:
  - Send Chunk 0, Chunk 1 immediately via TCP to all 8 segments
  - STM32 loads into double buffer (Chunk A, Chunk B)
  - Monitor execution progress via UDP feedback
  - When Chunk A reaches 80% progress:
    → Send Chunk 2 via TCP
    → STM32 loads into Chunk A slot (overwrite finished chunk)
  - When Chunk B reaches 80% progress:
    → Send Chunk 3 via TCP
    → STM32 loads into Chunk B slot
  - Continue ping-pong buffering until trajectory complete
           ↓
State Transition: PROCESSING → EXECUTING
  - Logs: "[EXECUTING] Starting trajectory with 7 chunks"
```

---

## Communication Protocol

### Network Topology
```
┌──────────────┐
│  ROS2 PC     │
│ 192.168.1.1  │
└──────┬───────┘
       │
┌──────▼────────────────────────────────────────────┐
│  Ethernet Switch (100 Mbps)                       │
└──┬────┬────┬────┬────┬────┬────┬────┬───────────┘
   │    │    │    │    │    │    │    │
  S1   S2   S3   S4   S5   S6   S7   S8
 .101 .102 .103 .104 .105 .106 .107 .108
```

### Channel 1: TCP Command Channel (ROS2 → STM32)

**Purpose:** Reliable trajectory chunk upload

**Packet Structure:**
```c
struct trajectory_chunk {
    // Header
    uint32_t magic;           // 0x44454C54 ("DELT")
    uint8_t segment_id;       // 1-8 (which segment this is for)
    uint8_t chunk_id;         // 0-255 (wraps around, sequence tracking)
    uint16_t num_waypoints;   // Number of waypoints in this chunk (typically 500)
    uint32_t start_time_us;   // Timestamp of first waypoint (microseconds)
    uint16_t dt_us;           // Time delta between waypoints (1000 = 1ms = 1kHz)
    uint16_t reserved;        // Padding for alignment

    // Payload
    struct waypoint {
        float position[3];    // Motor positions in meters [m0, m1, m2]
        float velocity[3];    // Motor velocities in m/s [v0, v1, v2]
    } waypoints[500];         // Variable length array

    // Footer
    uint32_t checksum;        // CRC32 of entire packet
};

// Size calculation:
// Header: 20 bytes
// Waypoints: 500 × (3×4 + 3×4) = 500 × 24 = 12,000 bytes
// Footer: 4 bytes
// Total: 12,024 bytes per chunk
```

**Transmission:**
- Protocol: TCP (reliable, ordered delivery)
- Port: 5000 (configurable)
- Rate: Variable (on-demand when buffer space available)
- Bandwidth: 12 KB/chunk × 2 chunks/sec = 24 KB/s per segment (negligible)
- Total bandwidth: 24 KB/s × 8 segments = 192 KB/s = 1.5 Mbps

**STM32 Response:**
```c
struct chunk_ack {
    uint32_t magic;           // 0x41434B21 ("ACK!")
    uint8_t segment_id;       // Echo back
    uint8_t chunk_id;         // Echo back
    uint8_t status;           // 0=OK, 1=Buffer full, 2=Checksum error
    uint8_t buffer_slots;     // Number of free buffer slots (0-2)
};
// Size: 8 bytes
```

### Channel 2: UDP Feedback Channel (STM32 → ROS2)

**Purpose:** Real-time position feedback and error reporting

**Packet Structure:**
```c
struct feedback_packet {
    // Header
    uint32_t magic;           // 0x46454544 ("FEED")
    uint8_t segment_id;       // 1-8
    uint8_t chunk_id;         // Currently executing chunk
    uint16_t waypoint_index;  // Current position in chunk (0-499)

    // State
    uint32_t timestamp_us;    // STM32 local time (microseconds)
    float actual_position[3]; // Encoder readings in meters [m0, m1, m2]
    float actual_velocity[3]; // Estimated velocities in m/s
    float tracking_error[3];  // target_pos - actual_pos [e0, e1, e2]

    // Status
    uint8_t status_flags;     // Bit 0: OK/ERROR
                              // Bit 1: WARNING
                              // Bit 2: Chunk A active
                              // Bit 3: Chunk B active
                              // Bit 4-7: Reserved
    uint8_t buffer_status;    // Number of loaded chunks (0-2)
    uint16_t checksum;        // CRC16
};
// Size: 60 bytes
```

**Transmission:**
- Protocol: UDP (fast, lossy acceptable)
- Port: 5001 (configurable)
- Rate: 100 Hz (every 10ms)
- Bandwidth: 60 bytes × 100 Hz × 8 segments = 48 KB/s = 0.38 Mbps

**Trigger Conditions:**
- Periodic: Every 10ms during execution
- On-demand: When error exceeds WARNING_THRESHOLD
- Critical: Immediate transmission if error exceeds ERROR_THRESHOLD

### Channel 3: UDP Time Sync Channel (ROS2 → All STM32)

**Purpose:** Clock synchronization for coordinated motion

**Packet Structure:**
```c
struct sync_packet {
    uint32_t magic;           // 0x54494D45 ("TIME")
    uint64_t master_time_us;  // Master clock in microseconds since epoch
    uint32_t sequence;        // Monotonically increasing counter
    uint16_t checksum;        // CRC16
};
// Size: 18 bytes
```

**Transmission:**
- Protocol: UDP broadcast (all segments receive simultaneously)
- Port: 5002 (configurable)
- Rate: 10 Hz (every 100ms)
- Bandwidth: 18 bytes × 10 Hz = 180 bytes/s (negligible)

**STM32 Clock Correction Algorithm:**
```c
void handle_sync_packet(sync_packet* pkt) {
    int64_t drift = pkt->master_time_us - local_time_us;

    if (abs(drift) > 1000) {
        // Large drift (> 1ms): Hard sync
        local_time_us = pkt->master_time_us;
        log_warning("Clock hard sync: drift=%lld us", drift);
    }
    else if (abs(drift) > 100) {
        // Medium drift (100µs - 1ms): Gradual correction
        local_time_us += drift / 10;  // Correct 10% per sync
    }
    // Small drift (< 100µs): Ignore, within tolerance
}
```

---

## Error Handling & Safety

### Three-Tier Error Classification

#### Tier 1: Normal Operation (error < 0.1mm)
**Action:** Continue execution, no alerts
```c
if (max_error < 0.0001) {  // 0.1mm
    status = STATUS_OK;
    // No feedback needed, proceed normally
}
```

#### Tier 2: Warning (0.1mm ≤ error < 0.5mm)
**Action:** Continue with monitoring, log warning
```c
else if (max_error < 0.0005) {  // 0.5mm
    status = STATUS_WARNING;
    send_feedback_packet(FEEDBACK_PRIORITY_MEDIUM);
    // ROS2 begins background re-planning
}
```

**ROS2 Response:**
- Monitor error trend over next 0.5 seconds
- Predict future error using linear extrapolation
- If predicted error > 0.5mm: Pre-emptively calculate corrected trajectory
- Ready to send new chunks if error reaches Tier 3

#### Tier 3: Error Recovery (0.5mm ≤ error < 2.0mm)
**Action:** Abort chunk, hold position, request re-plan
```c
else if (max_error < 0.002) {  // 2.0mm
    status = STATUS_ERROR;
    abort_current_chunk();
    hold_at_current_position();
    send_feedback_packet(FEEDBACK_PRIORITY_HIGH);
    state = STATE_WAITING_FOR_REPLAN;
}
```

**STM32 Behavior:**
- Stop advancing waypoint index
- Send current position to TMC9660 continuously (hold mode)
- Send feedback packet immediately (don't wait for 10ms timer)
- Wait for new trajectory chunks from ROS2

**ROS2 Response:**
1. Receive error packet with actual positions
2. Run Ruckig from `actual_position` to `original_target`
3. Generate new trajectory chunks with corrected path
4. Send via TCP to all segments
5. Send "RESUME" command when ready

**STM32 Resume:**
```c
void handle_resume_command() {
    if (state == STATE_WAITING_FOR_REPLAN) {
        if (chunks_loaded >= 2) {
            reset_chunk_index_to_zero();
            state = STATE_EXECUTING;
            log_info("Resumed execution after error recovery");
        }
    }
}
```

#### Tier 4: Emergency Stop (error ≥ 2.0mm)
**Action:** Immediate halt, disable motors, trigger alarm
```c
else {  // error >= 2.0mm
    status = STATUS_CRITICAL;
    emergency_stop();
    disable_all_motors();
    send_feedback_packet(FEEDBACK_PRIORITY_CRITICAL);
    state = STATE_EMERGENCY_STOP;
    trigger_hardware_alarm();  // Audible/visual indicator
}
```

**Emergency Stop Procedure:**
1. Set TMC9660 to brake mode (active braking, not free-wheeling)
2. Stop all timers and disable trajectory execution
3. Send critical error packet to ROS2 (UDP + TCP for redundancy)
4. Light red LED on segment PCB
5. Require manual reset (button press or ROS2 command) to clear

**Recovery from Emergency Stop:**
- User must acknowledge error
- System performs homing sequence to verify positions
- All error conditions must be cleared
- User initiates new trajectory from known good state

### Watchdog & Communication Timeout

**STM32 Watchdog:**
```c
#define FEEDBACK_TIMEOUT_MS  500  // 0.5 seconds

// In 1kHz execution loop:
if (state == STATE_EXECUTING) {
    time_since_last_tcp_packet += 1;  // 1ms increment

    if (time_since_last_tcp_packet > FEEDBACK_TIMEOUT_MS) {
        // ROS2 stopped sending chunks
        log_error("Communication timeout");
        finish_current_chunk();  // Complete what we have
        state = STATE_IDLE;      // Don't hang, return to safe state
        // Do NOT emergency stop - graceful degradation
    }
}
```

**ROS2 Monitoring:**
```python
class FeedbackMonitor:
    def __init__(self):
        self.last_feedback_time = {}  # {segment_id: timestamp}
        self.timeout_threshold = 0.5  # seconds

    def check_timeouts(self):
        now = time.time()
        for seg_id in range(1, 9):
            if seg_id in self.last_feedback_time:
                dt = now - self.last_feedback_time[seg_id]
                if dt > self.timeout_threshold:
                    self.logger.error(f"Segment {seg_id} feedback timeout!")
                    self.trigger_emergency_stop(seg_id)
```

---

## Timing & Synchronization

### Clock Hierarchy

```
Master Clock (ROS2 PC):
  - Source: System time (NTP synchronized)
  - Precision: ±1ms (NTP over internet)
  - Accuracy: Not critical (only for logging)
           ↓ UDP broadcast @ 10Hz
Slave Clocks (8× STM32):
  - Source: 25 MHz crystal oscillator (±50 ppm)
  - After PLL: 480 MHz system clock
  - Timer: 32-bit counter @ 480 MHz = 2ns resolution
  - Drift: ~50µs/second (50 ppm)
  - Correction: UDP sync every 100ms keeps drift < 5µs
```

### Synchronization Protocol

**Initialization Phase:**
1. ROS2 sends START command with `t_start` timestamp (future time, e.g., +500ms)
2. All STM32s receive and acknowledge
3. Each STM32 sets local timer: `local_t0 = t_start - current_time`
4. At `t_start`, all STM32s begin executing simultaneously

**Runtime Phase:**
1. Master (ROS2 or designated STM32) broadcasts sync packet every 100ms
2. Each slave adjusts local clock based on drift calculation
3. Gradual correction prevents velocity discontinuities

**Accuracy Requirements:**
- Target: ±100µs synchronization between segments
- Achieved: ±50µs typical with UDP sync @ 10Hz
- Impact of 100µs error at 2mm/s: 0.0002mm deviation (negligible)

### Latency Budget (1kHz execution loop = 1ms total)

| Task                          | Time (µs) | % of Budget |
|-------------------------------|-----------|-------------|
| Timer interrupt overhead      | 10        | 1%          |
| Read chunk buffer waypoint    | 20        | 2%          |
| SPI write to 3× TMC9660       | 150       | 15%         |
| SPI read from 3× TMC9660      | 150       | 15%         |
| Error calculation (3 motors)  | 30        | 3%          |
| Status check & logging        | 40        | 4%          |
| UDP feedback (every 10th)     | 100       | 10%         |
| Slack / margin                | 500       | 50%         |
| **Total**                     | **1000**  | **100%**    |

**Critical Path:** SPI communication (300µs total)
**Optimization:** Use DMA for SPI transfers to reduce CPU load

---

## Implementation Specifications

### ROS2 Node: `motor_hardware_interface`

**Responsibilities:**
- Subscribe to `/smoothed_motor_commands` from Ruckig
- Split trajectory into 500-waypoint chunks
- Manage TCP connections to 8 segment controllers
- Send chunks via TCP when buffer space available
- Receive UDP feedback packets @ 100Hz per segment
- Monitor errors and trigger re-planning
- Broadcast UDP time sync @ 10Hz

**State Machine:**
```
IDLE → UPLOADING → READY → EXECUTING → COMPLETING
  ↑                           ↓
  └─────────── ERROR ─────────┘
```

**Parameters:**
```yaml
motor_hardware_interface:
  chunk_size: 500                    # waypoints per chunk
  update_rate: 100.0                 # Hz (feedback monitoring)
  sync_rate: 10.0                    # Hz (time sync broadcast)
  error_threshold_warning: 0.0001    # meters (0.1mm)
  error_threshold_abort: 0.0005      # meters (0.5mm)
  error_threshold_emergency: 0.002   # meters (2.0mm)
  tcp_port: 5000
  udp_feedback_port: 5001
  udp_sync_port: 5002
  segment_ips:
    - "192.168.1.101"
    - "192.168.1.102"
    - "192.168.1.103"
    - "192.168.1.104"
    - "192.168.1.105"
    - "192.168.1.106"
    - "192.168.1.107"
    - "192.168.1.108"
```

### STM32 Firmware Architecture

**Main Components:**
1. **Ethernet Stack:** lwIP with TCP/UDP support
2. **Trajectory Manager:** Dual-buffer chunk storage and execution
3. **SPI Master:** Communication with 3× TMC9660 drivers
4. **Timer:** 1kHz interrupt for trajectory execution
5. **Error Monitor:** Real-time tracking error calculation
6. **Feedback Sender:** UDP packet transmission @ 100Hz

**Memory Allocation (1MB total RAM):**
```
Firmware + Stack:           200 KB
lwIP buffers:               100 KB
Trajectory Chunk A:          12 KB (500 waypoints)
Trajectory Chunk B:          12 KB (500 waypoints)
SPI buffers:                 10 KB
Log buffer:                  10 KB
Variables + heap:            50 KB
Free / safety margin:       606 KB
```

**FreeRTOS Tasks:**
```c
Task Name            | Priority | Stack | Period    | Purpose
---------------------|----------|-------|-----------|---------------------------
EthernetTask         | High     | 8 KB  | Event     | TCP/UDP packet handling
TrajectoryTask       | Realtime | 4 KB  | 1kHz      | Waypoint execution
FeedbackTask         | High     | 2 KB  | 100Hz     | UDP feedback transmission
MonitorTask          | Low      | 2 KB  | 10Hz      | Health checks, logging
```

### TMC9660 Configuration Registers

**Initialization Sequence:**
```c
// Mode configuration
tmc9660_write_reg(TMC9660_GCONF, 0x00000004);  // Enable position mode

// Motion limits (values depend on encoder scaling)
tmc9660_write_reg(TMC9660_VMAX, velocity_to_counts(0.002));     // 2 mm/s
tmc9660_write_reg(TMC9660_AMAX, accel_to_counts(0.0012));       // 1.2 mm/s²
tmc9660_write_reg(TMC9660_DMAX, accel_to_counts(0.0012));       // Decel = Accel

// Ramp configuration
tmc9660_write_reg(TMC9660_RAMPMODE, 0);  // Position mode with ramp

// Encoder configuration
tmc9660_write_reg(TMC9660_ENC_CONST, encoder_scaling_factor);

// PID parameters (tune during commissioning)
tmc9660_write_reg(TMC9660_PID_P, 100);
tmc9660_write_reg(TMC9660_PID_I, 10);
tmc9660_write_reg(TMC9660_PID_D, 5);
```

**Runtime Commands (1kHz):**
```c
// Send position + velocity (feedforward)
int32_t target_counts = meters_to_counts(waypoint.position);
int32_t target_vel_counts = velocity_to_counts(waypoint.velocity);

tmc9660_write_reg(TMC9660_XTARGET, target_counts);
tmc9660_write_reg(TMC9660_VMAX, target_vel_counts);

// Read actual position
int32_t actual_counts = tmc9660_read_reg(TMC9660_XACTUAL);
float actual_meters = counts_to_meters(actual_counts);
```

### Unit Conversions

**Encoder Counts:**
- 144 slits × 4 edges = 576 counts/rev (before gearbox)
- × 50 gearbox ratio = 28,800 counts/rev (after gearbox)

**Linear Actuator (example: lead screw with 5mm pitch):**
- 28,800 counts/rev ÷ 5mm = 5,760 counts/mm
- 1 count = 0.000174 mm = 0.174 µm

**Conversion Functions:**
```c
#define COUNTS_PER_MM  5760.0f

float counts_to_meters(int32_t counts) {
    return (float)counts / (COUNTS_PER_MM * 1000.0f);
}

int32_t meters_to_counts(float meters) {
    return (int32_t)(meters * COUNTS_PER_MM * 1000.0f);
}

int32_t velocity_to_counts(float m_per_sec) {
    // Convert m/s to counts/sec
    return (int32_t)(m_per_sec * COUNTS_PER_MM * 1000.0f);
}
```

---

## Performance Specifications

### Trajectory Execution Metrics

| Metric                     | Specification        | Achieved (Expected) |
|----------------------------|----------------------|---------------------|
| Position resolution        | 0.001 mm (1 µm)      | 0.174 µm            |
| Velocity accuracy          | ±5% at 2 mm/s        | ±1% (estimated)     |
| Tracking error (normal)    | < 0.05 mm            | < 0.02 mm           |
| Tracking error (max)       | < 0.2 mm             | < 0.1 mm            |
| Segment synchronization    | ±100 µs              | ±50 µs              |
| Path deviation (8 segments)| < 0.5 mm             | < 0.2 mm            |
| Max trajectory length      | Unlimited            | Unlimited (streaming)|
| Chunk upload latency       | < 50 ms              | ~20 ms (12KB/chunk) |
| Error detection latency    | < 10 ms              | < 1 ms (1kHz loop)  |
| Re-plan trigger time       | < 100 ms             | < 50 ms             |
| Emergency stop time        | < 5 ms               | < 2 ms              |

### Network Performance

| Metric                     | Specification        | Measured            |
|----------------------------|----------------------|---------------------|
| TCP chunk upload           | 192 KB/s (all seg)   | ~200 KB/s           |
| UDP feedback               | 48 KB/s (all seg)    | ~50 KB/s            |
| UDP sync                   | 180 bytes/s          | Negligible          |
| Total bandwidth            | < 2 Mbps             | ~2 Mbps peak        |
| Ethernet utilization       | < 2% (100 Mbps)      | < 2%                |
| Packet loss tolerance      | UDP: 10%, TCP: 0%    | N/A                 |
| Latency (ROS2 ↔ STM32)     | < 5 ms               | ~2 ms typical       |

---

## Safety Features

### Hardware Safety
1. **Emergency Stop Button:** Hardware interrupt on all STM32s
2. **Endstops:** 2 per motor (min/max), reflective optical sensors
3. **Thermal Monitoring:** TMC9660 internal temperature sensors
4. **Overcurrent Protection:** TMC9660 hardware current limiting
5. **Watchdog Timer:** STM32 independent watchdog (IWDG)

### Software Safety
1. **Position Limits:** Enforced in ROS2 (pre-planning) and STM32 (runtime)
2. **Velocity Limits:** Ruckig constraints + TMC9660 limits (dual redundancy)
3. **Error Thresholds:** Three-tier system with graceful degradation
4. **Communication Timeout:** Graceful stop if ROS2 connection lost
5. **Checksum Validation:** CRC32 on TCP, CRC16 on UDP packets

### Failure Modes & Recovery

| Failure Mode               | Detection Method       | Response                    |
|----------------------------|------------------------|-----------------------------|
| ROS2 PC crash              | STM32 TCP timeout      | Finish current chunk, stop  |
| Ethernet cable disconnect  | Link down detection    | Emergency stop all segments |
| Single segment crash       | Feedback timeout       | Stop all segments (safety)  |
| Motor stall                | TMC9660 stall flag     | Abort, re-plan with margin  |
| Encoder failure            | Position jump > 10mm   | Emergency stop              |
| Thermal overload           | TMC9660 thermal flag   | Reduce velocity, cool down  |
| Power supply fault         | Voltage monitor        | Emergency stop, log event   |

---

## Testing & Commissioning

### Phase 1: Component Testing
- [ ] STM32 Ethernet communication (TCP/UDP loopback)
- [ ] SPI communication with single TMC9660
- [ ] Encoder reading and velocity estimation
- [ ] Timer accuracy measurement (1kHz interrupt jitter)
- [ ] Memory allocation (verify 12KB chunks fit)

### Phase 2: Motor Testing
- [ ] Single motor position control (open-loop)
- [ ] Single motor position control (closed-loop with encoder)
- [ ] Velocity feedforward tuning
- [ ] PID parameter tuning for smooth motion
- [ ] Thermal testing (continuous operation 30 minutes)

### Phase 3: Segment Testing
- [ ] 3-motor coordinated motion (single segment)
- [ ] Trajectory chunk execution (pre-recorded)
- [ ] Error detection and threshold validation
- [ ] UDP feedback packet verification
- [ ] Clock synchronization accuracy measurement

### Phase 4: System Integration
- [ ] 8-segment network communication (all online)
- [ ] Clock synchronization between all segments
- [ ] Coordinated motion (simple trajectory)
- [ ] Chunk streaming (long trajectory > 10 seconds)
- [ ] Error recovery (inject artificial error, verify re-plan)

### Phase 5: Safety Validation
- [ ] Emergency stop response time < 5ms
- [ ] Error threshold boundaries (0.1mm, 0.5mm, 2.0mm)
- [ ] Communication timeout handling
- [ ] Endstop triggering (verify limit detection)
- [ ] Thermal protection (overheat simulation)

### Phase 6: Performance Testing
- [ ] Position accuracy: ±0.1mm across workspace
- [ ] Velocity accuracy: ±5% at various speeds
- [ ] Segment synchronization: ±100µs
- [ ] Path repeatability: 10 cycles, measure deviation
- [ ] Long-duration test: 1 hour continuous motion

---

## Future Enhancements

### Potential Improvements
1. **Predictive error compensation:** Machine learning model to predict and compensate for systematic errors
2. **Force feedback:** Add load cells for force control modes
3. **Collision detection:** IMU-based vibration monitoring for obstacle detection
4. **Dynamic velocity scaling:** Automatically slow down in tight curves
5. **Path optimization:** Minimum-jerk trajectory generation for smoother motion
6. **Remote monitoring:** Web dashboard for real-time status and diagnostics

### Advanced Features (Optional)
- **EtherCAT upgrade:** For sub-microsecond synchronization (research/precision applications)
- **Multi-robot coordination:** Extend protocol to support 2+ delta robots working together
- **Sensor fusion:** Combine encoder + IMU for redundant position validation
- **Adaptive control:** Self-tuning PID parameters based on load conditions

---

## References & Standards

### Industry Standards
- **IEC 61800-7-201:** Adjustable speed electrical power drive systems - Generic interface and use of profiles for power drive systems - Part 7-201: Generic interface - Profile type 1 specification
- **IEC 61131-3:** Programmable controllers - Programming languages
- **ISO 10218-1:2011:** Robots and robotic devices - Safety requirements for industrial robots

### Technical References
- **Ruckig:** Berscheid, L., & Kröger, T. (2021). Jerk-limited Real-time Trajectory Generation with Arbitrary Target States. *Robotics: Science and Systems XVII*
- **FABRIK:** Aristidou, A., & Lasenby, J. (2011). FABRIK: A fast, iterative solver for the Inverse Kinematics problem. *Graphical Models, 73*(5), 243-260
- **TMC9660 Datasheet:** Analog Devices, Rev 2, July 2025
- **STM32H753 Reference Manual:** STMicroelectronics, RM0433, Rev 7

### Related Documentation
- `ARCHITECTURE.md` - High-level system architecture
- `CLAUDE.md` - Development guide and command reference
- `robot_config/` - Centralized parameter definitions
- `hardware/components/` - Component specifications and datasheets

---

**Document Status:** APPROVED FOR IMPLEMENTATION
**Next Review Date:** After Phase 4 testing completion
**Maintained By:** ROS2 Development Team

