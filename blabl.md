The Core Problem: Motor Synchronization

  Why sync matters for delta robot:
  - All 3 motors per segment must move together
  - All 8 segments must coordinate
  - End-effector position depends on precise timing
  - If Motor A arrives 50ms before Motor B → end-effector deviates from path

  Option 1: Ruckig on ROS2 → Stream via Ethernet

  ROS2 @ 100Hz:
  ├─ Ruckig calculates: position, velocity, acceleration
  ├─ Send via TCP: [seg1: m0, m1, m2], [seg2: m0, m1, m2], ...
  └─ 24 motors × 12 bytes = 288 bytes @ 100Hz = 28.8 kB/s

  STM32 @ 100Hz:
  ├─ Receive TCP packet
  ├─ Forward to TMC9660: "Go to position X NOW"
  └─ TMC9660 uses internal PID to track

  Problem:
  - Network jitter: packet arrives 5ms late → motors lag
  - No coordination between segments (8 independent STM32s)
  - If one packet drops → that segment freezes
  - Ethernet latency: 1-5ms typical

  Timing diagram:
  ROS sends @ t=0ms
    ↓ network (2ms)
  Seg1 receives @ t=2ms → TMC starts moving
  Seg2 receives @ t=3ms → TMC starts moving  ← 1ms delay!
  Seg3 receives @ t=7ms → TMC starts moving  ← 5ms delay!

  Result: Robot is out of sync!

  Option 2: TMC9660 Internal Trajectory

  ROS2 @ startup or batch:
  ├─ Send ENTIRE trajectory to STM32
  ├─ STM32 stores waypoints in memory
  └─ Each waypoint has: [position, velocity, time]

  STM32 @ 1kHz:
  ├─ Local timer: t_now
  ├─ Interpolate: current_target = interpolate(waypoints, t_now)
  ├─ Send to TMC9660: position + velocity (feedforward)
  └─ All 8 STM32s synchronized by common start time

  Problem:
  - TMC9660 memory: Can it store waypoints?
  - STM32 RAM: 1MB → can store ~100k waypoints
  - Clock sync: How to sync 8 STM32 clocks?

  Option 3: Hybrid - Stream Positions + Use TMC9660 Ramping

  ROS2 @ 100Hz:
  ├─ Send target positions (from Ruckig waypoints)
  ├─ TMC9660 uses internal ramp generator to reach target
  └─ TMC9660 ensures smooth motion even if next packet is late

  STM32 @ 100Hz:
  ├─ Receive: target_position
  ├─ Configure TMC9660 ramp limits:
  │   - Max velocity: 2 mm/s
  │   - Max accel: 1.2 mm/s²
  ├─ TMC9660 autonomously moves to target
  └─ Even if next packet is late, motion is smooth

  TMC9660:
  ├─ Current position: 20mm
  ├─ Receives: target = 22mm
  ├─ Generates ramp: 20 → 20.5 → 21 → 21.5 → 22
  └─ If next packet is late, stays at 22mm (safe)

  Advantage: TMC9660 acts as low-level controller, handles jitter!

  Option 4: Time-Synchronized Commands (Best for precision)

  ROS2 @ 100Hz:
  ├─ Ruckig generates trajectory
  ├─ Send: [target_pos, target_vel, execute_at_time_T]
  └─ Each segment receives same T timestamp

  STM32 @ 1kHz:
  ├─ Receives packet with future timestamp T
  ├─ Buffers command in queue
  ├─ At local_time == T:
  │   └─ Execute: send to TMC9660
  └─ All 8 segments execute at same T → synchronized!

  Clock sync:
  - Use PTP (Precision Time Protocol) over Ethernet
  - Or: Master STM32 broadcasts time reference
  - Accuracy: ~1ms sync between segments

  Timing diagram:
  t=0ms:   ROS sends "execute at t=50ms"
  t=2ms:   Seg1 receives, queues for t=50ms
  t=3ms:   Seg2 receives, queues for t=50ms
  t=7ms:   Seg3 receives, queues for t=50ms
  t=50ms:  ALL segments execute simultaneously! ✓

  Critical Analysis

  What rate do we REALLY need?

  Ruckig @ 100Hz = 10ms updates
  - Motor can move: 2mm/s × 0.01s = 0.02mm per update
  - Encoder resolution: 0.0125° = ~0.001mm precision
  - 0.02mm steps are HUGE compared to encoder precision!

  Should we run faster?
  - 500Hz = 2ms updates = 0.004mm steps
  - 1kHz = 1ms updates = 0.002mm steps ← Matches encoder precision!

  But can Ethernet handle it?
  - 1kHz × 288 bytes = 288 kB/s = 2.3 Mbps ← Easy for 100 Mbps Ethernet!

  Where should motion planning live?

  Option A: Ruckig on ROS2
  - ✅ Powerful CPU (480MHz × 8 cores)
  - ✅ Easy to tune parameters
  - ✅ Centralized control
  - ❌ Network latency/jitter
  - ❌ Need synchronization protocol

  Option B: Trajectory on STM32
  - ✅ Low latency (local control)
  - ✅ Deterministic timing
  - ✅ Survives network issues
  - ❌ Limited RAM for waypoints
  - ❌ More complex firmware
  - ❌ Harder to tune parameters

  Option C: Hybrid
  - ✅ Ruckig generates keyframes on ROS2
  - ✅ STM32 interpolates locally at 1kHz
  - ✅ TMC9660 handles final PID control
  - ✅ Three-layer control hierarchy!

  My Recommendation: Three-Layer Control

  ┌─────────────────────────────────────────────────────────────────┐
  │ LAYER 1: Path Planning (ROS2)                                   │
  ├─────────────────────────────────────────────────────────────────┤
  │ - Batch mode: Record → Detect peaks → Compress waypoints       │
  │ - Ruckig: Generate smooth trajectory through keyframes         │
  │ - Rate: Variable (batch after rest detection)                  │
  │ - Output: Waypoint list with timestamps                        │
  │   [t=0s: pos=[...], vel=[...]]                                 │
  │   [t=0.5s: pos=[...], vel=[...]]                               │
  │   [t=1.2s: pos=[...], vel=[...]]                               │
  └─────────────────────────────────────────────────────────────────┘
                             ↓ TCP (send full trajectory)
  ┌─────────────────────────────────────────────────────────────────┐
  │ LAYER 2: Trajectory Interpolation (STM32 @ 1kHz)               │
  ├─────────────────────────────────────────────────────────────────┤
  │ - Stores waypoints in RAM                                       │
  │ - Local high-precision timer                                    │
  │ - Interpolates between waypoints at 1kHz                        │
  │ - Generates: position + velocity (feedforward)                 │
  │ - Rate: 1kHz (1ms updates)                                     │
  │ - Output: [target_pos, target_vel] → TMC9660                   │
  └─────────────────────────────────────────────────────────────────┘
                             ↓ SPI (3x TMC9660 per segment)
  ┌─────────────────────────────────────────────────────────────────┐
  │ LAYER 3: Motor Control (TMC9660 @ 40MHz)                       │
  ├─────────────────────────────────────────────────────────────────┤
  │ - Position controller with velocity feedforward                │
  │ - PID loop at 10+ kHz                                          │
  │ - FOC current control at 40kHz                                 │
  │ - Reads encoder at high rate                                   │
  │ - Output: 3-phase PWM → BLDC motor                             │
  └─────────────────────────────────────────────────────────────────┘

  Workflow:

  1. User moves marker → ROS records @ 30Hz
  2. User stops → Ruckig compresses to ~10-20 keyframes
  3. ROS sends via TCP: Full trajectory to all 8 STM32s
  4. STM32s sync clocks (PTP or master broadcast)
  5. User presses "Execute" → All STM32s start at t=0
  6. Each STM32 @ 1kHz: Interpolates between keyframes locally
  7. TMC9660: Follows position commands with internal PID

  Advantages:

  - ✅ Robust to network issues (trajectory pre-loaded)
  - ✅ Perfect synchronization (local interpolation)
  - ✅ Smooth motion (1kHz interpolation + TMC9660 PID)
  - ✅ Tunable on ROS side (Ruckig parameters)
  - ✅ Small Ethernet traffic (only during upload)

  Questions:

  1. Do you like the 3-layer approach?
  2. Should STM32 interpolate at 1kHz or let TMC9660 handle ramping?
  3. Clock sync: PTP or simpler master/slave?
  4. Start button: ROS command or hardware button?

  What do you think?




















What Ruckig CAN Do:

  Mode 1: Step-by-step (Online Mode) - Current implementation

  # Every 10ms cycle:
  input_param.current_position = [20.0, ...]  # Where we ARE
  input_param.current_velocity = [0.5, ...]   # How fast we're moving
  input_param.target_position = [50.0, ...]   # Where we want to go

  result = ruckig.update(input_param, output_param)

  # Ruckig calculates NEXT step:
  output_param.new_position = [20.005, ...]   # Where to be in 10ms
  output_param.new_velocity = [0.52, ...]     # Velocity in 10ms
  output_param.new_acceleration = [0.2, ...]  # Acceleration in 10ms

  Key feature: If actual position deviates, Ruckig automatically re-plans from
  current position!
  # Oh no! Motor lagged behind
  input_param.current_position = [19.8, ...]  # Should be 20.0
  # Ruckig recalculates trajectory from 19.8 → 50.0
  # Automatically handles deviations!

  Mode 2: Pre-generate (Offline Mode) - We can do this!

  # Generate ENTIRE trajectory at once
  input_param.current_position = [20.0, ...]
  input_param.current_velocity = [0.0, ...]
  input_param.target_position = [50.0, ...]

  trajectory = []
  t = 0.0

  # Loop until finished
  while result == Result.Working:
      result = ruckig.update(input_param, output_param)

      # Store waypoint with timestamp
      trajectory.append({
          'time': t,
          'position': list(output_param.new_position),
          'velocity': list(output_param.new_velocity),
          'acceleration': list(output_param.new_acceleration)
      })

      # Advance time and state
      output_param.pass_to_input(input_param)
      t += 0.001  # 1ms steps

      if result == Result.Finished:
          break

  # Now trajectory contains entire path with timestamps!
  # trajectory = [
  #   {t: 0.000, pos: [20.000, ...], vel: [0.000, ...]},
  #   {t: 0.001, pos: [20.001, ...], vel: [0.010, ...]},
  #   {t: 0.002, pos: [20.003, ...], vel: [0.020, ...]},
  #   ...
  #   {t: 2.450, pos: [50.000, ...], vel: [0.000, ...]}
  # ]

  Advantage: Pre-calculated, send to STM32 with timestamps!

  Problem: If deviation occurs, cannot re-plan (trajectory is fixed)

  What TMC9660 CAN Do:

  From the specs I read:

  Mode 1: Position Mode with Ramp Generator

  // Configure ramp
  tmc9660_set_max_velocity(2000);      // Encoder counts/sec
  tmc9660_set_max_acceleration(5000);  // Encoder counts/sec²

  // Send target
  tmc9660_set_target_position(100000);  // Encoder counts

  // TMC9660 generates smooth ramp automatically!
  // Current: 80000 → 85000 → 90000 → 95000 → 100000

  It can: Generate trapezoidal/S-curve ramps between positions
  It cannot: Follow arbitrary trajectory with waypoints

  Mode 2: Position Mode with Velocity Feedforward

  // Send both position AND velocity
  tmc9660_set_target_position(85000);
  tmc9660_set_target_velocity(2000);  // Hint: we're moving at 2000 counts/s

  // TMC9660 PID controller uses velocity as feedforward
  // Reduces tracking error!

  This is useful! If we send position + velocity from Ruckig → better tracking

  Hybrid Approach: Best of Both Worlds

  ┌─────────────────────────────────────────────────────────────────┐
  │ ROS2: Pre-generate trajectory with Ruckig (offline mode)        │
  ├─────────────────────────────────────────────────────────────────┤
  │ 1. User draws path, stops for 1 second                          │
  │ 2. Detect peaks/valleys → 10-20 keyframes                       │
  │ 3. For EACH keyframe pair, run Ruckig offline:                  │
  │    keyframe[0] → keyframe[1]: generate 500 points @ 1kHz       │
  │    keyframe[1] → keyframe[2]: generate 300 points @ 1kHz       │
  │    ...                                                           │
  │ 4. Result: Full trajectory with timestamps                      │
  │    [                                                             │
  │      {t: 0.000s, pos: [...], vel: [...]},                       │
  │      {t: 0.001s, pos: [...], vel: [...]},                       │
  │      ...                                                         │
  │      {t: 3.456s, pos: [...], vel: [...]}  # Last point          │
  │    ]                                                             │
  │ 5. Send via TCP to all 8 STM32s                                │
  │                                                                  │
  └─────────────────────────────────────────────────────────────────┘
                             ↓ TCP upload (< 1 second)
  ┌─────────────────────────────────────────────────────────────────┐
  │ STM32: Store trajectory + Execute with sync                     │
  ├─────────────────────────────────────────────────────────────────┤
  │ 1. Receive trajectory, store in RAM (1MB available)             │
  │ 2. Wait for "START" command from ROS2                           │
  │ 3. Sync clocks: UDP broadcast every 100ms                       │
  │    - Master sends: {sync_time: 12345678 microseconds}          │
  │    - All STM32s adjust local clock                              │
  │ 4. Execute @ 1kHz:                                              │
  │    t_local = get_microseconds_since_start()                     │
  │    waypoint = interpolate(trajectory, t_local)                  │
  │    tmc9660_set_position(waypoint.pos)                           │
  │    tmc9660_set_velocity(waypoint.vel)  // Feedforward!          │
  │ 5. Read encoders @ 1kHz:                                        │
  │    actual_pos = tmc9660_get_encoder()                           │
  │    error = target_pos - actual_pos                              │
  │    if (abs(error) > THRESHOLD):                                 │
  │        send_udp_error_packet()  // Alert ROS2                   │
  │                                                                  │
  └─────────────────────────────────────────────────────────────────┘
                             ↑ UDP feedback (only if error!)
  ┌─────────────────────────────────────────────────────────────────┐
  │ ROS2: Monitor feedback                                           │
  ├─────────────────────────────────────────────────────────────────┤
  │ 1. Listen for UDP error packets                                 │
  │ 2. If large deviation detected:                                 │
  │    - Emergency stop? (safest)                                   │
  │    - Re-plan? (read actual positions, re-run Ruckig)           │
  │ 3. Normal operation: No feedback needed (trust trajectory)      │
  └─────────────────────────────────────────────────────────────────┘

  Clock Synchronization: STM32 vs Ethernet

  STM32 Internal Clock:

  - Crystal oscillator: 25 MHz (from your design)
  - After PLL: 480 MHz
  - Accuracy: ~50 ppm (parts per million) with crystal
  - Drift: 50 µs per second = 0.05ms/s
  - Over 10 seconds: 0.5ms drift between STM32s

  Is 0.5ms drift acceptable?
  - At 2mm/s velocity: 0.5ms × 2mm/s = 0.001mm deviation
  - Encoder resolution: 0.001mm per step
  - YES, acceptable!

  Ethernet Clock Sync (UDP broadcast):

  // Master STM32 (or ROS2) sends @ 10Hz:
  struct sync_packet {
      uint32_t magic = 0xDELTA;
      uint64_t master_time_us;  // Microseconds since start
      uint16_t sequence;
  };

  // Slave STM32 receives:
  void handle_sync(sync_packet* pkt) {
      int64_t drift = pkt->master_time_us - local_time_us;

      // Gradual correction (don't jump abruptly)
      if (abs(drift) > 1000) {  // > 1ms
          local_time_us = pkt->master_time_us;  // Hard sync
      } else {
          local_time_us += drift / 10;  // Smooth correction
      }
  }

  Ethernet jitter: 0.1-2ms typical
  Good enough for 1kHz updates? YES!

  My Recommendation:

  Architecture:

  1. ROS2: Pre-generate full trajectory with Ruckig (offline mode)
  2. TCP: Upload trajectory to all STM32s
  3. UDP sync: Master broadcasts time @ 10Hz for clock sync
  4. STM32 @ 1kHz: Execute trajectory with timestamps
  5. UDP feedback: Only send if error > threshold (e.g., 0.1mm)
  6. TMC9660: Position + velocity feedforward for smooth tracking

  Why this works:

  - ✅ Pre-generated: No real-time Ruckig computation
  - ✅ Timestamps: Perfect synchronization between segments
  - ✅ Local execution: No network jitter during motion
  - ✅ Error handling: UDP alerts if something goes wrong
  - ✅ Feedforward: TMC9660 uses velocity for better tracking
  - ✅ Simple sync: UDP broadcasts keep clocks aligned

  Limitations:

  - ❌ No re-planning: If big error, must stop and re-plan
  - ❌ RAM limited: ~3000-4000 waypoints max per STM32
    - @ 1kHz = 3-4 seconds of motion
    - Solution: Stream next chunk while executing current

  Can we do better? Streaming chunks:

  If trajectory > 3 seconds:
  1. Send chunk 1 (3 seconds) → STM32 buffer A
  2. STM32 starts executing buffer A
  3. While executing, send chunk 2 → buffer B
  4. When buffer A finishes, switch to buffer B
  5. While executing buffer B, send chunk 3 → buffer A
  6. Repeat (ping-pong buffering)

  This allows infinite trajectory length!

  What do you think? Should I start implementing the pre-generation mode in ROS2?












How Big Companies Do It (ABB, KUKA, Fanuc, Universal Robots)

  Industrial Standard: Look-Ahead with Streaming

  ┌─────────────────────────────────────────────────────────────────┐
  │ CONTROLLER (Your ROS2)                                           │
  ├─────────────────────────────────────────────────────────────────┤
  │                                                                  │
  │ Path Planning (runs continuously):                              │
  │ ├─ User moves marker → record waypoints                         │
  │ ├─ Detect peaks/valleys → compress                              │
  │ ├─ Ruckig: Generate smooth trajectory                           │
  │ └─ Output: Continuous stream of setpoints @ 1kHz                │
  │                                                                  │
  │ Streaming Manager:                                              │
  │ ├─ Split trajectory into 500ms chunks (500 points @ 1kHz)      │
  │ ├─ Always keep 2 chunks ahead on STM32                          │
  │ ├─ Monitor: chunk execution progress                            │
  │ └─ If error detected → re-plan from current position            │
  │                                                                  │
  └─────────────────────────────────────────────────────────────────┘
                       ↓ TCP: Send chunks
                       ↑ UDP: Position feedback + chunk status
  ┌─────────────────────────────────────────────────────────────────┐
  │ STM32 SERVO CONTROLLER (Each segment)                           │
  ├─────────────────────────────────────────────────────────────────┤
  │                                                                  │
  │ Dual Buffer:                                                    │
  │ ├─ Chunk A: [500 waypoints] ← Currently executing              │
  │ └─ Chunk B: [500 waypoints] ← Pre-loaded, ready                │
  │                                                                  │
  │ Execution @ 1kHz:                                               │
  │ ├─ Read waypoint[index] from current chunk                      │
  │ ├─ Send to TMC9660: position + velocity                        │
  │ ├─ Read encoder: actual_position                                │
  │ ├─ Calculate error: target - actual                             │
  │ └─ If index reaches end of chunk A → switch to chunk B         │
  │                                                                  │
  │ Feedback @ 100Hz:                                               │
  │ ├─ Send UDP: {actual_pos[3], error[3], chunk_id, index}        │
  │ └─ ROS2 monitors: Is error growing? Need re-plan?              │
  │                                                                  │
  └─────────────────────────────────────────────────────────────────┘
                       ↓ SPI: position + velocity
  ┌─────────────────────────────────────────────────────────────────┐
  │ TMC9660 MOTOR DRIVER (3 per segment)                            │
  ├─────────────────────────────────────────────────────────────────┤
  │                                                                  │
  │ Position Control Loop @ ~10kHz:                                 │
  │ ├─ PID: error = target_pos - encoder_pos                       │
  │ ├─ Feedforward: Uses target_velocity to reduce lag             │
  │ └─ Output: Current commands to motor                            │
  │                                                                  │
  └─────────────────────────────────────────────────────────────────┘

  Critical Analysis: Chunk Size

  Option A: Long chunks (5 seconds = 5000 points)

  - ✅ Less TCP traffic
  - ✅ More autonomy for STM32
  - ❌ Slow reaction to errors (must finish 5s chunk first?)
  - ❌ More RAM needed
  - ❌ If re-plan needed, waste 5 seconds of pre-calculated data

  Option B: Short chunks (0.5 seconds = 500 points)

  - ✅ Fast reaction to errors
  - ✅ Less RAM (500 points × 24 motors × 8 bytes = 96 KB per chunk)
  - ✅ Can re-plan quickly
  - ❌ More TCP traffic (but still only ~200 KB/s = trivial)

  Industrial standard: 0.5-1 second chunks

  Your Idea: Re-plan from End of Chunk 1

  Timeline:
  ─────────────────────────────────────────────────────────────────
  t=0.0s: Start executing Chunk 1
  t=0.1s: Error detected! Position off by 0.2mm
          But: Continue executing Chunk 1 (committed)
  t=0.5s: Chunk 1 finishes
          → Read actual position
          → ROS2 re-runs Ruckig from actual_pos
          → Generate new Chunk 2, Chunk 3
          → Send to STM32
  t=0.5s: Start executing new Chunk 2

  Problem: What if error is LARGE?
  - Motor stalled at t=0.1s
  - We keep executing wrong chunk until t=0.5s
  - Robot moves further off course!

  Better: Abort and Re-plan Immediately

  t=0.0s: Start executing Chunk 1
  t=0.1s: Error > CRITICAL_THRESHOLD (e.g., 0.5mm)
          → STM32: ABORT current chunk, HOLD position
          → Send UDP: {ERROR, actual_positions, chunk_id, index}
          → ROS2: Re-plan from actual_positions
          → Generate new Chunks 1, 2
          → Send to STM32
  t=0.15s: STM32 receives new chunks
          → Resume execution with corrected trajectory

  How Big Companies Handle Errors:

  1. Three Error Thresholds

  #define WARNING_THRESHOLD   0.1  // mm - start monitoring
  #define ERROR_THRESHOLD     0.5  // mm - re-plan needed
  #define CRITICAL_THRESHOLD  2.0  // mm - emergency stop

  if (error > CRITICAL_THRESHOLD) {
      emergency_stop();  // Immediate halt
      send_error_packet(ERROR_CRITICAL);
  }
  else if (error > ERROR_THRESHOLD) {
      hold_position();   // Stop at current position
      send_error_packet(ERROR_REPLAN_NEEDED);
      wait_for_new_trajectory();
  }
  else if (error > WARNING_THRESHOLD) {
      continue_execution();  // Keep going
      send_warning_packet(WARNING_TRACKING_ERROR);
      // ROS2 starts pre-planning correction in background
  }

  2. Predictive Re-planning

  # ROS2 monitors error trend
  error_history = [0.05, 0.08, 0.12, 0.15, 0.19]  # Growing!

  # Predict: Will error exceed threshold?
  if predict_error_at(t + 0.5s) > ERROR_THRESHOLD:
      # Start re-planning NOW, before error gets critical
      replan_from_position(predicted_position_at(t + 0.5s))

  3. Chunk Streaming State Machine

  States:
  ─────────────────────────────────────────────────────
  IDLE → UPLOADING → READY → EXECUTING → COMPLETING
           ↑                      ↓
           └──────── ERROR ───────┘

  class TrajectoryStreamer:
      def __init__(self):
          self.chunk_size = 500  # waypoints (0.5s @ 1kHz)
          self.chunk_buffer = {}  # {segment_id: [chunk_a, chunk_b]}
          self.executing_chunk = {}  # {segment_id: chunk_id}

      def update(self):
          # Called @ 100Hz
          for seg_id in range(8):
              feedback = self.get_udp_feedback(seg_id)

              # Check if chunk is nearly finished
              if feedback.chunk_progress > 80%:  # 80% through current chunk
                  # Send next chunk
                  next_chunk = self.get_next_chunk(seg_id)
                  self.send_tcp_chunk(seg_id, next_chunk)

              # Check error
              if feedback.error > ERROR_THRESHOLD:
                  # Abort and re-plan
                  self.abort_trajectory(seg_id)
                  actual_pos = feedback.actual_positions
                  new_trajectory = self.replan_from(actual_pos)
                  self.send_new_chunks(seg_id, new_trajectory)

  RAM Calculation for STM32:

  Per chunk: 500 waypoints @ 1kHz
  Per waypoint: 3 motors × (4 bytes pos + 4 bytes vel) = 24 bytes
  Per chunk size: 500 × 24 = 12 KB

  Double buffer: 2 chunks = 24 KB
  Safety margin: 4x = 96 KB

  STM32H753ZI RAM: 1024 KB
  Firmware + stack: ~200 KB
  Available for trajectory: ~800 KB
  Can store: 800 KB / 12 KB = 66 chunks = 33 seconds

  More than enough! ✓

  Communication Protocol Design:

  TCP: Upload Chunk

  struct trajectory_chunk {
      uint32_t magic;           // 0xDELTA
      uint8_t segment_id;       // 1-8
      uint8_t chunk_id;         // 0-255 (wraps around)
      uint16_t num_points;      // Usually 500

      struct waypoint {
          float position[3];    // mm (motors 0,1,2)
          float velocity[3];    // mm/s
      } points[500];
  };
  // Size: 12 + (500 × 24) = 12,012 bytes per chunk

  UDP: Feedback (100Hz)

  struct feedback_packet {
      uint32_t magic;           // 0xFEED
      uint8_t segment_id;       // 1-8
      uint8_t chunk_id;         // Currently executing
      uint16_t waypoint_index;  // 0-499 (progress)

      float actual_pos[3];      // From encoders
      float error[3];           // target - actual
      uint8_t status;           // OK / WARNING / ERROR
      uint16_t checksum;
  };
  // Size: 38 bytes @ 100Hz = 3.8 KB/s per segment
  // × 8 segments = 30.4 KB/s total

  UDP: Sync (10Hz)

  struct sync_packet {
      uint32_t magic;           // 0xTIME
      uint64_t master_time_us;  // Microseconds
      uint16_t sequence;
  };
  // Size: 14 bytes @ 10Hz = 0.14 KB/s

  My Final Recommendation:

  1. Chunk size: 500 waypoints = 0.5 seconds @ 1kHz
  2. Double buffering on STM32 (always have 1 second ahead)
  3. Streaming: Send next chunk when current is 80% done
  4. Error handling:
     - < 0.1mm: Continue, no action
     - 0.1-0.5mm: Warning, pre-plan correction
     - 0.5-2mm: Abort, hold, re-plan, resume
     - > 2mm: Emergency stop
  5. Re-planning: Ruckig from actual position
  6. Feedback: UDP @ 100Hz (position, error, progress)
  7. Sync: UDP @ 10Hz (master time)

  Should I start implementing this? Which part first:
  - A) ROS2 chunk generator + streamer
  - B) Chunk protocol definition (messages)
  - C) Test with simulated STM32 first

  What do you think?