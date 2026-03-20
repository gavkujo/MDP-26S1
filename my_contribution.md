# My Contribution (Design + Architecture)

Focus areas:
1. STM RTOS (Task 1 + Task 2)
2. RPi image recognition orchestration

## 1) STM RTOS (Task 1 + Task 2)

### RTOS architecture intent
I designed the STM's RTOS side as a deterministic execution core for motion and safety-critical control.
The main rule was: parsing and decision logic can be asynchronous, but motor actuation must remain predictable and bounded.

### Task decomposition (what each task owns)
- CommunicationTask
  - receives UART bytes (interrupt-assisted ingress)
  - assembles fixed command frames
  - validates/parses command syntax
  - pushes valid `RobotCommand_t` into the command queue
- ControlTask
  - consumes parsed commands
  - maps command intent into motion goals and motor-level actions
  - handles command lifecycle (`idle -> executing -> complete/error`)
  - decides when to acknowledge completion back to upstream flow
- MotorTask
  - executes `MotorCommand_t` primitives
  - applies speed, steering, and duration parameters
  - performs stop/brake semantics for command boundaries and faults
- IMUTask and EncoderTask
  - perform periodic sensor acquisition/update
  - refresh shared state used by control decisions
- OLEDTask
  - provides lightweight runtime visibility (state/debug summaries)

### Scheduling and priority model
- Priority strategy:
  - communication at higher priority to avoid losing command ingress
  - sensor tasks above normal for stable feedback cadence
  - control/motor/oled at normal for predictable cooperative throughput
- Timer-driven periodic jobs:
  - IMU timer (fastest cadence)
  - encoder timer
  - ultrasonic timer
- Synchronization:
  - semaphores gate sensor worker release from timer callbacks
  - queues isolate producer/consumer rates between parser, controller, and motor executor

This makes the design tolerant to bursty UART traffic without starving actuator control.

### STM internal flow (ASCII)
```text
                     UART IRQ/RX
                         |
                         v
                [CommunicationTask]
                         |
             parse + validate command
                         |
                         v
                    [CommandQueue]
                         |
                         v
                    [ControlTask] -----> updates -----> [RobotState]
                         |
             plan/translate motor action
                         |
                         v
                     [MotorQueue]
                         |
                         v
                     [MotorTask] -----> PWM/servo outputs

Timers --> semaphores --> [IMUTask/EncoderTask] --> [SharedSensorData] --> used by ControlTask
```

### Command protocol design
- Frame format: `DDCCCCC` (7 visible chars)
  - `DD`: detection context code from RPi side
  - `CCCCC`: motion opcode + argument payload
- Supported command classes:
  - linear moves: `FWddd`, `BWddd`
  - turns: `FL090`, `FR090`
  - backward pivot turns: `BL090`, `BR090`
  - streaming/stop: `FW---`, `BW---`, `STOP-`
- Parser hardening:
  - strict length check after CR/LF trimming
  - numeric field validation
  - explicit invalid classification (`CMD_INVALID`) for unknown patterns

Design outcome: malformed packets are rejected before entering control/motion execution.

### Shared data and safety model
I used a centralized shared-data contract for cross-task safety decisions:
- sensor values and freshness timestamps (IMU, encoder, ultrasonic)
- obstacle/emergency-stop flags
- latest detection code from command ingress
- motion progress references used by control logic

Safety behavior is built around data freshness and explicit emergency states.
If sensor freshness degrades or a stop condition is raised, control can force safe motor commands.

### Startup and integration sequence
The initialization path was structured to minimize undefined first-motion behavior:
1. base peripheral bring-up
2. actuator init (motors/servo)
3. sensor init (ultrasonic/encoders/IMU)
4. IMU calibration step
5. RTOS object creation (queues, semaphores, timers, tasks)
6. shared-data initialization
7. scheduler start

This ensures the first command runs with initialized actuators and valid sensor context.

### Task 2 adaptation
For Task 2, I reused the same RTOS skeleton and command contract to avoid rewriting low-level control paths.
The main adaptation was at behavior level (mission sequence/command stream), while keeping:
- identical parser guarantees
- identical queue-based decoupling
- identical safety and state model

### Impact
- Deterministic command pipeline from UART ingress to motor output.
- Better isolation of failures (parser/sensor issues do not directly corrupt motor loops).
- Easier scaling for new opcodes and sensor policies without monolithic task growth.
- Reusable RTOS architecture across Task 1 and Task 2 mission variants.

## 2) RPi Image Recognition

### Architecture I built/extended
- Multiprocess runtime to isolate communication and action workers.
- Queue-driven command pipeline with lock-gated ACK progression.
- Camera capture + API inference integrated into mission flow.
- Recovery path for Android link drop and child-process restart.

### RPi process model (ASCII)
```text
                   +----------------------+
Android RX --------> rpi_action_queue     |
                   |                      |
Android TX <------- android_queue         |   (Manager IPC)
                   |                      |
STM RX ------------> command unlock/ACK   |
                   +----------+-----------+
                              |
                              v
                       [Command Follower]
                              |
                              v
                             STM
```

### Snap-and-rec flow (ASCII)
```text
Mission trigger --> Capture (Picamera2) --> Save frame --> POST /image --> Parse label --> Send to Android
```

### Task-specific behavior
- `task1.py.txt`: path mission + obstacle/image flow
- `task2.py.txt`: ACK-triggered image decisions (left/right branching)
- `test_task2.py.txt`: mock STM for integration testing without hardware

### Impact
- Reliable bridge between embedded control and perception
- Safer sequencing (one command in flight)
- Faster dev cycles via mock-based testing

## Validation Checklist
- Bluetooth link to Android works
- Serial link to STM is stable
- API host/port in `rpi/settings.py.txt` reachable
- Camera works with Picamera2
