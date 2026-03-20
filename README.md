# MDP 2025/26 S1 Team 34

## What This Repo Is
A distributed robotics stack:
- Android UI (operator input + status)
- Raspberry Pi (orchestration + camera + API bridge)
- STM32 (real-time motion/sensors, FreeRTOS)
- Algorithm/API services (path + image recognition)

## Quick Structure
- `algo/` path + image API, planner logic, web assets
- `rpi/` runtime orchestrator and communication links
- `stm_task_1/` STM32 FreeRTOS firmware
- `stm_task_2/` STM Task 2 snapshots
- `android/` Android snapshots (`.txt` export)
- `image rec/` data prep/training scripts

## System Architecture (ASCII)

```text
+-------------------+        Bluetooth JSON        +-------------------+
|     Android UI    | <--------------------------> |   Raspberry Pi    |
| (commands/status) |                              | (orchestrator)    |
+-------------------+                              +-------------------+
                                                           |
                                                           | UART commands + ACK
                                                           v
                                                  +-------------------+
                                                  |       STM32       |
                                                  | FreeRTOS control  |
                                                  +-------------------+
                                                           ^
                                                           |
                                             actuator/sensor control loop

+-------------------+   HTTP (/path,/image,/status)   +-------------------+
|   Raspberry Pi    | <------------------------------> |   Algo/Image API  |
+-------------------+                                   +-------------------+
```

## Runtime Design

### Control flow
1. Android sends obstacles/start.
2. RPi plans path via API.
3. RPi sends one command to STM.
4. STM executes and returns `ACK`.
5. RPi sends next command.
6. Snapshot points trigger image capture + `/image` call.

### Why this works
- STM handles deterministic, timing-critical work.
- RPi handles integration and mission logic.
- ACK-gated pipeline prevents command overrun.

## STM32 Design Snapshot
- Tasks: Communication, Control, Motor, IMU, Encoder, OLED
- Primitives: queues + semaphores + software timers
- Parser: strict 7-char command grammar (`DDCCCCC`)
- Shared model: sensor freshness + emergency-stop state

## RPi Design Snapshot
- Multiprocess workers: Android RX/TX, STM RX, command follower, action worker
- IPC: manager queues/events + movement lock
- Recovery: Android reconnect + process restart
- Task profiles: manual (`task0`), mission (`task1`), image-decision mission (`task2`)

## Setup (Minimal)

### Algo/API
```bash
cd algo
npm install
npm run dev
pip install -r requirements.txt
python main.txt
```

### RPi
```bash
cd rpi
pip install -r requirements.txt
python task1.py.txt
# or task2.py.txt
```

### STM32
1. Open `stm_task_1` in STM32CubeIDE.
2. Build + flash.
3. Verify UART ACK behavior.

## Key Config Files
- `rpi/settings.py.txt` serial + API host/port
- `rpi/consts.py.txt` image label map
- `stm_task_1/Inc/task_priorities.h` RTOS cadence/priorities
- `stm_task_1/Inc/command_parser.h` command format

## Notes
- Some modules are snapshot exports (`.txt`) and may need restoring for direct IDE builds.
- Detailed personal contribution doc: `README_my_contribution.md`.
