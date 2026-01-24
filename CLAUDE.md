# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **robotics competition project** based on RT-Thread RTOS for a supermarket robot challenge. The system runs on STM32F103ZET6 (Cortex-M3, 72MHz) with 512KB Flash and 64KB RAM.

**Core functionality:**
- 4-wheel independent drive with closed-loop stepper motors (Emm25 V5.0 drivers)
- 3-DOF robotic arm with bus servos (LX-16A)
- 9-axis IMU navigation (HWT101 gyroscope)
- YOLO visual recognition system
- Multi-sensor fusion (ultrasonic + encoder + IMU)
- Competition state machine management (6-minute match)
- One-key restart mechanism (10-second wait)

## Build Commands

### Primary Build Method (SCons)
```bash
# Clean and rebuild
scons -c && scons

# Menu configuration
scons --menuconfig

# Build only (no clean)
scons
```

### Alternative: RT-Thread Studio IDE
- Uses Eclipse-based GUI with project file `.project`
- Build: Project -> Build Project (Ctrl+B)
- Download: Project -> Download (F11)

### Output
- Build target: `rt-thread.elf`
- Build directory: `build/` or `Debug/`

### Flashing
- J-Link (recommended)
- ST-Link V2
- DAPLink

### Serial Console
- Port: UART1 (baudrate 115200, 8N1)
- Used for FinSH shell and debug output

## Project Architecture

### Directory Structure

```
applications/              # Application layer code
├── arm/                   # Robotic arm control (LX-16A servos on UART3)
│   ├── Src/Arm_thread.c
│   ├── Src/LobotServoController.c
│   └── Inc/LobotServoController.h
├── motor/                 # Motor control (Emm25 drivers on UART4)
│   ├── Src/motor_thread.c
│   ├── Src/motor_cmd.c
│   ├── Src/motor_state.c
│   └── Inc/motor_*.h
├── gyro/                  # IMU module (HWT101 on UART2)
│   ├── Src/sensor_thread.c
│   ├── Src/wit_c_sdk.c
│   └── Inc/wit_c_sdk.h
├── task/                  # Task modules (shelf grab, vision pick)
│   ├── Src/shelf_grab_task.c
│   └── Inc/shelf_grab_task.h
├── competition/           # Competition control state machine
│   ├── competition_control.c
│   └── competition_control.h
├── yolo_comm/             # Vision communication module
│   ├── Src/yolo_serial_com.c
│   └── Inc/yolo_serial_com.h
├── ultrasonic_ver2.0/     # Ultrasonic distance sensors
│   ├── Src/ultrasonic.c
│   └── Inc/ultrasonic.h
├── PID/                   # PID control algorithm
│   ├── Src/pid.c
│   └── Inc/pid.h
└── main.c                 # Main entry point

drivers/                   # BSP driver layer
├── board.c                # Board initialization
├── drv_usart.c            # UART driver (UART1-5)
├── drv_gpio.c             # GPIO driver
├── drv_pwm.c              # PWM driver (TIM5/TIM8)
├── drv_hwtimer.c          # Hardware timer driver
└── include/               # Driver headers

cubemx/                    # STM32CubeMX generated files
└── Drivers/               # HAL library

libraries/                 # STM32 HAL and CMSIS
├── STM32F1xx_HAL_Driver/
└── CMSIS/

rt-thread/                 # RT-Thread kernel (27MB)
├── src/                   # Kernel source
├── components/            # Components (FinSH, libc, drivers)
└── tools/                 # Build tools
```

### Module Dependencies

```
competition_control  (competition state machine)
    │
    ├── task (shelf grab / vision pick tasks)
    │       │
    │       ├── arm (robotic arm)
    │       ├── gyro (IMU)
    │       ├── yolo_comm (vision)
    │       └── ultrasonic (distance)
    │
    └── motor (motion control)
            └── PID (closed-loop control)
```

## Hardware Configuration

### UART Assignments
- **UART1**: Console (FinSH shell) - 115200 baud
- **UART2**: HWT101 IMU - 9600/115200 baud
- **UART3**: LX-16A Bus Servos (3 servos) - 115200 baud
- **UART4**: Emm25 Stepper Drivers (4 drivers) - 115200 baud
- **UART?**: YOLO vision module

### Timer Assignments
- **TIM1**: Encoder (front-left wheel)
- **TIM5**: PWM + Encoder (front wheels)
- **TIM8**: PWM + Encoder (rear wheels)

### GPIO Buttons
- **PC13**: Start button
- **PD0**: Restart button

### Motor IDs (Emm25)
- `LF` - Left-Front
- `RF` - Right-Front
- `LB` - Left-Rear
- `RB` - Right-Rear

### Servo IDs (LX-16A)
- ID 1: Rotation base (ARM_ROTATE_SERVO_ID)
- ID 2: Extension arm (ARM_EXTEND_SERVO_ID)
- ID 3: Gripper claw (ARM_GRAB_SERVO_ID)

## RT-Thread Configuration

### System Configuration (rtconfig.h)
- CPU: STM32F103ZET6 (Cortex-M3)
- RT-Thread version: 4.1.1
- Max threads: 32 priority levels
- System tick: 1000 Hz (1ms tick)
- Console device: "uart1"

### Thread Priority Scheme
Lower number = higher priority
- Priority 10: Competition control (comp_ctrl)
- Priority 16: Motor state scan (motor_state)
- Priority 20: FinSH shell (tshell)

### Memory Management
- Uses Small Memory Allocator (RT_USING_SMALL_MEM)
- Heap from system RAM

## Communication Protocols

### Emm25 Stepper Driver Protocol (UART4)
Frame structure:
```
0x7E | ID | Len | Cmd | Params | CRC
```

Common commands:
- `0xA2`: Speed closed-loop (parameter: RPM)
- `0xFD`: Position mode
- `0xA8`: Return to zero
- `0x23`: Read speed

### LX-16A Servo Protocol (UART3)
Frame structure:
```
0x55 | ID | Len | Cmd | Params | SUM
```

Common commands:
- MOVE_SERVO: Single servo position + time control
- ACTION_GROUP_RUN: Execute action group
- READ: Read servo parameters

### HWT101 IMU Protocol (UART2)
Output frame structure:
```
0x55 | Timestamp | Accel(3) | Gyro(3) | Angle(3) | Mag(3) | Checksum
```

Data includes:
- Acceleration (x, y, z) in g
- Angular velocity (x, y, z) in °/s
- Euler angles (roll, pitch, yaw) in °
- Magnetic field (x, y, z)

## Competition Control Flow

### Competition States
```c
typedef enum {
    COMP_IDLE = 0,            // Idle
    COMP_WAITING_START,       // Waiting for start button
    COMP_START_WAITING,       // 10-second start wait
    COMP_RUNNING,             // Competition running (6 min)
    COMP_RESTART_WAITING,     // Restart wait (10 sec)
    COMP_FINISHED             // Competition finished
} competition_state_t;
```

### Time Constants
```c
#define COMPETITION_TIME  (6 * 60 * RT_TICK_PER_SECOND)  // 6 minutes
#define RESTART_WAIT_TIME (10 * RT_TICK_PER_SECOND)      // 10 seconds
#define START_WAIT_TIME   (10 * RT_TICK_PER_SECOND)      // 10 seconds
```

## Development Guidelines

### Adding New Hardware
1. Create driver in `drivers/` following existing pattern (drv_*.c)
2. Create application module in `applications/`
3. Add to `applications/SConscript`
4. Configure pinmux in STM32CubeMX if needed
5. Update Kconfig for configuration options

### Adding New Tasks
1. Create task module in `applications/task/`
2. Implement state machine following `shelf_grab_task.c` pattern
3. Integrate with competition_control for timing
4. Test with FinSH commands before full integration

### Debugging with FinSH
Use UART1 console (115200 baud):
- `list_thread()` - List all threads
- `list_sem()` - List semaphores
- `ps()` - Process status
- `free()` - Show memory usage

### Thread Best Practices
- Use `rt_thread_mdelay()` instead of `rt_thread_sleep()`
- Keep thread stacks minimal (typically 512-2048 bytes)
- Use mutexes for shared resource protection
- Avoid busy loops - use events or mailboxes for synchronization

## Hardware Documentation

Protocol documents are in `说明书/`:
- `Emm_V5.0步进闭环驱动说明书Rev1.3.pdf`
- `HWT101协议.pdf`
- `02 总线舵机通信协议.pdf`
- `HWT的SDK API接口说明书.pdf`

## Common Issues

### Build Issues
- If SCons fails, try `scons -c` to clean first
- Ensure arm-none-eabi-gcc is in PATH
- Check rtconfig.py for correct toolchain path

### UART Communication
- All UARTs use DMA for reception
- Check baud rate matches in both hardware and code
- Use logic analyzer for protocol debugging

### Motor Control
- Motors require closed-loop command (0xA2) before movement
- Read back RPM for verification (0x23 command)
- Motor IDs must match Emm25 DIP switch settings

### Servo Control
- Servo IDs configured via hardware
- Position range: 0-1000
- Time parameter in milliseconds for movement speed
