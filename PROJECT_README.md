# RTT-Robotic-Project

> 基于 RT-Thread RTOS 的智能机器人竞赛项目 - 超市机器人挑战赛

[![RT-Thread](https://img.shields.io/badge/RT--Thread-4.1.1-blue)](https://www.rt-thread.io/)
[![STM32](https://img.shields.io/badge/STM32-F103ZET6-green)](https://www.st.com/resource/en/datasheet/stm32f103zet6.pdf)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow.svg)](LICENSE)

---

## 项目简介

本项目是为参加**创新机器人制作竞赛 - 超市机器人挑战赛**而开发的嵌入式机器人控制系统。机器人需要在超市模拟环境中自主完成物品识别、抓取、配送等任务。

### 核心功能

- 🤖 **四轮独立驱动** - 闭环步进电机精确控制
- 🦾 **三自由度机械臂** - 总线舵机控制抓取系统
- 🧭 **九轴姿态融合** - HWT101陀螺仪实时导航
- 👁️ **视觉识别系统** - YOLO目标检测与定位
- 📡 **多传感器融合** - 超声波+编码器+IMU
- ⏱️ **比赛流程控制** - 状态机管理6分钟赛程
- 🔄 **一键重启功能** - 10秒等待恢复机制

---

## 硬件配置

### 主控平台

| 组件 | 型号 | 说明 |
|------|------|------|
| **MCU** | STM32F103ZET6 | Cortex-M3, 72MHz, 512KB Flash, 64KB RAM |
| **RTOS** | RT-Thread 4.1.1 | 实时操作系统 |
| **开发环境** | RT-Thread Studio | 基于 Eclipse 的 IDE |

### 核心硬件模块

```
┌─────────────────────────────────────────────────────────┐
│                    STM32F103ZET6                        │
│                                                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │ UART4    │  │ UART3    │  │ UART2    │             │
│  │ Emm25×4  │  │ 舵机×3   │  │ HWT101   │             │
│  │ 步进电机 │  │ 机械臂   │  │ 陀螺仪   │             │
│  └──────────┘  └──────────┘  └──────────┘             │
│                                                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │ GPIO     │  │ UART?    │  │ TIM1/5/8 │             │
│  │ 超声波   │  │ YOLO模块 │  │ 编码器   │             │
│  └──────────┘  └──────────┘  └──────────┘             │
└─────────────────────────────────────────────────────────┘
```

| 模块 | 型号 | 接口 | 功能 |
|------|------|------|------|
| **步进驱动器** | Emm25 V5.0 ×4 | UART4 (115200) | 四轮独立闭环控制 |
| **总线舵机** | LX-16A ×3 | UART3 (115200) | 机械臂3自由度 |
| **陀螺仪** | HWT101 | UART2 (9600/115200) | 姿态角度输出 |
| **超声波** | HC-SR04 ×N | GPIO | 距离检测 |
| **视觉模块** | YOLO | UART? | 物品识别定位 |
| **编码器** | AB相 | TIM1/5/8 | 速度反馈 |

---

## 软件架构

### 目录结构

```
RTT-Robotic-Project/
├── applications/              # 应用层代码 (231KB)
│   ├── arm/                  # 机械臂控制
│   │   ├── Src/
│   │   │   ├── Arm_thread.c              # 机械臂控制线程
│   │   │   ├── LobotServoController.c    # 舵机控制器
│   │   │   └── servo_uart3.c             # UART3通信
│   │   └── Inc/
│   │       ├── LobotServoController.h
│   │       ├── servo_uart3.h
│   │       └── bool.h
│   │
│   ├── motor/                # 电机控制
│   │   ├── Src/
│   │   │   ├── motor_thread.c             # 电机控制线程
│   │   │   ├── motor_cmd.c                # UART4命令接口
│   │   │   └── motor_state.c              # 运动状态管理
│   │   └── Inc/
│   │       ├── motor_cmd.h
│   │       └── motor_state.h
│   │
│   ├── gyro/                 # 陀螺仪模块
│   │   ├── Src/
│   │   │   ├── sensor_thread.c            # 传感器数据线程
│   │   │   ├── wit_c_sdk.c                # HWT101 SDK
│   │   │   └── sensor_port.c              # 底层接口
│   │   └── Inc/
│   │       ├── wit_c_sdk.h
│   │       ├── sensor_port.h
│   │       └── REG.h
│   │
│   ├── task/                 # 任务模块
│   │   ├── Src/
│   │   │   ├── shelf_grab_task.c          # 货架抓取任务
│   │   │   └── vision_pick_task.c         # 视觉抓取任务
│   │   └── Inc/
│   │       └── shelf_grab_task.h
│   │
│   ├── competition/          # 比赛控制
│   │   ├── competition_control.c          # 比赛流程状态机
│   │   └── competition_control.h
│   │
│   ├── yolo_comm/            # 视觉通信
│   │   ├── Src/
│   │   │   └── yolo_serial_com.c
│   │   └── Inc/
│   │       └── yolo_serial_com.h
│   │
│   ├── ultrasonic_ver2.0/    # 超声波模块
│   │   ├── Src/
│   │   │   └── ultrasonic.c
│   │   └── Inc/
│   │       └── ultrasonic.h
│   │
│   ├── PID/                  # PID算法
│   │   ├── Src/
│   │   │   └── pid.c
│   │   └── Inc/
│   │       └── pid.h
│   │
│   └── main.c                # 主程序入口
│
├── drivers/                  # BSP驱动层 (383KB)
│   ├── board.c               # 板级初始化
│   ├── drv_usart.c           # 串口驱动
│   ├── drv_gpio.c            # GPIO驱动
│   ├── drv_pwm.c             # PWM驱动
│   ├── drv_hwtimer.c         # 硬件定时器
│   └── ...
│
├── cubemx/                   # STM32CubeMX配置
│   └── Drivers/              # HAL库文件
│
├── libraries/                # STM32 HAL库
│   ├── STM32F1xx_HAL_Driver/
│   └── CMSIS/
│
├── rt-thread/                # RT-Thread内核 (27MB)
│   ├── src/                  # 内核源码
│   ├── components/           # 组件
│   └── tools/                # 构建工具
│
├── 说明书/                   # 硬件协议文档
│   ├── （非正式版）附件1 - 创新机器人制作竞赛规则.pdf
│   ├── HWT101协议.pdf
│   ├── 02 总线舵机通信协议.pdf
│   ├── Emm_V5.0步进闭环驱动说明书Rev1.3.pdf
│   └── HWT的SDK API接口说明书.pdf
│
├── .config                   # Kconfig配置
├── rtconfig.h                # RT-Thread配置头文件
├── SConstruct                # SCons构建脚本
├── Kconfig                   # 配置菜单
└── README.md                 # 本文档
```

### 模块依赖关系

```
┌─────────────────────────────────────────────────────────┐
│                  competition_control                    │
│                   (比赛流程控制)                         │
└────────────┬────────────────────────────────────────────┘
             │
     ┌───────┴───────┐
     ▼               ▼
┌─────────┐    ┌──────────────┐
│  task   │    │    motor     │
│ (任务)  │◄───┤  (运动控制)   │
└────┬────┘    └──────┬───────┘
     │                │
     ├─────────┬──────┘
     │         │
     ▼         ▼
┌─────────┐ ┌───────┐ ┌──────────┐
│   arm   │ │ gyro  │ │yolo_comm │
│ (机械臂) │ │(陀螺仪)│ │ (视觉)   │
└─────────┘ └───────┘ └──────────┘
     │                     │
     ▼                     ▼
┌─────────┐         ┌──────────┐
│ultrasonic│        │ PID控制  │
│ (超声波) │         └──────────┘
└─────────┘
```

---

## 功能模块详解

### 1. 比赛控制模块 (`competition/`)

**功能**: 管理比赛全流程状态机

**状态定义**:
```c
typedef enum {
    COMP_IDLE = 0,            // 空闲状态
    COMP_WAITING_START,       // 等待启动信号
    COMP_START_WAITING,       // 10秒启动等待
    COMP_RUNNING,             // 比赛运行中
    COMP_RESTART_WAITING,     // 重启等待(10秒)
    COMP_FINISHED             // 比赛结束
} competition_state_t;
```

**关键功能**:
- 6分钟比赛倒计时
- 1次重启权利控制
- 启动按钮检测 (PC13)
- 重启按钮检测 (PD0)

**时间常量**:
```c
#define COMPETITION_TIME  (6 * 60 * RT_TICK_PER_SECOND)  // 6分钟
#define RESTART_WAIT_TIME (10 * RT_TICK_PER_SECOND)      // 10秒
#define START_WAIT_TIME   (10 * RT_TICK_PER_SECOND)      // 10秒
```

---

### 2. 电机控制模块 (`motor/`)

**功能**: 四轮步进电机闭环控制

**硬件**: Emm25 V5.0 闭环步进驱动器 × 4

**通信协议** (UART4, 115200):
```
帧头 | ID | 长度 | 命令 | 参数 | 校验和
0x7E | 01 | 03  | 0xFD| ...  | CRC
```

**关键API**:
```c
// 速度控制
motor_cmd_set_speed(LF, speed);     // 设置左前轮速度
motor_cmd_read_rpm_nb(LF, &rpm);    // 非阻塞读取转速

// 运动状态
motor_state_set(ST_FORWARD);        // 前进
motor_state_set(ST_LEFT);           // 左转
motor_state_set(ST_STOP);           // 停止
```

**电机编号**:
- `LF` - 左前轮
- `RF` - 右前轮
- `LB` - 左后轮
- `RB` - 右后轮

---

### 3. 机械臂控制模块 (`arm/`)

**功能**: 3自由度机械臂抓取系统

**硬件**: LX-16A 总线舵机 × 3

**舵机配置**:
```c
#define ARM_ROTATE_SERVO_ID    1    // 旋转台舵机
#define ARM_EXTEND_SERVO_ID    2    // 伸展臂舵机
#define ARM_GRAB_SERVO_ID      3    // 抓爪舵机
```

**关键位置**:
```c
// 旋转台
#define ROTATE_FRONT_POS      0     // 朝向车头
#define ROTATE_BACK_POS     750     // 朝向货架(180°)

// 伸展臂
#define ARM_EXTENDED_POS      0     // 伸出状态
#define ARM_RETRACTED_POS   135     // 收回状态

// 抓爪
#define GRAB_OPEN_POS       300     // 张开
#define GRAB_CLOSE_POS      700     // 闭合
```

**通信** (UART3, 115200):
```c
moveServo(id, position, time);      // 单舵机控制
actionGroupRun(group, times);       // 动作组执行
```

---

### 4. 陀螺仪模块 (`gyro/`)

**功能**: 9轴姿态融合 (加速度+陀螺+磁力计)

**硬件**: HWT101 姿态传感器

**通信** (UART2, 9600/115200):

**输出数据**:
```c
typedef struct {
    float accel_x, accel_y, accel_z;      // 加速度 (g)
    float gyro_x, gyro_y, gyro_z;         // 角速度 (°/s)
    float roll, pitch, yaw;               // 欧拉角 (°)
    float mag_x, mag_y, mag_z;            // 磁场强度
} sensor_data_t;
```

**数据帧格式**:
```
帧头(0x55) | 时间戳 | 加速度 | 角速度 | 角度 | 磁场 | 校验和
```

**校准**: 需要电子校准（传感器水平静止30秒）

---

### 5. 任务模块 (`task/`)

**功能**: 货架抓取任务状态机

**货架抓取任务** (`shelf_grab_task.c`):

**状态流程**:
```c
typedef enum {
    SHELF_TASK_INIT = 0,      // 初始化
    SHELF_ADJUST_POSITION,    // 位置调整
    SHELF_SCAN_ITEMS,         // 扫描物品
    SHELF_GRAB_ITEM,          // 抓取物品
    SHELF_PLACE_ITEM,         // 放置物品
    SHELF_LEAVE_AREA,         // 离开区域
    SHELF_TASK_COMPLETE       // 任务完成
} shelf_task_state_t;
```

**任务参数**:
```c
#define TOTAL_ITEMS_TO_GRAB 4      // 目标抓取数量
#define MAX_SHELVES 2              // 货架数量(2个)
```

**视觉抓取任务** (`vision_pick_task.c`):
- YOLO目标识别
- 物品坐标计算
- 精准定位抓取

---

### 6. 超声波模块 (`ultrasonic_ver2.0/`)

**功能**: 距离检测与避障

**硬件**: HC-SR04 × N

**接口**: GPIO (Trigger + Echo)

**测量范围**: 2cm ~ 400cm

**精度**: ~3mm

---

### 7. PID控制模块 (`PID/`)

**功能**: 闭环控制算法

**应用场景**:
- 电机速度闭环
- 位置精确控制
- 姿态稳定控制

---

## 竞赛规则对应

### 超市机器人挑战赛任务流程

```
┌──────────┐
│ 启动区   │ 机器人上电，等待启动按钮
│ (充电)   │
└────┬─────┘
     │ 10秒启动等待
     ▼
┌──────────┐
│ 导航     │ 自主导航到货架区
│ 到货架   │
└────┬─────┘
     │
     ▼
┌──────────┐
│ 识别物品 │ YOLO视觉识别二维码/颜色
│         │ 计算物品位置
└────┬─────┘
     │
     ▼
┌──────────┐
│ 抓取物品 │ 机械臂伸展+抓爪闭合
│ (×4件)  │ 依次抓取4件物品
└────┬─────┘
     │
     ▼
┌──────────┐
│ 运送     │ 四轮运动控制
│ 到配送区 │ 陀螺仪导航
└────┬─────┘
     │
     ▼
┌──────────┐
│ 精准投放 │ 按识别结果放置到对应区域
│         │ A/B/C三个投放区
└────┬─────┘
     │
     ▼
┌──────────┐
│ 返回     │ 可选：返回充电区
│ 充电区   │ 获得+30分奖励
└──────────┘
```

### 得分规则

| 项目 | 分数 |
|------|------|
| 成功抓取+投放1件 | +50分 |
| 完美返回充电区 | +30分 |
| 总分上限 | 230分 (4件×50 + 30) |

---

## 编译与烧录

### 开发环境

1. **RT-Thread Studio** (推荐)
2. **Keil MDK-ARM**
3. **GCC ARM Embedded**

### 编译步骤

#### 方法1: RT-Thread Studio
```bash
# 1. 打开项目
File -> Open Projects from File System -> 选择项目目录

# 2. 编译项目
Project -> Build Project (或按 Ctrl+B)

# 3. 下载到板子
Project -> Download (或按 F11)
```

#### 方法2: 命令行 (SCons)
```bash
# 配置编译参数
scons --menuconfig

# 编译
scons

# 清理
scons -c
```

#### 方法3: Keil MDK
```bash
# 1. 打开 .project 文件
# 2. 选择 Build Target (Debug/Release)
# 3. 点击 Build (F7)
# 4. 点击 Download (F8)
```

### 烧录工具

- **J-Link** (推荐)
- **ST-Link V2**
- **DAPLink**

### 串口调试

```bash
# 串口参数
波特率: 115200
数据位: 8
停止位: 1
校验位: None

# 常用串口工具
- PuTTY
- SecureCRT
- 串口助手
```

---

## RT-Thread 配置

### 内核配置 (.config)

```c
// 线程优先级
RT_THREAD_PRIORITY_MAX = 32

// 系统节拍
RT_TICK_PER_SECOND = 1000

// 内存管理
RT_USING_SMALL_MEM
RT_USING_SMALL_MEM_AS_HEAP

// IPC机制
RT_USING_SEMAPHORE
RT_USING_MUTEX
RT_USING_EVENT
RT_USING_MAILBOX
RT_USING_MESSAGEQUEUE

// 控制台
RT_CONSOLE_DEVICE_NAME = "uart1"
```

### 线程优先级分配

| 线程名称 | 优先级 | 栈大小 | 功能 |
|---------|--------|--------|------|
| tshell | 20 | 4096 | FinSH控制台 |
| motor_state | 16 | 1024 | 运动状态扫描 |
| comp_ctrl | 10 | 1024 | 比赛流程控制 |
| sensor_thread | ? | ? | 传感器数据采集 |

---

## 硬件协议说明

### Emm25 V5.0 步进驱动器

**通信**: UART4, 115200, 8N1

**常用命令**:
| 命令码 | 功能 | 参数 |
|--------|------|------|
| 0xA2 | 速度闭环 | 速度值(RPM) |
| 0xFD | 位置模式 | 目标位置 |
| 0xA8 | 回零 | - |
| 0x23 | 读速度 | - |

**细分设置**: 1~256 (默认32)

---

### LX-16A 总线舵机

**通信**: UART3, 115200, 8N1

**指令格式**:
```
帧头 | ID | 长度 | 命令 | 参数 | 校验
0x55 | 01 | 04  | 03  | ...  | SUM
```

**常用命令**:
| 命令 | 功能 |
|------|------|
| MOVE_SERVO | 单舵机位置+时间控制 |
| ACTION_GROUP_RUN | 动作组执行 |
| READ | 读取舵机参数 |

---

### HWT101 陀螺仪

**通信**: UART2, 9600 (可设115200)

**输出频率**: 默认10Hz

**数据内容**:
- 时间戳
- 加速度 (3轴)
- 角速度 (3轴)
- 欧拉角 (Roll/Pitch/Yaw)
- 磁场 (3轴)

---

## 项目特点

### ✨ 技术亮点

1. **模块化设计** - 各功能独立解耦，易于维护
2. **状态机架构** - 清晰的任务流程控制
3. **RTOS多线程** - 实时性保障，任务并行
4. **传感器融合** - IMU + 编码器 + 超声波 + 视觉
5. **闭环控制** - PID算法 + 步进闭环驱动
6. **错误恢复** - 重启机制 + 超时保护

### 📊 代码统计

| 项目 | 数量 |
|------|------|
| 应用层文件 | 29个 (.c/.h) |
| 应用层代码 | 231KB |
| 驱动层代码 | 383KB |
| RT-Thread内核 | 27MB |

### 🔧 依赖库

- RT-Thread 4.1.1
- STM32F1xx HAL Driver
- CMSIS-Core
- Lobot舵机控制库
- WIT陀螺仪SDK

---

## 开发团队

### 项目信息
- **竞赛**: 创新机器人制作竞赛
- **主题**: 超市机器人挑战赛
- **版本**: 赛场初版本 (v1.0)
- **最后更新**: 2025-01-16

### 技术栈
- **嵌入式**: STM32 + RT-Thread
- **传感器**: IMU + 超声波 + 视觉
- **控制算法**: PID + 状态机
- **通信**: 多UART并发

---

## 许可证

```
Copyright (c) 2006-2025, RT-Thread Development Team

SPDX-License-Identifier: Apache-2.0
```

---

## 相关文档

- [RT-Thread 官方文档](https://www.rt-thread.io/document/)
- [STM32F103 参考手册](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [Emm25 驱动器说明书](./说明书/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf)
- [HWT101 协议文档](./说明书/HWT101协议.pdf)
- [总线舵机协议](./说明书/02%20总线舵机通信协议.pdf)
- [竞赛规则](./说明书/（非正式版）附件1%20-%20创新机器人制作竞赛规则.pdf)

---

## 问题反馈

如有问题或建议，请通过以下方式联系：

- 📧 提交 Issue
- 💬 RT-Thread 论坛
- 📚 RT-Thread 文档中心

---

<div align="center">

**Made with ❤️ for RT-Thread Robot Competition**

[⬆ 返回顶部](#rtt-robotic-project)

</div>
