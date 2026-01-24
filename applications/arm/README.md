# 机械臂控制模块 (ARM)

## 📋 概述

本模块用于控制3自由度机械臂系统，采用LX-16A总线舵机，通过UART3与RT-Thread系统通信。

### 硬件组成
- **舵机型号**: LX-16A 总线舵机
- **通信接口**: UART3 (115200 baud, 8N1)
- **舵机数量**: 3个
  - ID 1: 旋转台舵机 (ARM_ROTATE_SERVO_ID)
  - ID 2: 伸缩臂舵机 (ARM_EXTEND_SERVO_ID)
  - ID 3: 爪子舵机 (ARM_GRAB_SERVO_ID)

---

## 🔧 硬件配置

### UART引脚分配 (USART3)
| 信号 | 引脚 | 说明 |
|------|------|------|
| TX | PB10 | STM32 → 舵机 |
| RX | PB11 | 舵机 → STM32 |

### 舵机ID配置
LX-16A舵机ID通过硬件按钮配置，需与代码定义一致：
```
ID 1 → 旋转台 (0°/180° 旋转)
ID 2 → 伸缩臂 (0-135mm 伸缩)
ID 3 → 爪子 (开合动作)
```

---

## 📐 位置参数说明

### 1. 旋转台 (ID: 1)
| 常量 | 数值 | 功能 |
|------|------|------|
| `ROTATE_FRONT_POS` | 0 | 朝向车头方向 |
| `ROTATE_BACK_POS` | 750 | 朝向货架(180°) |

### 2. 伸缩臂 (ID: 2)
| 常量 | 数值 | 功能 |
|------|------|------|
| `ARM_EXTENDED_POS` | 0 | 机械臂伸出到货架 |
| `ARM_RETRACTED_POS` | 135 | 机械臂完全收回 |

### 3. 爪子 (ID: 3)
| 常量 | 数值 | 功能 |
|------|------|------|
| `GRAB_OPEN_POS` | 300 | 爪子张开(准备抓取) |
| `GRAB_CLOSE_POS` | 700 | 爪子闭合(抓住物品) |

---

## 💻 API接口

### 单舵机控制
```c
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
```
**参数:**
- `servoID`: 舵机ID (0-31)
- `Position`: 目标位置 (0-1000)
- `Time`: 运动时间 (ms)

**示例:**
```c
// 1秒内将旋转台转到朝前位置
moveServo(ARM_ROTATE_SERVO_ID, ROTATE_FRONT_POS, 1000);
rt_thread_mdelay(1200);  // 等待运动完成

// 1秒内将爪子张开
moveServo(ARM_GRAB_SERVO_ID, GRAB_OPEN_POS, 1000);
```

---

### 多舵机控制 (数组方式)
```c
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
```

**示例:**
```c
LobotServo servos[2];
servos[0].ID = ARM_ROTATE_SERVO_ID;
servos[0].Position = ROTATE_BACK_POS;
servos[1].ID = ARM_EXTEND_SERVO_ID;
servos[1].Position = ARM_EXTENDED_POS;

// 同时控制旋转和伸缩，2秒完成
moveServosByArray(servos, 2, 2000);
```

---

### 多舵机控制 (可变参数)
```c
void moveServos(uint8_t Num, uint16_t Time, ...);
```

**示例:**
```c
// 同时控制3个舵机
moveServos(3, 1500,
    ARM_ROTATE_SERVO_ID, ROTATE_BACK_POS,
    ARM_EXTEND_SERVO_ID, ARM_EXTENDED_POS,
    ARM_GRAB_SERVO_ID, GRAB_OPEN_POS
);
```

---

### 动作组控制
```c
// 运行动作组 (Times=0表示无限循环)
void runActionGroup(uint8_t numOfAction, uint16_t Times);

// 停止动作组
void stopActionGroup(void);

// 设置动作组速度
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
```

---

## 📡 通信协议

### 帧格式
```
| 0x55 | 0x55 | 长度 | 命令 | 参数... | 校验 |
```

### 单舵机移动命令 (0x03)
```
| 0x55 | 0x55 | 8 | 0x03 | 1 | Time_L | Time_H | ID | Pos_L | Pos_H |
```

### 命令定义
| 命令码 | 功能 | 说明 |
|--------|------|------|
| 0x03 | CMD_SERVO_MOVE | 舵机移动 |
| 0x06 | CMD_ACTION_GROUP_RUN | 运行动作组 |
| 0x07 | CMD_ACTION_GROUP_STOP | 停止动作组 |
| 0x0B | CMD_ACTION_GROUP_SPEED | 设置动作组速度 |
| 0x0F | CMD_GET_BATTERY_VOLTAGE | 获取电池电压 |

---

## 🎯 典型应用场景

### 场景1: 货架抓取任务
```c
// 1. 旋转到货架方向
moveServo(ARM_ROTATE_SERVO_ID, ROTATE_BACK_POS, 1000);
rt_thread_mdelay(1200);

// 2. 伸出机械臂
moveServo(ARM_EXTEND_SERVO_ID, ARM_EXTENDED_POS, 1000);
rt_thread_mdelay(1200);

// 3. 张开爪子
moveServo(ARM_GRAB_SERVO_ID, GRAB_OPEN_POS, 500);
rt_thread_mdelay(600);

// 4. 闭合爪子抓取
moveServo(ARM_GRAB_SERVO_ID, GRAB_CLOSE_POS, 500);
rt_thread_mdelay(600);

// 5. 收回机械臂
moveServo(ARM_EXTEND_SERVO_ID, ARM_RETRACTED_POS, 1000);
rt_thread_mdelay(1200);

// 6. 旋转回车头方向
moveServo(ARM_ROTATE_SERVO_ID, ROTATE_FRONT_POS, 1000);
rt_thread_mdelay(1200);
```

### 场景2: 快速复位
```c
// 同时控制所有舵机回到初始位置
LobotServo servos[3] = {
    {ARM_ROTATE_SERVO_ID, ROTATE_FRONT_POS},
    {ARM_EXTEND_SERVO_ID, ARM_RETRACTED_POS},
    {ARM_GRAB_SERVO_ID, GRAB_OPEN_POS}
};
moveServosByArray(servos, 3, 1000);
```

---

## ⚙️ 文件结构

```
applications/arm/
├── Src/
│   ├── Arm_thread.c              # RT-Thread线程入口 (已注释)
│   └── LobotServoController.c    # 舵机协议实现
├── Inc/
│   ├── LobotServoController.h    # API接口定义
│   └── bool.h                    # 布尔类型定义
└── README.md                     # 本文档
```

---

## 🔍 调试说明

### FinSH命令测试
通过UART1控制台(115200 baud)可以直接测试：

```c
// 测试单个舵机
moveServo(1, 500, 1000);

// 测试多舵机
LobotServo test[2] = {{1, 0}, {2, 100}};
moveServosByArray(test, 2, 1500);
```

### 常见问题

| 问题 | 原因 | 解决方法 |
|------|------|----------|
| 舵机无响应 | UART未正确初始化 | 检查drv_usart.c中UART3配置 |
| 运动不到位 | 位置参数超限 | 确认Position在0-1000范围内 |
| 运动抖动 | Time参数过小 | 增加运动时间到≥500ms |
| ID冲突 | 硬件ID与代码不一致 | 重新配置舵机ID |

---

## 📚 参考资料

- **协议文档**: `说明书/02 总线舵机通信协议.pdf`
- **SDK示例**: `LobotServoController.c` (深圳乐幻索尔科技)
- **RT-Thread文档**: https://www.rt-thread.org/document/site/

---

## 📝 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2025-01 | 初始版本，基于Lobot SDK移植 |
| v1.1 | 2026-01-21 | 添加本文档 |

---

## 👥 维护者

- **原始SDK**: 深圳乐幻索尔科技
- **RT-Thread移植**: 机器人竞赛项目组
