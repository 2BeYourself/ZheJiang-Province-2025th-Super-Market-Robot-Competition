# 步进电机延迟问题诊断报告

**日期**: 2025-01-18
**项目**: RTT-Robotic-Project
**问题**: UART4蓝牙控制步进电机经常延迟1-20秒才执行

---

## 1. 系统架构

### 硬件连接

```
[PC终端] --UART1(有线)--> [MCU控制板] --UART4 TX--> [蓝牙HC-05]
                                                 ↓
                                          无线配对(115200)
                                                 ↓
[步进电机] <--UART-- [蓝牙HC-05]         [蓝牙HC-05]
 (Emm25 V5.0)
```

### 串口配置

| UART | 设备 | 波特率 | 功能 | DMA配置 |
|------|------|--------|------|----------|
| **UART1** | 有线串口 | 115200 | FinSH控制台 | ❌ 禁用（避免IDLE中断延迟） |
| **UART2** | 陀螺仪HWT101 | 115200 | 姿态数据采集 | ✅ 启用（高频数据流） |
| **UART3** | 舵机LX-16A | 115200 | 机械臂控制 | ❌ 禁用 |
| **UART4** | 蓝牙HC-05 | 115200 | 步进电机控制 | ✅ 启用（仅TX，RX影响不大） |
| **UART5** | YOLO视觉模块 | 115200 | 视觉通信 | 可选 |

---

## 2. 问题现象

### 主要问题

**通过PC发送命令后，步进电机经常延迟1-20秒才执行，偶尔延迟无限长**

### 具体日志

```bash
# PC通过UART1发送命令
msh_elevator_down 100

# MCU立即响应（<10ms）
[21:42:28.999] msh >msh_elevator_down 100
[21:42:29.005] [Elevator] 下降 - 设置状态: ST_DOWN, RPM=100

# 步进电机延迟1-20秒才开始动作 ❌
```

### 延迟统计

| 测试项 | 结果 | 结论 |
|--------|------|------|
| **UART1命令接收** | bt_test_ping立即响应 | ✅ UART1正常，无延迟 |
| **UART1时间戳** | 正常递增（~1000 ticks/次） | ✅ 系统Tick正常 |
| **UART4 TX发送** | 耗时0-1 ticks (<1ms) | ✅ UART4发送正常 |
| **命令重复次数** | 每次发送3次 | ⚠️ poll循环重复调用 |

---

## 3. 根本原因分析

### 3.1 延迟来源确认

**✅ 延迟不在MCU端**

- MCU立即显示命令信息
- UART4 TX发送耗时 < 1ms
- 蓝牙配对成功，波特率匹配

**❌ 延迟在步进电机执行**

根据Emm25 V5.0说明书，当前配置：
```c
// motor_state.c line 493-501
rt_err_t elevator_up(uint16_t rpm)
{
  motor_cmd_position(Elevator,
                      CW,
                      rpm,
                      0,
                      145000,   /* 绝对位置脉冲数 */
                      Absolute,
                      RT_NULL);
  return RT_EOK;
}
```

### 3.2 执行时间计算

**参数**:
- 位置脉冲数: **145,000**
- 速度: **100 RPM**
- 假设细分: **2000脉冲/转** (需确认实际设置)

**计算**:
```
执行时间(s) = 位置脉冲数 / (RPM × 脉冲数/转 / 60)
           = 145,000 / (100 × 2000 / 60)
           = 145,000 / 3333.33
           ≈ 43.5秒
```

**结论**: 1-20秒延迟是正常的！步进电机需要43.5秒才能执行完145,000脉冲的位置移动。(这里的结论是错误的，1-20s指的是反应时间，而非动作执行时间)

### 3.3 命令重复发送问题

**现象**: 每次命令被发送3次

**原因**: `motor_state_poll()` 在20Hz循环中被调用
```c
// motor_thread.c line 15-16
while (1)
{
    motor_state_poll();  // 每50ms执行一次
    rt_thread_mdelay(50);
}
```

每次调用都会执行：
```c
case ST_UP: elevator_up(g_elevator_rpm); break;  // 重复发送
```

---

## 4. 诊断测试过程

### 测试1: UART1响应测试

```bash
msh >bt_test_ping
[BT] PONG - 立即响应测试  # ✅ 立即响应
```

**结论**: UART1（有线串口）通信正常

### 测试2: 系统Tick测试

```bash
msh >bt_test_timestamp
[BT] Tick=12260683 - 时间戳响应

msh >bt_test_timestamp
[BT] Tick=12261683 - 时间戳响应  # ✅ 递增1000 ticks

msh >bt_test_timestamp
[BT] Tick=12262634 - 时间戳响应  # ✅ 正常
```

**结论**: 系统时钟正常，无卡顿

### 测试3: UART4 TX发送测试

```bash
msh_elevator_down 100
[Elevator] 下降 - 设置状态: ST_DOWN, RPM=100
[UART4 TX] 发送 13 字节, 耗时 1 ticks  # ✅ < 1ms
[UART4 TX] 发送 13 字节, 耗时 1 ticks
[UART4 TX] 发送 13 字节, 耗时 1 ticks
```

**结论**:
- UART4 TX发送速度正常（0-1 ticks）
- 命令重复发送3次（poll循环导致）

---

## 5. 已尝试的解决方案

### 5.1 UART4 DMA接收配置

**尝试**: 禁用UART4的DMA接收

**结果**:
- ❌ 未解决问题
- **发现**: UART4主要用于TX发送，DMA RX配置影响不大

**最终配置**: 保持DMA启用

### 5.2 添加状态变化检测

**尝试**: 只在状态切换时发送一次命令

```c
// 失败的尝试
case ST_UP:
    if (cur_state != last_state) {
        elevator_up(g_elevator_rpm);
        last_state = cur_state;
    }
    break;
```

**结果**:
- ❌ 命令无法发送
- 原因: 状态初始化逻辑有问题

**当前状态**: 已恢复原始版本（重复发送3次，但可用）

---

## 6. 优化建议

### 6.1 短期方案（快速验证）

#### 方案A: 减少目标位置

```c
// 当前
elevator_up(100) → 145000脉冲 → ~43.5秒

// 优化后
elevator_up(100) → 20000脉冲 → ~6秒
elevator_up(100) → 10000脉冲 → ~3秒
```

#### 方案B: 提高速度

```c
// 当前
msh_elevator_up 100  → 43.5秒

// 优化后
msh_elevator_up 200  → 21.8秒（时间减半）
msh_elevator_up 300  → 14.5秒
```

#### 方案C: 使用速度模式（推荐）

根据Emm25说明书，速度模式(0xA2)立即响应：

```c
// 替代位置模式
motor_cmd_speed(Elevator, CW, rpm, accel, RT_NULL);
// 优点: 立即开始转动，无固定执行时间
// 缺点: 需要手动控制停止时机
```

### 6.2 长期方案（彻底解决）

#### 1. 优化状态机

添加状态变化检测，避免重复发送：

```c
// 伪代码
static motor_state_t last_executed_state = ST_IDLE;

void motor_state_poll(void) {
    switch (cur_state) {
        case ST_UP:
            if (cur_state != last_executed_state) {
                elevator_up(g_elevator_rpm);
                last_executed_state = cur_state;
            }
            break;
        // ... 其他状态
    }
}
```

#### 2. 添加命令完成检测

根据Emm25说明书，可以读取电机状态：

```c
// 检查电机是否执行完成
uint8_t status = motor_cmd_read_status(Elevator);
if (status & STATUS_COMPLETE) {
    // 切换到下一个状态
}
```

#### 3. 使用非阻塞式控制

```c
// 发送命令后立即返回
elevator_start(rpm);

// 在后台线程中监控
void elevator_monitor_thread() {
    while (1) {
        if (elevator_is_complete()) {
            // 处理完成事件
        }
        rt_thread_mdelay(100);
    }
}
```

---

## 7. 参考信息

### 7.1 Emm25 V5.0 关键参数

- **通信协议**: UART串口
- **波特率**: 115200 (8N1)
- **位置模式命令**: 0xFD
- **速度模式命令**: 0xA2
- **停止命令**: 0xFE
- **读速度命令**: 0x23

### 7.2 相关文件

| 文件 | 位置 | 说明 |
|------|------|------|
| **电机控制** | `applications/motor/Src/motor_cmd.c` | UART4通信底层驱动 |
| **状态机** | `applications/motor/Src/motor_state.c` | 运动状态轮询 |
| **线程** | `applications/motor/Src/motor_thread.c` | MSH命令接口 |
| **UART配置** | `drivers/board.h` | DMA/引脚配置 |
| **说明书** | `说明书/Emm_V5.0步进闭环驱动说明书Rev1.3.pdf` | 完整协议文档 |

### 7.3 测试命令

```bash
# 诊断命令
bt_test_ping          # 测试立即响应
bt_test_timestamp     # 测试系统时钟
bt_test_loop 10       # 测试循环输出
msh_elevator_status   # 查询升降电机状态

# 控制命令
msh_elevator_up 100   # 上升（100 RPM）
msh_elevator_down 100 # 下降（100 RPM）
msh_elevator_stop     # 停止
```

---

## 8. 下一步行动计划

### 立即执行（高优先级）

1. ✅ 确认步进电机细分设置（2000脉冲/转？）
2. ✅ 测试不同RPM下的实际执行时间
3. ⬜ 修改目标位置脉冲数（145000 → 50000）
4. ⬜ 验证执行时间是否符合预期

### 短期优化（中优先级）

5. ⬜ 实现状态变化检测（避免重复发送）
6. ⬜ 添加命令发送确认机制
7. ⬜ 测试速度模式的可行性

### 长期改进（低优先级）

8. ⬜ 实现非阻塞式控制
9. ⬜ 添加电机状态反馈
10. ⬜ 优化蓝牙通信协议

---

## 9. 总结

**问题本质**: 
####1.状态切换线程中的:motor_state_poll();

该函数一直在发送数据,使用有线连接的时候是没有延迟的；替换为HC-05蓝牙模式就会有1-20s甚至是时间无限长的延迟。无限发送数据，那么也会无限得到返回值，导致串口占用率过高(甚至是接收端一直被中断)

但是，如果更改该函数一次只发一次数据，使用无线是绝对没有响应的，经过测试使用有线也没有反应。但是发送三次就会有反应。

问题解决：有线传输3次基本没有什么问题，对于速度要求强度高的确实需要一直进行该状态进行发送，像上升/下降一层则不需要一直发，发3次就够了。

HC-05传输效果尚可，但需要发送10次才勉勉强强稳定一些。

####2.motor_cmd.c文件里的send_frame();

该函数使用了rt_thread_mdelay(5);阻塞式发送,其实这一点非常不靠谱,因为rt_device_write()发送完数据后，按理来说应当在原地等(或者对数据发送函数改写，采取线程发送)等到数据发完再说，而不是采取阻塞式延时。

问题解决：rt_device_write()本质是发送完数据后才进入rt_thread_mdelay(5)的，而非数据没发完就走了。



**文档版本**: v1.1
**最后更新**: 2025-01-19
**负责人**: Claude
