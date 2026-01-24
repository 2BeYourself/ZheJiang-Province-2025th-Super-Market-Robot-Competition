#include "motor/Inc/motor_cmd.h"

static rt_device_t motor_uart = RT_NULL;
#define READ_TIMEOUT_MS  20   // 最长等待 20ms，可根据波特率/场景调
#define MS_TO_TICKS(ms)  ((ms) * RT_TICK_PER_SECOND / 1000)

/* 打开步进电机串口uart4 */
rt_err_t motor_cmd_init(const char *uart_name)
{
   /* 如果已经初始化，直接返回成功 */
   if (motor_uart != RT_NULL)
   {
       return RT_EOK;
   }

   motor_uart = rt_device_find(uart_name);

   if (motor_uart != RT_NULL)
   {
       rt_kprintf("Device %s found.\n", uart_name);
       /* UART4已配置DMA接收，使用RT_DEVICE_FLAG_DMA_RX标志 */
       rt_err_t open_result = rt_device_open(motor_uart, RT_DEVICE_FLAG_RDWR);

       if (open_result == RT_EOK)
       {
           rt_kprintf("Device %s opened successfully.\n", uart_name);
           return RT_EOK;
       }

       else
       {
           rt_kprintf("Failed to open device %s, error code: %d\n", uart_name, open_result);
           motor_uart = RT_NULL; // 打开失败，将其重置为NULL
           return open_result;
       }
   }

   else
   {
       rt_kprintf("Device %s not found.\n", uart_name);
       return -RT_ERROR;
   }
}

/* 串口发送数据 - DMA 模式（旧版本，已弃用） */
/*
static void send_frame(const uint8_t *buf, size_t len)
{
   rt_device_write(motor_uart, 0, buf, len);
   rt_thread_mdelay(2);
}
*/

/**
 * @brief 发送命令并等待应答（带重试机制）
 * @param buf 命令缓冲区
 * @param len 命令长度
 * @param max_retry 最大重试次数（0=无限重试直到成功）
 * @return RT_EOK 收到正确应答，-RT_ETIMEOUT 超时，-RT_ERROR 收到错误应答
 *
 * @note 应答格式（4字节）：地址 + 功能码 + 状态码 + 校验字节
 *       - 状态码 0x02 = 成功
 *       - 状态码 0xE2 = 条件不满足（堵转保护、电机没使能）
 *       - 状态码 0xEE = 错误命令
 */
static rt_err_t send_frame_with_ack(const uint8_t *buf, size_t len, uint8_t max_retry)
{
    uint8_t cmd_addr = buf[0];           // 命令中的设备地址
    uint8_t cmd_func = buf[1];           // 命令功能码
    uint8_t ack_buf[4];                  // 应答缓冲区
    uint8_t retry_count = 0;
    rt_err_t result = -RT_ETIMEOUT;

    while (max_retry == 0 || retry_count < max_retry)
    {
        /* 1. 发送命令 */
        rt_device_write(motor_uart, 0, buf, len);

        /* 2. 等待 DMA 发送完成（2ms 足够最大帧发送） */
        rt_thread_mdelay(2);

        /* 3. 读取应答（超时 50ms） */
        rt_tick_t start = rt_tick_get();
        rt_size_t recv_len = 0;
        uint8_t rx_byte;

        while ((rt_tick_get() - start) < RT_TICK_PER_SECOND / 20)  // 50ms 超时
        {
            if (rt_device_read(motor_uart, 0, &rx_byte, 1) == 1)
            {
                ack_buf[recv_len++] = rx_byte;

                /* 收到完整应答帧（4字节） */
                if (recv_len == 4)
                {
                    /* 校验应答帧格式 */
                    if (ack_buf[0] == cmd_addr &&    // 地址匹配
                        ack_buf[1] == cmd_func &&    // 功能码匹配
                        ack_buf[3] == 0x6B)          // 校验字节匹配
                    {
                        uint8_t status = ack_buf[2];

                        if (status == 0x02)  // 成功
                        {
                            rt_kprintf("[Motor] CMD %02X OK (addr=%02X)\n",
                                       cmd_func, cmd_addr);
                            return RT_EOK;
                        }
                        else if (status == 0xE2)  // 条件不满足
                        {
                            rt_kprintf("[Motor] CMD %02X FAIL: 条件不满足 (堵转保护/电机未使能) (addr=%02X)\n",
                                       cmd_func, cmd_addr);
                            result = -RT_ERROR;
                            break;  // 跳出接收循环，进入重试
                        }
                        else if (status == 0xEE)  // 错误命令
                        {
                            rt_kprintf("[Motor] CMD %02X ERROR: 无效命令 (addr=%02X)\n",
                                       cmd_func, cmd_addr);
                            result = -RT_ERROR;
                            break;  // 跳出接收循环，进入重试
                        }
                        else  // 未知状态码
                        {
                            rt_kprintf("[Motor] CMD %02X UNKNOWN STATUS: %02X (addr=%02X)\n",
                                       cmd_func, status, cmd_addr);
                            result = -RT_ERROR;
                            break;
                        }
                    }
                    else  // 应答帧格式错误
                    {
                        rt_kprintf("[Motor] Invalid ACK frame: %02X %02X %02X %02X\n",
                                   ack_buf[0], ack_buf[1], ack_buf[2], ack_buf[3]);
                        recv_len = 0;  // 重置接收，继续等待
                    }
                }
            }
            else
            {
                rt_thread_mdelay(1);  // 没有数据，稍微延时
            }
        }

        /* 4. 超时处理 */
        if (recv_len < 4)
        {
            rt_kprintf("[Motor] CMD %02X Timeout (retry=%d/%d, addr=%02X)\n",
                       cmd_func, retry_count + 1, max_retry, cmd_addr);
        }

        /* 5. 重试计数 */
        retry_count++;
        if (max_retry == 0)
        {
            rt_kprintf("[Motor] Retrying...\n");
        }
        else if (retry_count >= max_retry)
        {
            rt_kprintf("[Motor] Max retry reached, give up.\n");
            break;
        }

        /* 6. 重试前延时（避免连续发送过快） */
        rt_thread_mdelay(10);
    }

    return result;
}

/**
 * @brief 简单发送命令（不带应答确认）
 */
static void send_frame(const uint8_t *buf, size_t len)
{
   rt_device_write(motor_uart, 0, buf, len);
   rt_thread_mdelay(2);  /* 等待 DMA 发送完成 */
}

/* 速度模式控制（不带应答确认，用于高频状态机） */
rt_err_t motor_cmd_speed(uint8_t addr,
                        uint8_t dir,
                        uint16_t rpm,
                        uint8_t accel,
                        uint8_t sync_flag)
{
   uint8_t cmd[8];

   cmd[0] = addr;                    // 设备地址
   cmd[1] = 0xF6;                    // 功能码：速度模式
   cmd[2] = dir;                     // 方向
   cmd[3] = (rpm >> 8) & 0xFF;       // 速度高字节
   cmd[4] =  rpm        & 0xFF;      // 速度低字节
   cmd[5] = accel;                   // 加速度档位
   cmd[6] = sync_flag;               // 同步标志：1 同步，0 不同步
   cmd[7] = 0x6B;                    // 校验字节

   send_frame(cmd, sizeof(cmd));     // 发送整帧
   return RT_EOK;                    // 返回成功
}

/**
 * @brief 速度模式控制（带应答确认）
 * @param addr 电机地址
 * @param dir 方向（0x00=CW顺时针, 0x01=CCW逆时针）
 * @param rpm 速度（0-3000 RPM）
 * @param accel 加速度档位（0=不使用曲线加减速，1-255=使用曲线加减速）
 * @param sync_flag 同步标志（0=立即执行，1=等待同步命令）
 * @param max_retry 最大重试次数（0=无限重试直到成功，3=重试3次）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_speed_ack(uint8_t addr,
                             uint8_t dir,
                             uint16_t rpm,
                             uint8_t accel,
                             uint8_t sync_flag,
                             uint8_t max_retry)
{
   uint8_t cmd[8];

   cmd[0] = addr;
   cmd[1] = 0xF6;
   cmd[2] = dir;
   cmd[3] = (rpm >> 8) & 0xFF;
   cmd[4] =  rpm        & 0xFF;
   cmd[5] = accel;
   cmd[6] = sync_flag;
   cmd[7] = 0x6B;

   return send_frame_with_ack(cmd, sizeof(cmd), max_retry);
}


/* 位置闭环命令（不带应答确认，用于高频状态机） */
rt_err_t motor_cmd_position(uint8_t addr,
                                 uint8_t dir,
                                 uint16_t rpm,
                                 uint8_t accel,
                                 uint32_t pulses,
                                 uint8_t rel_abs,
                                 uint8_t sync_flag)
{
   uint8_t cmd[13];
   cmd[0] = addr;                    // 地址
   cmd[1] = 0xFD;                    // 功能码：位置模式
   cmd[2] = dir;                     // 方向
   cmd[3] = (rpm >> 8) & 0xFF;       // 速度高字节
   cmd[4] = rpm & 0xFF;              // 速度低字节
   cmd[5] = accel;                   // 加速度档位
   cmd[6] = (pulses >> 24) & 0xFF;   // 脉冲数字节3
   cmd[7] = (pulses >> 16) & 0xFF;   // 脉冲数字节2
   cmd[8] = (pulses >> 8) & 0xFF;    // 脉冲数字节1
   cmd[9] = pulses & 0xFF;           // 脉冲数字节0
   cmd[10] = rel_abs;                // 相对/绝对
   cmd[11] = sync_flag;              // 同步标志
   cmd[12] = 0x6B;                   // 校验字节

   send_frame(cmd, sizeof(cmd));
   return RT_EOK;
}

/**
 * @brief 位置模式控制（带应答确认）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_position_ack(uint8_t addr,
                                uint8_t dir,
                                uint16_t rpm,
                                uint8_t accel,
                                uint32_t pulses,
                                uint8_t rel_abs,
                                uint8_t sync_flag,
                                uint8_t max_retry)
{
   uint8_t cmd[13];
   cmd[0] = addr;
   cmd[1] = 0xFD;
   cmd[2] = dir;
   cmd[3] = (rpm >> 8) & 0xFF;
   cmd[4] = rpm & 0xFF;
   cmd[5] = accel;
   cmd[6] = (pulses >> 24) & 0xFF;
   cmd[7] = (pulses >> 16) & 0xFF;
   cmd[8] = (pulses >> 8) & 0xFF;
   cmd[9] = pulses & 0xFF;
   cmd[10] = rel_abs;
   cmd[11] = sync_flag;
   cmd[12] = 0x6B;

   return send_frame_with_ack(cmd, sizeof(cmd), max_retry);
}

/* 立即停止命令（不带应答确认，用于高频状态机） */
rt_err_t motor_cmd_stop(uint8_t addr, uint8_t sync_flag)
{
   uint8_t cmd[5];
   cmd[0] = addr;        /* 地址 */
   cmd[1] = 0xFE;        /* 功能码：立即停止 */
   cmd[2] = 0x98;        /* 停止子码 */
   cmd[3] = sync_flag;   /* 同步标志 */
   cmd[4] = 0x6B;        /* 校验 */

   send_frame(cmd, sizeof(cmd));
   return RT_EOK;
}

/**
 * @brief 立即停止命令（带应答确认）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_stop_ack(uint8_t addr, uint8_t sync_flag, uint8_t max_retry)
{
   uint8_t cmd[5];
   cmd[0] = addr;
   cmd[1] = 0xFE;
   cmd[2] = 0x98;
   cmd[3] = sync_flag;
   cmd[4] = 0x6B;

   return send_frame_with_ack(cmd, sizeof(cmd), max_retry);
}

/* 多机同步运动（不带应答确认） */
rt_err_t motor_cmd_sync(void)
{
   uint8_t cmd[4];
   cmd[0] = 0x00;       /* 地址: 使用广播地址如 0x00 */
   cmd[1] = 0xFF;       /* 功能码：多机同步运动 */
   cmd[2] = 0x66;       /* 子码: 同步启动 */
   cmd[3] = 0x6B;       /* 校验字节*/

   send_frame(cmd, sizeof(cmd));
   return RT_EOK;
}

/**
 * @brief 多机同步运动（带应答确认）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_sync_ack(uint8_t max_retry)
{
   uint8_t cmd[4];
   cmd[0] = 0x00;
   cmd[1] = 0xFF;
   cmd[2] = 0x66;
   cmd[3] = 0x6B;

   return send_frame_with_ack(cmd, sizeof(cmd), max_retry);
}

///* 读取实时转速命令(非阻塞式) */
//rt_err_t motor_cmd_read_rpm_nb(uint8_t addr, int16_t *rpm)
//{
//  uint8_t cmd[3] = { addr, 0x35, 0x6B };
//  uint8_t buf[16];
//  uint8_t rx;
//  int     idx = 0;
//
//  rt_tick_t start = rt_tick_get();
//  rt_tick_t timeout = MS_TO_TICKS(READ_TIMEOUT_MS);
//
//  // 1) 丢掉旧数据
//  while (rt_device_read(motor_uart, 0, &rx, 1) == 1) { }
//
//  // 2) 发命令
//  send_frame(cmd, sizeof(cmd));
//
//  // 3) 逐字节非阻塞读，直到超时或找到一帧
//  while ((rt_tick_get() - start) < timeout)
//  {
//      if (rt_device_read(motor_uart, 0, &rx, 1) == 1)
//      {
//          rt_kprintf("RX: %02X | ", rx);
//          // 把新字节推入滑动窗口缓冲
//          if (idx < sizeof(buf)) buf[idx++] = rx;
//          else                   // 缓冲满就保留最新 16 字节
//          {
//              memmove(buf, buf + 1, sizeof(buf) - 1);
//              buf[sizeof(buf) - 1] = rx;
//          }
//
//          // 只要缓冲 >=6，就检查最后一段是否为一帧
//          if (idx >= 6)
//          {
//              int base = idx - 6;
//              if (buf[base]   == addr    &&
//                  buf[base+1] == 0x35    &&
//                  buf[base+5] == 0x6B)
//              {
//                  // 解析
//                  uint16_t mag = (buf[base+3] << 8) | buf[base+4];
//                  int16_t  v   = (int16_t)mag;
//                  // 按方向打负号
//                  if (buf[base+2] & 0x01) v = -v;
//                  *rpm = v;
////                   rt_thread_mdelay(50);
////                    rt_kprintf("%d\n",v);
//                  return RT_EOK;
//              }
//          }
//      }
//      else
//      {
//          // 没读到数据，就稍微休息一下，yield CPU
//          rt_thread_mdelay(1);
//      }
//  }
//  rt_kprintf("Read RPM timeout\n");
//  return -RT_ETIMEOUT;
//}

/************************************************************************/
