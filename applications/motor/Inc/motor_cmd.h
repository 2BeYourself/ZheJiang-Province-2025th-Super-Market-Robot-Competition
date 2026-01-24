//————— motor_cmd 配置 —————//
#ifndef MOTOR_CMD_CONFIG_H
#define MOTOR_CMD_CONFIG_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_common.h"
#include <string.h>

rt_err_t motor_cmd_init(const char *uart_name); //串口4初始化

/* ========== 不带应答确认的命令（用于高频状态机） ========== */

/* 速度模式 */
rt_err_t motor_cmd_speed(uint8_t addr,
                         uint8_t dir,
                         uint16_t rpm,
                         uint8_t accel,
                         uint8_t sync_flag);

/* 相对位置模式 */
rt_err_t motor_cmd_position(uint8_t addr,
                            uint8_t dir,
                            uint16_t rpm,
                            uint8_t accel,
                            uint32_t pulses,
                            uint8_t rel_abs,
                            uint8_t sync_flag);

/* 立即停止 */
rt_err_t motor_cmd_stop(uint8_t addr, uint8_t sync_flag);

/* 同步启动 */
rt_err_t motor_cmd_sync(void);   /* 广播 0x00 一般 */

/* 读取速度 */
rt_err_t motor_cmd_read_rpm_nb(uint8_t addr, int16_t *rpm);

/* ========== 带应答确认的命令（用于关键操作） ========== */

/**
 * @brief 速度模式控制（带应答确认和重试机制）
 * @param addr 电机地址
 * @param dir 方向（0x00=CW顺时针, 0x01=CCW逆时针）
 * @param rpm 速度（0-3000 RPM）
 * @param accel 加速度档位（0=不使用曲线加减速，1-255=使用曲线加减速）
 * @param sync_flag 同步标志（0=立即执行，1=等待同步命令）
 * @param max_retry 最大重试次数（0=无限重试直到成功，建议3次）
 * @return RT_EOK 成功收到正确应答（0x02）
 *         -RT_ETIMEOUT 超时未收到应答
 *         -RT_ERROR 收到错误应答（0xE2=条件不满足, 0xEE=无效命令）
 *
 * @note 应答格式：地址 + 功能码 + 状态码 + 校验字节
 *       - 状态码 0x02：成功
 *       - 状态码 0xE2：条件不满足（堵转保护/电机未使能）
 *       - 状态码 0xEE：错误命令
 *       - 其他状态码：未知错误
 */
rt_err_t motor_cmd_speed_ack(uint8_t addr,
                             uint8_t dir,
                             uint16_t rpm,
                             uint8_t accel,
                             uint8_t sync_flag,
                             uint8_t max_retry);

/**
 * @brief 位置模式控制（带应答确认和重试机制）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_position_ack(uint8_t addr,
                                uint8_t dir,
                                uint16_t rpm,
                                uint8_t accel,
                                uint32_t pulses,
                                uint8_t rel_abs,
                                uint8_t sync_flag,
                                uint8_t max_retry);

/**
 * @brief 立即停止命令（带应答确认和重试机制）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_stop_ack(uint8_t addr, uint8_t sync_flag, uint8_t max_retry);

/**
 * @brief 多机同步运动（带应答确认和重试机制）
 * @return RT_EOK 成功，-RT_ETIMEOUT 超时，-RT_ERROR 错误
 */
rt_err_t motor_cmd_sync_ack(uint8_t max_retry);

#endif
