/**
 * @file uart_fast.c
 * @brief UART快速刷新 - 解决蓝牙延迟问题
 */

#include <rtthread.h>
#include <rtdevice.h>

static rt_thread_t uart_poll_thread = RT_NULL;
static rt_sem_t uart_sem = RT_NULL;

/* UART轮询线程 - 强制刷新接收缓冲区 */
static void uart_poll_entry(void *parameter)
{
    rt_device_t uart1 = rt_device_find("uart1");

    if (uart1 == RT_NULL)
    {
        rt_kprintf("[UART Poll] UART1 not found!\n");
        return;
    }

    rt_kprintf("[UART Poll] Started - 每10ms刷新一次\n");

    while (1)
    {
        /* 尝试读取并触发接收中断 */
        rt_uint8_t ch;
        while (rt_device_read(uart1, 0, &ch, 1) > 0)
        {
            /* 数据会被中断处理，这里只是触发 */
        }

        rt_thread_mdelay(10);  /* 10Hz轮询 */
    }
}

/* 初始化UART轮询线程 */
int uart_poll_init(void)
{
    /* 创建信号量 */
    uart_sem = rt_sem_create("uart_poll", 1, RT_IPC_FLAG_FIFO);

    /* 创建线程 */
    uart_poll_thread = rt_thread_create("uart_poll",
                                        uart_poll_entry,
                                        RT_NULL,
                                        512,
                                        5,  /* 高优先级 */
                                        10);
    if (uart_poll_thread != RT_NULL)
    {
        rt_thread_startup(uart_poll_thread);
        return 0;
    }
    return -1;
}

/* 手动刷新命令 */
static void cmd_uart_flush(int argc, char *argv[])
{
    rt_device_t uart1 = rt_device_find("uart1");
    rt_uint8_t ch;
    int count = 0;

    if (uart1 == RT_NULL)
    {
        rt_kprintf("[UART] UART1 not found\n");
        return;
    }

    /* 强制读取缓冲区 */
    while (rt_device_read(uart1, 0, &ch, 1) > 0)
    {
        count++;
    }

    rt_kprintf("[UART] 刷新完成，读取 %d 字节\n", count);
}
MSH_CMD_EXPORT(cmd_uart_flush, cmd_uart_flush - 手动刷新UART缓冲区);

/* 启用自动轮询（默认禁用） */
// INIT_APP_EXPORT(uart_poll_init);
