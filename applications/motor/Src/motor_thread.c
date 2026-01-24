#include "motor/Inc/motor_state.h"
#include "motor/Inc/motor_cmd.h"
#include <stdlib.h>

extern volatile int16_t Curent_Speed[4];

/***************************** 运动状态扫描线程 ****************************/
static void state_thread(void *p)
{
    /* 初始化底层串口uart4 */
    motor_cmd_init("uart4");
    rt_kprintf("[Motor State] UART4 initialized, state thread running...\n");

    while (1)
    {
        motor_state_poll();           /* 每循环一次执行一次动作 */
        rt_thread_mdelay(50);         /* 20Hz 刷新 */
    }
}
/******************************************************************/

/***************************** 运动状态线程扫描开启函数 ****************************/
int state_thread_init(void)
{
    rt_thread_t tid = rt_thread_create("motor_state",
                                       state_thread,
                                       RT_NULL,
                                       1024,
                                       16,
                                       10);
    if (tid) rt_thread_startup(tid);
    return 0;
}
INIT_APP_EXPORT(state_thread_init);

/**********************************************************************************/
/** MSH命令 - 升降电机控制 **/
/**********************************************************************************/

/* 升降电机上升 */
static void msh_elevator_up(int argc, char *argv[])
{
    uint16_t rpm = 100;

    if (argc > 1)
    {
        rpm = atoi(argv[1]);
    }

    rt_kprintf("[Elevator] 上升 - 设置状态: ST_UP, RPM=%u\n", rpm);
    g_elevator_rpm = rpm;  /* 更新全局RPM */
    motor_state_set(ST_UP);
}
MSH_CMD_EXPORT(msh_elevator_up, msh_elevator_up [rpm] - 升降电机上升);

/* 升降电机下降 */
static void msh_elevator_down(int argc, char *argv[])
{
    uint16_t rpm = 100;

    if (argc > 1)
    {
        rpm = atoi(argv[1]);
    }

    rt_kprintf("[Elevator] 下降 - 设置状态: ST_DOWN, RPM=%u\n", rpm);
    g_elevator_rpm = rpm;  /* 更新全局RPM */
    motor_state_set(ST_DOWN);
}
MSH_CMD_EXPORT(msh_elevator_down, msh_elevator_down [rpm] - 升降电机下降);

/* 升降电机停止 */
static void msh_elevator_stop(int argc, char *argv[])
{
    RT_ASSERT(argc > 0);

    rt_kprintf("[Elevator] 停止 - 设置状态: ST_Elevator_Stop\n");
    motor_state_set(ST_Elevator_Stop);
}
MSH_CMD_EXPORT(msh_elevator_stop, msh_elevator_stop - 升降电机停止);

/* 查询当前状态 */
static void msh_elevator_status(int argc, char *argv[])
{
    motor_state_t current_state = motor_get_current_state();

    rt_kprintf("[Elevator] 当前状态: ");
    switch (current_state)
    {
        case ST_IDLE:
            rt_kprintf("ST_IDLE (空闲)\n");
            break;
        case ST_UP:
            rt_kprintf("ST_UP (上升), RPM=%u\n", g_elevator_rpm);
            break;
        case ST_DOWN:
            rt_kprintf("ST_DOWN (下降), RPM=%u\n", g_elevator_rpm);
            break;
        case ST_Elevator_Stop:
            rt_kprintf("ST_Elevator_Stop (停止)\n");
            break;
        default:
            rt_kprintf("UNKNOWN (未知状态: %d)\n", current_state);
            break;
    }
}
MSH_CMD_EXPORT(msh_elevator_status, msh_elevator_status - 查询升降电机状态);

/**********************************************************************************/
/**********************************************************************************/
///
///
///
/////***************************** 测速线程 ****************************/
///
//static void motor_rpm_thread_entry(void *parameter)
//{
///
//  /* 局部变量保存单次读取结果 */
//  int16_t LF_RPM = 0;
//  int16_t RF_RPM = 0;
//  int16_t LB_RPM = 0;
//  int16_t RB_RPM = 0;
///
//  while (1)
//  {
//      /* 依次读取四路电机转速 */
//      if (motor_cmd_read_rpm_nb(LF, &LF_RPM) != RT_EOK) LF_RPM = 0;
//      if (motor_cmd_read_rpm_nb(RF, &RF_RPM) != RT_EOK) RF_RPM = 0;
//      if (motor_cmd_read_rpm_nb(LB, &LB_RPM) != RT_EOK) LB_RPM = 0;
//      if (motor_cmd_read_rpm_nb(RB, &RB_RPM) != RT_EOK) RB_RPM = 0;
///
//      /* 将局部变量批量写入全局数组 */
//      int16_t rpm_local[4] = { LF_RPM, RF_RPM, LB_RPM, RB_RPM };
//      for (int i = 0; i < 4; i++)
//      {
//          Curent_Speed[i] = rpm_local[i];
//      }
///
//       /* （可选）打印四路速度，便于调试 */
//        rt_kprintf("Speed LF:%d  RF:%d  LB:%d  RB:%d\r\n",
//                   Curent_Speed[0],
//                   Curent_Speed[1],
//                   Curent_Speed[2],
//                   Curent_Speed[3]);
///
//      /* 20 ms 延时，50 Hz 刷新 */
//      rt_thread_mdelay(2);
//  }
//}
///
/////******************************************************************/
////
////
////
/////************************************ 测速线程开启函数 ********************************/
//static int init_motor_rpm_thread(void)
//{
//  rt_thread_t tid = rt_thread_create("rpm_thread",
//                                     motor_rpm_thread_entry,
//                                     RT_NULL,
//                                     2048,   /* 栈大小 */
//                                     18,    /* 优先级调整为12，使其高于 get_yaw 和 motor_state */
//                                     20);   /* tick，影响 mdelay 精度 */
//  if (tid)
//      rt_thread_startup(tid);
//  return 0;
//}
//INIT_COMPONENT_EXPORT(init_motor_rpm_thread);
/////**********************************************************************************/
////
////
////
