### Task 4: motor.c — 30ms ISR 添加 A 点检测逻辑

**Files:**
- Modify: `ucdriver/motor/motor.c`

**接口:**
- Consumes: `lap_state`, `lap_start_ms`, `lap_total_ms`（来自 Task 2）, `huidu_value[5]`（来自 Task 3）, `question`, `enter`, `sys_tick_ms`
- Produces: 修改 `enter` 标志以停止电机

- [ ] **Step 1: 在 motor.c 的 extern 声明区添加 lap 相关变量**

在 `extern volatile uint8_t key_debounce_cnt;` 后面添加：

```c
/* 第二问 lap 状态机 */
extern volatile int question;
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
extern volatile uint32_t sys_tick_ms;
```

- [ ] **Step 2: 修改 motor_pid_INST_IRQHandler**

将原 ISR 函数体替换为：

```c
//30ms定时器中断服务函数
void motor_pid_INST_IRQHandler(void)
{
    //按键消抖计数器递减（60ms消抖窗口）
    if(key_debounce_cnt) key_debounce_cnt--;
   switch(DL_Timer_getPendingInterrupt(motor_pid_INST))
   {
    case DL_TIMER_IIDX_LOAD:
        //发车
        if(enter == 1)
        {
            //灰度控制
            adjust_motor();
            //计算电机速度
            calculate_speed(1);
            calculate_speed(2);
            //pid控制
            motor_PID(1);
            motor_PID(2);

            /* 第二问: 跑圈计时 + 自动停止 */
            if(question == 2)
            {
                if(lap_state == LAP_IDLE || lap_state == LAP_COMPLETE)
                {
                    // 从待命或完成状态 -> 开始跑圈
                    lap_state = LAP_RUNNING;
                    lap_start_ms = sys_tick_ms;
                }
                else if(lap_state == LAP_RUNNING)
                {
                    // 起步后延迟2s开始检测A点，避免出发位置误判
                    if(sys_tick_ms - lap_start_ms > 2000)
                    {
                        /* A点特征: L1+M+R1同时检测到黑线, L2+R2丢线 */
                        if(huidu_value[1] && huidu_value[2] && huidu_value[3]
                            && !huidu_value[0] && !huidu_value[4])
                        {
                            lap_state = LAP_COMPLETE;
                            lap_total_ms = sys_tick_ms - lap_start_ms;
                            enter = 0;  // 停止电机
                        }
                    }
                }
            }
        }
        //未发车时（包括Q2完成后的状态），电机停止
        if(enter == 0)
        {
            motor_set_duty(1, 0);
            motor_set_duty(2, 0);
        }

    default:
        break;
   }
}
```
