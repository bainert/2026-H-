### Task 5: main.c — OLED 显示计时信息

**Files:**
- Modify: `main.c`

**接口:**
- Consumes: `lap_state`, `lap_start_ms`, `lap_total_ms`（来自 Task 2）, `question`, `enter`, `sys_tick_ms`

- [ ] **Step 1: 添加 lap 变量 extern 声明**

在 `extern volatile uint32_t sys_tick_ms;` 后面添加：

```c
// 第二问: 跑圈状态
extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;
```

- [ ] **Step 2: 修改 oled_proc() 函数**

在原有显示之后、`OLED_Refresh()` 之前添加 Q2 计时显示：

```c
//oled显示
void oled_proc(void)
{
    //第一行：发车状态
    OLED_ShowString(0, 0, (u8 *)"Status:", 12);
    if(enter)
        OLED_ShowString(42, 0, (u8 *)"RUN ", 12);
    else
        OLED_ShowString(42, 0, (u8 *)"STOP", 12);

    //第二行：题号
    OLED_ShowString(0,12,(u8 *)"question: ", 12);
    OLED_ShowNum(54, 12, question, 2, 12);

    /* 第二问: 跑圈计时显示 */
    if(question == 2)
    {
        if(lap_state == LAP_RUNNING)
        {
            // 实时显示 elapsed time (秒)
            uint32_t elapsed = sys_tick_ms - lap_start_ms;
            OLED_ShowString(0, 24, (u8 *)"T:", 12);
            OLED_ShowFloat(12, 24, (float)elapsed / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 24, (u8 *)"s", 12);
        }
        else if(lap_state == LAP_COMPLETE)
        {
            // 显示完成总时间
            OLED_ShowString(0, 24, (u8 *)"F:", 12);
            OLED_ShowFloat(12, 24, (float)lap_total_ms / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 24, (u8 *)"s!", 12);
        }
        else  // LAP_IDLE
        {
            OLED_ShowString(0, 24, (u8 *)"Press ENTER", 12);
        }
    }

    //刷新显示
    OLED_Refresh();
}
```
