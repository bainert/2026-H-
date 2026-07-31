### Task 3: 鎺ョ嚎灞傦紙uart.c / motor.c / main.c锛?
**Files:**
- Modify: `ucdriver/uart/uart.c:20-36`锛圲ART0 ISR 鏀硅瑙夎В鏋愶級
- Modify: `ucdriver/motor/motor.c:176-217`锛圦3 灞忚斀寰抗 + 璋?ball_proc锛?- Modify: `main.c:33-38`锛坕nclude锛夈€乣main.c:120-153`锛堝垵濮嬪寲锛夈€乣main.c:70-117`锛坥led_proc Q3 鍒嗘敮锛?
**Interfaces:**
- Consumes: Task 2 鐨?`ball_uart_rx/ball_proc/ball_position_mm/ball_frame_lost/ball_running`銆乣UART_0_INST`锛坱i_msp_dl_config.h锛?- Produces: 鏃犳柊鎺ュ彛

- [ ] **Step 1: 淇敼 `ucdriver/uart/uart.c` 鈥?UART0 ISR 鐢卞洖鏄炬敼涓鸿瑙夊抚瑙ｆ瀽**

`uart.c` 寮€澶村姞 `#include "ball.h"`锛堝湪 `#include "uart.h"` 涔嬪悗锛夛細

```c
#include "uart.h"
#include "ball.h"
#include "ti_msp_dl_config.h"
```

灏?`UART_0_INST_IRQHandler` 鏁翠釜鏇挎崲涓猴細

```c
//涓插彛鎺ユ敹涓柇澶勭悊鍑芥暟锛堣瑙夋暟鎹級
void UART_0_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
    case DL_UART_IIDX_RX:
    {
        //灏嗘帴鏀跺瓧鑺傞€佸叆瑙嗚甯цВ鏋?        uint8_t data = DL_UART_receiveDataBlocking(UART_0_INST);
        ball_uart_rx(data);
        break;
    }

    default:
        break;
    }
}
```

- [ ] **Step 2: 淇敼 `ucdriver/motor/motor.c` 鈥?Q3 灞忚斀灏忚溅椹卞姩 + 璋冪敤 ball_proc**

`motor.c` 寮€澶村姞 `#include "ball.h"`锛堝湪 `#include "key.h"` 涔嬪悗锛夛細

```c
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "huidu.h"
#include "key.h"
#include "ball.h"
```

灏?`motor_pid_INST_IRQHandler` 涓?`case DL_TIMER_IIDX_LOAD:` 鍒嗘敮鏇挎崲涓猴細

```c
    case DL_TIMER_IIDX_LOAD:
        //鍙戣溅
        if(enter == 1)
        {
            if(question == 3)
            {
                //Q3 闈欐€佹憜鐞冿細灏忚溅鐢垫満鍋滐紝涓嶅惊杩癸紝浜ょ粰鎽嗙悆鎺у埗
                motor_set_duty(1, 0);
                motor_set_duty(2, 0);
            }
            else
            {
                //鐏板害鎺у埗
                adjust_motor();
                //閲岀▼绱锛堜娇鐢ㄧ紪鐮佸櫒璁℃暟鍊硷紝绮惧害鏇撮珮锛?                odometer += ((float)counter_1_A + (float)counter_2_A) * 0.5f
                            / motor_xianshu * PI * motor_wheel_d;
                //璁＄畻鐢垫満閫熷害
                calculate_speed(1);
                calculate_speed(2);
                //pid鎺у埗
                motor_PID(1);
                motor_PID(2);

                /* 绗簩闂? 璺戝湀璁℃椂 + 鑷姩鍋滄 */
                if(question == 2)
                {
                    if(lap_state == LAP_IDLE || lap_state == LAP_COMPLETE)
                    {
                        // 浠庡緟鍛芥垨瀹屾垚鐘舵€?-> 寮€濮嬭窇鍦堬紝閲岀▼娓呴浂
                        odometer = 0.0f;
                        lap_state = LAP_RUNNING;
                        lap_start_ms = sys_tick_ms;
                    }
                    else if(lap_state == LAP_RUNNING)
                    {
                        /* 閲岀▼璁″埌杈?.20m鏃跺仠杞?*/
                        if(odometer >= 5000.0f)
                        {
                            lap_state = LAP_COMPLETE;
                            lap_total_ms = sys_tick_ms - lap_start_ms;
                            enter = 0;  // 鍋滄鐢垫満
                        }
                    }
                }
                else
                {
                    // 闈濹2棰樺彿鏃堕噸缃湀鐘舵€侊紝闃叉鍒囨崲棰樺彿鍥炴潵鏃剁姸鎬佹贩涔?                    lap_state = LAP_IDLE;
                }
            }
        }
        //鏈彂杞︽椂锛堝寘鎷琎2瀹屾垚鍚庣殑鐘舵€侊級锛岀數鏈哄仠姝?        if(enter == 0)
        {
            motor_set_duty(1, 0);
            motor_set_duty(2, 0);
        }

        //鎽嗙悆鎺у埗 30ms 鍛ㄦ湡璋冪敤锛圦3 鏃剁敓鏁堬紝闈?Q3 鑷姩寰呭懡锛?        ball_proc();

    default:
        break;
```

- [ ] **Step 3: 淇敼 `main.c` 鈥?include + 鍒濆鍖?+ OLED Q3 鏄剧ず**

`main.c` 鐨?include 鍖猴紙`#include "motor.h"` 鍚庯級鍔狅細

```c
#include "ball.h"
```

extern 鍖猴紙`#include "key.h"` 闄勮繎锛宍extern volatile uint32_t sys_tick_ms;` 鍚庨潰锛夊姞锛?
```c
// Q3 鎽嗙悆鎺у埗
extern volatile uint8_t ball_position_mm;
extern volatile uint8_t ball_frame_lost;
extern volatile uint8_t ball_running;
```

`main()` 涓?`OLED_Clear();` 涔嬪悗鍔狅紙鏀惧湪鎸夐敭/缂栫爜鍣ㄤ腑鏂娇鑳戒箣鍚庛€乵otor_init 涔嬪墠锛夛細

```c
    //Q3 鎽嗙悆鎺у埗鍒濆鍖栵紙鍚杩涚數鏈哄洖闆讹紝闃诲绾?s锛?    ball_init();
```

`oled_proc()` 涓涓夎锛堥噷绋嬭锛変笌绗洓琛岋紙Q2 璁℃椂锛夋浛鎹负锛?
```c
    //绗笁琛岋細Q3 鏄剧ず灏忕悆浣嶇疆锛屽惁鍒欓噷绋嬭
    if(question == 3)
    {
        OLED_ShowString(0, 24, (u8 *)"P:", 12);
        if(ball_frame_lost)
            OLED_ShowString(12, 24, (u8 *)"LOST", 12);
        else
            OLED_ShowNum(12, 24, (u32)ball_position_mm, 3, 12);
        OLED_ShowString(36, 24, (u8 *)"mm", 12);
    }
    else
    {
        OLED_ShowString(0, 24, (u8 *)"D:", 12);
        OLED_ShowFloat(12, 24, odometer / 1000.0f, 1, 2, 12);
        OLED_ShowString(34, 24, (u8 *)"m", 12);
    }

    //绗洓琛岋細Q2 璁℃椂 / Q3 鎽嗙悆鐘舵€?/ 鍏朵粬娓呯┖
    if(question == 2)
    {
        if(lap_state == LAP_RUNNING)
        {
            uint32_t elapsed = sys_tick_ms - lap_start_ms;
            OLED_ShowString(0, 36, (u8 *)"T:", 12);
            OLED_ShowFloat(12, 36, (float)elapsed / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 36, (u8 *)"s", 12);
        }
        else if(lap_state == LAP_COMPLETE)
        {
            OLED_ShowString(0, 36, (u8 *)"F:", 12);
            OLED_ShowFloat(12, 36, (float)lap_total_ms / 1000.0f, 2, 2, 12);
            OLED_ShowString(36, 36, (u8 *)"s!   ", 12);
        }
        else  // LAP_IDLE
        {
            OLED_ShowString(0, 36, (u8 *)"Press ENTER", 12);
        }
    }
    else if(question == 3)
    {
        OLED_ShowString(0, 36, (u8 *)"BAL:", 12);
        if(ball_running)
            OLED_ShowString(36, 36, (u8 *)"RUN ", 12);
        else
            OLED_ShowString(36, 36, (u8 *)"STOP", 12);
    }
    else
    {
        // 闈濹2/Q3鏃舵竻闄ょ鍥涜
        OLED_ShowString(0, 36, (u8 *)"            ", 12);
    }
```

- [ ] **Step 4: 鑷**

妫€鏌ワ細`uart.c` 寮曠敤 `ball_uart_rx` 闇€ `ball.h`锛圱ask 2锛夛紱`motor.c` 寮曠敤 `ball_proc` 闇€ `ball.h`锛決3 鍒嗘敮淇濈暀浜?`enter==0` 鏃剁殑鍋滆溅閫昏緫锛沗ball_proc()` 鏃犳潯浠惰皟鐢紙闈?Q3 鍐呴儴 return锛屽紑閿€鍙拷鐣ワ級锛沗main.c` 鐨?`ball_init()` 鏀惧湪涓柇浣胯兘涔嬪悗锛堝洖闆剁殑 1s 闃诲鏈熼棿 OLED 宸插垵濮嬪寲锛屾樉绀烘甯革級銆?
- [ ] **Step 5: Commit**

```bash
git add ucdriver/uart/uart.c ucdriver/motor/motor.c main.c
git commit -m "feat: Q3 鎺ョ嚎灞?鈥?UART0 瑙嗚瑙ｆ瀽/鐢垫満Q3鍋滆浆/OLED鏄剧ず灏忕悆浣嶇疆"
```

---

