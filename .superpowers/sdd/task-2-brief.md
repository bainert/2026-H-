### Task 2: ball 鎺у埗灞傦紙瑙嗚瑙ｆ瀽 + 澧為噺 PID + 鐘舵€佹満锛?
**Files:**
- Create: `ucdriver/ball/ball.h`
- Create: `ucdriver/ball/ball.c`

**Interfaces:**
- Consumes: Task 1 鐨?`stepper_init/stepper_home_start/stepper_move`銆乣key.h` 鐨?`enter/question`銆乣delay.h` 鐨?`delay_ms`銆乣sys_tick_ms`锛坢ain.c 涓畾涔夊苟閫掑锛夈€乣UART_0_INST`锛坱i_msp_dl_config.h锛?- Produces: `void ball_init(void)`銆乣void ball_proc(void)`銆乣void ball_uart_rx(uint8_t byte)`銆乣volatile uint8_t ball_position_mm`銆乣volatile uint8_t ball_frame_lost`銆乣volatile uint8_t ball_running`

- [ ] **Step 1: 鍒涘缓 `ucdriver/ball/ball.h`**

```c
#ifndef __BALL_H__
#define __BALL_H__

#include <stdint.h>

//灏忕悆浣嶇疆锛坢m锛?~250锛夛紝鐢?UART0 瑙嗚甯ф洿鏂?extern volatile uint8_t ball_position_mm;
//瑙嗚涓㈠抚鏍囧織锛?-姝ｅ父 1-瓒呰繃500ms鏃犳柊甯?extern volatile uint8_t ball_frame_lost;
//鎽嗙悆鎺у埗杩愯鏍囧織锛歈3 涓?enter=1 鏃朵负 1
extern volatile uint8_t ball_running;

//鍒濆鍖栵細浣胯兘 UART3 + 瑙﹀彂鍥為浂 + 闃诲寤舵椂1s
void ball_init(void);
//30ms 鍛ㄦ湡璋冪敤锛堝湪 motor_pid 瀹氭椂鍣?ISR 涓級
void ball_proc(void);
//瑙嗚涓插彛瀛楄妭鍥炶皟锛圲ART0 ISR 涓€愬瓧鑺傝皟鐢級
void ball_uart_rx(uint8_t byte);

#endif
```

- [ ] **Step 2: 鍒涘缓 `ucdriver/ball/ball.c`**

```c
#include "ball.h"
#include "ti_msp_dl_config.h"
#include "stepper.h"
#include "key.h"
#include "delay.h"

//姘寸涓偣锛坢m锛?#define BALL_CENTER_MM      125.0f
//鐢垫満瑙掑害闄愪綅锛堣剦鍐诧級锛?24掳 ~ +80掳 @16缁嗗垎 0.1125掳/鑴夊啿
#define BALL_MIN_PULSE      (-213)
#define BALL_MAX_PULSE      (711)
//瑙嗚涓㈠抚鍒ゅ畾鏃堕棿锛坢s锛?#define BALL_FRAME_TIMEOUT_MS  500
//澧為噺 PID 鍙傛暟锛堝崟浣嶏細鑴夊啿/mm锛屽疄杞︽爣瀹氾級
#define BALL_KP             5.0f
#define BALL_KI             0.1f
#define BALL_KD             30.0f
//鍗曞懆鏈熻緭鍑洪檺骞咃紙鑴夊啿锛岄槻鐚涜浆锛?#define BALL_DU_MAX         100

//灏忕悆浣嶇疆锛坢m锛?volatile uint8_t ball_position_mm = 0;
//涓㈠抚鏍囧織
volatile uint8_t ball_frame_lost = 0;
//杩愯鏍囧織
volatile uint8_t ball_running = 0;

//甯цВ鏋愮姸鎬佹満锛?-绛夊抚澶?1-宸叉敹data 2-绛夊抚灏?static uint8_t frame_state = 0;
static uint8_t frame_data = 0;

//鏈€杩戜竴甯ф椂闂存埑锛坰ys_tick_ms锛?static volatile uint32_t last_frame_ms = 0;

//PID 鍘嗗彶璇樊
static float pid_e1 = 0.0f;
static float pid_e2 = 0.0f;
//褰撳墠鐩爣鑴夊啿锛堢粷瀵逛綅缃紝鐩稿闆剁偣锛?static int32_t target_pulse = 0;
//涓婁竴娆′笅鍙戠殑鑴夊啿锛堢洰鏍囧彉鍖栨墠涓嬪彂锛屽幓閲嶏級
static int32_t last_sent_pulse = 0;

extern volatile uint32_t sys_tick_ms;

//瑙嗚甯цВ鏋愶細0xAA data 0x55 涓夊瓧鑺傚抚
void ball_uart_rx(uint8_t byte)
{
    if(frame_state == 0)
    {
        //绛夊抚澶?        if(byte == 0xAA) frame_state = 1;
    }
    else if(frame_state == 1)
    {
        //鏀?data
        frame_data = byte;
        frame_state = 2;
    }
    else
    {
        if(byte == 0x55)
        {
            //瀹屾暣甯э細鏇存柊灏忕悆浣嶇疆涓庢椂闂存埑
            ball_position_mm = frame_data;
            last_frame_ms = sys_tick_ms;
            ball_frame_lost = 0;
        }
        else if(byte == 0xAA)
        {
            //鍙兘涓㈠け data 瀛楄妭锛屾寜鏂板抚澶撮噸鏂拌В鏋?            frame_state = 1;
        }
        else
        {
            frame_state = 0;
        }
    }
}

//鍒濆鍖栵細浣胯兘 UART3 + 瑙﹀彂鍥為浂 + 闃诲寤舵椂1s绛夊緟鍥為浂瀹屾垚
void ball_init(void)
{
    stepper_init();
    stepper_home_start();
    delay_ms(1000);
    //鍥為浂鍚庣數鏈轰綅浜庨浂鐐癸紙涓綅锛夛紝鍚屾鐩爣涓庡凡涓嬪彂鍊?    target_pulse = 0;
    last_sent_pulse = 0;
}

//30ms 鍛ㄦ湡璋冪敤锛坢otor_pid 瀹氭椂鍣?ISR 涓級
void ball_proc(void)
{
    //闈?Q3锛氬緟鍛斤紝娓呯悊 PID 鐘舵€侊紝鐢垫満涓嶅姩浣?    if(question != 3)
    {
        pid_e1 = 0.0f;
        pid_e2 = 0.0f;
        ball_running = 0;
        return;
    }

    //涓㈠抚妫€娴嬶細瓒呰繃500ms鏃犳柊甯?    if(sys_tick_ms - last_frame_ms > BALL_FRAME_TIMEOUT_MS)
    {
        ball_frame_lost = 1;
    }

    if(enter == 1 && ball_frame_lost == 0)
    {
        //RUN锛氬閲忓紡 PID锛岃宸?= 涓偣 - 褰撳墠浣嶇疆
        float e = BALL_CENTER_MM - (float)ball_position_mm;
        float du = BALL_KP * (e - pid_e1)
                 + BALL_KI * e
                 + BALL_KD * (e - 2.0f * pid_e1 + pid_e2);
        pid_e2 = pid_e1;
        pid_e1 = e;

        //鍗曞懆鏈熻緭鍑洪檺骞咃紝闃叉鐚涜浆
        if(du >  BALL_DU_MAX) du =  BALL_DU_MAX;
        if(du < -BALL_DU_MAX) du = -BALL_DU_MAX;

        target_pulse += (int32_t)du;

        //瑙掑害闄愬箙 -24掳 ~ +80掳
        if(target_pulse > BALL_MAX_PULSE) target_pulse = BALL_MAX_PULSE;
        if(target_pulse < BALL_MIN_PULSE) target_pulse = BALL_MIN_PULSE;

        ball_running = 1;
    }
    else if(enter == 1)
    {
        //涓㈠抚涓細淇濇寔褰撳墠瑙掑害锛屼笉鏇存柊鐩爣
        ball_running = 1;
    }
    else
    {
        //enter=0锛氬洖涓綅锛堥浂鐐癸級锛屾竻 PID 鐘舵€?        target_pulse = 0;
        pid_e1 = 0.0f;
        pid_e2 = 0.0f;
        ball_running = 0;
    }

    //鐩爣鍙樺寲鎵嶄笅鍙戯紝閬垮厤閲嶅鍙戠浉鍚屽懡浠?    if(target_pulse != last_sent_pulse)
    {
        last_sent_pulse = target_pulse;
        stepper_move(target_pulse);
    }
}
```

- [ ] **Step 3: 鑷**

妫€鏌ワ細`ball_uart_rx` 鐘舵€佹満瑕嗙洊 0xAA/0x55 閲嶅叆鎯呭舰锛沗question/enter` 鏉ヨ嚜 key.h extern锛沗sys_tick_ms` 鍦?main.c 瀹氫箟锛汸ID 鍘嗗彶鍙湪 RUN 鍒嗘敮鏇存柊锛堜涪甯?寰呭懡鏃朵笉鍔紝闃叉绉彂锛夛紱闄愬箙甯搁噺涓庤鏍间竴鑷达紱涓㈠抚鏃?`last_sent_pulse` 涓嶅彉 鈫?涓嶉噸澶嶅彂鍛戒护銆?
- [ ] **Step 4: Commit**

```bash
git add ucdriver/ball/ball.h ucdriver/ball/ball.c
git commit -m "feat: 鏂板 ball 鎽嗙悆鎺у埗灞傦紙瑙嗚瑙ｆ瀽/涓㈠抚妫€娴?澧為噺PID/鐘舵€佹満锛?
```

---

