#include "ball.h"
#include "ti_msp_dl_config.h"
#include "stepper.h"
#include "key.h"
#include "delay.h"

//水管中点（mm）
#define BALL_CENTER_MM      125.0f
//电机角度限位（脉冲）：-24° ~ +80° @16细分 0.1125°/脉冲
#define BALL_MIN_PULSE      (-213)
#define BALL_MAX_PULSE      (711)
//视觉丢帧判定时间（ms）
#define BALL_FRAME_TIMEOUT_MS  500
//增量 PID 参数（单位：脉冲/mm，实车标定）
#define BALL_KP             5.0f
#define BALL_KI             0.1f
#define BALL_KD             30.0f
//单周期输出限幅（脉冲，防猛转）
#define BALL_DU_MAX         100

//小球位置（mm）
volatile uint8_t ball_position_mm = 0;
//丢帧标志
volatile uint8_t ball_frame_lost = 0;
//运行标志
volatile uint8_t ball_running = 0;

//帧解析状态机：0-等帧头 1-已收data 2-等帧尾
static uint8_t frame_state = 0;
static uint8_t frame_data = 0;

//最近一帧时间戳（sys_tick_ms）
static volatile uint32_t last_frame_ms = 0;

//PID 历史误差
static float pid_e1 = 0.0f;
static float pid_e2 = 0.0f;
//当前目标脉冲（绝对位置，相对零点）
static int32_t target_pulse = 0;
//上一次下发的脉冲（目标变化才下发，去重）
static int32_t last_sent_pulse = 0;

extern volatile uint32_t sys_tick_ms;

//视觉帧解析：0xAA data 0x55 三字节帧
void ball_uart_rx(uint8_t byte)
{
    if(frame_state == 0)
    {
        //等帧头
        if(byte == 0xAA) frame_state = 1;
    }
    else if(frame_state == 1)
    {
        //收 data
        frame_data = byte;
        frame_state = 2;
    }
    else
    {
        if(byte == 0x55)
        {
            //完整帧：更新小球位置与时间戳
            ball_position_mm = frame_data;
            last_frame_ms = sys_tick_ms;
            ball_frame_lost = 0;
        }
        else if(byte == 0xAA)
        {
            //可能丢失 data 字节，按新帧头重新解析
            frame_state = 1;
        }
        else
        {
            frame_state = 0;
        }
    }
}

//初始化：使能 UART3 + 触发回零 + 阻塞延时1s等待回零完成
void ball_init(void)
{
    stepper_init();
    stepper_home_start();
    delay_ms(1000);
    //回零后电机位于零点（中位），同步目标与已下发值
    target_pulse = 0;
    last_sent_pulse = 0;
}

//30ms 周期调用（motor_pid 定时器 ISR 中）
void ball_proc(void)
{
    //非 Q3：待命，清理 PID 状态，电机不动
    if(question != 3)
    {
        pid_e1 = 0.0f;
        pid_e2 = 0.0f;
        ball_running = 0;
        return;
    }

    //丢帧检测：超过500ms无新帧
    if(sys_tick_ms - last_frame_ms > BALL_FRAME_TIMEOUT_MS)
    {
        ball_frame_lost = 1;
    }

    if(enter == 1 && ball_frame_lost == 0)
    {
        //RUN：增量式 PID，误差 = 中点 - 当前位置
        float e = BALL_CENTER_MM - (float)ball_position_mm;
        float du = BALL_KP * (e - pid_e1)
                 + BALL_KI * e
                 + BALL_KD * (e - 2.0f * pid_e1 + pid_e2);
        pid_e2 = pid_e1;
        pid_e1 = e;

        //单周期输出限幅，防止猛转
        if(du >  BALL_DU_MAX) du =  BALL_DU_MAX;
        if(du < -BALL_DU_MAX) du = -BALL_DU_MAX;

        target_pulse += (int32_t)du;

        //角度限幅 -24° ~ +80°
        if(target_pulse > BALL_MAX_PULSE) target_pulse = BALL_MAX_PULSE;
        if(target_pulse < BALL_MIN_PULSE) target_pulse = BALL_MIN_PULSE;

        ball_running = 1;
    }
    else if(enter == 1)
    {
        //丢帧中：保持当前角度，不更新目标
        ball_running = 1;
    }
    else
    {
        //enter=0：回中位（零点），清 PID 状态
        target_pulse = 0;
        pid_e1 = 0.0f;
        pid_e2 = 0.0f;
        ball_running = 0;
    }

    //目标变化才下发，避免重复发相同命令
    if(target_pulse != last_sent_pulse)
    {
        last_sent_pulse = target_pulse;
        stepper_move(target_pulse);
    }
}
