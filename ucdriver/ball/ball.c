#include "ball.h"
#include "ti_msp_dl_config.h"
#include "stepper.h"
#include "key.h"
#include "delay.h"

//水管中点（mm）
#define BALL_CENTER_MM      125.0f
//电机角度限位（脉冲）：-24° ~ +70° @16细分 0.1125°/脉冲
#define BALL_MIN_PULSE      (-213)
#define BALL_MAX_PULSE      (622)
//视觉丢帧判定时间（ms）
#define BALL_FRAME_TIMEOUT_MS  500
//误差死区（mm）：视觉分辨率 1mm，|e|<=1mm 视为到位
//注意：死区必须加在误差 e 上，不能加在增量 du 上（原因见 ball_proc 注释）
#define BALL_ERR_DEADZONE_MM   1.0f
//位置式 PID 参数（输出 = 目标角度脉冲，单位：脉冲/mm）
//  KP：球偏 1mm → 摆杆倾斜 KP 脉冲（1.5 脉冲≈0.17°，50mm 偏差≈8.4°→加速度 1.4m/s²）
//  KI：修摩擦静差（小即可，过大引起低频振荡）
//  KD：阻尼，抑制过冲振荡（对 1mm 量化敏感，若高速抖动就调小）
#define BALL_KP             1.10f
#define BALL_KI             0.01f
#define BALL_KD             10.5f
//误差积分限幅（脉冲，防 windup）：±200 ≈ ±22°，远小于机械限位
#define BALL_INTEGRAL_MAX   200.0f
//调参指引：回中慢 → 加大 KP（注意过冲，配 KD 压制）；稳态振铃 → 减小 KP/加大 KD
//过冲不回弹 → KD 太小或 KP 太大；到不了位（静差）→ 加大 KI

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

//上一帧误差（微分项用）
static float pid_e1 = 0.0f;
//误差积分（抗摩擦静差，限幅防 windup）
static float ball_integral = 0.0f;
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

//10ms 周期调用（motor_pid 定时器 ISR 中）
void ball_proc(void)
{
    //非 Q3：待命，清理 PID 状态，电机不动
    if(question != 3)
    {
        pid_e1 = 0.0f;
        ball_integral = 0.0f;
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
        //RUN：位置式 PID，输出直接为目标角度（脉冲），
        //球一偏离中心立即得到大角度 → 加速度瞬间建立，响应快
        float e = BALL_CENTER_MM - (float)ball_position_mm;
        //误差死区：|e|<=1mm 视为到位（视觉分辨率 1mm，更小无法分辨）
        //【关键】死区若加在增量 du 上：稳态时 KP/KD 项恒为 0，只剩 KI*e=0.1*e，
        //任何 <180mm 的稳态误差产生的 du 都 < 18 脉冲被丢弃 → 永不回中，KP 调多大都无效
        if(e > -BALL_ERR_DEADZONE_MM && e < BALL_ERR_DEADZONE_MM) e = 0.0f;

        //积分项（限幅防 windup：球必须能停在偏离中心的位置，积分负责补足
        //"摆杆保持固定角度"所需的持续输出——增量式 PID 做不到这点）
        ball_integral += e;
        if(ball_integral >  BALL_INTEGRAL_MAX) ball_integral =  BALL_INTEGRAL_MAX;
        if(ball_integral < -BALL_INTEGRAL_MAX) ball_integral = -BALL_INTEGRAL_MAX;

        //位置式输出：target_pulse = KP*e + KI*Σe + KD*de/dt
        //（de/dt 用相邻两帧误差差分近似，一阶差分即足够）
        target_pulse = (int32_t)(BALL_KP * e
                               + BALL_KI * ball_integral
                               + BALL_KD * (e - pid_e1));
        pid_e1 = e;

        //角度限幅 -24° ~ +80°（即输出限幅，位置式无需额外的单周期限幅）
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
        ball_integral = 0.0f;
        ball_running = 0;
    }

    //目标变化才下发，避免重复发相同命令
    if(target_pulse != last_sent_pulse)
    {
        last_sent_pulse = target_pulse;
        stepper_move(target_pulse);
    }
}
