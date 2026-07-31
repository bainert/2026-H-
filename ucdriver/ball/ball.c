#include "ball.h"
#include "ti_msp_dl_config.h"
#include "stepper.h"
#include "key.h"
#include "delay.h"

//目标点（mm）：中心O / ±5cm（视觉坐标系 0~250，中心 125）
#define BALL_TARGET_CENTER_MM  125.0f
#define BALL_TARGET_PLUS5_MM   175.0f
#define BALL_TARGET_MINUS5_MM  75.0f
#define BALL_POSITION_MAX_MM   250U
#define BALL_VISION_REVERSE_AXIS  1
//到达判定：|位置-目标|≤8mm 且持续 30×10ms=300ms 视为到位
//（8mm 留 2mm 裕量给 ±1cm 赛题要求，300ms 抗视觉抖动误判）
#define BALL_ARRIVE_ERR_MM     8.0f
#define BALL_ARRIVE_CNT_MAX    30
//行程段比例增益倍率：实测跑得慢（>4s）时调大，如 1.5/2.0；回中过冲则调小
//（只乘在 KP 上，不污染原参数，稳态精度由 KI 积分保证）
//电机角度限位（脉冲）：-24° ~ +70° @16细分 0.1125°/脉冲
#define BALL_MIN_PULSE      (-213)
#define BALL_MAX_PULSE      (622)
//视觉丢帧判定时间（ms）
#define BALL_FRAME_TIMEOUT_MS  500
//误差死区（mm）：视觉分辨率 1mm，|e|<=3mm 视为到位
//注意：死区必须加在误差 e 上，不能加在增量 du 上（原因见 ball_proc 注释）
#define BALL_ERR_DEADZONE_MM   1.0f
//位置式 PID 参数（输出 = 目标角度脉冲，单位：脉冲/mm）
//  KP：球偏 1mm → 摆杆倾斜 KP 脉冲（1.5 脉冲≈0.17°，50mm 偏差≈8.4°→加速度 1.4m/s²）
//  KI：修摩擦静差（小即可，过大引起低频振荡）
//  KD：阻尼，抑制过冲振荡（对 1mm 量化敏感，若高速抖动就调小）
//微分项单次限幅（脉冲）：限制视觉帧位置跳变产生的反向冲击
//目标角度斜率限幅（脉冲/10ms）：给钢球留出减速时间
#define BALL_PULSE_SLEW_MAX      32
#define BALL_PULSE_SLEW_MAX_Q4   28
#define BALL_PULSE_SLEW_MAX_Q3_REVERSE  12
#define BALL_REVERSE_TARGET_STEP_MM  1.0f
#define BALL_ENDPOINT_BOOST_ERR_MM   25.0f
#define BALL_ENDPOINT_BOOST_PULSE    24
// Q3 折返时限制“虚拟目标”的移动速度，避免摆杆由正向角度瞬间反向。
// 每 10ms 移动 2mm，+5cm 到 -5cm 的目标过渡约 0.5s。
//误差积分限幅（脉冲，防 windup）：±200 ≈ ±22°，远小于机械限位
#define BALL_INTEGRAL_MAX   300.0f
// 仅在接近目标点时启用积分，避免运动阶段积分累积造成连续过冲
#define BALL_INTEGRAL_ENABLE_ERR_MM  30.0f

// 不同题目的摆球动态不同：Q4 行驶时车体扰动更大，需要更强阻尼。
typedef struct
{
    float kp;
    float ki;
    float kd;
    float d_term_max;
} ball_pid_param_t;

static const ball_pid_param_t ball_pid_q3 = {0.95f, 0.025f, 26.0f, 60.0f};
// Q4 行驶时车体扰动较大：减小积分避免累积过冲，增大阻尼并限制微分峰值。
static const ball_pid_param_t ball_pid_q4 = {0.80f, 0.015f, 44.0f, 52.0f};
static const ball_pid_param_t ball_pid_q5 = {0.95f, 0.015f, 28.0f, 60.0f};

static const ball_pid_param_t *ball_get_pid_param(void)
{
    switch(question)
    {
    case 3: return &ball_pid_q3;
    case 4: return &ball_pid_q4;
    case 5: return &ball_pid_q5;
    default: return &ball_pid_q5;
    }
}
//调参指引：回中慢 → 加大 KP（注意过冲，配 KD 压制）；稳态振铃 → 减小 KP/加大 KD
//过冲不回弹 → KD 太小或 KP 太大；到不了位（静差）→ 加大 KI

//小球位置（mm）
volatile uint8_t ball_position_mm = 0;
//丢帧标志
volatile uint8_t ball_frame_lost = 0;
//运行标志
volatile uint8_t ball_running = 0;
//摆球运动阶段：0-待命(中心) 1-前往+5cm 2-折返-5cm 3-完成保持（仅Q3使用，Q4/Q5恒为中心）
volatile uint8_t ball_phase = 0;
//当前目标点（mm，由状态机更新，替代原固定 BALL_CENTER_MM）
static float ball_target_mm = BALL_TARGET_CENTER_MM;
// PID 实际使用的目标。ball_target_mm 保存最终目标，折返阶段由本变量平滑追踪。
static float ball_control_target_mm = BALL_TARGET_CENTER_MM;
//到达判定计数器（10ms 周期累加，>=30 判到达）
static uint8_t arrive_cnt = 0;
//目标切换防冲击标志：置位后 PID 第一拍微分项清零
static uint8_t target_changed = 0;

//帧解析状态机：0-等帧头 1-已收data 2-等帧尾
static uint8_t frame_state = 0;
static uint8_t frame_data = 0;

//最近一帧时间戳（sys_tick_ms）
static volatile uint32_t last_frame_ms = 0;

//上一帧误差（微分项用）
static float pid_e1 = 0.0f;
// 视觉约 20Hz，而控制中断为 100Hz；微分项保持在两帧之间使用，
// 避免出现“打一拍、松四拍”的反复冲击。
// 上一次参与微分计算的视觉帧时间戳；重复控制周期不重复计算微分
static uint32_t pid_frame_ms = 0;
static float pid_d_term = 0.0f;
//误差积分（抗摩擦静差，限幅防 windup）
static float ball_integral = 0.0f;
//当前目标脉冲（绝对位置，相对零点）
static int32_t target_pulse = 0;
//上一次下发的脉冲（目标变化才下发，去重）
static int32_t last_sent_pulse = 0;

extern volatile uint32_t sys_tick_ms;

// 限制绝对位置指令的单周期变化量，current/desired 单位均为脉冲
static int32_t ball_slew_pulse(int32_t current, int32_t desired)
{
    int32_t slew_max;

    if(question == 4)
        slew_max = BALL_PULSE_SLEW_MAX_Q4;
    else if(question == 3 && ball_phase == 2)
        slew_max = BALL_PULSE_SLEW_MAX_Q3_REVERSE;
    else
        slew_max = BALL_PULSE_SLEW_MAX;

    if(desired > current + slew_max)
        return current + slew_max;
    if(desired < current - slew_max)
        return current - slew_max;
    return desired;
}

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
            uint8_t position_mm = frame_data;

            if(position_mm > BALL_POSITION_MAX_MM)
                position_mm = BALL_POSITION_MAX_MM;
#if BALL_VISION_REVERSE_AXIS
            position_mm = (uint8_t)BALL_POSITION_MAX_MM - position_mm;
#endif
            ball_position_mm = position_mm;
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
    ball_target_mm = BALL_TARGET_CENTER_MM;
    ball_control_target_mm = BALL_TARGET_CENTER_MM;
    pid_frame_ms = 0;
    pid_d_term = 0.0f;
}

//10ms 周期调用（motor_pid 定时器 ISR 中）
void ball_proc(void)
{
    //非 Q3/Q4/Q5：待命，清理 PID 状态，电机不动
    if(question != 3 && question != 4 && question != 5)
    {
        pid_e1 = 0.0f;
        pid_d_term = 0.0f;
        ball_integral = 0.0f;
        ball_running = 0;
        ball_phase = 0;
        ball_target_mm = BALL_TARGET_CENTER_MM;
        ball_control_target_mm = BALL_TARGET_CENTER_MM;
        arrive_cnt = 0;
        target_changed = 0;
        return;
    }

    const ball_pid_param_t *pid = ball_get_pid_param();

    //丢帧检测：超过500ms无新帧
    if(sys_tick_ms - last_frame_ms > BALL_FRAME_TIMEOUT_MS)
    {
        ball_frame_lost = 1;
    }

    if(enter == 1 && ball_frame_lost == 0)
    {
        if(question == 4 || question == 5)
        {
            //Q4 带球行驶 / Q5 测试：目标恒为中心 O（125mm），闭环一直把球拉回中心。
            //Q5 时车体由 motor.c 停止，仅测试摆球在 O 点的稳定性。
            if(ball_target_mm != BALL_TARGET_CENTER_MM || ball_phase != 0)
            {
                ball_target_mm = BALL_TARGET_CENTER_MM;
                ball_control_target_mm = BALL_TARGET_CENTER_MM;
                ball_phase = 0;
                arrive_cnt = 0;
                target_changed = 1;
            }
        }
        else
        {
        //目标点状态机（Q3）：中心O → +5cm → 折返 → -5cm 保持
        switch(ball_phase)
        {
        case 0:
            //S0→S1：enter=1 启动，目标改为 +5cm（175mm）
            ball_target_mm = BALL_TARGET_PLUS5_MM;
            ball_control_target_mm = BALL_TARGET_PLUS5_MM;
            ball_phase = 1;
            arrive_cnt = 0;
            target_changed = 1;   //防冲击：第一拍微分清零
            break;
        case 1:
            //S1→S2：到达 +5cm（|误差|≤8mm 持续 300ms）后折返 -5cm
            if(ball_position_mm >= BALL_TARGET_PLUS5_MM - BALL_ARRIVE_ERR_MM
            && ball_position_mm <= BALL_TARGET_PLUS5_MM + BALL_ARRIVE_ERR_MM)
            {
                if(++arrive_cnt >= BALL_ARRIVE_CNT_MAX)
                {
                    ball_target_mm = BALL_TARGET_MINUS5_MM;
                    // 折返先经过中心点刹车，再继续向 -5cm 运动。
                    ball_control_target_mm = BALL_TARGET_CENTER_MM;
                    ball_phase = 2;
                    ball_control_target_mm = BALL_TARGET_CENTER_MM;
                    arrive_cnt = 0;
                    target_changed = 1;
                }
            }
            else
            {
                //一次出窗就清零，必须连续 300ms 才算到达
                arrive_cnt = 0;
            }
            break;
        case 2:
            //S2→S3：到达 -5cm 后进入保持阶段
            if(ball_position_mm >= BALL_TARGET_MINUS5_MM - BALL_ARRIVE_ERR_MM
            && ball_position_mm <= BALL_TARGET_MINUS5_MM + BALL_ARRIVE_ERR_MM)
            {
                if(++arrive_cnt >= BALL_ARRIVE_CNT_MAX)
                {
                    ball_phase = 3;
                    arrive_cnt = 0;
                }
            }
            else
            {
                arrive_cnt = 0;
            }
            break;
        case 3:
        default:
            //S3：保持 -5cm 不动（闭环继续，直到 enter=0）
            break;
        }
        }

        //RUN：位置式 PID，输出直接为目标角度（脉冲），
        //球一偏离目标点立即得到大角度 → 加速度瞬间建立，响应快
        // 折返阶段只把 PID 目标缓慢移向 -5cm。最终到位判定仍然使用
        // ball_target_mm，避免“目标平滑”导致提前结束测试。
        if(ball_phase == 2)
        {
            if(ball_control_target_mm > ball_target_mm)
            {
                ball_control_target_mm -= BALL_REVERSE_TARGET_STEP_MM;
                if(ball_control_target_mm < ball_target_mm)
                    ball_control_target_mm = ball_target_mm;
            }
            else if(ball_control_target_mm < ball_target_mm)
            {
                ball_control_target_mm += BALL_REVERSE_TARGET_STEP_MM;
                if(ball_control_target_mm > ball_target_mm)
                    ball_control_target_mm = ball_target_mm;
            }
        }
        else
        {
            ball_control_target_mm = ball_target_mm;
        }

        float e = ball_control_target_mm - (float)ball_position_mm;
        //误差死区：|e|<=3mm 视为到位，过滤视觉量化噪声
        //【关键】死区若加在增量 du 上：稳态时 KP/KD 项恒为 0，只剩 KI*e=0.1*e，
        //任何 <180mm 的稳态误差产生的 du 都 < 18 脉冲被丢弃 → 永不回中，KP 调多大都无效
        if(e > -BALL_ERR_DEADZONE_MM && e < BALL_ERR_DEADZONE_MM) e = 0.0f;

        //【防冲击】目标刚切换的第一拍：微分记忆对齐当前误差，
        //KD*(e-e1)=0，避免 e 突变 50mm 时微分项输出 ~525 脉冲被打满限幅
        if(target_changed)
        {
            target_changed = 0;
            pid_e1 = e;
            pid_d_term = 0.0f;
            pid_frame_ms = last_frame_ms;
            // 从一个目标切换到另一个目标时，旧方向的积分会抵消新的制动动作
            ball_integral = 0.0f;
        }

        //位置式输出：target_pulse = KP*e + KI*Σe + KD*de/dt
        //（de/dt 用相邻两帧误差差分近似，一阶差分即足够；KP 可乘行程倍率）
        // UART0 视觉帧约 20Hz，而控制 ISR 为 100Hz；只在新帧到达时计算微分，
        // 避免同一帧数据被重复计算造成控制量抖动。
        if(last_frame_ms != pid_frame_ms)
        {
            //视觉帧约 20Hz，而控制中断为 100Hz；积分只在新帧到达时更新，
            //并且只在接近目标点时启用，避免同一帧误差重复积分和运动阶段 windup。
            if(e > -BALL_INTEGRAL_ENABLE_ERR_MM && e < BALL_INTEGRAL_ENABLE_ERR_MM)
            {
                ball_integral += e;
                if(ball_integral >  BALL_INTEGRAL_MAX) ball_integral =  BALL_INTEGRAL_MAX;
                if(ball_integral < -BALL_INTEGRAL_MAX) ball_integral = -BALL_INTEGRAL_MAX;
            }
            else
            {
                ball_integral = 0.0f;
            }

            // 进入死区时清除微分记忆，避免视觉量化造成一次反向脉冲。
            if(e == 0.0f)
            {
                pid_d_term = 0.0f;
                pid_e1 = 0.0f;
            }
            else
            {
                float control_kd = pid->kd;
                if(question == 3)
                {
                    if(ball_phase == 1)
                        control_kd = 18.0f;
                    else if(ball_phase == 2)
                        control_kd = 42.0f;
                }
                float raw_d_term = control_kd * (e - pid_e1);
                if(raw_d_term > pid->d_term_max) raw_d_term = pid->d_term_max;
                if(raw_d_term < -pid->d_term_max) raw_d_term = -pid->d_term_max;
                // 一阶低通，抑制视觉量化和车体振动造成的反向尖峰。
                pid_d_term = 0.55f * pid_d_term + 0.45f * raw_d_term;
                pid_e1 = e;
            }
            pid_frame_ms = last_frame_ms;
        }

        float control_kp = pid->kp;
        if(question == 3)
        {
            if(ball_phase == 1)
                control_kp = 1.20f;
            else if(ball_phase == 2)
                control_kp = 0.65f;
        }

        int32_t desired_pulse = (int32_t)(control_kp * e
                                        + pid->ki * ball_integral
                                        + pid_d_term);

        // 接近 Q3 端点时补偿静摩擦，避免小球停在 3~4cm 后无法继续靠近目标。
        // 只在误差方向与目标一致时补偿，进入目标附近或反向时不会继续加力。
        if(ball_phase == 1 && e > BALL_ERR_DEADZONE_MM
        && e < BALL_ENDPOINT_BOOST_ERR_MM)
        {
            desired_pulse += BALL_ENDPOINT_BOOST_PULSE;
        }
        //角度限幅 -24° ~ +80°（即输出限幅，位置式无需额外的单周期限幅）
        if(desired_pulse > BALL_MAX_PULSE) desired_pulse = BALL_MAX_PULSE;
        if(desired_pulse > BALL_MAX_PULSE) desired_pulse = BALL_MAX_PULSE;
        if(desired_pulse < BALL_MIN_PULSE) desired_pulse = BALL_MIN_PULSE;

        // 绝对位置命令也要限制变化速度，防止目标切换时摆杆瞬间反向
        target_pulse = ball_slew_pulse(target_pulse, desired_pulse);

        ball_running = 1;
    }
    else if(enter == 1)
    {
        //丢帧中：保持当前角度，不更新目标（到达计数也不走，见 case 分支条件）
        ball_running = 1;
    }
    else
    {
        //enter=0：回中位（零点），清 PID 状态，状态机复位待命
        // 松开 ENTER 回中时也采用斜率限制，避免摆杆瞬间回零造成钢球二次过冲
        target_pulse = ball_slew_pulse(target_pulse, 0);
        pid_e1 = 0.0f;
        pid_d_term = 0.0f;
        ball_integral = 0.0f;
        ball_running = 0;
        ball_phase = 0;
        ball_target_mm = BALL_TARGET_CENTER_MM;
        ball_control_target_mm = BALL_TARGET_CENTER_MM;
        arrive_cnt = 0;
        target_changed = 0;
    }

    //目标变化才下发，避免重复发相同命令
    if(target_pulse != last_sent_pulse)
    {
        last_sent_pulse = target_pulse;
        stepper_move(target_pulse);
    }
}
