#include "huidu.h"
#include "key.h"

volatile uint8_t huidu_value[] = {0, 0, 0, 0, 0};
//直角弯计数变量
volatile uint8_t huidu_straight_count = 0;

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio) {
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio); 
    if((high_bits & gpio) != 0) return 1;
    else return 0;
}

void huidu_get_value()
{
    huidu_value[0] = get_gpio_state(HUIDU_L2_PORT, HUIDU_L2_PIN);
    huidu_value[1] = get_gpio_state(HUIDU_L1_PORT, HUIDU_L1_PIN);
    huidu_value[2] = get_gpio_state(HUIDU_M_PORT, HUIDU_M_PIN);
    huidu_value[3] = get_gpio_state(HUIDU_R1_PORT, HUIDU_R1_PIN);
    huidu_value[4] = get_gpio_state(HUIDU_R2_PORT, HUIDU_R2_PIN);
}
extern volatile float target_speed_1;
extern volatile float target_speed_2;
static float q4_base_speed_cmd = 0.0f;
static float q4_error_filtered = 0.0f;
// Q4 的循迹参数全部独立配置，调试 Q4 时不会改变 Q2 已调好的参数。
#define Q4_BASE_SPEED             300.0f  // 两轮平均目标速度，单位 mm/s
#define Q4_START_SPEED_STEP         0.75f // 每 10ms 增加 0.75mm/s，约 4s 到达巡航速度
#define Q4_ERROR_FILTER_ALPHA       0.30f // 灰度误差低通系数
#define Q4_ERROR_DEAD_ZONE          0.30f // 直线附近的小误差死区
#define Q4_CENTER_LOCK_ERROR        0.55f // 中心传感器覆盖范围内直接保持零差速
#define Q4_STRAIGHT_ERROR_LIMIT     0.50f // 小于该偏差时按直线小力度修正
#define Q4_STRAIGHT_KP             20.0f  // Q4 直线段专用比例系数
#define Q4_TURN_KP                 60.0f  // Q4 弯道增量比例系数
#define Q4_CORRECTION_MAX         100.0f  // Q4 单轮最大差速修正，单位 mm/s

// 比例控制参数
static float huidu_base_speed = 350.0f;  // 可调基准速度 mm/s
static float huidu_Kp = 40.0f;           // 比例系数
static float last_error = 0.0f;          // 上次有效偏差，丢线时保持

void adjust_motor()
{
    uint8_t i;
    float error = 0.0f;
    uint8_t sum = 0;
    static uint8_t lost_count = 0;
    static uint8_t was_lost = 0;        //丢线标志：丢线→恢复时计数一次

    // 权重数组: L2=-2, L1=-1, M=0, R1=+1, R2=+2
    const int8_t weights[5] = {-2, -1, 0, 1, 2};

    huidu_get_value();

    // 计算加权偏差
    for(i = 0; i < 5; i++)
    {
        error += (float)(huidu_value[i] * weights[i]);
        sum += huidu_value[i];
    }

    if(sum > 0 && sum < 5)
    {
        if(was_lost)                      //刚从直角转弯恢复 → 计数一次
        {
            huidu_straight_count++;
            was_lost = 0;
        }
        // 正常循迹: 至少一个传感器检测到线，但不是全白
        lost_count = 0;
        error /= (float)sum;  // error 范围 [-2, +2]
        last_error = error;   // 记录有效偏差
    }
    else if(sum == 0)
    {
        // 丢线(逻辑待实现)
        lost_count++;
        if(lost_count >= 30)
        {
            // 连续丢线 >= 30×10ms=300ms → 进入丢线处理
            was_lost = 1;                 //标记丢线，恢复循迹时计数
           

            return;  // 跳过比例计算和钳位
        }
        else
        {
            // 短暂丢线（传感器间隙），保持上次有效偏差
            error = last_error;
        }
    }
    else  // sum == 5, 全黑
    {
        lost_count = 0;
        // 全黑: 直行
        error = 0.0f;
    }

    // Q4/Q5 软启动完成后匀速循迹：两轮只做等量反向差速，巡航平均速度保持 300mm/s。
    // 两道带球行驶题共用独立的滤波、死区、比例系数和修正限幅，不影响 Q2。
    if(question == 4 || question == 5)
    {
        float q4_correction;
        float q4_correction_limit;
        float q4_error_cmd;
        float q4_error_abs;

        // 停车后再次发车时，同时清除速度斜坡和循迹滤波的历史状态。
        if(target_speed_1 == 0.0f && target_speed_2 == 0.0f)
        {
            q4_base_speed_cmd = 0.0f;
            q4_error_filtered = 0.0f;
        }

        // 只在发车阶段做一次软启动，到达 300mm/s 后全程保持固定平均速度。
        // 降低车体的纵向起步加速度，减小钢球因惯性直接冲向车尾的幅度。
        if(q4_base_speed_cmd < Q4_BASE_SPEED)
        {
            q4_base_speed_cmd += Q4_START_SPEED_STEP;
            if(q4_base_speed_cmd > Q4_BASE_SPEED)
                q4_base_speed_cmd = Q4_BASE_SPEED;
        }

        // 直线中心附近直接锁定零差速，避免数字灰度量在相邻探头之间切换时
        // 产生反向修正；偏差超过中心覆盖范围后再恢复滤波和分段转向。
        if(error > -Q4_CENTER_LOCK_ERROR && error < Q4_CENTER_LOCK_ERROR)
            q4_error_filtered = 0.0f;
        else
            q4_error_filtered += Q4_ERROR_FILTER_ALPHA *
                                 (error - q4_error_filtered);
        q4_error_cmd = q4_error_filtered;
        if(q4_error_cmd > -Q4_ERROR_DEAD_ZONE &&
           q4_error_cmd < Q4_ERROR_DEAD_ZONE)
            q4_error_cmd = 0.0f;

        // 中心附近只用小力度修正，避免车辆越过黑线后反复左右摆动。
        // 偏差超过 0.5 后连续增加弯道修正，最大偏差时仍可达到 ±100mm/s。
        q4_error_abs = (q4_error_cmd >= 0.0f) ? q4_error_cmd : -q4_error_cmd;
        if(q4_error_abs <= Q4_STRAIGHT_ERROR_LIMIT)
        {
            q4_correction = Q4_STRAIGHT_KP * q4_error_cmd;
        }
        else
        {
            q4_correction = Q4_STRAIGHT_KP * Q4_STRAIGHT_ERROR_LIMIT;
            q4_correction += Q4_TURN_KP *
                             (q4_error_abs - Q4_STRAIGHT_ERROR_LIMIT);
            if(q4_error_cmd < 0.0f)
                q4_correction = -q4_correction;
        }

        // 软启动阶段差速不能超过当前基础速度，保证两轮都只向前转。
        q4_correction_limit = Q4_CORRECTION_MAX;
        if(q4_correction_limit > q4_base_speed_cmd)
            q4_correction_limit = q4_base_speed_cmd;
        if(q4_correction > q4_correction_limit)
            q4_correction = q4_correction_limit;
        if(q4_correction < -q4_correction_limit)
            q4_correction = -q4_correction_limit;

        target_speed_1 = q4_base_speed_cmd + q4_correction;
        target_speed_2 = q4_base_speed_cmd - q4_correction;
        return;
    }

    q4_base_speed_cmd = 0.0f;
    q4_error_filtered = 0.0f;

    // 其他题目保持原来的循迹参数。
    float correction = huidu_Kp * error;
    if(correction > 80.0f) correction = 80.0f;
    if(correction < -80.0f) correction = -80.0f;

    target_speed_1 = huidu_base_speed + correction;   // motor 1 = 左轮
    target_speed_2 = huidu_base_speed - correction;   // motor 2 = 右轮

    if(target_speed_1 < 0.0f) target_speed_1 = 0.0f;
    if(target_speed_2 < 0.0f) target_speed_2 = 0.0f;
}
