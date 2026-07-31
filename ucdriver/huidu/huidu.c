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
static float q4_target_speed_1_cmd = 0.0f;
static float q4_target_speed_2_cmd = 0.0f;

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

    // Q4 需要边行驶边稳球，降低基础速度可以明显减小车体纵向加速度对摆杆的冲击。
    // AB 段 1.5m 以 260mm/s 行驶约 5.8s，仍满足 8s 限时；其他题目保持原速度。
    // Q4 提高巡航速度保证 AB 段时间，同时用斜坡限制起步加速度。
    // 每个 10ms 周期最多增加 10mm/s，约 0.32s 到达 320mm/s。
    if(question != 4 || (target_speed_1 == 0.0f && target_speed_2 == 0.0f))
    {
        q4_base_speed_cmd = 0.0f;
        q4_error_filtered = 0.0f;
        q4_target_speed_1_cmd = 0.0f;
        q4_target_speed_2_cmd = 0.0f;
    }
    if(question == 4)
    {
        // 直线起步采用 2.5mm/s 每周期的慢斜坡，约 1.2s 达到 300mm/s。
        if(q4_base_speed_cmd < 300.0f)
        {
            q4_base_speed_cmd += 2.5f;
            if(q4_base_speed_cmd > 300.0f) q4_base_speed_cmd = 300.0f;
        }
    }

    float base_speed = (question == 4) ? q4_base_speed_cmd : huidu_base_speed;
    if(question == 4)
    {
        q4_error_filtered += 0.25f * (error - q4_error_filtered);
        error = q4_error_filtered;
        if(error > -0.35f && error < 0.35f)
            error = 0.0f;
    }
    // Q4 进入弯道时降低基础速度，减小横向修正和车体加速度对摆球的冲击。
    if(question == 4)
    {
        float turn_abs = (error >= 0.0f) ? error : -error;
        if(turn_abs > 1.0f)
            base_speed -= 100.0f;
        else if(turn_abs > 0.5f)
            base_speed -= 60.0f;
        if(base_speed < 200.0f) base_speed = 200.0f;
    }
    float line_kp = (question == 4) ? 20.0f : huidu_Kp;
    float correction = line_kp * error;
    float correction_limit = (question == 4) ? 40.0f : 80.0f;
    if(correction > correction_limit) correction = correction_limit;
    if(correction < -correction_limit) correction = -correction_limit;

    // 比例控制: 左轮减速 = 右轮加速
    target_speed_1 = base_speed + huidu_Kp * error;   // motor 1 = 左轮
    target_speed_2 = base_speed - huidu_Kp * error;   // motor 2 = 右轮

    // 钳位到有效范围
    target_speed_1 = base_speed + correction;
    target_speed_2 = base_speed - correction;

    if(question == 4)
    {
        // 限制左右轮目标速度的单周期变化，避免循迹量化跳变造成忽快忽慢。
        float delta_1 = target_speed_1 - q4_target_speed_1_cmd;
        float delta_2 = target_speed_2 - q4_target_speed_2_cmd;
        if(delta_1 > 5.0f) delta_1 = 5.0f;
        if(delta_1 < -5.0f) delta_1 = -5.0f;
        if(delta_2 > 5.0f) delta_2 = 5.0f;
        if(delta_2 < -5.0f) delta_2 = -5.0f;
        q4_target_speed_1_cmd += delta_1;
        q4_target_speed_2_cmd += delta_2;
        target_speed_1 = q4_target_speed_1_cmd;
        target_speed_2 = q4_target_speed_2_cmd;
    }

    if(target_speed_1 < 0.0f) target_speed_1 = 0.0f;
    if(target_speed_2 < 0.0f) target_speed_2 = 0.0f;
}
