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
static float q4_error_filtered = 0.0f;
// Q4 的循迹参数全部独立配置，调试 Q4 时不会改变 Q2 已调好的参数。
#define Q4_BASE_SPEED             300.0f  // 两轮平均目标速度，单位 mm/s
#define Q4_ERROR_FILTER_ALPHA       0.35f // 灰度误差低通系数，增大后弯道响应更快
#define Q4_ERROR_DEAD_ZONE          0.25f // 直线附近的小误差死区
#define Q4_LINE_KP                 50.0f  // Q4 专用循迹比例系数
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

    // Q4 匀速循迹：两轮只做等量反向差速，平均目标速度始终保持 300mm/s。
    // Q4 使用独立的滤波、死区、比例系数和修正限幅，不影响 Q2 的循迹参数。
    if(question == 4)
    {
        float q4_correction;
        float q4_error_cmd;

        // 停车后再次发车时，清除上一次留下的循迹偏差。
        if(target_speed_1 == 0.0f && target_speed_2 == 0.0f)
            q4_error_filtered = 0.0f;

        q4_error_filtered += Q4_ERROR_FILTER_ALPHA * (error - q4_error_filtered);
        q4_error_cmd = q4_error_filtered;
        if(q4_error_cmd > -Q4_ERROR_DEAD_ZONE &&
           q4_error_cmd < Q4_ERROR_DEAD_ZONE)
            q4_error_cmd = 0.0f;

        q4_correction = Q4_LINE_KP * q4_error_cmd;
        if(q4_correction > Q4_CORRECTION_MAX)
            q4_correction = Q4_CORRECTION_MAX;
        if(q4_correction < -Q4_CORRECTION_MAX)
            q4_correction = -Q4_CORRECTION_MAX;

        target_speed_1 = Q4_BASE_SPEED + q4_correction;
        target_speed_2 = Q4_BASE_SPEED - q4_correction;
        return;
    }

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
