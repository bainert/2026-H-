#include "motor.h"
#include "ti_msp_dl_config.h"
#include "huidu.h"

//30ms定时器计数变量
extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;

//是否发车 0-未发车 1-发车
extern volatile int enter;

//按键消抖计数器
extern volatile uint8_t key_debounce_cnt;

volatile float speed_1 = 0.0f;
volatile float speed_2 = 0.0f;

//电机占空比
int PWM_1_duty = 0;
int PWM_2_duty = 0;

//目标速度
volatile float target_speed_1 = 0.0f; //单位：mm/s
volatile float target_speed_2 = 0.0f; //单位：mm/s

//上一次误差，当前误差
float last_error_1 = 0.0f;
float current_error_1 = 0.0f;
float last_error_2 = 0.0f;
float current_error_2 = 0.0f;

//电机pid参数
float motor_Kp = 0.6f;
float motor_Ki = 1.9f;

void motor_init(uint8_t motor_id)
{
    //初始化电机引脚
    DL_GPIO_setPins(dc_motor_STBY_PORT, dc_motor_STBY_PIN);
    //开启定时器
    DL_Timer_startCounter(PWMAB_INST);

    //开启30ms定时器
    DL_Timer_startCounter(motor_pid_INST);
    //开启30ms定时器中断
    NVIC_EnableIRQ(motor_pid_INST_INT_IRQN);

    if(motor_id == 1)
    {
        DL_GPIO_setPins(dc_motor_AIN1_PORT, dc_motor_AIN1_PIN);
        DL_GPIO_setPins(dc_motor_AIN2_PORT, dc_motor_AIN2_PIN);
         //设置pwm占空比为0
        DL_Timer_setCaptureCompareValue(PWMA_INST,0,GPIO_PWMA_C1_IDX);
    }
    if(motor_id == 2)
    {
        DL_GPIO_setPins(dc_motor_BIN1_PORT, dc_motor_BIN1_PIN);
        DL_GPIO_setPins(dc_motor_BIN2_PORT, dc_motor_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST,0,GPIO_PWMAB_C1_IDX);
    }
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    //占空比范围0-4000
    if(duty > 4000)
    {
        duty = 4000;
    }

    //设置pwm占空比
    if(motor_id == 1)
    {
        DL_Timer_setCaptureCompareValue(PWMA_INST,duty,GPIO_PWMA_C1_IDX);
    }
    if(motor_id == 2)
    {
        DL_Timer_setCaptureCompareValue(PWMAB_INST,duty,GPIO_PWMAB_C1_IDX);
    }
}

//设置电机方向
//direction: 0 - 正向, 1 - 反向
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1)
    {
        if(direction == 0)
        {
            DL_GPIO_setPins(dc_motor_AIN1_PORT, dc_motor_AIN1_PIN);
            DL_GPIO_clearPins(dc_motor_AIN2_PORT, dc_motor_AIN2_PIN);
        }
        else
        {
            DL_GPIO_clearPins(dc_motor_AIN1_PORT, dc_motor_AIN1_PIN);
            DL_GPIO_setPins(dc_motor_AIN2_PORT, dc_motor_AIN2_PIN);
        }
    }
    if(motor_id == 2)
    {
        if(direction == 0)
        {
            DL_GPIO_setPins(dc_motor_BIN1_PORT, dc_motor_BIN1_PIN);
            DL_GPIO_clearPins(dc_motor_BIN2_PORT, dc_motor_BIN2_PIN);
        }
        else
        {
            DL_GPIO_clearPins(dc_motor_BIN1_PORT, dc_motor_BIN1_PIN);
            DL_GPIO_setPins(dc_motor_BIN2_PORT, dc_motor_BIN2_PIN);
        }
    }
}

//计算电机速度
void calculate_speed(uint8_t motor_id)
{
    if(motor_id == 1)
    {
        speed_1 = (float)counter_1_A / motor_xianshu * PI * motor_wheel_d / 0.03f; //单位：mm/s
        counter_1_A = 0;
    }
    if(motor_id == 2)
    {
        speed_2 = (float)counter_2_A / motor_xianshu * PI * motor_wheel_d / 0.03f;
        counter_2_A = 0;
    }
}

//增量式pid公式：Δu(k) = Kp · [e(k) - e(k-1)] + Ki · e(k)
void motor_PID(uint8_t motor_id)
{
    if(motor_id == 1)
    {
        //计算当前误差
        current_error_1 = target_speed_1 - speed_1;
        //计算pid输出
        PWM_1_duty += (int)(motor_Kp * (current_error_1 - last_error_1) + motor_Ki * current_error_1);
        //钳位占空比到0-4000
        if(PWM_1_duty > 4000) PWM_1_duty = 4000;
        if(PWM_1_duty < 0) PWM_1_duty = 0;
        //设置pwm占空比
        motor_set_duty(1, (uint32_t)PWM_1_duty);
        //更新上一次误差
        last_error_1 = current_error_1;
    }
    if(motor_id == 2)
    {
        current_error_2 = target_speed_2 - speed_2;
        PWM_2_duty += (int)(motor_Kp * (current_error_2 - last_error_2) + motor_Ki * current_error_2);
        if(PWM_2_duty > 4000) PWM_2_duty = 4000;
        if(PWM_2_duty < 0) PWM_2_duty = 0;
        motor_set_duty(2, (uint32_t)PWM_2_duty);
        last_error_2 = current_error_2;
    }
}

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
            break;
        }
        //未发车时，电机停止
        if(enter == 0)
        {
            motor_set_duty(1, 0);
            motor_set_duty(2, 0);
        }

    default:
        break;
   }
}