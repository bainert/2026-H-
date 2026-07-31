#include "stepper.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

//电机地址（出厂默认1），不可为0广播
#define STEPPER_ADDR        0x01
//校验码（出厂默认0x6B）
#define STEPPER_CRC         0x6B

//位置模式运动参数（可调）
#define STEPPER_SPEED_RPM   30     //速度 0-3000 RPM
#define STEPPER_ACC         0     //加速度档位 0-255，越大加速越快
//最近一次电机应答字节
volatile uint8_t stepper_ack = 0;

//发送一帧命令
static void stepper_send_frame(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        uart_send_char(UART_3_INST, frame[i]);
    }
}

//初始化：使能 UART3 接收中断
void stepper_init(void)
{
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

//触发单圈就近回零（01 9A 00 00 6B）（模式0单圈就近，同步0立即）
void stepper_home_start(void)
{
    static const uint8_t frame[] = {0x01, 0x9A, 0x00, 0x00, 0x6B};
    stepper_send_frame(frame, sizeof(frame));
}

//位置模式绝对运动
//帧: 01 FD 方向 速度(2B) 加速度 脉冲(4B) 运动模式01(绝对) 同步00 6B
void stepper_move(int32_t pulses)
{
    uint8_t frame[13];
    uint8_t dir;
    uint32_t abs_pulses;

    //正负脉冲拆分：正=CW(00) 负=CCW(01)，脉冲数取绝对值
    if(pulses >= 0)
    {
        dir = 0x00;
        abs_pulses = (uint32_t)pulses;
    }
    else
    {
        dir = 0x01;
        abs_pulses = (uint32_t)(-pulses);
    }

    frame[0] = STEPPER_ADDR;
    frame[1] = 0xFD;                                  //位置模式功能码
    frame[2] = dir;
    frame[3] = (STEPPER_SPEED_RPM >> 8) & 0xFF;       //速度高字节
    frame[4] = STEPPER_SPEED_RPM & 0xFF;              //速度低字节
    frame[5] = STEPPER_ACC;                           //加速度档位
    frame[6]  = (abs_pulses >> 24) & 0xFF;            //脉冲数 4B 大端
    frame[7]  = (abs_pulses >> 16) & 0xFF;
    frame[8]  = (abs_pulses >> 8) & 0xFF;
    frame[9]  = abs_pulses & 0xFF;
    frame[10] = 0x01;                                 //运动模式：相对坐标零点绝对位置
    frame[11] = 0x00;                                 //同步标志：立即执行
    frame[12] = STEPPER_CRC;

    stepper_send_frame(frame, sizeof(frame));
}

//UART3 接收中断：解析电机应答（只取应答字节，供上层/调试使用）
void UART_3_INST_IRQHandler(void)
{
    switch(DL_UART_getPendingInterrupt(UART_3_INST))
    {
    case DL_UART_IIDX_RX:
        stepper_ack = DL_UART_receiveDataBlocking(UART_3_INST);
        break;
    default:
        break;
    }
}
