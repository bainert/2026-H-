### Task 1: stepper 鍗忚灞傦紙EMM 甯ф墦鍖?+ UART3 搴旂瓟锛?
**Files:**
- Create: `ucdriver/stepper/stepper.h`
- Create: `ucdriver/stepper/stepper.c`

**Interfaces:**
- Consumes: `uart.h` 鐨?`uart_send_char(UART_Regs*, uint8_t)`銆乣UART_3_INST`/`UART_3_INST_INT_IRQN`锛坱i_msp_dl_config.h锛夈€乣DL_UART_getPendingInterrupt`/`DL_UART_receiveDataBlocking`/`DL_UART_IIDX_RX`
- Produces: `void stepper_init(void)`銆乣void stepper_home_start(void)`銆乣void stepper_move(int32_t pulses)`銆乣volatile uint8_t stepper_ack`锛堟渶杩戝簲绛斿瓧鑺傦紝渚涜皟璇曪級

- [ ] **Step 1: 鍒涘缓 `ucdriver/stepper/stepper.h`**

```c
#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <stdint.h>

// 鏈€杩戜竴娆＄數鏈哄簲绛斿瓧鑺傦紙02=鍛戒护姝ｇ‘ 12=宸插湪闆剁偣 E2=鍙傛暟閿?EE=鏍煎紡閿?9F=鍔ㄤ綔瀹屾垚锛?extern volatile uint8_t stepper_ack;

// 鍒濆鍖栵細浣胯兘 UART3 鎺ユ敹涓柇
void stepper_init(void);
// 瑙﹀彂鍗曞湀灏辫繎鍥為浂锛?A 鍛戒护锛?void stepper_home_start(void);
// 浣嶇疆妯″紡缁濆杩愬姩锛歱ulses 涓虹浉瀵归浂鐐圭殑鏈夌鍙疯剦鍐诧紙姝?CW 璐?CCW锛?void stepper_move(int32_t pulses);

#endif
```

- [ ] **Step 2: 鍒涘缓 `ucdriver/stepper/stepper.c`**

```c
#include "stepper.h"
#include "ti_msp_dl_config.h"
#include "uart.h"

//鐢垫満鍦板潃锛堝嚭鍘傞粯璁?锛屼笉鍙负0骞挎挱锛?#define STEPPER_ADDR        0x01
//鏍￠獙鐮侊紙鍑哄巶榛樿0x6B锛?#define STEPPER_CRC         0x6B

//浣嶇疆妯″紡杩愬姩鍙傛暟锛堝彲璋冿級
#define STEPPER_SPEED_RPM   500     //閫熷害 0-3000 RPM
#define STEPPER_ACC         100     //鍔犻€熷害妗ｄ綅 0-255锛岃秺澶у姞閫熻秺蹇?
//鏈€杩戜竴娆＄數鏈哄簲绛斿瓧鑺?volatile uint8_t stepper_ack = 0;

//鍙戦€佷竴甯у懡浠?static void stepper_send_frame(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    for(i = 0; i < len; i++)
    {
        uart_send_char(UART_3_INST, frame[i]);
    }
}

//鍒濆鍖栵細浣胯兘 UART3 鎺ユ敹涓柇
void stepper_init(void)
{
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

//瑙﹀彂鍗曞湀灏辫繎鍥為浂锛?1 9A 00 00 6B锛堟ā寮?0鍗曞湀灏辫繎锛屽悓姝?0绔嬪嵆锛?void stepper_home_start(void)
{
    static const uint8_t frame[] = {0x01, 0x9A, 0x00, 0x00, 0x6B};
    stepper_send_frame(frame, sizeof(frame));
}

//浣嶇疆妯″紡缁濆杩愬姩
//甯? 01 FD 鏂瑰悜 閫熷害(2B) 鍔犻€熷害 鑴夊啿(4B) 杩愬姩妯″紡01(缁濆) 鍚屾00 6B
void stepper_move(int32_t pulses)
{
    uint8_t frame[13];
    uint8_t dir;
    uint32_t abs_pulses;

    //姝ｈ礋鑴夊啿鎷嗗垎锛氭=CW(00) 璐?CCW(01)锛岃剦鍐叉暟鍙栫粷瀵瑰€?    if(pulses >= 0)
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
    frame[1] = 0xFD;                                  //浣嶇疆妯″紡鍔熻兘鐮?    frame[2] = dir;
    frame[3] = (STEPPER_SPEED_RPM >> 8) & 0xFF;       //閫熷害楂樺瓧鑺?    frame[4] = STEPPER_SPEED_RPM & 0xFF;              //閫熷害浣庡瓧鑺?    frame[5] = STEPPER_ACC;                           //鍔犻€熷害妗ｄ綅
    frame[6]  = (abs_pulses >> 24) & 0xFF;            //鑴夊啿鏁?4B 澶х
    frame[7]  = (abs_pulses >> 16) & 0xFF;
    frame[8]  = (abs_pulses >> 8) & 0xFF;
    frame[9]  = abs_pulses & 0xFF;
    frame[10] = 0x01;                                 //杩愬姩妯″紡锛氱浉瀵瑰潗鏍囬浂鐐圭粷瀵逛綅缃?    frame[11] = 0x00;                                 //鍚屾鏍囧織锛氱珛鍗虫墽琛?    frame[12] = STEPPER_CRC;

    stepper_send_frame(frame, sizeof(frame));
}

//UART3 鎺ユ敹涓柇锛氳В鏋愮數鏈哄簲绛旓紙鍙彇搴旂瓟瀛楄妭锛屼緵涓婂眰/璋冭瘯浣跨敤锛?void UART_3_INST_IRQHandler(void)
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
```

- [ ] **Step 3: 鑷**

妫€鏌ワ細`stepper_send_frame` 浣跨敤 `UART_3_INST`锛坱i_msp_dl_config.h 宸插畾涔夛紝瑙?`Debug/ti_msp_dl_config.h:169`锛夛紱`uart_send_char` 澹版槑浜?uart.h锛涘抚闀垮害 13 瀛楄妭 = Addr+Code+鏂瑰悜+閫熷害2+鍔犻€熷害+鑴夊啿4+杩愬姩妯″紡+鍚屾+鏍￠獙锛汧D 甯у瓧娈典笌鎵嬪唽 5.3.12 涓€鑷淬€?
- [ ] **Step 4: Commit**

```bash
git add ucdriver/stepper/stepper.h ucdriver/stepper/stepper.c
git commit -m "feat: 鏂板 stepper 姝ヨ繘鐢垫満鍗忚灞傦紙EMM 鍥為浂/浣嶇疆妯″紡/UART3搴旂瓟锛?
```

---

