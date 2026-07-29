#ifndef __KEY_H__
#define __KEY_H__

#include "ti_msp_dl_config.h"

extern volatile int enter;
extern volatile int question;
extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;
extern volatile uint8_t key_debounce_cnt;

typedef enum {
    LAP_IDLE = 0,      // 待命
    LAP_RUNNING,       // 行驶中
    LAP_COMPLETE       // 完成一圈回到A点
} lap_state_t;

extern volatile lap_state_t lap_state;
extern volatile uint32_t lap_start_ms;
extern volatile uint32_t lap_total_ms;

uint8_t get_key_state(uint32_t key_state);

#endif