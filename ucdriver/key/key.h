#ifndef __KEY_H__
#define __KEY_H__

#include "ti_msp_dl_config.h"

extern volatile int enter;
extern volatile int question;
extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;
extern volatile uint8_t key_debounce_cnt;

uint8_t get_key_state(uint32_t key_state);

#endif