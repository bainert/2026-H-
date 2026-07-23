#include "delay.h"
#include "ti_msp_dl_config.h"

void delay_ms(uint32_t ms)
{
    uint32_t cycles = ms * (CPUCLK_FREQ / 1000);
    delay_cycles(cycles);
}
