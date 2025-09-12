#include "gd32e11x.h"
#include "syswork.hpp"
#include <cstdint>
#include <stdlib.h>

#define highfreq_multiplier 1000

uint64_t total_tick = 0;

uint64_t get_current_highfreq_tick_us(void)
{
#if 1
    uint64_t load = SysTick->LOAD;
    uint64_t val  = SysTick->VAL;

    return total_tick * highfreq_multiplier + (load - val) * highfreq_multiplier / load;
#else
    return total_tick * highfreq_multiplier;
#endif
}

__weak_symbol void usr_tick() { }

extern "C" void SysTick_Handler(void)
{
    /* Clear overflow flag */
    SysTick->CTRL;
    total_tick++;
    get_sys_timer().tick_update();
    usr_tick();
}

void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        /* capture error */
        while (1) { }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

#include <_syslist.h>
#include <reent.h>
#include <sys/time.h>
#include <sys/times.h>

extern "C" int _gettimeofday_r(struct _reent* ptr, struct timeval* ptimeval, void* ptimezone)
{
    (void)ptimezone;
    uint64_t _tick = get_current_highfreq_tick_us();

    ptimeval->tv_sec  = _tick / 1'000'000;
    ptimeval->tv_usec = _tick % 1'000'000;
    ptr->_errno       = 0;
    return 0;
}
