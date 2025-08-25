#include "cmsis_compiler.h"
#include "lock.h"
#include <stdint.h>

uint32_t call_counter = 0;
uint32_t gflag        = 0;

static inline void stm32_lock_acquire(void)
{
    uint32_t flag = __get_PRIMASK();
    __disable_irq();
    if (call_counter == 0) {
        gflag = flag;
    }
    call_counter++;
}

static inline void stm32_lock_release(void)
{
    call_counter--;
    if (call_counter == 0) {
        __set_PRIMASK(gflag);
    }
}

uint32_t lock_acquire(void)
{
    stm32_lock_acquire();
    return 0;
}
void lock_release(uint32_t magic)
{
    stm32_lock_release();
    (void)magic;
}
