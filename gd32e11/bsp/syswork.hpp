#pragma once

#include "lock.hpp"
#include "timer.hpp"
uint32_t lock_acquire(void);
void     lock_release(uint32_t magic);
struct cortex_lock {
public:
    void lock() { lock_acquire(); }
    void unlock() { lock_release(0); }
};

co_wq::workqueue<cortex_lock>&         get_sys_workqueue(void);
co_wq::Timer_check_queue<cortex_lock>& get_sys_timer(void);
