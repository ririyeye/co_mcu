#pragma once

#include "gd32e11x_rcu.h"

enum {
    HARDWARE_RX = 0,
    HARDWARE_TX = 1,
};

struct hard_dma {
    rcu_periph_enum  dma_rcu;
    IRQn_Type        dma_irq_num;
    dma_channel_enum dma_channel;
    uint32_t         dma_periph;
};
