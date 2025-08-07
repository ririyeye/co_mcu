extern "C" {
#include "gd32e11x_rcu.h"
}
#include "semaphore.hpp"
#include "syswork.hpp"

struct uart_handle : worknode {
    explicit uart_handle(const struct uart_hard_info* info) : mInfo(info), sem(get_sys_workqueue(), 1, 1) { }

    uint32_t is_init : 1;

    int baud_rate;

    const struct uart_hard_info* mInfo;

    LIST_HEAD(list_work_tx);
    LIST_HEAD(list_work_rx);

    co_mcu::semaphore sem;
};

struct uart_hard_info {
    rcu_periph_enum rcu_uart;
    uint32_t        uart_periph;
    IRQn_Type       uart_irq_num;

    void (*uart_pin_cfg)(const struct uart_hard_info* pinfo);
};

static void uart0_pin_cfg(const struct uart_hard_info* pinfo)
{
    (void)pinfo;
    rcu_periph_clock_enable(RCU_GPIOA);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
}

static const struct uart_hard_info hard_u0 = {
    .rcu_uart     = RCU_USART0,
    .uart_periph  = USART0,
    .uart_irq_num = USART0_IRQn,

    .uart_pin_cfg = uart0_pin_cfg,
};

struct uart_handle uart_0(&hard_u0);

static void uart_hard_init(const struct uart_hard_info* phard, int baud_rate)
{
    NVIC_SetPriority(phard->uart_irq_num, 0x00U);

    rcu_periph_clock_enable(phard->rcu_uart);
    /* USART configure */
    usart_deinit(phard->uart_periph);
    usart_baudrate_set(phard->uart_periph, baud_rate);
    usart_receive_config(phard->uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(phard->uart_periph, USART_TRANSMIT_ENABLE);
    usart_enable(phard->uart_periph);

    if (phard->uart_pin_cfg) {
        phard->uart_pin_cfg(phard);
    }
}

static struct uart_handle* _uart_ext_handle_require(struct uart_handle* puart)
{
    if (puart->is_init) {
        return puart;
    }

    puart->is_init = 1;

    uart_hard_init(puart->mInfo, puart->baud_rate);

    return puart;
}

struct uart_handle* uart_handle_get(int num)
{
    struct uart_handle* phandle = NULL;
    switch (num) {
    case 0:
        phandle = _uart_ext_handle_require(&uart_0);
        break;

    default:
        break;
    }

    return phandle;
}

co_mcu::semaphore& uart_handle_get(struct uart_handle* puart)
{
    return puart->sem;
}
