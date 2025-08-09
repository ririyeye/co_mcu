extern "C" {
#include "gd32e11x_rcu.h"
}
#include "co_uart.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"
struct UART_FLG {
    uint32_t tx_not_rx  : 1;
    uint32_t rx_timeout : 8;
    uint32_t fast_cb    : 1;
};

struct uart_session : worknode {
    uint8_t*        buff;
    uint32_t        len;
    uint32_t        cur_len; // 当前已传输长度
    struct UART_FLG flg;
};

struct uart_handle : worknode {
    explicit uart_handle(const struct uart_hard_info* info, workqueue& wq) : wq_(wq), mInfo(info), sem(wq_, 1, 1) { }

    struct workqueue& wq_;

    uint32_t is_init : 1;

    int baud_rate = 1152000;

    const struct uart_hard_info* mInfo;

    LIST_HEAD(list_work_tx);
    LIST_HEAD(list_work_rx);

    co_mcu::Semaphore sem;
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

struct uart_handle uart_0(&hard_u0, get_sys_workqueue());

static void uart_hard_init(const struct uart_hard_info* phard, int baud_rate)
{
    nvic_irq_enable(phard->uart_irq_num, 0, 0);

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

co_mcu::Semaphore& uart_handle_get(struct uart_handle* puart)
{
    return puart->sem;
}

static void uart_stop_rx(const struct uart_hard_info* phard)
{
    usart_receiver_timeout_disable(phard->uart_periph);
    usart_interrupt_disable(phard->uart_periph, USART_INT_RBNE);
    usart_interrupt_disable(phard->uart_periph, USART_INT_RT);
    usart_flag_clear(phard->uart_periph, USART_FLAG_RT);
}

static void uart_setup_rx(struct uart_handle* phandle, struct uart_session* pnod);

static int uart_rx_handle(struct uart_handle* phandle, struct uart_session* pnod)
{
    workqueue_add_new_nolock(&phandle->wq_, pnod);

    if (!list_empty(&phandle->list_work_rx)) {

        worknode*     pbase = list_first_entry(&phandle->list_work_rx, worknode, ws_node);
        uart_session* pnxt  = static_cast<uart_session*>(pbase);

        uart_setup_rx(phandle, pnxt);
    }
    return 1;
}

static int _uart_irq_handle(struct uart_handle* phandle)
{
    int retcnt = 0;

    const struct uart_hard_info* phard = phandle->mInfo;
    // receive timeout
    if (RESET != usart_interrupt_flag_get(phard->uart_periph, USART_INT_FLAG_RT)) {
        uart_stop_rx(phard);
        if (list_empty(&phandle->list_work_rx)) {
            return retcnt;
        }
        worknode*     pbase = list_first_entry(&phandle->list_work_rx, worknode, ws_node);
        uart_session* pnod  = static_cast<uart_session*>(pbase);

        retcnt += uart_rx_handle(phandle, pnod);
    }

    /* receive data */
    if (RESET != usart_interrupt_flag_get(phard->uart_periph, USART_INT_FLAG_RBNE)) {
        if (list_empty(&phandle->list_work_rx)) {
            uart_stop_rx(phard);
            return retcnt;
        }

        worknode*     pbase = list_first_entry(&phandle->list_work_rx, worknode, ws_node);
        uart_session* pnod  = static_cast<uart_session*>(pbase);

        pnod->buff[pnod->cur_len++] = usart_data_receive(phard->uart_periph);

        if (pnod->cur_len >= pnod->len) {
            uart_stop_rx(phard);
            retcnt += uart_rx_handle(phandle, pnod);
        }
    }
    /* transmit data */
    if (RESET != usart_interrupt_flag_get(phard->uart_periph, USART_INT_FLAG_TBE)) {
        if (list_empty(&phandle->list_work_tx)) {
            usart_interrupt_disable(phard->uart_periph, USART_INT_TBE);
            return retcnt;
        }

        worknode*     pbase = list_first_entry(&phandle->list_work_tx, worknode, ws_node);
        uart_session* pnod  = static_cast<uart_session*>(pbase);
        usart_data_transmit(phard->uart_periph, pnod->buff[pnod->cur_len++]);
        if (pnod->cur_len >= pnod->len) {
            workqueue_add_new_nolock(&phandle->wq_, pnod);
            retcnt++;
            if (list_empty(&phandle->list_work_tx)) {
                usart_interrupt_disable(phard->uart_periph, USART_INT_TBE);
            }
        }
    }

    return retcnt;
}

static void uart_irq_handle(struct uart_handle* phandle)
{
    uint32_t lk  = lock_acquire();
    int      evt = _uart_irq_handle(phandle);
    lock_release(lk);

    if (evt) {
        workqueue_trig_once(&phandle->wq_);
    }
}

extern "C" void USART0_IRQHandler(void)
{
    uart_irq_handle(&uart_0);
}

static void uart_setup_tx(struct uart_handle* phandle)
{
    usart_interrupt_enable(phandle->mInfo->uart_periph, USART_INT_TBE);
}

static void uart_setup_rx(struct uart_handle* phandle, struct uart_session* pnod)
{
    usart_interrupt_enable(phandle->mInfo->uart_periph, USART_INT_RBNE);

    if (pnod->flg.rx_timeout > 0) {
        usart_receiver_timeout_enable(phandle->mInfo->uart_periph);
        usart_receiver_timeout_threshold_config(phandle->mInfo->uart_periph, pnod->flg.rx_timeout);

        usart_interrupt_enable(phandle->mInfo->uart_periph, USART_INT_RT);
    }
}

static int _uart_ext_transfer_cb(struct uart_handle* handle, uart_session& psess)
{
    uint32_t tx_setup = 0;
    uint32_t rx_setup = 0;

    auto flg = psess.flg;

    // 第一个工作节点 使能 UART
    if (flg.tx_not_rx && list_empty(&handle->list_work_tx)) {
        tx_setup = 1;
    }

    if (!flg.tx_not_rx && list_empty(&handle->list_work_rx)) {
        rx_setup = 1;
    }

    if (flg.tx_not_rx) {
        list_add_tail(&psess.ws_node, &handle->list_work_tx);
    } else {
        list_add_tail(&psess.ws_node, &handle->list_work_rx);
    }

    if (tx_setup) {
        uart_setup_tx(handle);
    }

    if (rx_setup) {
        uart_setup_rx(handle, &psess);
    }

    return 0;
}

static int uart_ext_transfer_cb(struct uart_handle* handle, uart_session& psess)
{
    uint32_t lk  = lock_acquire();
    int      ret = _uart_ext_transfer_cb(handle, psess);
    lock_release(lk);
    return ret;
}

co_mcu::Task<int, co_mcu::Work_Promise<int>> UartManager::uart_transfer(uint8_t* data, size_t len, int tx)
{
    struct tx_uart_session : uart_session {
        explicit tx_uart_session(workqueue& wq) : cpl_inotify(wq, 0, 1) { INIT_LIST_HEAD(&ws_node); }
        co_mcu::Semaphore cpl_inotify;
    };

    tx_uart_session node(handle_->wq_);
    node.buff          = const_cast<uint8_t*>(data);
    node.len           = len;
    node.cur_len       = 0;
    node.flg.tx_not_rx = !!tx; // 仅发送数据，不接收

    node.func = [](struct worknode* pws) {
        tx_uart_session* psess = static_cast<tx_uart_session*>(pws);
        psess->cpl_inotify.release();
    };

    uart_ext_transfer_cb(handle_, node);

    co_await co_mcu::SemReqAwaiter(node.cpl_inotify);

    co_return node.cur_len;
}

co_mcu::Task<bool, co_mcu::Work_Promise<bool>> UartManager::init()
{
    if (handle_) {
        co_return true; // 已经初始化
    }

    auto tmp_handle = uart_handle_get(uart_num_);
    if (!tmp_handle) {
        co_return false; // 获取句柄失败
    }

    co_await co_mcu::SemReqAwaiter(uart_handle_get(tmp_handle));

    handle_ = tmp_handle;

    co_return true;
}

UartManager::~UartManager()
{
    if (handle_) {
        uart_handle_get(handle_).release();
        handle_ = nullptr;
    }
}
