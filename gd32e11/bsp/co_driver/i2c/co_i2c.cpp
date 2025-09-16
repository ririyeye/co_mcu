extern "C" {
#include "gd32e11x_gpio.h"
#include "gd32e11x_i2c.h"
#include "gd32e11x_rcu.h"
}
#include "co_i2c.hpp"
#include "co_i2c_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

using namespace co_wq;

// 轻量 I2C 后端：借用 gd32 底层寄存器，使用工作队列回调完成一次会话。
// 相比旧版 i2c_ext（复杂状态机+FreeRTOS workqueue），这里使用简化流程：
//  - 每个 session 在中断驱动下完成；完成后在 wq 上触发回调。

struct i2c_hard_info_s {
    uint32_t        i2c_periph;
    rcu_periph_enum rcu_i2c;
    rcu_periph_enum rcu_gpio;
    uint32_t        scl_port, scl_pin;
    uint32_t        sda_port, sda_pin;
    IRQn_Type       irq_evt;
    IRQn_Type       irq_err;
};

struct i2c_handle : worknode {
    explicit i2c_handle(const struct i2c_hard_info_s& info, workqueue<cortex_lock>& wq)
        : wq_(wq), mInfo(info), sem(wq_, 1, 1)
    {
        INIT_LIST_HEAD(&list_work);
    }

    workqueue<cortex_lock>&       wq_;
    const struct i2c_hard_info_s& mInfo;
    Semaphore<cortex_lock>        sem;
    list_head                     list_work;
    uint8_t                       addr7 { 0x00 };
    uint32_t                      clock_hz { 100000 };
    uint32_t                      is_init : 1;
    uint32_t                      err_sta : 1;
    i2c_sta                       cursta { i2c_stop };
};

static void i2c0_gpio_init(const struct i2c_hard_info_s* p)
{
    rcu_periph_clock_enable(p->rcu_gpio);
    gpio_init(p->scl_port, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, p->scl_pin);
    gpio_init(p->sda_port, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, p->sda_pin);
}

static void i2c_hw_config(const struct i2c_hard_info_s* p, uint8_t addr7, uint32_t clk)
{
    rcu_periph_clock_enable(p->rcu_i2c);
    i2c_deinit(p->i2c_periph);
    i2c_clock_config(p->i2c_periph, clk ? clk : 100000, I2C_DTCY_16_9);
    i2c_mode_addr_config(p->i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, addr7);
    i2c_enable(p->i2c_periph);
}

static const struct i2c_hard_info_s i2c_info_0 = {
    I2C0, RCU_I2C0, RCU_GPIOB, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, I2C0_EV_IRQn, I2C0_ER_IRQn,
};

static i2c_handle i2c0(i2c_info_0, get_sys_workqueue());

static void i2c_start_sequence(const struct i2c_hard_info_s& p)
{
    i2c_start_on_bus(p.i2c_periph);
}
static void i2c_stop_sequence(const struct i2c_hard_info_s& p)
{
    i2c_stop_on_bus(p.i2c_periph);
}

static void i2c_enable_irqs(const struct i2c_hard_info_s& p)
{
    nvic_irq_enable(p.irq_evt, 4U, 0);
    nvic_irq_enable(p.irq_err, 4U, 0);
    i2c_interrupt_enable(p.i2c_periph, I2C_INT_ERR);
    i2c_interrupt_enable(p.i2c_periph, I2C_INT_BUF);
    i2c_interrupt_enable(p.i2c_periph, I2C_INT_EV);
}

static void i2c_hw_init(const struct i2c_hard_info_s* p)
{
    i2c0_gpio_init(p);
    i2c_hw_config(p, 0, 100000);
    i2c_enable_irqs(*p);
}

struct i2c_handle* i2c_handle_get_init(int num)
{
    switch (num) {
    case 0:
        if (!i2c0.is_init) {
            i2c_hw_init(&i2c0.mInfo);
            i2c0.is_init = 1;
        }
        return &i2c0;
    default:
        return nullptr;
    }
}

// 完成回调设置：将节点加入工作队列以唤醒 Awaiter
static void i2c_complete_node(i2c_handle* handle, i2c_session* node)
{
    node->len = node->trans_len;
    handle->wq_.add_new_nolock(*static_cast<worknode*>(node));
    handle->wq_.trig_once();
}

// 结束一次会话并启动下一个
static void i2c_end_proc(i2c_handle* handle, i2c_session* node)
{
    const auto& p = handle->mInfo;

    if (node->ctrl_bit & i2c_end_i2c) {
        handle->cursta = i2c_stop;
        i2c_stop_sequence(p);
        i2c_ack_config(p.i2c_periph, I2C_ACK_ENABLE);
        i2c_ackpos_config(p.i2c_periph, I2C_ACKPOS_CURRENT);
    }

    if (handle->err_sta) {
        handle->cursta = i2c_stop;
        i2c_stop_sequence(p);
        // 软复位总线（简版：重新配置硬件）
        i2c_hw_config(&p, handle->addr7, handle->clock_hz);
        handle->err_sta = 0;
    }

    list_del(&node->ws_node);
    i2c_complete_node(handle, node);

    // 还有剩余工作节点 使能 i2c
    if (!list_empty(&handle->list_work)) {
        worknode* pbase = list_first_entry(&handle->list_work, worknode, ws_node);
        auto*     next  = static_cast<i2c_session*>(pbase);
        // 启动下一个会话
        // 若需要起始，发起 START；否则根据当前状态继续或 RESTART
        if (next->ctrl_bit & i2c_start_i2c) {
            i2c_start_sequence(p);
            handle->cursta = i2c_set_start;
        } else {
            if (handle->cursta == i2c_send_tx_data && (next->ctrl_bit & i2c_tx_not_rx)) {
                // 继续 TX，由中断在 TBE 推进
            } else {
                // 需要 RESTART
                i2c_start_sequence(p);
                handle->cursta = i2c_set_start;
            }
        }
    }
}

// 根据当前状态与中断标志推进状态机
static void i2c_ev_irq_critical(i2c_handle* handle)
{
    const auto& p = handle->mInfo;
    if (list_empty(&handle->list_work)) {
        // 清理可能的多余标志
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_BTC)) {
            // no-op
        }
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_TBE)) {
            // no-op
        }
        return;
    }

    worknode* pbase = list_first_entry(&handle->list_work, worknode, ws_node);
    auto*     node  = static_cast<i2c_session*>(pbase);
    switch (handle->cursta) {
    case i2c_stop:
        break;
    case i2c_set_start:
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_SBSEND)) {
            if (node->ctrl_bit & i2c_tx_not_rx) {
                handle->cursta = i2c_send_tx_addr;
                i2c_master_addressing(p.i2c_periph, handle->addr7, I2C_TRANSMITTER);
                i2c_ack_config(p.i2c_periph, I2C_ACK_ENABLE);
            } else {
                handle->cursta = i2c_send_rx_addr;
                i2c_master_addressing(p.i2c_periph, handle->addr7, I2C_RECEIVER);
                i2c_ack_config(p.i2c_periph, I2C_ACK_ENABLE);
            }
        }
        break;
    case i2c_send_tx_addr:
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_ADDSEND)) {
            i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_ADDSEND);
        }
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_TBE)) {
            i2c_data_transmit(p.i2c_periph, node->buf[node->trans_len++]);
            handle->cursta = i2c_send_tx_data;
        }
        break;
    case i2c_send_tx_data:
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_TBE)) {
            if (node->trans_len >= node->trans_max) {
                i2c_end_proc(handle, node);
            } else {
                i2c_data_transmit(p.i2c_periph, node->buf[node->trans_len++]);
                handle->cursta = i2c_send_tx_data;
            }
        }
        break;
    case i2c_send_rx_addr:
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_ADDSEND)) {
            i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_ADDSEND);
            uint32_t left = node->trans_max - node->trans_len;
            if (left <= 1) {
                i2c_ack_config(p.i2c_periph, I2C_ACK_DISABLE);
            }
            handle->cursta = i2c_send_rx_data;
        }
        break;
    case i2c_send_rx_data:
        if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_RBNE)) {
            uint8_t rx                   = i2c_data_receive(p.i2c_periph);
            node->buf[node->trans_len++] = rx;
            uint32_t left                = node->trans_max - node->trans_len;
            if (left <= 1) {
                i2c_ack_config(p.i2c_periph, I2C_ACK_DISABLE);
            }
            if (node->trans_max == node->trans_len) {
                handle->err_sta = 0;
                i2c_end_proc(handle, node);
            }
        }
        break;
    default:
        break;
    }
}

static void i2c_err_irq_critical(i2c_handle* handle)
{
    const auto& p = handle->mInfo;
    // 清错误标志
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_AERR))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_AERR);
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_SMBALT))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_SMBALT);
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_SMBTO))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_SMBTO);
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_OUERR))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_OUERR);
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_LOSTARB))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_LOSTARB);
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_BERR))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_BERR);
    if (i2c_interrupt_flag_get(p.i2c_periph, I2C_INT_FLAG_PECERR))
        i2c_interrupt_flag_clear(p.i2c_periph, I2C_INT_FLAG_PECERR);

    // 结束当前节点
    if (!list_empty(&handle->list_work)) {
        worknode* pbase = list_first_entry(&handle->list_work, worknode, ws_node);
        auto*     node  = static_cast<i2c_session*>(pbase);
        handle->err_sta = 1;
        i2c_end_proc(handle, node);
    }
}

static void i2c_setup_transfer(i2c_handle* phandle, i2c_session* node)
{
    const auto& p = phandle->mInfo;
    // 重新配置当前地址/时钟
    i2c_hw_config(&p, phandle->addr7, phandle->clock_hz);

    if ((node->ctrl_bit & i2c_start_i2c) == 0) {
        if (phandle->cursta == i2c_stop) {
            // 无效的继续请求：直接完成并处理下一个
            list_del(&node->ws_node);
            i2c_complete_node(phandle, node);
            if (!list_empty(&phandle->list_work)) {
                worknode* pbase = list_first_entry(&phandle->list_work, worknode, ws_node);
                auto*     next  = static_cast<i2c_session*>(pbase);
                i2c_setup_transfer(phandle, next);
            }
            return;
        }
    }

    if (node->ctrl_bit & i2c_start_i2c) {
        i2c_start_sequence(p);
        phandle->cursta = i2c_set_start;
    } else {
        if (phandle->cursta == i2c_send_tx_data) {
            if (node->ctrl_bit & i2c_tx_not_rx) {
                // 继续发送：由 TBE 中断推进
            } else {
                // 重启进入接收
                i2c_start_sequence(p);
                phandle->cursta = i2c_set_start;
            }
        }
    }
}

static void i2c_enqueue_session_nolock(i2c_handle& ph, i2c_session& ps)
{
    bool empty = list_empty(&ph.list_work);
    list_add_tail(&ps.ws_node, &ph.list_work);
    if (empty) {
        i2c_setup_transfer(&ph, &ps);
    }
}

void i2c_enqueue_session(i2c_handle& ph, i2c_session& ps)
{
    uint32_t lk = lock_acquire();
    i2c_enqueue_session_nolock(ph, ps);
    lock_release(lk);
    ph.wq_.trig_once();
}

// ===== 提供给模板的访问函数 =====
co_wq::workqueue<cortex_lock>& i2c_handle_wq(i2c_handle* h)
{
    return h->wq_;
}
co_wq::Semaphore<cortex_lock>& i2c_handle_sem(i2c_handle* h)
{
    return h->sem;
}
void i2c_handle_set_addr(i2c_handle* h, uint8_t addr7)
{
    h->addr7 = addr7 & 0x7F;
}
uint8_t i2c_handle_get_addr(i2c_handle* h)
{
    return h->addr7;
}
void i2c_handle_set_clock(i2c_handle* h, uint32_t hz)
{
    if (hz)
        h->clock_hz = hz;
}
uint32_t i2c_handle_get_clock(i2c_handle* h)
{
    return h->clock_hz;
}

void I2cManager::release()
{
    if (handle_) {
        handle_->sem.release();
        handle_ = nullptr;
    }
}
I2cManager::~I2cManager()
{
    release();
}

extern "C" void I2C0_EV_IRQHandler(void)
{
    uint32_t lk = lock_acquire();
    i2c_ev_irq_critical(&i2c0);
    lock_release(lk);
    i2c0.wq_.trig_once();
}

extern "C" void I2C0_ER_IRQHandler(void)
{
    uint32_t lk = lock_acquire();
    i2c_err_irq_critical(&i2c0);
    lock_release(lk);
    i2c0.wq_.trig_once();
}
