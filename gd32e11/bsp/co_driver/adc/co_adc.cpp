extern "C" {
#include "gd32e11x_adc.h"
#include "gd32e11x_gpio.h"
#include "gd32e11x_rcu.h"
}
#include "co_adc_internal.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

using namespace co_wq;

struct adc_hard_info {
    rcu_periph_enum rcu_adc;
    IRQn_Type       adc_irq_num;
    uint32_t        adc_periph;

    struct channel {
        uint32_t        gpio_periph;
        uint32_t        gpio_pin;
        rcu_periph_enum gpio_rcu;
        uint8_t         adc_channel;
    } chan[18];
    uint32_t avail_map { 0 };
    uint8_t  idx { 0 };

    void (*spi_pin_cfg)(const struct adc_hard_info* pinfo) { nullptr }; // 兼容接口名（未使用）
};

struct adc_handle : worknode {
    explicit adc_handle(const struct adc_hard_info& info, workqueue<cortex_lock>& wq)
        : wq_(wq), mInfo(info), sem(wq_, 1, 1)
    {
        INIT_LIST_HEAD(&list_work);
    }

    workqueue<cortex_lock>& wq_;
    uint32_t                is_init : 1;

    const struct adc_hard_info& mInfo;

    list_head              list_work;
    Semaphore<cortex_lock> sem; // 作为“独占”句柄互斥
};

static void adc0_pin_cfg(const struct adc_hard_info* pinfo)
{
    (void)pinfo;
    // 内部通道无需 GPIO；若后续开放外部通道，可在此配置
}

static const struct adc_hard_info adc_info_0 = {
    RCU_ADC0,
    ADC0_1_IRQn,
    ADC0,
    // 仅开放内部温感/Vref
    {
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     {},
     { 0, 0, (rcu_periph_enum)0, ADC_CHANNEL_16 }, // 16: temp
        { 0, 0, (rcu_periph_enum)0, ADC_CHANNEL_17 }, // 17: vref
    },
    (1u << 16) | (1u << 17),
    0,
    adc0_pin_cfg,
};

static const struct adc_hard_info adc_info_1 = {
    RCU_ADC1,
    ADC0_1_IRQn,
    ADC1,
    {
      { GPIOA, GPIO_PIN_0, RCU_GPIOA, ADC_CHANNEL_0 },
      { GPIOA, GPIO_PIN_1, RCU_GPIOA, ADC_CHANNEL_1 },
      },
    (1u << 0) | (1u << 1),
    1,
    nullptr,
};

static void adc_chan_gpio_cfg(const struct adc_hard_info* padc, int chan_idx)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(padc->chan[chan_idx].gpio_rcu);
    gpio_init(padc->chan[chan_idx].gpio_periph, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, padc->chan[chan_idx].gpio_pin);
}

static void adc_nvic_config(const struct adc_hard_info* padc)
{
    nvic_irq_enable(padc->adc_irq_num, 4U, 0);
}

static void adc_periph_init(const struct adc_hard_info* phd)
{
    adc_deinit(phd->adc_periph);
    rcu_periph_clock_enable(phd->rcu_adc);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    adc_deinit(phd->adc_periph);

    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(phd->adc_periph, ADC_SCAN_MODE, ENABLE);
    adc_data_alignment_config(phd->adc_periph, ADC_DATAALIGN_RIGHT);

    adc_oversample_mode_config(phd->adc_periph,
                               ADC_OVERSAMPLING_ALL_CONVERT,
                               ADC_OVERSAMPLING_SHIFT_8B,
                               ADC_OVERSAMPLING_RATIO_MUL256);
    adc_oversample_mode_enable(phd->adc_periph);

    adc_channel_length_config(phd->adc_periph, ADC_REGULAR_CHANNEL, 1);
    adc_interrupt_flag_clear(phd->adc_periph, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(phd->adc_periph, ADC_INT_FLAG_EOIC);
    adc_interrupt_enable(phd->adc_periph, ADC_INT_EOC);
    adc_external_trigger_source_config(phd->adc_periph, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(phd->adc_periph, ADC_REGULAR_CHANNEL, ENABLE);

    adc_enable(phd->adc_periph);
    adc_calibration_enable(phd->adc_periph);
}

static void adc_setup_once(adc_handle* ph, adc_session* sess)
{
    const auto* padc = &ph->mInfo;
    const int   ch   = sess->chan_idx;
    adc_regular_channel_config(padc->adc_periph, 0U, padc->chan[ch].adc_channel, sess->sample_time);
    adc_software_trigger_enable(padc->adc_periph, ADC_REGULAR_CHANNEL);
}

static int adc_irq_cpl_critical(adc_handle* handle, uint16_t val)
{
    if (list_empty(&handle->list_work)) {
        return 0;
    }

    worknode*    pbase = list_first_entry(&handle->list_work, worknode, ws_node);
    adc_session* pnod  = static_cast<adc_session*>(pbase);
    list_del(&pnod->ws_node);

    pnod->result = val;
    handle->wq_.add_new_nolock(*pnod); // 唤醒等待的协程

    // 若还有剩余会话，继续触发
    if (!list_empty(&handle->list_work)) {
        auto* next_base = list_first_entry(&handle->list_work, worknode, ws_node);
        adc_setup_once(handle, static_cast<adc_session*>(next_base));
    }
    return 1;
}

static void adc_irq_handle(adc_handle* h)
{
    auto* padc = &h->mInfo;
    if (adc_interrupt_flag_get(padc->adc_periph, ADC_INT_FLAG_EOC)) {
        adc_interrupt_flag_clear(padc->adc_periph, ADC_INT_FLAG_EOC);
        uint16_t val = adc_regular_data_read(padc->adc_periph);
        uint32_t lk  = lock_acquire();
        int      ret = adc_irq_cpl_critical(h, val);
        lock_release(lk);
        if (ret) {
            h->wq_.trig_once();
        }
    }
}

static adc_handle adc0(adc_info_0, get_sys_workqueue());
static adc_handle adc1(adc_info_1, get_sys_workqueue());

extern "C" void ADC0_1_IRQHandler(void)
{
    adc_irq_handle(&adc0);
    adc_irq_handle(&adc1);
}

static void adc_hard_init(const adc_hard_info& hard)
{
    adc_nvic_config(&hard);
    adc_periph_init(&hard);
    // 配置通道对应 GPIO（内部通道无需）
    for (size_t idx = 0; idx < 18; ++idx) {
        if (idx == 16 || idx == 17) {
            if (hard.idx == 0) {
                adc_tempsensor_vrefint_enable();
            }
        } else if (hard.avail_map & (1u << idx)) {
            adc_chan_gpio_cfg(&hard, (int)idx);
        }
    }
}

adc_handle* adc_handle_get_init(int num)
{
    switch (num) {
    case 0:
        if (!adc0.is_init) {
            adc_hard_init(adc0.mInfo);
            adc0.is_init = 1;
        }
        return &adc0;
    case 1:
        if (!adc1.is_init) {
            adc_hard_init(adc1.mInfo);
            adc1.is_init = 1;
        }
        return &adc1;
    default:
        return nullptr;
    }
}

static void adc_enqueue_session_nolock(adc_handle& handle, adc_session& sess)
{
    bool empty = list_empty(&handle.list_work);
    list_add_tail(&sess.ws_node, &handle.list_work);
    if (empty) {
        adc_setup_once(&handle, &sess);
    }
}

void adc_enqueue_session(adc_handle& handle, adc_session& sess)
{
    uint32_t lk = lock_acquire();
    adc_enqueue_session_nolock(handle, sess);
    lock_release(lk);
}

// 供 AdcManager 使用的访问函数
co_wq::workqueue<cortex_lock>& adc_handle_wq(adc_handle* h)
{
    return h->wq_;
}
co_wq::Semaphore<cortex_lock>& adc_handle_sem(adc_handle* h)
{
    return h->sem;
}

// RAII：析构释放信号量
#include "co_adc.hpp"

void AdcManager::release()
{
    if (handle_) {
        adc_handle_sem(handle_).release();
        handle_ = nullptr;
    }
}

AdcManager::~AdcManager()
{
    release();
}
