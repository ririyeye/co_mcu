#include "adc_ext.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os2.h"
#include "freertos_ext.h"
#include "gd32e11x_rcu.h"
#include "sys_workqueue.h"
#include "task.h"
#include "usrlist.h"
#include "workqueue.h"
#include <stddef.h>
#include <stdio.h>

struct gd32_adc {
    // adc irq
    rcu_periph_enum rcu_adc;
    IRQn_Type       adc_irq_num;
    uint32_t        adc_periph;

    // adc gpio
    struct channel {
        uint32_t        gpio_periph;
        uint32_t        gpio_pin;
        rcu_periph_enum gpio_rcu;
        uint8_t         adc_channel;
        uint32_t        sample_time;
    } chan[18];
    uint32_t avail_map;
    uint32_t idx;

    void* handle;
};

struct adc_node {
    struct work_struct wk;

    adc_cb pcb_fun;
    void*  pcb_priv;

    uint16_t chan_idx;
    uint16_t adc_val;

    void* adc_handle;
};

#define SESSION_NUM 4

struct adc_ext {
    uint32_t inited : 1;

    struct adc_node sessions[SESSION_NUM];

    struct list_head list_free;
    struct list_head list_work;

    struct workqueue_struct adc_cb_wq;

    struct os_sem_ext sync_mtx;
    struct os_sem_ext sync_sem;
};

struct adc_ext adc0_ext = {
    .inited = 0,
};

struct adc_ext adc1_ext = {
    .inited = 0,
};

const struct gd32_adc adc0 = {
    .rcu_adc     = RCU_ADC0,
    .adc_irq_num = ADC0_1_IRQn,
    .adc_periph  = ADC0,

    .avail_map = BIT(16) | BIT(17),

    .chan[16].adc_channel = ADC_CHANNEL_16,
    .chan[16].sample_time = ADC_SAMPLETIME_13POINT5,

    .chan[17].adc_channel = ADC_CHANNEL_17,
    .chan[17].sample_time = ADC_SAMPLETIME_13POINT5,

    .idx = 0,

    .handle = &adc0_ext,
};

const struct gd32_adc adc1 = {
    .rcu_adc     = RCU_ADC1,
    .adc_irq_num = ADC0_1_IRQn,
    .adc_periph  = ADC1,

    .chan[0].gpio_periph = GPIOA,
    .chan[0].gpio_pin    = GPIO_PIN_0,
    .chan[0].gpio_rcu    = RCU_GPIOA,
    .chan[0].adc_channel = ADC_CHANNEL_0,
    .chan[0].sample_time = ADC_SAMPLETIME_13POINT5,

    .chan[1].gpio_periph = GPIOA,
    .chan[1].gpio_pin    = GPIO_PIN_1,
    .chan[1].gpio_rcu    = RCU_GPIOA,
    .chan[1].adc_channel = ADC_CHANNEL_1,
    .chan[1].sample_time = ADC_SAMPLETIME_13POINT5,

    .avail_map = BIT(0) | BIT(1),

    .idx = 1,

    .handle = &adc1_ext,
};

void adc_chan_gpio_cfg(const struct gd32_adc* padc, int chan_idx)
{
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(padc->chan[chan_idx].gpio_rcu);
    // adc1_in0
    gpio_init(padc->chan[chan_idx].gpio_periph, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_MAX, padc->chan[chan_idx].gpio_pin);
}

void adc_nvic_config(const struct gd32_adc* padc)
{
    nvic_irq_enable(padc->adc_irq_num, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4U, 0);
}

void adc_ext_init(const struct gd32_adc* padc)
{
    struct adc_ext* pext = padc->handle;
    if (pext->inited) {
        return;
    }
    pext->inited = 1;

    INIT_LIST_HEAD(&pext->list_free);
    INIT_LIST_HEAD(&pext->list_work);
    for (int i = 0; i < SESSION_NUM; i++) {
        list_add_tail(&pext->sessions[i].wk.entry, &pext->list_free);
    }

    char buff[32];
    snprintf(buff, 31, "adc_sync_sem_%d", (int)padc->idx);
    os_sem_ext_init(&pext->sync_sem, buff, 1, 0);

    snprintf(buff, 31, "adc_sync_mtx_%d", (int)padc->idx);
    os_sem_ext_init(&pext->sync_mtx, buff, 1, 1);

    workqueue_struct_init(&pext->adc_cb_wq, NULL, wq_single_thread, "adc_cb");
}

static struct adc_node* get_free_session_nolock(struct adc_ext* ctrl)
{
    struct adc_node* pnod = NULL;
    if (list_empty(&ctrl->list_free)) {
        pnod = NULL;
    } else {
        pnod = list_first_entry(&ctrl->list_free, struct adc_node, wk.entry);
        list_del(&pnod->wk.entry);
    }
    return pnod;
}

static void trig_adc_work_once(const struct gd32_adc* padc, struct adc_node* pnode)
{
    int chan_idx = pnode->chan_idx;
    /* ADC regular channel config */
    adc_regular_channel_config(padc->adc_periph,
                               0U,
                               padc->chan[chan_idx].adc_channel,
                               padc->chan[chan_idx].sample_time);
    /* ADC software trigger enable */
    adc_software_trigger_enable(padc->adc_periph, ADC_REGULAR_CHANNEL);
}

static int adc_channel_sample_cb(const struct gd32_adc* padc, int chan_idx, adc_cb pcb_fun, void* pcb_priv)
{
    struct adc_ext* pext     = padc->handle;
    int             emptyflg = 0;
    taskENTER_CRITICAL();
    if (list_empty(&pext->list_work)) {
        emptyflg = 1;
    }

    struct adc_node* pnode = get_free_session_nolock(pext);
    if (pnode) {
        pnode->pcb_fun  = pcb_fun;
        pnode->pcb_priv = pcb_priv;
        pnode->chan_idx = chan_idx;
        list_add_tail(&pnode->wk.entry, &pext->list_work);
    }
    taskEXIT_CRITICAL();

    if (!pnode) {
        return -__LINE__;
    }

    if (!emptyflg) {
        return 0;
    }

    trig_adc_work_once(padc, pnode);
#if 0
    /* wait the end of conversion flag */
    while (!adc_flag_get(padc->adc_periph, ADC_FLAG_EOC))
        ;
    /* clear the end of conversion flag */
    adc_flag_clear(padc->adc_periph, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(padc->adc_periph));
#endif
    return 0;
}

void adc_periph_init(const struct gd32_adc* phd)
{
    adc_deinit(phd->adc_periph);

    rcu_periph_clock_enable(phd->rcu_adc);

    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);

    adc_deinit(phd->adc_periph);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC continuous function enable */
    adc_special_function_config(phd->adc_periph, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(phd->adc_periph, ADC_DATAALIGN_RIGHT);

#if 0
    adc_resolution_config(phd->adc_periph, ADC_RESOLUTION_12B);
#endif
    /* 64 times sample, 6 bits shift */
    adc_oversample_mode_config(phd->adc_periph,
                               ADC_OVERSAMPLING_ALL_CONVERT,
                               ADC_OVERSAMPLING_SHIFT_8B,
                               ADC_OVERSAMPLING_RATIO_MUL256);
    adc_oversample_mode_enable(phd->adc_periph);

    /* ADC channel length config */
    adc_channel_length_config(phd->adc_periph, ADC_REGULAR_CHANNEL, 1);
    /* ADC regular channel config */
#if 0
    adc_regular_channel_config(phd->adc_periph, 0, phd->chan[chan_idx].adc_channel, phd->sample_time);
#endif
    /* clear the ADC flag */
    adc_interrupt_flag_clear(phd->adc_periph, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(phd->adc_periph, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    adc_interrupt_enable(phd->adc_periph, ADC_INT_EOC);

    /* ADC trigger config */
    adc_external_trigger_source_config(phd->adc_periph, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(phd->adc_periph, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(phd->adc_periph);

    adc_calibration_enable(phd->adc_periph);

    osDelay(1);
}

static void adc_irq_work_cb(struct work_struct* work)
{
    struct adc_node* pnode = container_of(work, struct adc_node, wk);

    if (pnode->pcb_fun) {
        pnode->pcb_fun(pnode->pcb_priv, pnode->adc_val);
    }
    taskENTER_CRITICAL();
    struct adc_ext* handle = pnode->adc_handle;
    list_add_tail(&pnode->wk.entry, &handle->list_free);
    taskEXIT_CRITICAL();
}

static void adc_set_cb(struct adc_node* pnod, uint16_t adc_val, const struct gd32_adc* padc, struct adc_ext* handle)
{
    pnod->adc_val    = adc_val;
    pnod->adc_handle = padc->handle;
    queue_work_on_irq(&handle->adc_cb_wq, &pnod->wk);
}

static void adc_cpl_cb_critical(const struct gd32_adc* padc, uint16_t adc_val)
{
    struct adc_ext* handle = padc->handle;

    if (list_empty(&handle->list_work)) {
        return;
    }
    struct adc_node* pnod = list_first_entry(&handle->list_work, struct adc_node, wk.entry);
    list_del(&pnod->wk.entry);

    INIT_WORK(&pnod->wk, adc_irq_work_cb);
    adc_set_cb(pnod, adc_val, padc, handle);

    // 还有剩余工作节点 使能 ADC
    if (!list_empty(&handle->list_work)) {
        struct adc_node* pnod_new = list_first_entry(&handle->list_work, struct adc_node, wk.entry);
        trig_adc_work_once(padc, pnod_new);
    }
}

static void adc_irq_handle(const struct gd32_adc* padc)
{
    if (adc_interrupt_flag_get(padc->adc_periph, ADC_INT_FLAG_EOC)) {
        adc_interrupt_flag_clear(padc->adc_periph, ADC_INT_FLAG_EOC);
        uint16_t val = adc_regular_data_read(padc->adc_periph);
        uint32_t x   = taskENTER_CRITICAL_FROM_ISR();
        adc_cpl_cb_critical(padc, val);
        taskEXIT_CRITICAL_FROM_ISR(x);
    }
}

void ADC0_1_IRQHandler(void)
{
    adc_irq_handle(&adc0);
    adc_irq_handle(&adc1);
}

static const struct gd32_adc* adc_tab[] = {
    &adc0,
    &adc1,
};

static const struct gd32_adc* find_idx(uint32_t idx)
{
    for (size_t i = 0; i < sizeof(adc_tab) / sizeof(adc_tab[0]); i++) {
        if (adc_tab[i]->idx == idx) {
            return adc_tab[i];
        }
    }
    return NULL;
}

int adc_ext_enable(int adc_idx)
{
    const struct gd32_adc* padc = find_idx(adc_idx);

    if (!padc) {
        return -1;
    }

    adc_ext_init(padc);
    adc_nvic_config(padc);
    adc_periph_init(padc);

    for (size_t idx = 0; idx < sizeof(uint32_t) * 8; idx++) {
        if (idx == 16 || idx == 17) {
            // 16是内部温度 只有adc0有
            if (adc_idx == 0) {
                adc_tempsensor_vrefint_enable();
            }
        } else if (padc->avail_map & BIT(idx)) {
            adc_chan_gpio_cfg(padc, idx);
        }
    }

    return 0;
}

int adc_sample_once_cb(int adc_idx, int chan_idx, adc_cb pcb_fun, void* pcb_priv)
{
    const struct gd32_adc* padc = find_idx(adc_idx);

    if (!padc) {
        return -__LINE__;
    }

    if (!(padc->avail_map & BIT(chan_idx))) {
        return -__LINE__;
    }

    adc_channel_sample_cb(padc, chan_idx, pcb_fun, pcb_priv);

    return 0;
}

struct adc_sync_data {
    uint16_t        adc_val;
    struct adc_ext* pext;
};

static int adc_sync_cb(void* priv, uint16_t adc_val)
{
    struct adc_sync_data* pdat = priv;
    pdat->adc_val              = adc_val;
    os_sem_release(&pdat->pext->sync_sem);
    return 0;
}

static int adc_sample_once_sync_nolock(struct adc_ext* pext, int adc_idx, int chan_idx)
{
    struct adc_sync_data dat = {
        .pext = pext,
    };

    int ret = adc_sample_once_cb(adc_idx, chan_idx, adc_sync_cb, &dat);

    if (!ret) {
        os_sem_acquire(&pext->sync_sem, osWaitForever);
        return dat.adc_val;
    } else {
        return ret;
    }
}

int adc_sample_once_sync(int adc_idx, int chan_idx)
{
    const struct gd32_adc* padc = find_idx(adc_idx);

    if (!padc) {
        return -__LINE__;
    }

    struct adc_ext* pext = padc->handle;

    os_sem_acquire(&pext->sync_mtx, osWaitForever);

    int ret = adc_sample_once_sync_nolock(pext, adc_idx, chan_idx);

    os_sem_release(&pext->sync_mtx);

    return ret;
}
