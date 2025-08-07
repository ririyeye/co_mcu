#include "dma_def.h"
#include "gd32e11x_spi.h"
// #include "spi_ext.h"

#if 0
struct spi_handle;
struct spi_session : worknode {
    struct work_struct ws;

    const uint8_t* tx_buf;
    const uint8_t* rx_buf;
    uint32_t       len;
    spi_ctrl_bit   ctrl_bit;

    spi_ext_callback_t cb_fun;
    void*              cb_priv;

    struct spi_handle* spi_handle;
};

struct spi_hard_info {
    rcu_periph_enum rcu_spi;
    struct hard_dma dma[2];

    rcu_periph_enum soft_cs_gpio_rcu;
    uint32_t        soft_cs_gpio_periph;
    uint32_t        soft_cs_gpio_pin;

    void (*spi_pin_cfg)(const struct spi_hard_info* pinfo);

    uint32_t spi_periph;
};

struct spi_handle {
    uint32_t is_init : 1;
    uint8_t  cur_mode;

    uint8_t dummy_tx[4]; // 4 bytes for 32-bit dummy tx

    const struct spi_hard_info* info;

    struct spi_session sessions[SESSION_NUM];

    struct workqueue_struct cb_wq;

    struct list_head list_free;
    struct list_head list_work;

    struct os_sem_ext mtx;
};
static void spi0_pin_cfg(const struct spi_hard_info* pinfo);

static const struct spi_hard_info spi_info_0 = {
    .dma[HARDWARE_TX].dma_rcu     = RCU_DMA0,
    .dma[HARDWARE_TX].dma_irq_num = DMA0_Channel2_IRQn,
    .dma[HARDWARE_TX].dma_channel = DMA_CH2,
    .dma[HARDWARE_TX].dma_periph  = DMA0,

    .dma[HARDWARE_RX].dma_rcu     = RCU_DMA0,
    .dma[HARDWARE_RX].dma_irq_num = DMA0_Channel1_IRQn,
    .dma[HARDWARE_RX].dma_channel = DMA_CH1,
    .dma[HARDWARE_RX].dma_periph  = DMA0,

    .soft_cs_gpio_periph = GPIOA,
    .soft_cs_gpio_pin    = GPIO_PIN_4,
    .soft_cs_gpio_rcu    = RCU_GPIOA,

    .spi_pin_cfg = spi0_pin_cfg,

    .rcu_spi    = RCU_SPI0,
    .spi_periph = SPI0,
};

static void spi_master_config(const struct spi_hard_info* phard, uint32_t mode)
{
    rcu_periph_clock_enable(phard->rcu_spi);

    spi_parameter_struct spi_init_struct;
    /* deinitialize SPI and the parameters */
    spi_i2s_deinit(phard->spi_periph);
    spi_struct_para_init(&spi_init_struct);

    uint32_t clock_polarity_phase = 0;
    if (mode & spi_cpha) {
        clock_polarity_phase |= SPI_CTL0_CKPH;
    }
    if (mode & spi_cpol) {
        clock_polarity_phase |= SPI_CTL0_CKPL;
    }

    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = (mode & spi_is_8bit_not_16bit) ? SPI_FRAMESIZE_8BIT : SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = clock_polarity_phase;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = mode & spi_high_speed ? SPI_PSC_8 : SPI_PSC_64;
    spi_init_struct.endian               = (mode & spi_is_msb_not_lsb) ? SPI_ENDIAN_MSB : SPI_ENDIAN_LSB;
    spi_init(phard->spi_periph, &spi_init_struct);
}

static void spi0_pin_cfg(const struct spi_hard_info* pinfo)
{
    (void)pinfo;
    rcu_periph_clock_enable(RCU_GPIOA);

    /* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
}

static void spi_cs_gpio_config(const struct spi_hard_info* pinfo)
{
    rcu_periph_clock_enable(pinfo->soft_cs_gpio_rcu);
    gpio_init(pinfo->soft_cs_gpio_periph, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pinfo->soft_cs_gpio_pin);

    gpio_bit_write(pinfo->soft_cs_gpio_periph, pinfo->soft_cs_gpio_pin, 1);
}

static void
spi_dma_config(uint8_t tx_dummy_buff[4], const struct spi_hard_info* phard, const uint8_t* addr, int len, int is_tx)
{
    static uint8_t rx_dummy_buff[4];

    rcu_periph_clock_enable(phard->dma[is_tx].dma_rcu);

    dma_parameter_struct dma_init_struct;

    dma_struct_para_init(&dma_init_struct);

    uint32_t dma_periph  = phard->dma[is_tx].dma_periph;
    uint32_t dma_channel = phard->dma[is_tx].dma_channel;

    dma_deinit(dma_periph, dma_channel);

    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(phard->spi_periph);
    dma_init_struct.direction    = is_tx ? DMA_MEMORY_TO_PERIPHERAL : DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_LOW;
    dma_init_struct.number       = len;

    if (addr == NULL) {
        dma_init_struct.memory_addr = is_tx ? (uint32_t)tx_dummy_buff : (uint32_t)rx_dummy_buff;
        dma_init_struct.memory_inc  = DMA_MEMORY_INCREASE_DISABLE;
    } else {
        dma_init_struct.memory_addr = (uint32_t)addr;
        dma_init_struct.memory_inc  = DMA_MEMORY_INCREASE_ENABLE;
    }
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init(dma_periph, dma_channel, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(dma_periph, dma_channel);
    dma_memory_to_memory_disable(dma_periph, dma_channel);
}
static void spi_setup_transfer(struct spi_handle* phandle, struct spi_session* pnod);

static void spi_irq_cb(struct work_struct* work)
{
    struct spi_session* pnod = container_of(work, struct spi_session, ws.entry);

    if (pnod->cb_fun) {
        pnod->cb_fun(pnod->spi_handle, pnod->tx_buf, pnod->rx_buf, pnod->len, pnod->cb_priv);
    }

    taskENTER_CRITICAL();
    list_add_tail(&pnod->ws.entry, &pnod->spi_handle->list_free);
    taskEXIT_CRITICAL();
}

static void spi_set_cb(struct spi_session* pnod, struct spi_handle* handle)
{
    pnod->spi_handle = handle;

    if (!pnod->cb_fun) {
        list_add_tail(&pnod->ws.entry, &pnod->spi_handle->list_free);
    } else {
        if (pnod->ctrl_bit & spi_fast_cb) {
            if (pnod->cb_fun) {
                pnod->cb_fun(pnod->spi_handle, pnod->tx_buf, pnod->rx_buf, pnod->len, pnod->cb_priv);
            }
            list_add_tail(&pnod->ws.entry, &pnod->spi_handle->list_free);
        } else {
            INIT_WORK(&pnod->ws, spi_irq_cb);
            queue_work_on_irq(&handle->cb_wq, &pnod->ws);
        }
    }
}

static void spi_dma_cpl_cb_critical(struct spi_handle* handle, int is_tx)
{
    (void)is_tx;
    if (list_empty(&handle->list_work)) {
        return;
    }
    struct spi_session* pnod = list_first_entry(&handle->list_work, struct spi_session, ws.entry);

    list_del(&pnod->ws.entry);

    spi_set_cb(pnod, handle);

    const struct spi_hard_info* pinfo = handle->info;

    if (pnod->ctrl_bit & spi_cs_not_set_at_end) {
        // 结束后不拉高CS
    } else {
        gpio_bit_write(pinfo->soft_cs_gpio_periph, pinfo->soft_cs_gpio_pin, 1);
    }

    // 还有剩余工作节点 使能 SPI
    if (!list_empty(&handle->list_work)) {
        struct spi_session* pnod_new = list_first_entry(&handle->list_work, struct spi_session, ws.entry);
        spi_setup_transfer(handle, pnod_new);
    } else {
        spi_disable(pinfo->spi_periph);
    }
}

static void spi_dma_cpl_cb(struct spi_handle* handle, int is_tx)
{
    uint32_t dma_flag    = DMA_INT_FLAG_FTF;
    uint32_t dma_periph  = handle->info->dma[is_tx].dma_periph;
    uint32_t dma_channel = handle->info->dma[is_tx].dma_channel;

    if (dma_interrupt_flag_get(dma_periph, dma_channel, dma_flag)) {
        dma_interrupt_flag_clear(dma_periph, dma_channel, dma_flag);
    } else {
        return;
    }

    uint32_t x = taskENTER_CRITICAL_FROM_ISR();
    spi_dma_cpl_cb_critical(handle, is_tx);
    taskEXIT_CRITICAL_FROM_ISR(x);
}

static void spi_hard_init(const struct spi_hard_info* phard, uint32_t mode)
{
    spi_master_config(phard, mode);
    spi_cs_gpio_config(phard);
    if (phard->spi_pin_cfg) {
        phard->spi_pin_cfg(phard);
    }

    nvic_irq_enable(phard->dma[HARDWARE_RX].dma_irq_num, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4U, 0);
}

static void spi_handle_init_com(struct spi_handle* handle)
{
    INIT_LIST_HEAD(&handle->list_free);
    INIT_LIST_HEAD(&handle->list_work);
    for (int i = 0; i < SESSION_NUM; i++) {
        list_add_tail(&handle->sessions[i].ws.entry, &handle->list_free);
    }

    workqueue_struct_init(&handle->cb_wq, NULL, wq_single_thread, "spi_cb");

    os_sem_ext_init(&handle->mtx, "spi_mtx", 1, 1);
}

static void spi_setup_transfer(struct spi_handle* phandle, struct spi_session* pnod)
{
    const struct spi_hard_info* pinfo = phandle->info;

    gpio_bit_write(pinfo->soft_cs_gpio_periph, pinfo->soft_cs_gpio_pin, 0);

    const uint8_t* tx_buf = pnod->tx_buf;
    const uint8_t* rx_buf = pnod->rx_buf;
    int            len    = pnod->len;

    spi_dma_config(phandle->dummy_tx, pinfo, tx_buf, len, HARDWARE_TX);
    spi_dma_config(phandle->dummy_tx, pinfo, rx_buf, len, HARDWARE_RX);

    dma_channel_enable(pinfo->dma[HARDWARE_TX].dma_periph, pinfo->dma[HARDWARE_TX].dma_channel);

    dma_interrupt_enable(pinfo->dma[HARDWARE_RX].dma_periph, pinfo->dma[HARDWARE_RX].dma_channel, DMA_INT_FTF);
    dma_channel_enable(pinfo->dma[HARDWARE_RX].dma_periph, pinfo->dma[HARDWARE_RX].dma_channel);

    spi_enable(pinfo->spi_periph);

    spi_dma_enable(pinfo->spi_periph, SPI_DMA_TRANSMIT);
    spi_dma_enable(pinfo->spi_periph, SPI_DMA_RECEIVE);
}

static struct spi_handle* _spi_ext_handle_require(struct spi_handle* phandle, uint32_t mode)
{
    if (phandle->is_init) {
        return phandle;
    }

    spi_handle_init_com(phandle);

    spi_hard_init(phandle->info, mode);

    phandle->is_init = 1;

    phandle->cur_mode = mode;

    return phandle;
}

void spi_ext_set_sp_dummy_byte(struct spi_handle* phandle, uint8_t dummy_byte)
{
    phandle->dummy_tx[0] = dummy_byte;
    phandle->dummy_tx[1] = dummy_byte;
    phandle->dummy_tx[2] = dummy_byte;
    phandle->dummy_tx[3] = dummy_byte;
}

void spi_ext_handle_Release(struct spi_handle* phandle)
{
    os_sem_release(&phandle->mtx);
}

static int spi_ext_transfer_cb_nolock(struct spi_handle* handle,
                                      const uint8_t*     tx_buf[],
                                      const uint8_t*     rx_buf[],
                                      int                len[],
                                      int                sz,
                                      spi_ctrl_bit       ctrl_bit,
                                      spi_ext_callback_t cb,
                                      void*              cb_priv)
{
    if (list_empty(&handle->list_free)) {
        return -__LINE__;
    }

    struct spi_session* tmp_node[sz];

    struct spi_session* tmp;
    int                 tmp_len = 0;

    list_for_each_entry (tmp, &handle->list_free, ws.entry, struct spi_session) {
        tmp_node[tmp_len] = tmp;
        tmp_len++;
        if (tmp_len >= sz) {
            break;
        }
    }

    if (ctrl_bit & spi_only_together) {
        if (tmp_len != sz) {
            return -__LINE__;
        }
    }

    int emptyflg = 0;
    // 第一个工作节点 使能 SPI
    if (list_empty(&handle->list_work)) {
        emptyflg = 1;
    }

    for (int i = 0; i < tmp_len; i++) {
        list_del(&tmp_node[i]->ws.entry);
        tmp_node[i]->tx_buf  = tx_buf[i];
        tmp_node[i]->rx_buf  = rx_buf[i];
        tmp_node[i]->len     = len[i];
        tmp_node[i]->cb_fun  = NULL;
        tmp_node[i]->cb_priv = NULL;

        tmp_node[i]->ctrl_bit = ctrl_bit & ~spi_cs_not_set_at_end;
        if (ctrl_bit & spi_cs_not_set_at_half) {
            tmp_node[i]->ctrl_bit = ctrl_bit | spi_cs_not_set_at_end;
        }

        list_add_tail(&tmp_node[i]->ws.entry, &handle->list_work);
    }

    tmp_node[tmp_len - 1]->cb_fun  = cb;
    tmp_node[tmp_len - 1]->cb_priv = cb_priv;

    tmp_node[tmp_len - 1]->ctrl_bit = ctrl_bit & ~spi_cs_not_set_at_end;
    tmp_node[tmp_len - 1]->ctrl_bit = ctrl_bit & ~spi_cs_not_set_at_half;
    if (ctrl_bit & spi_cs_not_set_at_end) {
        tmp_node[tmp_len - 1]->ctrl_bit = ctrl_bit;
        tmp_node[tmp_len - 1]->ctrl_bit |= spi_cs_not_set_at_end;
    }

    if (emptyflg) {
        spi_setup_transfer(handle, tmp_node[0]);
    }

    return tmp_len;
}

static int spi_ext_transfer_cb_lock(struct spi_handle* handle,
                                    const uint8_t*     tx_buf[],
                                    const uint8_t*     rx_buf[],
                                    int                len[],
                                    int                sz,
                                    spi_ctrl_bit       ctrl_bit,
                                    spi_ext_callback_t cb,
                                    void*              cb_priv)
{
    taskENTER_CRITICAL();
    int ret = spi_ext_transfer_cb_nolock(handle, tx_buf, rx_buf, len, sz, ctrl_bit, cb, cb_priv);
    taskEXIT_CRITICAL();
    return ret;
}

int spi_ext_transfer_multi_cb(struct spi_handle* handle,
                              const uint8_t*     tx_buff[],
                              const uint8_t*     rx_buff[],
                              int                len[],
                              int                sz,
                              spi_ctrl_bit       ctrl_bit,
                              spi_ext_callback_t cb,
                              void*              cb_priv)
{
    if (!handle) {
        return -__LINE__;
    }

    if (!handle->is_init) {
        return -__LINE__;
    }

    if (!tx_buff && !rx_buff) {
        return -__LINE__;
    }

    if (sz < 0) {
        return -__LINE__;
    }

    int ret = spi_ext_transfer_cb_lock(handle, tx_buff, rx_buff, len, sz, ctrl_bit, cb, cb_priv);

    return ret;
}

int spi_ext_transfer_cb(struct spi_handle* handle,
                        const uint8_t*     tx_buff,
                        const uint8_t*     rx_buff,
                        int                len,
                        spi_ctrl_bit       ctrl_bit,
                        spi_ext_callback_t cb,
                        void*              cb_priv)
{
    if (!tx_buff && !rx_buff) {
        return -__LINE__;
    }

    if (len <= 0) {
        return -__LINE__;
    }

    const uint8_t* txarray[]  = { tx_buff };
    const uint8_t* rxarray[]  = { rx_buff };
    int            lenarray[] = { len };

    int ret = spi_ext_transfer_cb_lock(handle,
                                       txarray,
                                       rxarray,
                                       lenarray,
                                       1,
                                       ctrl_bit & ~spi_cs_not_set_at_half,
                                       cb,
                                       cb_priv);
    return ret;
}

struct spi_sync_data {
    osThreadId_t       tid;
    struct spi_handle* handle;
    int                recv_cnt;
};

static int spi_ext_transfer_sync_cb_proc(struct spi_handle* handle,
                                         const uint8_t*     tx_buff,
                                         const uint8_t*     rx_buff,
                                         int                len,
                                         void*              priv)
{
    (void)tx_buff;
    (void)rx_buff;

    struct spi_sync_data* sync_data = priv;

    if (sync_data->handle != handle) {
        return -__LINE__;
    }
    sync_data->recv_cnt = len;
    if (sync_data->tid) {
        osThreadFlagsSet(sync_data->tid, 1);
    }
    return 0;
}

int spi_ext_transfer_sync(struct spi_handle* handle, const uint8_t* tx_buff, const uint8_t* rx_buff, int len)
{
    int ret = len;

    struct spi_sync_data sync_data = {
        .tid    = osThreadGetId(),
        .handle = handle,
    };

    spi_ext_transfer_cb(handle, tx_buff, rx_buff, len, 0, spi_ext_transfer_sync_cb_proc, &sync_data);

    osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

    return ret;
}

static int spi_ext_transfer_sync_cb_proc_muliblk(struct spi_handle* handle,
                                                 const uint8_t*     tx_buff,
                                                 const uint8_t*     rx_buff,
                                                 int                len,
                                                 void*              priv)
{
    (void)handle;
    (void)tx_buff;
    (void)rx_buff;
    (void)len;

    osThreadId_t tid = priv;
    // 完成传输
    if (tid) {
        osThreadFlagsSet(tid, 1);
    }

    return 0;
}

int spi_ext_transfer_sync_multiblock(struct spi_handle* handle,
                                     const uint8_t*     tx_buff[],
                                     const uint8_t*     rx_buff[],
                                     int                len[],
                                     int                blk_cnt)
{
    int ret = spi_ext_transfer_multi_cb(handle,
                                        tx_buff,
                                        rx_buff,
                                        len,
                                        blk_cnt,
                                        0,
                                        spi_ext_transfer_sync_cb_proc_muliblk,
                                        osThreadGetId());

    if (ret > 0) {
        osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
    }

    return ret;
}

struct spi_handle spi_ext_0 = {
    .is_init = 0,
    .info    = &spi_info_0,
};

struct spi_handle* spi_ext_handle_Acquire(int num, uint32_t mode)
{
    struct spi_handle* phandle = NULL;
    switch (num) {
    case 0:
        phandle = _spi_ext_handle_require(&spi_ext_0, mode);
        break;
    default:
        break;
    }

    if (!phandle) {
        return NULL;
    }

    os_sem_acquire(&phandle->mtx, osWaitForever);

    if (mode != phandle->cur_mode) {
        spi_master_config(phandle->info, mode);
    }

    spi_ext_set_sp_dummy_byte(phandle, 0xff);

    return phandle;
}

extern "C" void DMA0_Channel1_IRQHandler(void)
{
    spi_dma_cpl_cb(&spi_ext_0, HARDWARE_RX);
}

extern "C" void DMA0_Channel2_IRQHandler(void)
{
    spi_dma_cpl_cb(&spi_ext_0, HARDWARE_TX);
}
#endif