extern "C" {
#include "dma_def.h"
#include "gd32e11x_dma.h"
#include "gd32e11x_gpio.h"
#include "gd32e11x_rcu.h"
}
#include "co_spi.hpp"
#include "semaphore.hpp"
#include "syswork.hpp"

using namespace co_wq;

struct spi_session : worknode {
    const uint8_t* tx_buf;
    const uint8_t* rx_buf;
    uint32_t       len;
    uint32_t       ctrl_bit;
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

struct spi_handle : worknode {
    explicit spi_handle(const struct spi_hard_info& info, workqueue<cortex_lock>& wq)
        : wq_(wq), mInfo(info), sem(wq_, 1, 1)
    {
    }

    workqueue<cortex_lock>& wq_;

    uint32_t     is_init : 1;
    spi_mode_bit cur_mode;
    uint8_t      dummy_tx[4]; // 4 bytes for 32-bit dummy tx

    const struct spi_hard_info& mInfo;

    list_head              list_work;
    Semaphore<cortex_lock> sem;
};

static void spi0_pin_cfg(const struct spi_hard_info* pinfo)
{
    (void)pinfo;
    rcu_periph_clock_enable(RCU_GPIOA);

    /* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
}

static const struct spi_hard_info spi_info_0 = {

    RCU_SPI0,
    {
      {
            RCU_DMA0,
            DMA0_Channel1_IRQn,
            DMA_CH1,
            DMA0,
        }, {
            RCU_DMA0,
            DMA0_Channel2_IRQn,
            DMA_CH2,
            DMA0,
        }, },
    RCU_GPIOA,
    GPIOA,
    GPIO_PIN_4,
    spi0_pin_cfg,
    SPI0
};

struct spi_handle  spi_ext_0(spi_info_0, get_sys_workqueue());
struct spi_handle* spi_handle_get()
{
    return &spi_ext_0;
}

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

static void spi_cs_gpio_config(const struct spi_hard_info* pinfo)
{
    rcu_periph_clock_enable(pinfo->soft_cs_gpio_rcu);
    gpio_init(pinfo->soft_cs_gpio_periph, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pinfo->soft_cs_gpio_pin);

    gpio_bit_write(pinfo->soft_cs_gpio_periph, pinfo->soft_cs_gpio_pin, FlagStatus::SET);
}

static void spi_hard_init(const struct spi_hard_info* phard, uint32_t mode)
{
    spi_master_config(phard, mode);
    spi_cs_gpio_config(phard);
    if (phard->spi_pin_cfg) {
        phard->spi_pin_cfg(phard);
    }

    nvic_irq_enable(phard->dma[HARDWARE_RX].dma_irq_num, 4U, 0);
}

struct spi_handle* spi_handle_get_init()
{
    if (spi_ext_0.is_init) {
        return &spi_ext_0;
    }

    spi_hard_init(&spi_ext_0.mInfo, 0);

    spi_ext_0.is_init = 1;

    return &spi_ext_0;
}

static void
spi_dma_config(uint8_t tx_dummy_buff[4], const struct spi_hard_info* phard, const uint8_t* addr, int len, int is_tx)
{
    static uint8_t rx_dummy_buff[4];

    rcu_periph_clock_enable(phard->dma[is_tx].dma_rcu);

    dma_parameter_struct dma_init_struct;

    dma_struct_para_init(&dma_init_struct);

    uint32_t         dma_periph  = phard->dma[is_tx].dma_periph;
    dma_channel_enum dma_channel = phard->dma[is_tx].dma_channel;

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

static void spi_setup_transfer(struct spi_handle* phandle, struct spi_session* pnod)
{
    const struct spi_hard_info* pinfo = &phandle->mInfo;

    gpio_bit_write(pinfo->soft_cs_gpio_periph, pinfo->soft_cs_gpio_pin, FlagStatus::RESET);

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

static int spi_dma_cpl_cb_critical(struct spi_handle* handle, int is_tx)
{
    (void)is_tx;
    if (list_empty(&handle->list_work)) {
        return 0;
    }

    worknode* pbase = list_first_entry(&handle->list_work, worknode, ws_node);
    spi_session* pnod  = static_cast<spi_session*>(pbase);

    handle->wq_.add_new_nolock(*pnod);

    const struct spi_hard_info& pinfo = handle->mInfo;

    if (pnod->ctrl_bit & spi_cs_not_set_at_end) {
        // 结束后不拉高CS
    } else {
        gpio_bit_write(pinfo.soft_cs_gpio_periph, pinfo.soft_cs_gpio_pin, FlagStatus::SET);
    }

    // 还有剩余工作节点 使能 SPI
    if (!list_empty(&handle->list_work)) {
        worknode*    pbase_new = list_first_entry(&handle->list_work, worknode, ws_node);
        spi_session* pnod_new  = static_cast<spi_session*>(pbase_new);

        spi_setup_transfer(handle, pnod_new);
    } else {
        spi_disable(pinfo.spi_periph);
    }

    return 1;
}

static void spi_dma_cpl_cb(struct spi_handle* handle, int is_tx)
{
    uint32_t         dma_flag    = DMA_INT_FLAG_FTF;
    uint32_t         dma_periph  = handle->mInfo.dma[is_tx].dma_periph;
    dma_channel_enum dma_channel = handle->mInfo.dma[is_tx].dma_channel;

    if (dma_interrupt_flag_get(dma_periph, dma_channel, dma_flag)) {
        dma_interrupt_flag_clear(dma_periph, dma_channel, dma_flag);
    } else {
        return;
    }
    uint32_t lk  = lock_acquire();
    int      ret = spi_dma_cpl_cb_critical(handle, is_tx);
    lock_release(lk);

    if (ret) {
        handle->wq_.trig_once();
    }
}

extern "C" void DMA0_Channel1_IRQHandler(void)
{
    spi_dma_cpl_cb(&spi_ext_0, HARDWARE_RX);
}

extern "C" void DMA0_Channel2_IRQHandler(void)
{
    spi_dma_cpl_cb(&spi_ext_0, HARDWARE_TX);
}

static void spi_transfer_setup_nolock(struct spi_handle& phandle, spi_session& psess)
{
    int emptyflg = 0;
    if (list_empty(&phandle.list_work)) {
        emptyflg = 1;
    }

    list_add_tail(&psess.ws_node, &phandle.list_work);

    if (emptyflg) {
        spi_setup_transfer(&phandle, &psess);
    }
}

static void spi_transfer_setup(struct spi_handle& phandle, spi_session& psess)
{
    uint32_t lk = lock_acquire();
    spi_transfer_setup_nolock(phandle, psess);
    lock_release(lk);
}

Task<int, Work_Promise<cortex_lock, int>>
SpiManager::transfer(const uint8_t* tx_buff, const uint8_t* rx_buff, size_t len, uint32_t ctrl_bit)
{
    struct co_spi_session : spi_session {
        explicit co_spi_session(workqueue<cortex_lock>& wq) : cpl_inotify(wq, 0, 1) { INIT_LIST_HEAD(&ws_node); }
        Semaphore<cortex_lock> cpl_inotify;
    };

    co_spi_session node(handle_->wq_);
    node.tx_buf   = tx_buff;
    node.rx_buf   = rx_buff;
    node.len      = len;
    node.ctrl_bit = ctrl_bit;

    node.func = [](struct worknode* pws) {
        co_spi_session* psess = static_cast<co_spi_session*>(pws);
        psess->cpl_inotify.release();
    };

    spi_transfer_setup(*handle_, node);

    co_await SemReqAwaiter(node.cpl_inotify);

    co_return node.len;
}

void spi_ext_set_sp_dummy_byte(struct spi_handle* phandle, uint8_t dummy_byte)
{
    phandle->dummy_tx[0] = dummy_byte;
    phandle->dummy_tx[1] = dummy_byte;
    phandle->dummy_tx[2] = dummy_byte;
    phandle->dummy_tx[3] = dummy_byte;
}

Task<bool, Work_Promise<cortex_lock, bool>> SpiManager::init(uint32_t mode)
{
    if (handle_) {
        co_return true; // 已经初始化
    }

    auto tmp_handle = spi_handle_get_init();
    if (!tmp_handle) {
        co_return false; // 获取句柄失败
    }

    co_await SemReqAwaiter(tmp_handle->sem);

    handle_ = tmp_handle;
    if (mode != handle_->cur_mode) {
        spi_master_config(&handle_->mInfo, mode);
    }

    spi_ext_set_sp_dummy_byte(handle_, 0xff);

    co_return true;
}

SpiManager::~SpiManager()
{
    if (handle_) {
        handle_->sem.release();
        handle_ = nullptr;
    }
}
