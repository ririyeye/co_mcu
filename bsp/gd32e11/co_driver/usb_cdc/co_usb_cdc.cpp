#include "semaphore.hpp"
extern "C" {
#include "cdc_acm_core.h"
#include "drv_usb_core.h"
#include "drv_usb_hw.h"
#include "usbd_core.h"
}
usb_core_driver cdc_acm;
#include "co_usb_cdc.hpp"
#include "syswork.hpp"

using namespace co_wq;

typedef enum {
    cdc_ret_error    = -1,
    cdc_ret_null_pkt = -2,
    cdc_ret_continue = 0,
    cdc_ret_pkt_cpl  = 1,
} cdc_ret;

// cdc_usr_node 定义移到 internal 头（供模板协程使用）

struct cdc_usr : worknode {
    explicit cdc_usr(workqueue<cortex_lock>& wq) : wq_(wq), sem(wq_, 1, 1)
    {
        is_init       = 0;
        usb_ready_flg = 0;
    }
    struct workqueue<cortex_lock>& wq_;

    usb_dev* udev = nullptr;

    uint32_t is_init       : 1;
    uint32_t usb_ready_flg : 1;

    co_wq::list_head list_work_tx;
    co_wq::list_head list_work_rx;

    Semaphore<cortex_lock> sem;
};

cdc_usr         cdc_data(get_sys_workqueue());
struct cdc_usr* get_cdc(void);
void            cdc_usr_init(void)
{
    if (cdc_data.is_init) {
        return;
    }
    cdc_data.is_init = 1;

    usb_rcu_config();
    usb_timer_init();
    usbd_init(&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
    usb_intr_config();
}

static cdc_ret cdc_usr_cpl_cb_nolock(int len, uint8_t* addr, co_wq::list_head* phead, int txflg, struct cdc_usr* pcdc)
{
    // 没有可以接收的 数据节点
    if (list_empty(phead)) {
        printf("error usb recv sta!!");
        return cdc_ret_null_pkt;
    }

    struct worknode* pbase = list_first_entry(phead, worknode, ws_node);
    cdc_usr_node*    pnod  = static_cast<cdc_usr_node*>(pbase);

    if (pnod->data + pnod->dat_cur != addr) {
        printf("error usb recv addr!!");
        return cdc_ret_null_pkt;
    }

    // 更新work 节点
    pnod->dat_cur += len;
    if (txflg) {
        if (pnod->dat_cur >= pnod->dat_max) {
            list_del(&pnod->ws_node);
            pcdc->wq_.add_new_nolock(*pnod);
            return cdc_ret_pkt_cpl;
        }
        return cdc_ret_continue;
    } else {
        list_del(&pnod->ws_node);
        pcdc->wq_.add_new_nolock(*pnod);
        return cdc_ret_pkt_cpl;
    }
}

static int cdc_usr_setup_recv_nolock(usb_dev* udev, struct cdc_usr* pcdc)
{
    // 没有可以接收的 数据节点
    if (list_empty(&pcdc->list_work_rx)) {
        return -1;
    }

    struct worknode*     pbase = list_first_entry(&pcdc->list_work_rx, worknode, ws_node);
    struct cdc_usr_node* pnod  = static_cast<cdc_usr_node*>(pbase);
    usbd_ep_recev(udev,
                  CDC_DATA_OUT_EP,
                  (uint8_t*)pnod->data,
                  pnod->dat_max < USB_CDC_DATA_PACKET_SIZE ? pnod->dat_max : USB_CDC_DATA_PACKET_SIZE);

    return 0;
}

static int cdc_usr_setup_send_nolock(usb_dev* udev, struct cdc_usr* pcdc)
{
    // 没有可以发送数据节点
    if (list_empty(&pcdc->list_work_tx)) {
        return -1;
    }

    struct worknode*     pbase = list_first_entry(&pcdc->list_work_tx, worknode, ws_node);
    struct cdc_usr_node* pnod  = static_cast<cdc_usr_node*>(pbase);

    int left = pnod->dat_max - pnod->dat_cur;
    usbd_ep_send(udev,
                 CDC_DATA_IN_EP,
                 (uint8_t*)pnod->data + pnod->dat_cur,
                 left < USB_CDC_DATA_PACKET_SIZE ? left : USB_CDC_DATA_PACKET_SIZE);

    return 0;
}

extern "C" void cdc_usr_recv_cpl(usb_dev* udev, int len, uint8_t* addr)
{
    struct cdc_usr* pcdc = get_cdc();

    uint32_t lk  = lock_acquire();
    int      ret = cdc_usr_cpl_cb_nolock(len, addr, &pcdc->list_work_rx, 0, pcdc);
    if (ret < 0) {
        lock_release(lk);
        return;
    }
    cdc_usr_setup_recv_nolock(udev, pcdc);
    lock_release(lk);

    if (ret == cdc_ret_pkt_cpl) {
        pcdc->wq_.trig_once();
    }
}
extern "C" void cdc_usr_send_cpl(usb_dev* udev, int len, uint8_t* addr)
{

    struct cdc_usr* pcdc = get_cdc();
    uint32_t        lk   = lock_acquire();
    int             ret  = cdc_usr_cpl_cb_nolock(len, addr, &pcdc->list_work_tx, 1, pcdc);
    if (ret < 0) {
        lock_release(lk);
        return;
    }
    cdc_usr_setup_send_nolock(udev, pcdc);
    lock_release(lk);

    if (ret == cdc_ret_pkt_cpl) {
        pcdc->wq_.trig_once();
    }
}

extern "C" void cdc_usr_set_ready(usb_dev* udev)
{

    struct cdc_usr* pcdc = get_cdc();

    uint32_t lk         = lock_acquire();
    pcdc->usb_ready_flg = 1;
    pcdc->udev          = udev;
    cdc_usr_setup_recv_nolock(udev, pcdc); // 开始接收数据
    cdc_usr_setup_send_nolock(udev, pcdc); // 开始发送数据
    lock_release(lk);
}

void cdc_usr_async_setup_nolock(co_wq::list_head* plist,
                                cdc_usr_node*     pnod,
                                int               usb_ready_flg,
                                usb_dev*          udev,
                                int               txflg)
{
    int instance_trx = 0;
    if (list_empty(plist)) {
        if (usb_ready_flg) {
            instance_trx = 1;
        }
    }

    list_add_tail(&pnod->ws_node, plist);

    if (instance_trx) {
        if (txflg) {
            usbd_ep_send(udev,
                         CDC_DATA_IN_EP,
                         (uint8_t*)pnod->data,
                         pnod->dat_max < USB_CDC_DATA_PACKET_SIZE ? pnod->dat_max : USB_CDC_DATA_PACKET_SIZE);
        } else {
            usbd_ep_recev(udev,
                          CDC_DATA_OUT_EP,
                          (uint8_t*)pnod->data,
                          pnod->dat_max < USB_CDC_DATA_PACKET_SIZE ? pnod->dat_max : USB_CDC_DATA_PACKET_SIZE);
        }
    }
}

struct cdc_usr* get_cdc(void)
{
    return &cdc_data;
}

struct cdc_usr* get_cdc_init(void)
{
    cdc_usr_init();
    return &cdc_data;
}

struct cdc_usr* get_cdc_init_by_num(int num)
{
    switch (num) {
    case 0:
        return get_cdc_init();
    default:
        return nullptr;
    }
}

// 模板版本实现已迁移到头文件

// ===== 提供给模板的访问函数 =====
co_wq::workqueue<cortex_lock>& cdc_usr_wq(struct cdc_usr* h)
{
    return h->wq_;
}
co_wq::Semaphore<cortex_lock>& cdc_usr_sem(struct cdc_usr* h)
{
    return h->sem;
}
co_wq::list_head* cdc_usr_list_tx(struct cdc_usr* h)
{
    return &h->list_work_tx;
}
co_wq::list_head* cdc_usr_list_rx(struct cdc_usr* h)
{
    return &h->list_work_rx;
}
int cdc_usr_ready(struct cdc_usr* h)
{
    return h->usb_ready_flg;
}
usb_dev* cdc_usr_dev(struct cdc_usr* h)
{
    return h->udev;
}

void UsbCDCManager::release()
{
    if (handle_) {
        handle_->sem.release();
        handle_ = nullptr;
    }
}

UsbCDCManager::~UsbCDCManager()
{
    release();
}
