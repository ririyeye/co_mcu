// USB CDC 内部结构与辅助函数声明，仅供协程模板实现使用。
#pragma once
#include "semaphore.hpp"
#include "syswork.hpp"
#include "worker.hpp"
extern "C" {
typedef struct _usb_core_driver usb_dev; // 与底层 USB 库保持一致的前置声明
}

struct cdc_usr;                         // 底层句柄
struct cdc_usr_node : co_wq::worknode { // 仅需在模板里继承使用
    uint8_t* data;
    uint16_t dat_max;
    uint16_t dat_cur;
};

// 初始化并返回 CDC 句柄
struct cdc_usr* get_cdc_init(void);
// 辅助访问函数（避免在模板中访问不完整类型成员）
co_wq::workqueue<cortex_lock>& cdc_usr_wq(struct cdc_usr* h);
co_wq::Semaphore<cortex_lock>& cdc_usr_sem(struct cdc_usr* h);
co_wq::list_head*              cdc_usr_list_tx(struct cdc_usr* h);
co_wq::list_head*              cdc_usr_list_rx(struct cdc_usr* h);
int                            cdc_usr_ready(struct cdc_usr* h);
usb_dev*                       cdc_usr_dev(struct cdc_usr* h);
// 挂接异步 I/O 节点（内部可能立即触发硬件传输）
void cdc_usr_async_setup_nolock(co_wq::list_head* plist,
                                cdc_usr_node*     pnod,
                                int               usb_ready_flg,
                                usb_dev*          udev,
                                int               txflg);
