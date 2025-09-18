// ADC 内部结构与辅助函数声明，仅供协程模板实现使用。
#pragma once
#include "semaphore.hpp"
#include "syswork.hpp"

struct adc_handle; // 前向声明

// 单次采样会话：仅需通道与采样时间，结果由驱动填充
struct adc_session : co_wq::worknode {
    uint8_t  chan_idx { 0xff };
    uint32_t sample_time { 0 }; // 使用 GD32E11x 的 ADC_SAMPLETIME_* 宏
    uint16_t result { 0 };      // 转换完成后写入
};

// 获取/初始化 ADC 句柄（目前支持 0 与 1）
adc_handle* adc_handle_get_init(int num);

// 访问底层工作队列与互斥信号量
co_wq::workqueue<cortex_lock>& adc_handle_wq(adc_handle* h);
co_wq::Semaphore<cortex_lock>& adc_handle_sem(adc_handle* h);

// 入队一次采样（队列空则立即启动转换）
void adc_enqueue_session(adc_handle& handle, adc_session& sess);
