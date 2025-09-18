// I2C 内部结构与辅助函数声明，仅供协程模板实现使用。
#pragma once
#include "semaphore.hpp"
#include "syswork.hpp"

struct i2c_handle; // 前向声明

// 方向与控制位（与旧 i2c_ext 含义一致，但简化实现）
typedef enum {
    i2c_start_i2c  = 1 << 0, // 本次传输前发送 START
    i2c_end_i2c    = 1 << 1, // 本次传输后发送 STOP
    i2c_tx_not_rx  = 1 << 4, // 方向：1=TX，0=RX
} i2c_ctrl_bit;

struct i2c_session : co_wq::worknode {
    uint8_t*     buf;
    uint32_t     trans_max;  // 期望传输长度
    uint32_t     trans_len;  // 已传输长度（完成时拷回到 len 以兼容 awaiter）
    uint32_t     len;        // 对外结果长度（完成时 = trans_len）
    uint32_t     ctrl_bit;   // 见 i2c_ctrl_bit
};

typedef enum {
    i2c_stop = 0,
    i2c_set_start,
    i2c_send_tx_addr,
    i2c_send_rx_addr,
    i2c_send_tx_data,
    i2c_send_rx_data,
} i2c_sta;

// 按编号获取/初始化 I2C（目前仅支持 0）
struct i2c_handle* i2c_handle_get_init(int num);
// 兼容旧接口（等价 num=0）
static inline struct i2c_handle* i2c_handle_get_init()
{
    return i2c_handle_get_init(0);
}
// 访问底层工作队列
co_wq::workqueue<cortex_lock>& i2c_handle_wq(struct i2c_handle* h);
// 访问句柄内部信号量
co_wq::Semaphore<cortex_lock>& i2c_handle_sem(struct i2c_handle* h);
// 设置/获取目标 7bit 设备地址
void i2c_handle_set_addr(struct i2c_handle* h, uint8_t addr7);
uint8_t i2c_handle_get_addr(struct i2c_handle* h);
// 设置工作时钟（单位 Hz），典型 100000 或 400000
void i2c_handle_set_clock(struct i2c_handle* h, uint32_t hz);
uint32_t i2c_handle_get_clock(struct i2c_handle* h);
// 入队一次传输（在空队列时立即启动）
void i2c_enqueue_session(struct i2c_handle& phandle, struct i2c_session& psess);
