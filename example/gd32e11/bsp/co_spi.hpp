#pragma once
#include <stdint.h>

struct spi_handle;
typedef enum {
    spi_is_master_not_slave = 1 << 0,
    spi_is_8bit_not_16bit   = 1 << 1,
    spi_is_msb_not_lsb      = 1 << 2,

    spi_cpol = 1 << 3,
    spi_cpha = 1 << 4,

    spi_cp_mode_0 = 0,
    spi_cp_mode_1 = (0 | spi_cpha),
    spi_cp_mode_2 = (spi_cpol | 0),
    spi_cp_mode_3 = (spi_cpol | spi_cpha),

    spi_high_speed = 1 << 5,
} spi_mode_bit;

typedef enum {
    spi_cs_not_set_at_end  = 1 << 0,
    spi_cs_not_set_at_half = 1 << 1,
    spi_only_together      = 1 << 2,
    spi_short_cb           = 1 << 3,
    spi_fast_cb            = 1 << 4,
} spi_ctrl_bit;

typedef int (*spi_ext_callback_t)(struct spi_handle* handle,
                                  const uint8_t*     tx_buff,
                                  const uint8_t*     rx_buff,
                                  int                len,
                                  void*              priv);

struct spi_handle* spi_ext_handle_Acquire(int num, uint32_t mode);
void               spi_ext_handle_Release(struct spi_handle* phandle);

void spi_ext_set_sp_dummy_byte(struct spi_handle* phandle, uint8_t dummy_byte);

// void spi_ext_deinit(struct spi_handle* handle);
int spi_ext_transfer_cb(struct spi_handle* handle,
                        const uint8_t*     tx_buff,
                        const uint8_t*     rx_buff,
                        int                len,
                        spi_ctrl_bit       ctrl_bit,
                        spi_ext_callback_t cb,
                        void*              cb_priv);