#pragma once
#include <stdint.h>
typedef int (*adc_cb)(void* priv, uint16_t adc_val);

int adc_ext_enable(int adc_idx);
int adc_sample_once_cb(int adc_idx, int chan_idx, adc_cb pcb_fun, void* pcb_priv);
int adc_sample_once_sync(int adc_idx, int chan_idx);
