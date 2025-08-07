#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

uint32_t lock_acquire(void);
void     lock_release(uint32_t magic);

#ifdef __cplusplus
}
#endif /* __cplusplus */
