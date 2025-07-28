#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BUILD_DAEMON_KO
#include <linux/types.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif

typedef struct ringbuffer {
    uint8_t* buffer;
    uint32_t size;
    uint32_t write;
    uint32_t read;
    uint32_t data_len;
} ringbuffer_t;

void     rb_reset(ringbuffer_t* fifo);
uint32_t rb_len(ringbuffer_t* fifo);
uint32_t rb_avail(ringbuffer_t* fifo);
bool     rb_is_empty(ringbuffer_t* fifo);
bool     rb_is_full(ringbuffer_t* fifo);
uint32_t rb_size(ringbuffer_t* fifo);

/*write to ringbuffer*/
uint32_t rb_in(ringbuffer_t* fifo, const void* in, uint32_t len);
uint32_t rb_in_with_limit(ringbuffer_t* fifo, const void* in, uint32_t len, uint32_t limit);

/*read to ringbuffer*/
uint32_t rb_out(ringbuffer_t* fifo, void* out, uint32_t len);

uint32_t rb_get_data(ringbuffer_t* fifo, void* outbuf, uint32_t start, uint32_t len);
uint32_t rb_move_ipt(ringbuffer_t* fifo, uint32_t len);
uint32_t rb_get_contiguous_len(ringbuffer_t* fifo);
uint8_t* rb_get_wr_ptr(ringbuffer_t* fifo);

#ifdef __cplusplus
}
#endif

#endif /* _RING_BUFFER_H_ */
