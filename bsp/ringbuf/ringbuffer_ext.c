#include "ringbuffer.h"
#ifdef BUILD_DAEMON_KO
#include <linux/string.h>
#else
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#endif

/**
 * \brief  Gets some data from the FIFO.
 * \param  [in] fifo: The fifo to be used.
 * \param  [in] out:  Where the data must be copied.
 * \param  [in] start:  Where the data must be copied start index.
 * \param  [in] len:  The size of the destination buffer.
 * \return The number of copied bytes.
 * \note   This function copies at most @len bytes from the FIFO into
 *         the @out and returns the number of copied bytes.
 */
uint32_t rb_get_data(ringbuffer_t* fifo, void* outbuf, uint32_t start, uint32_t len)
{
    uint32_t readlen = 0, tmplen = 0;
    if (rb_is_empty(fifo))
        return 0;

    if (start > fifo->data_len) {
        return 0;
    }

    readlen = len + start > fifo->data_len ? fifo->data_len - start : len;

    uint32_t newrd = (fifo->read + start) % fifo->size;

    tmplen = fifo->size - newrd;

    if (NULL != outbuf) {
        if (readlen <= tmplen) {
            memcpy((void*)outbuf, (void*)&fifo->buffer[newrd], readlen);
        } else {
            memcpy((void*)outbuf, (void*)&fifo->buffer[newrd], tmplen);
            memcpy((uint8_t*)outbuf + tmplen, (void*)fifo->buffer, readlen - tmplen);
        }
    }

    return readlen;
}

uint32_t rb_in_with_limit(ringbuffer_t* fifo, const void* in, uint32_t len, uint32_t limit)
{
    if (0 == limit) {
        return rb_in(fifo, in, len);
    }

    uint32_t rblen  = rb_len(fifo);
    int32_t  rbleft = (int32_t)limit - (int32_t)rblen;

    if (rbleft < 0) {
        return 0;
    }

    if ((uint32_t)rbleft < len) {
        len = rbleft;
    }

    return rb_in(fifo, in, len);
}

uint32_t rb_move_ipt(ringbuffer_t* fifo, uint32_t len)
{
    uint32_t writelen = 0, tmplen = 0;

    if (rb_is_full(fifo)) {
        return 0;
    }

    tmplen   = fifo->size - fifo->data_len;
    writelen = tmplen > len ? len : tmplen;

    fifo->write = (fifo->write + writelen) % fifo->size;
    fifo->data_len += writelen;

    return writelen;
}

uint32_t rb_get_contiguous_len(ringbuffer_t* fifo)
{
    if (rb_is_full(fifo)) {
        return 0;
    }

    if (fifo->write == fifo->read && rb_avail(fifo) > 0) {
        return fifo->size - fifo->write;
    }

    if (fifo->write > fifo->read) {
        return fifo->size - fifo->write;
    } else {
        return rb_avail(fifo);
    }

    return 0;
}

uint8_t* rb_get_wr_ptr(ringbuffer_t* fifo)
{
    return fifo->buffer + fifo->write;
}
