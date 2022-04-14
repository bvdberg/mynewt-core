/*
 * Copyright INBRAIN-Neuroelectronics 2021-2022
 * All rights reserved
 */

#ifndef RING_BUF_LF_H
#define RING_BUF_LF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint32_t head;      // write-index
    volatile uint32_t tail;      // read-index
    uint32_t bufsize;
    uint8_t* buf;
} ringbuf_lf;

/**
* @brief Function used to init the ring buffer
* @note The actual number of storage bytes it 1 less than bufsize.
* @param rb - pointer to the ring buffer struct
* @param buf - pointer to the data buffer
* @param bufsize - size of buffer. See note
*/
void ringbuf_lf_init(ringbuf_lf* rb, uint8_t* buf, uint32_t bufsize);

/**
* @brief Function used to get the number of bytes that can be added
* @param rb - pointer to the ring buffer struct
* @return the number of bytes that can be added to the ring buffer
*/
uint32_t ringbuf_lf_get_free(const ringbuf_lf* rb);

/**
* @brief Function used to get the number of bytes that can be retrieved
* @param rb - pointer to the ring buffer struct
* @return the number of bytes in the ring buffer
*/
uint32_t ringbuf_lf_get_count(const ringbuf_lf* rb);

/**
* @brief Function used to see if the ring buffer is empty
* @param rb - pointer to the ring buffer struct
* @return Whether the ring buffer is empty
*/
static inline bool ringbuf_lf_isempty(const ringbuf_lf* rb) {
    return rb->head == rb->tail;
}

/**
* @brief Function used to get bytes from the ring buffer
* @note This function only modifies the head member
* @param rb - pointer to the ring buffer struct
* @param data - pointer to the data buffer
* @param max - the maximum number of retrieved bytes
* @return The number of added bytes
*/
uint32_t ringbuf_lf_add(ringbuf_lf* rb, const uint8_t* data, uint32_t num);

/**
* @brief Function used to retrieve bytes from the ring buffer without changing buffer
* @param rb - pointer to the ring buffer struct
* @param data - pointer to the data buffer
* @param max - the maximum number of retrieved bytes
* @return The number of retrieved bytes
*/
uint32_t ringbuf_lf_peek(const ringbuf_lf* rb, uint8_t* data, uint32_t max);

/**
* @brief Function used to retrieve bytes from the ring buffer
* @note This function only modifies the tail member
* @param rb - pointer to the ring buffer struct
* @param data - pointer to the data buffer
* @param max - the maximum number of retrieved bytes
* @return The number of retrieved bytes
*/
uint32_t ringbuf_lf_get(ringbuf_lf* rb, uint8_t* data, uint32_t max);

/**
* @brief Function used to consume up to num bytes from the ring buffer
* @note This function only modifies the tail member.
* @param rb - pointer to the ring buffer struct
* @param num - the maximum number of consumed bytes
* @return The number of consumed bytes
*/
uint32_t ringbuf_lf_consume(ringbuf_lf* rb, uint32_t num);

#ifdef __cplusplus
}
#endif

#endif

