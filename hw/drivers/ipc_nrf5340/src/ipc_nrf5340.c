/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <ipc_nrf5340/ipc_nrf5340.h>
#include <errno.h>
#include <os/os.h>
#include <os/os_trace_api.h>
#include <nrfx_ipc.h>
#if MYNEWT_VAL(BLE_TRANSPORT_INT_FLOW_CTL)
#include <nimble/transport.h>
#endif

#if MYNEWT
#include "ringbuf_lf.h"
#else
#include "utils/ringbuf_lf.h"
#endif

// TODO MYNEWT_VAL(IPC_NRF5340_NET_GPIO)

#define IPC_MAX_CHANS MYNEWT_VAL(IPC_NRF5340_CHANNELS)
#define IPC_BUF_SIZE MYNEWT_VAL(IPC_NRF5340_BUF_SZ)

#define NET_CRASH_CHANNEL 2
#define IPC_IRQ_PRIORITY  5
//#define LOG_IPC

typedef enum {
    APP_WAITS_FOR_NET,
    APP_AND_NET_RUNNING,
    NET_RESTARTED,
} ipc_state_t;

typedef struct {
    volatile ipc_state_t ipc_state;
#if MYNEWT_VAL(BLE_TRANSPORT_INT_FLOW_CTL)
    uint8_t acl_from_ll_count;
#endif
    ringbuf_lf ring[IPC_MAX_CHANS];
    char ring_data[IPC_MAX_CHANS][IPC_BUF_SIZE];

#ifdef LOG_IPC
    uint8_t notify[IPC_MAX_CHANS];
    uint8_t notify_data[IPC_MAX_CHANS];
    uint32_t log_idx;
    uint8_t log_buf[2048];
#endif
} SharedMemory;


static SharedMemory mem[1] __attribute__((section(".ipc")));

#if MYNEWT_VAL(MCU_APP_CORE)
static void (*net_core_restart_cb)(void);
#endif

typedef struct {
    ipc_nrf5340_recv_cb cb;
    void* user_data;
} ipc_t;

static ipc_t ipcs[IPC_MAX_CHANS];

void IPC_IRQHandler(void) {
    os_trace_isr_enter();

    /* Handle only interrupts that were enabled */
    uint32_t irq_pend = NRF_IPC->INTPEND & NRF_IPC->INTEN;

    for (int i = 0; i < IPC_MAX_CHANS; i++) {
        if (irq_pend & (0x1UL << i)) {
            NRF_IPC->EVENTS_RECEIVE[i] = 0;
            ipcs[i].cb(i, ipcs[i].user_data);
            if (ipcs[i].cb) ipcs[i].cb(i, ipcs[i].user_data);
        }
    }

    os_trace_isr_exit();
}

static void ipc_init(void) {
    /* Enable IPC channels */
    for (int i = 0; i < IPC_MAX_CHANS; i++) {
        NRF_IPC->SEND_CNF[i] = (0x01UL << i);
        NRF_IPC->RECEIVE_CNF[i] = 0;
    }

    NVIC_SetPriority(IPC_IRQn, IPC_IRQ_PRIORITY);
    NRF_IPC->INTENCLR = 0xFFFF;
    NVIC_ClearPendingIRQ(IPC_IRQn);
#if MYNEWT
    NVIC_SetVector(IPC_IRQn, (uint32_t)IPC_IRQHandler);
    // Note: otherwise via startup.S
#endif
    NVIC_EnableIRQ(IPC_IRQn);
}

#if MYNEWT_VAL(MCU_APP_CORE)
static void ipc_on_net_crash(int channel, void *user_data) {
    (void)channel;
    (void)user_data;

    if (mem->ipc_state == NET_RESTARTED) {
        mem->ipc_state = APP_AND_NET_RUNNING;
        if (net_core_restart_cb) net_core_restart_cb();
    }
}
#endif

void ipc_nrf5340_init(void) {
#if MYNEWT_VAL(MCU_APP_CORE)
    /* Make sure network core if off when we set up IPC */
    NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;

    memset(mem, 0, sizeof(SharedMemory));

    for (unsigned i=0; i<IPC_MAX_CHANS; i++) {
        ringbuf_lf_init(&mem->ring[i], (uint8_t*)mem->ring_data[i], sizeof(mem->ring_data[i]));
    }
    mem->ipc_state = APP_WAITS_FOR_NET;

#if MYNEWT_VAL(BLE_TRANSPORT_INT_FLOW_CTL)
    mem->acl_from_ll_count = MYNEWT_VAL(BLE_TRANSPORT_ACL_FROM_LL_COUNT);
#endif

    ipc_init();

    ipc_nrf5340_recv(NET_CRASH_CHANNEL, ipc_on_net_crash, NULL);

    /* Start Network Core */
    NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;

    /*
     * Waits for NET core to start and init it's side of IPC.
     * It may take several seconds if there is net core
     * embedded image in the application flash.
     */
    while (mem->ipc_state != APP_AND_NET_RUNNING);
#else
    ipc_init();
    if (mem->ipc_state == APP_AND_NET_RUNNING) {
        mem->ipc_state = NET_RESTARTED;
        NRF_IPC->TASKS_SEND[NET_CRASH_CHANNEL] = 1;
        while (mem->ipc_state == NET_RESTARTED) ;
    }
    mem->ipc_state = APP_AND_NET_RUNNING;
#endif
}

#if MYNEWT_VAL(MCU_APP_CORE)
void ipc_nrf5340_set_net_core_restart_cb(void (*on_restart)(void))
{
    net_core_restart_cb = on_restart;
}
#endif

int ipc_nrf5340_net_online(void) {
    return (mem->ipc_state == APP_AND_NET_RUNNING);
}

void ipc_nrf5340_recv(int channel, ipc_nrf5340_recv_cb cb, void *user_data) {
        ipcs[channel].cb = cb;
        ipcs[channel].user_data = user_data;
        NRF_IPC->RECEIVE_CNF[channel] = (0x1UL << channel);
        NRF_IPC->INTENSET = (0x1UL << channel);
}

int ipc_nrf5340_send(int channel, const void *data, uint16_t len) {
    ringbuf_lf* ring = &mem->ring[channel];
#ifdef LOG_IPC
    if (channel != 3) {
        // TODO locking
        uint32_t log_idx = mem->log_idx;
        uint32_t req_len = len + sizeof(log_ipc_hdr_t);
        if (mem->notify[channel]) req_len++;
        if (log_idx + req_len < 2048) { // otherwise drop
            mem->log_idx += req_len;
            log_ipc_hdr_t* hdr = (log_ipc_hdr_t*)&mem->log_buf[log_idx];
            hdr->chan = channel;
            hdr->dlen = len;
            if (mem->notify[channel]) {
                hdr->data[0] = mem->notify_data[channel];
                memcpy(hdr->data+1, data, len);
                hdr->dlen++;
                mem->notify[channel] = 0;
            } else {
                memcpy(hdr->data, data, len);
            }
        }
    }
#endif
    while (len) {
        uint32_t avail = ringbuf_lf_get_free(ring);
        if (avail < 64) continue;   // blocking wait

        uint32_t block = min(len, avail);
        uint32_t inserted = ringbuf_lf_add(ring, data, block);
        len -= inserted;
        data += inserted;
        NRF_IPC->TASKS_SEND[channel] = 1;
    }
    return 0;
}

void ipc_nrf5340_send_no_notify(int channel, uint8_t data) {
    ringbuf_lf* ring = &mem->ring[channel];
#ifdef LOG_IPC
    if (channel != 3) {
        // just store single byte
        mem->notify[channel] = 1;
        mem->notify_data[channel] = data;
    }
#endif
    // TODO can simplify
    uint16_t len = 1;
    while (len) {
        uint32_t avail = ringbuf_lf_get_free(ring);
        if (avail < 64) {
            NRF_IPC->TASKS_SEND[channel] = 1;
            continue;   // blocking wait
        }

        uint32_t block = min(len, avail);
        uint32_t inserted = ringbuf_lf_add(ring, &data, block);
        len -= inserted;
        data += inserted;
    }
}

uint16_t ipc_nrf5340_available_buf(int channel, void **dptr) {
    // NOTE: only return biggest unwrapped buffer size since caller will read linearly!!
    const ringbuf_lf* rb = &mem->ring[channel];

    uint32_t head = rb->head;    // write-index
    uint32_t tail = rb->tail;    // read-index

    *dptr = &rb->buf[tail];

    if (head >= tail) return head - tail;

    return rb->bufsize - tail;  // only unwrapped part
}

uint16_t ipc_nrf5340_avail(int channel) {
    return ringbuf_lf_get_count(&mem->ring[channel]);
}

uint16_t ipc_nrf5340_consume(int channel, uint16_t len) {
    return ringbuf_lf_consume(&mem->ring[channel], len);
}

uint16_t ipc_nrf5340_free_space(int channel) {
    return ringbuf_lf_get_free(&mem->ring[channel]);
}

#if MYNEWT_VAL(BLE_TRANSPORT_INT_FLOW_CTL)
int
ble_transport_int_flow_ctl_get(void)
{
    int ret;

    __asm__ volatile (".syntax unified                \n"
                      "1: ldrexb r1, [%[addr]]        \n"
                      "   mov %[ret], r1              \n"
                      "   cmp r1, #0                  \n"
                      "   itte ne                     \n"
                      "   subne r2, r1, #1            \n"
                      "   strexbne r1, r2, [%[addr]]  \n"
                      "   clrexeq                     \n"
                      "   cmp r1, #0                  \n"
                      "   bne 1b                      \n"
                      : [ret] "=&r" (ret)
                      : [addr] "r"(&mem->acl_from_ll_count)
                      : "r1", "r2", "memory");

    return ret;
}

void
ble_transport_int_flow_ctl_put(void)
{
    __asm__ volatile (".syntax unified              \n"
                      "1: ldrexb r1, [%[addr]]      \n"
                      "   add r1, r1, #1            \n"
                      "   strexb r2, r1, [%[addr]]  \n"
                      "   cmp r2, #0                \n"
                      "   bne 1b                    \n"
                      :
                      : [addr] "r"(&mem->acl_from_ll_count)
                      : "r1", "r2", "memory");
}

#endif

void ipc_nrf5340_get_log(uint32_t* len, uint8_t** data) {
#ifdef LOG_IPC
    *len = mem->log_idx;
    *data = mem->log_buf;
#else
    *len = 0;
    *data = NULL;
#endif
}
