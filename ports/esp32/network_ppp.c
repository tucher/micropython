/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 "Eric Poulsen" <eric@zyxod.com>
 * Copyright (c) 2024 Damien P. George
 *
 * Based on the ESP IDF example code which is Public Domain / CC0
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// This file is intended to closely match extmod/network_ppp_lwip.c. Changes can
// and should probably be applied to both files. Compare them directly by using:
// git diff --no-index extmod/network_ppp_lwip.c ports/esp32/network_ppp.c

#include "py/runtime.h"
#include "py/mphal.h"
#include "py/stream.h"

#if defined(CONFIG_ESP_NETIF_TCPIP_LWIP) && defined(CONFIG_LWIP_PPP_SUPPORT)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/netifapi.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppapi.h"
#include "netif/ppp/pppos.h"

// Includes for port-specific changes compared to network_ppp_lwip.c
#include "shared/netutils/netutils.h"
#include "ppp_set_auth.h"

// Enable to trace what network_ppp_deinit_all() does at soft reset.
#define NETWORK_PPP_DEINIT_TRACE (0)

// Smallest C writer worth using. pppos builds its output in a buffer of
// PBUF_POOL_BUFSIZE and flushes it to the output callback when it fills or the
// frame ends, so that is the largest single write it can ask for -- measured
// at 1446 bytes while uploading, against 129 while downloading, where all we
// send is ACKs. A short write is an error to pppos and the frame is dropped,
// so a writer that cannot take that much loses every outbound data frame
// while a download still looks fine. Refuse it and use the blocking path.
#define NETWORK_PPP_MIN_C_WRITER (PBUF_POOL_BUFSIZE)

// Enable this to see the serial data going between the PPP layer.
#define PPP_TRACE_IN_OUT (0)

typedef enum {
    STATE_INACTIVE,
    STATE_ACTIVE,
    STATE_ERROR,
    STATE_CONNECTING,
    STATE_CONNECTED,
} network_ppp_state_t;

typedef struct _network_ppp_obj_t {
    mp_obj_base_t base;
    network_ppp_state_t state;
    int error_code;
    mp_obj_t stream;
    ppp_pcb *pcb;
    struct netif netif;
    // Live sessions are kept on a list so that shutdown can find them; see
    // network_ppp_deinit_all(). lwIP's global netif_list holds a pointer to the
    // `netif` member above and its callbacks hold `self`, neither of which the
    // GC can see, so the object's lifetime has to be managed explicitly rather
    // than left to reachability.
    //
    // In ordinary operation the object happens to be reachable anyway, via
    // machine_uart_objs -> uart -> mp_irq_obj -> handler (a bound method of
    // this object). That protection disappears when the stream IRQ is detached
    // -- config(stream=None) -- and it is irrelevant at soft reset, where
    // gc_sweep_all() frees everything regardless of reachability. Making the
    // list a GC root closes both cases.
    //
    // Entries are reaped lazily (see network_ppp_reap) because the status
    // callback that clears `pcb` runs on the lwIP thread and must not mutate
    // the list.
    struct _network_ppp_obj_t *next;
    bool listed;
    // True when the stream accepted MP_STREAM_SET_DATA_SINK, i.e. incoming data
    // is delivered straight to C instead of through the MicroPython scheduler.
    bool c_sink_installed;
    // VM-free, non-blocking writer obtained from the stream, used by the PPP
    // output callback so that lwIP's tcpip thread never enters the VM and never
    // waits on the UART. Zeroed when unavailable; then the mp_stream_rw()
    // fallback below is used.
    mp_stream_c_writer_t c_writer;
    // True when the fallback path installed a Python-level stream IRQ. Tracked
    // so that detaching is idempotent: network_ppp_stream_uart_irq_disable() is
    // also reached from the PPP status callback, which runs on the lwIP thread,
    // and calling the stream's .irq() method from there enters the VM and can
    // allocate GC memory. See the note above network_ppp_status_cb().
    bool py_irq_installed;
    // Set from the lwIP thread to stop the C sink feeding a pcb that is on its
    // way out. It is the only thing that callback is allowed to touch: writing
    // our own flag needs neither the VM nor another GC object, where reaching
    // the stream to detach the sink needed both. See network_ppp_status_cb().
    volatile bool sink_muted;
    // Set by the status callback once lwIP has taken the session to the dead
    // phase, which is the only phase in which ppp_free() may be called. The
    // callback no longer frees the pcb itself -- see network_ppp_teardown().
    volatile bool pcb_dead;
} network_ppp_obj_t;

static void network_ppp_list_add(network_ppp_obj_t *self) {
    if (self->listed) {
        return;
    }
    self->next = MP_STATE_PORT(network_ppp_obj_head);
    MP_STATE_PORT(network_ppp_obj_head) = self;
    self->listed = true;
}

// Drop finished sessions from the root list so they can be collected again.
// Only ever called from the MicroPython thread.
static void network_ppp_reap(void) {
    network_ppp_obj_t **link = &MP_STATE_PORT(network_ppp_obj_head);
    while (*link != NULL) {
        network_ppp_obj_t *self = *link;
        if (self->pcb == NULL) {
            *link = self->next;
            self->next = NULL;
            self->listed = false;
        } else {
            link = &self->next;
        }
    }
}

const mp_obj_type_t esp_network_ppp_lwip_type;

static void network_ppp_teardown(network_ppp_obj_t *self, bool shutdown);

// Receive path when the stream supports a C sink. Runs on the stream's own
// task (for machine.UART: uart_event_task), never on the MicroPython thread, so
// PPP keeps receiving even while the VM is blocked in a C call. Must not touch
// the VM. pppos_input_tcpip() is the thread-safe lwIP entry point.
static void network_ppp_data_sink(void *ctx, const uint8_t *buf, size_t len) {
    network_ppp_obj_t *self = ctx;
    if (self->sink_muted) {
        return;
    }
    ppp_pcb *pcb = self->pcb;
    if (pcb != NULL && self->state >= STATE_ACTIVE) {
        pppos_input_tcpip(pcb, (u8_t *)buf, len);
    }
}

// Returns true if the stream took the sink configuration.
static bool network_ppp_set_data_sink(network_ppp_obj_t *self, bool enable) {
    const mp_stream_p_t *stream_p = mp_get_stream(self->stream);
    if (stream_p == NULL || stream_p->ioctl == NULL) {
        return false;
    }
    mp_stream_data_sink_cfg_t cfg = {
        .handler = enable ? network_ppp_data_sink : NULL,
        .ctx = enable ? self : NULL,
    };
    int errcode = 0;
    mp_uint_t res = stream_p->ioctl(self->stream, MP_STREAM_SET_DATA_SINK, (uintptr_t)&cfg, &errcode);
    return res != MP_STREAM_ERROR;
}

static void network_ppp_stream_uart_irq_disable(network_ppp_obj_t *self) {
    if (self->stream == mp_const_none) {
        return;
    }

    self->c_writer.write = NULL;
    self->c_writer.ctx = NULL;
    self->c_writer.max_write = 0;

    if (self->c_sink_installed) {
        network_ppp_set_data_sink(self, false);
        self->c_sink_installed = false;
        return;
    }

    if (!self->py_irq_installed) {
        return;
    }
    self->py_irq_installed = false;

    // Disable UART IRQ.
    mp_obj_t dest[3];
    mp_load_method(self->stream, MP_QSTR_irq, dest);
    dest[2] = mp_const_none;
    mp_call_method_n_kw(1, 0, dest);
}

// NOTE: this runs on the lwIP tcpip thread, not the MicroPython thread, so it
// must not enter the VM or touch another GC object. It used to call
// network_ppp_stream_uart_irq_disable() here, which reached the stream through
// mp_get_stream() and then blocked this thread in the UART's ioctl waiting for
// an in-flight sink callback -- both wrong from here, and the second one stalls
// all of lwIP. It now only sets sink_muted, which is our own memory; the actual
// detach happens on the MicroPython thread, where every other caller of
// network_ppp_stream_uart_irq_disable() already lives.
static void network_ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx) {
    (void)pcb;
    network_ppp_obj_t *self = ctx;
    switch (err_code) {
        case PPPERR_NONE:
            self->state = STATE_CONNECTED;
            break;
        case PPPERR_USER:
            if (self->state >= STATE_ERROR) {
                // Indicate that we are no longer connected and thus
                // only need to free the PPP PCB, not close it.
                self->state = STATE_ACTIVE;
            }
            // Mark, do not free. Freeing here means freeing on the lwIP
            // thread while the C sink may already have read self->pcb and be
            // about to hand it to pppos_input_tcpip() -- and since that call
            // queues a message, even one queued earlier would land after the
            // free. lwIP's own contract says ppp_free() is the final destroy
            // and a closed pcb may simply be reconnected, so there is no
            // reason to do it here. network_ppp_teardown() does it, on the
            // MicroPython thread, after detaching the sink.
            self->sink_muted = true;
            self->pcb_dead = true;
            break;
        default:
            self->state = STATE_ERROR;
            self->error_code = err_code;
            break;
    }
}

static mp_obj_t network_ppp_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    mp_obj_t stream = all_args[0];

    if (stream != mp_const_none) {
        mp_get_stream_raise(stream, MP_STREAM_OP_READ | MP_STREAM_OP_WRITE);
    }

    network_ppp_reap();

    network_ppp_obj_t *self = mp_obj_malloc_with_finaliser(network_ppp_obj_t, type);
    self->state = STATE_INACTIVE;
    self->stream = stream;
    self->pcb = NULL;
    self->next = NULL;
    self->listed = false;
    self->c_sink_installed = false;
    self->py_irq_installed = false;
    self->sink_muted = false;
    self->pcb_dead = false;
    self->c_writer.write = NULL;
    self->c_writer.ctx = NULL;
    self->c_writer.max_write = 0;

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t network_ppp___del__(mp_obj_t self_in) {
    // Also the GC finaliser. That is safe only because a session holding a pcb
    // is on the root list and so never collected, and because the soft-reset
    // path runs network_ppp_deinit_all() before gc_sweep_all() -- so by the
    // time a finaliser sees this object the pcb is already gone and teardown
    // returns at once.
    network_ppp_teardown(MP_OBJ_TO_PTR(self_in), false);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(network_ppp___del___obj, network_ppp___del__);

static mp_obj_t network_ppp_poll(size_t n_args, const mp_obj_t *args) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (self->state <= STATE_ERROR) {
        return MP_OBJ_NEW_SMALL_INT(-MP_EPERM);
    }

    mp_int_t total_len = 0;
    mp_obj_t stream;
    while (self->state >= STATE_ACTIVE && (stream = self->stream) != mp_const_none) {
        uint8_t buf[256];
        int err;
        mp_uint_t len = mp_stream_rw(stream, buf, sizeof(buf), &err, 0);
        if (len == 0) {
            break;
        }
        #if PPP_TRACE_IN_OUT
        mp_printf(&mp_plat_print, "ppp_in(n=%u,data=", len);
        for (size_t i = 0; i < len; ++i) {
            mp_printf(&mp_plat_print, "%02x:", buf[i]);
        }
        mp_printf(&mp_plat_print, ")\n");
        #endif
        pppos_input_tcpip(self->pcb, (u8_t *)buf, len);
        total_len += len;
    }

    return MP_OBJ_NEW_SMALL_INT(total_len);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_ppp_poll_obj, 1, 2, network_ppp_poll);

static void network_ppp_stream_uart_irq_enable(network_ppp_obj_t *self) {
    if (self->stream == mp_const_none) {
        return;
    }

    // Preferred: a C-level sink, so the data path does not depend on the
    // MicroPython scheduler getting a turn.
    if (network_ppp_set_data_sink(self, true)) {
        self->c_sink_installed = true;
        self->sink_muted = false;
        // Ask for the matching outbound path. Optional: a stream may support
        // the sink but not the writer (e.g. a UART with no TX ring buffer).
        const mp_stream_p_t *stream_p = mp_get_stream(self->stream);
        mp_stream_c_writer_t writer = { NULL, NULL, 0 };
        int errcode = 0;
        if (stream_p != NULL && stream_p->ioctl != NULL
            && stream_p->ioctl(self->stream, MP_STREAM_GET_C_WRITER,
                (uintptr_t)&writer, &errcode) != MP_STREAM_ERROR
            && writer.max_write >= NETWORK_PPP_MIN_C_WRITER) {
            self->c_writer = writer;
        }
        // Otherwise c_writer stays empty and network_ppp_output_callback()
        // uses the stream protocol, which blocks but does not lose frames.
        return;
    }

    // Fallback for streams without that support: soft IRQ calling PPP.poll().
    mp_obj_t dest[4];
    mp_load_method(self->stream, MP_QSTR_irq, dest);
    dest[2] = mp_obj_new_bound_meth(MP_OBJ_FROM_PTR(&network_ppp_poll_obj), MP_OBJ_FROM_PTR(self));
    dest[3] = mp_load_attr(self->stream, MP_QSTR_IRQ_RXIDLE);
    mp_call_method_n_kw(2, 0, dest);
    self->py_irq_installed = true;
}

static mp_obj_t network_ppp_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    if (n_args != 1 && kwargs->used != 0) {
        mp_raise_TypeError(MP_ERROR_TEXT("either pos or kw args are allowed"));
    }
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (kwargs->used != 0) {
        for (size_t i = 0; i < kwargs->alloc; i++) {
            if (mp_map_slot_is_filled(kwargs, i)) {
                switch (mp_obj_str_get_qstr(kwargs->table[i].key)) {
                    case MP_QSTR_stream: {
                        if (kwargs->table[i].value != mp_const_none) {
                            mp_get_stream_raise(kwargs->table[i].value, MP_STREAM_OP_READ | MP_STREAM_OP_WRITE);
                        }
                        if (self->state >= STATE_ACTIVE) {
                            network_ppp_stream_uart_irq_disable(self);
                        }
                        self->stream = kwargs->table[i].value;
                        if (self->state >= STATE_ACTIVE) {
                            network_ppp_stream_uart_irq_enable(self);
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
        }
        return mp_const_none;
    }

    if (n_args != 2) {
        mp_raise_TypeError(MP_ERROR_TEXT("can query only one param"));
    }

    mp_obj_t val = mp_const_none;

    switch (mp_obj_str_get_qstr(args[1])) {
        case MP_QSTR_stream: {
            val = self->stream;
            break;
        }
        case MP_QSTR_ifname: {
            if (self->pcb != NULL) {
                struct netif *pppif = ppp_netif(self->pcb);
                char ifname[NETIF_NAMESIZE + 1] = {0};
                netif_index_to_name(netif_get_index(pppif), ifname);
                if (ifname[0] != 0) {
                    val = mp_obj_new_str_from_cstr((char *)ifname);
                }
            }
            break;
        }
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
    }

    return val;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(network_ppp_config_obj, 1, network_ppp_config);

static mp_obj_t network_ppp_status(mp_obj_t self_in) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->state == STATE_ERROR) {
        return MP_OBJ_NEW_SMALL_INT(-self->error_code);
    } else {
        return MP_OBJ_NEW_SMALL_INT(self->state);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_1(network_ppp_status_obj, network_ppp_status);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
static u32_t network_ppp_output_callback(ppp_pcb *pcb, const void *data, u32_t len, void *ctx)
#else
static u32_t network_ppp_output_callback(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
#endif
{
    network_ppp_obj_t *self = ctx;
    #if PPP_TRACE_IN_OUT
    mp_printf(&mp_plat_print, "ppp_out(n=%u,data=", len);
    for (size_t i = 0; i < len; ++i) {
        mp_printf(&mp_plat_print, "%02x:", ((const uint8_t *)data)[i]);
    }
    mp_printf(&mp_plat_print, ")\n");
    #endif
    // Preferred: write without entering the VM and without blocking. This
    // callback runs on the lwIP tcpip thread; blocking here stalls the whole
    // stack, and calling the stream protocol from this context is the defect
    // behind micropython#16701. A short return is read by lwIP as an error and
    // the frame is dropped, which is the correct meaning of a congested link.
    if (self->c_writer.write != NULL) {
        return self->c_writer.write(self->c_writer.ctx, (const uint8_t *)data, len);
    }

    mp_obj_t stream = self->stream;
    if (stream == mp_const_none) {
        return 0;
    }
    int err;
    // The return value from this output callback is the number of bytes written out.
    // If it's less than the requested number of bytes then lwIP will propagate out an error.
    return mp_stream_rw(stream, (void *)data, len, &err, MP_STREAM_RW_WRITE);
}

static mp_obj_t network_ppp_connect(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    enum { ARG_security, ARG_user, ARG_key, ARG_authmode, ARG_username, ARG_password };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_user, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_key, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        // Deprecated arguments for backwards compatibility
        { MP_QSTR_authmode, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = PPPAUTHTYPE_NONE} },
        { MP_QSTR_username, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_password, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
    };

    mp_arg_val_t parsed_args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed_args);

    // Use deprecated arguments as defaults
    if (parsed_args[ARG_security].u_int == -1) {
        parsed_args[ARG_security].u_int = parsed_args[ARG_authmode].u_int;
    }
    if (parsed_args[ARG_user].u_obj == mp_const_none) {
        parsed_args[ARG_user].u_obj = parsed_args[ARG_username].u_obj;
    }
    if (parsed_args[ARG_key].u_obj == mp_const_none) {
        parsed_args[ARG_key].u_obj = parsed_args[ARG_password].u_obj;
    }

    network_ppp_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (self->state == STATE_INACTIVE) {
        self->pcb = pppapi_pppos_create(&self->netif, network_ppp_output_callback, network_ppp_status_cb, self);
        if (self->pcb == NULL) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("pppos_create failed"));
        }
        // From here on lwIP holds a pointer to self->netif: keep the object alive.
        network_ppp_list_add(self);
        self->pcb_dead = false;
        self->state = STATE_ACTIVE;

        network_ppp_stream_uart_irq_enable(self);
    }

    if (self->state == STATE_CONNECTING || self->state == STATE_CONNECTED) {
        mp_raise_OSError(MP_EALREADY);
    }

    switch (parsed_args[ARG_security].u_int) {
        case PPPAUTHTYPE_NONE:
        case PPPAUTHTYPE_PAP:
        case PPPAUTHTYPE_CHAP:
            break;
        default:
            mp_raise_ValueError(MP_ERROR_TEXT("invalid auth"));
    }

    if (parsed_args[ARG_security].u_int != PPPAUTHTYPE_NONE) {
        const char *user_str = mp_obj_str_get_str(parsed_args[ARG_user].u_obj);
        const char *key_str = mp_obj_str_get_str(parsed_args[ARG_key].u_obj);
        pppapi_set_auth(self->pcb, parsed_args[ARG_security].u_int, user_str, key_str);
    }

    if (pppapi_set_default(self->pcb) != ERR_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ppp_set_default failed"));
    }

    ppp_set_usepeerdns(self->pcb, true);

    if (pppapi_connect(self->pcb, 0) != ERR_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ppp_connect failed"));
    }

    self->state = STATE_CONNECTING;

    if (!self->c_sink_installed) {
        // Do a poll in case there is data waiting on the input stream.
        // With a C sink the stream's own task is already draining it.
        network_ppp_poll(1, args);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(network_ppp_connect_obj, 1, network_ppp_connect);

static mp_obj_t network_ppp_disconnect(mp_obj_t self_in) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->state == STATE_CONNECTING || self->state == STATE_CONNECTED) {
        // Initiate close and wait for PPPERR_USER callback.
        pppapi_close(self->pcb, 0);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(network_ppp_disconnect_obj, network_ppp_disconnect);

static mp_obj_t network_ppp_isconnected(mp_obj_t self_in) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->state == STATE_CONNECTED);
}
static MP_DEFINE_CONST_FUN_OBJ_1(network_ppp_isconnected_obj, network_ppp_isconnected);

static mp_obj_t network_ppp_ifconfig(size_t n_args, const mp_obj_t *args) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 1) {
        // get
        const ip_addr_t *dns;
        if (self->pcb != NULL) {
            dns = dns_getserver(0);
            struct netif *pppif = ppp_netif(self->pcb);
            mp_obj_t tuple[4] = {
                netutils_format_ipv4_addr((uint8_t *)&pppif->ip_addr, NETUTILS_BIG),
                netutils_format_ipv4_addr((uint8_t *)&pppif->gw, NETUTILS_BIG),
                netutils_format_ipv4_addr((uint8_t *)&pppif->netmask, NETUTILS_BIG),
                netutils_format_ipv4_addr((uint8_t *)dns, NETUTILS_BIG),
            };
            return mp_obj_new_tuple(4, tuple);
        } else {
            mp_obj_t tuple[4] = { mp_const_none, mp_const_none, mp_const_none, mp_const_none };
            return mp_obj_new_tuple(4, tuple);
        }
    } else {
        ip_addr_t dns;
        mp_obj_t *items;
        mp_obj_get_array_fixed_n(args[1], 4, &items);
        #if CONFIG_LWIP_IPV6
        netutils_parse_ipv4_addr(items[3], (uint8_t *)&dns.u_addr.ip4, NETUTILS_BIG);
        #else
        netutils_parse_ipv4_addr(items[3], (uint8_t *)&dns, NETUTILS_BIG);
        #endif // CONFIG_LWIP_IPV6
        dns_setserver(0, &dns);
        return mp_const_none;
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_ppp_ifconfig_obj, 1, 2, network_ppp_ifconfig);

static mp_obj_t network_ppp_ipconfig(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (kwargs->used == 0) {
        if (self->pcb == NULL) {
            mp_raise_ValueError(MP_ERROR_TEXT("PPP not active"));
        }
        struct netif *netif = ppp_netif(self->pcb);
        // Get config value
        if (n_args != 2) {
            mp_raise_TypeError(MP_ERROR_TEXT("must query one param"));
        }

        switch (mp_obj_str_get_qstr(args[1])) {
            case MP_QSTR_addr4: {
                mp_obj_t tuple[2] = {
                    netutils_format_ipv4_addr((uint8_t *)&netif->ip_addr, NETUTILS_BIG),
                    netutils_format_ipv4_addr((uint8_t *)&netif->netmask, NETUTILS_BIG),
                };
                return mp_obj_new_tuple(2, tuple);
            }
            case MP_QSTR_gw4: {
                return netutils_format_ipv4_addr((uint8_t *)&netif->gw, NETUTILS_BIG);
            }
            default: {
                mp_raise_ValueError(MP_ERROR_TEXT("unexpected key"));
                break;
            }
        }
        return mp_const_none;
    } else {
        mp_raise_TypeError(MP_ERROR_TEXT("setting properties not supported"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(network_ppp_ipconfig_obj, 1, network_ppp_ipconfig);

static mp_obj_t network_ppp_active(size_t n_args, const mp_obj_t *args) {
    network_ppp_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args > 1) {
        if (mp_obj_is_true(args[1])) {
            if (self->state >= STATE_ACTIVE) {
                return mp_const_true;
            }

            self->pcb = pppapi_pppos_create(&self->netif, network_ppp_output_callback, network_ppp_status_cb, self);
            if (self->pcb == NULL) {
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("pppos_create failed"));
            }
            // lwIP now holds a pointer to self->netif; keep the object alive.
            network_ppp_list_add(self);
            self->pcb_dead = false;
            self->state = STATE_ACTIVE;

            network_ppp_stream_uart_irq_enable(self);
        } else {
            if (self->state < STATE_ACTIVE) {
                return mp_const_false;
            }

            network_ppp___del__(MP_OBJ_FROM_PTR(self));
            network_ppp_reap();
        }
    }
    return mp_obj_new_bool(self->state >= STATE_ACTIVE);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_ppp_active_obj, 1, 2, network_ppp_active);

// Tear one session down far enough that lwIP no longer references memory owned
// by the MicroPython heap. Must not raise: it runs from the soft-reset path,
// outside any nlr handler.
// The tcpip mailbox is FIFO, so once a message posted here has been processed,
// everything the sink queued ahead of it has been too. The sequence number
// makes a timed-out barrier harmless: its late callback publishes a number no
// later wait is looking for. Only ever used from the MicroPython thread.
static volatile uint32_t network_ppp_barrier_seq;
static volatile uint32_t network_ppp_barrier_done;

static void network_ppp_barrier_cb(void *ctx) {
    network_ppp_barrier_done = (uint32_t)(uintptr_t)ctx;
}

static void network_ppp_drain_tcpip(void) {
    uint32_t seq = ++network_ppp_barrier_seq;
    if (tcpip_callback(network_ppp_barrier_cb, (void *)(uintptr_t)seq) != ERR_OK) {
        // Could not post. Pausing is a poor substitute for the barrier, but it
        // beats freeing immediately behind input that may still be queued.
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
    }
    for (int i = 0; i < 200 && network_ppp_barrier_done != seq; ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// The one place a pcb is freed. Runs on the MicroPython thread, where blocking
// is allowed, which is what makes the order below possible:
//
//   detach the sink   -- the UART ioctl waits for a callback already running,
//                        so afterwards nothing new can be queued;
//   drain tcpip       -- so nothing queued earlier is still in flight;
//   free.
//
// `shutdown` is for the soft-reset path, which must also stop the output
// callback reaching a VM that is being torn down. Must not raise: it is called
// from outside any nlr handler.
static void network_ppp_teardown(network_ppp_obj_t *self, bool shutdown) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        network_ppp_stream_uart_irq_disable(self);
        nlr_pop();
    }
    if (shutdown) {
        self->stream = mp_const_none;
    }

    if (self->pcb == NULL) {
        self->state = STATE_INACTIVE;
        return;
    }

    if (self->state >= STATE_ERROR && !self->pcb_dead) {
        // nocarrier=1: do not wait for the peer. The status callback marks the
        // session dead; it does not free it.
        pppapi_close(self->pcb, 1);
        for (int i = 0; i < 200 && !self->pcb_dead; ++i) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    network_ppp_drain_tcpip();

    if (pppapi_free(self->pcb) == ERR_OK) {
        self->pcb = NULL;
    } else {
        // Wedged. Leaking the pcb is survivable; leaving a netif that points
        // into the MicroPython heap in lwIP's global list is not.
        netifapi_netif_remove(&self->netif);
        #if NETWORK_PPP_DEINIT_TRACE
        mp_printf(&mp_plat_print, "[ppp] teardown: forced netif_remove\n");
        #endif
    }
    #if NETWORK_PPP_DEINIT_TRACE
    mp_printf(&mp_plat_print, "[ppp] teardown: pcb=%p state=%d\n", self->pcb, self->state);
    #endif
    self->pcb_dead = false;
    self->state = STATE_INACTIVE;
}

// Called from the soft-reset path, BEFORE machine_uart_deinit_all(): detaching
// the stream IRQ needs the UART object to still be usable.
void network_ppp_deinit_all(void) {
    network_ppp_obj_t *self = MP_STATE_PORT(network_ppp_obj_head);
    MP_STATE_PORT(network_ppp_obj_head) = NULL;
    #if NETWORK_PPP_DEINIT_TRACE
    int n = 0;
    for (network_ppp_obj_t *t = self; t; t = t->next) {
        n++;
    }
    mp_printf(&mp_plat_print, "[ppp] deinit_all: %d live session(s)\n", n);
    #endif
    while (self != NULL) {
        network_ppp_obj_t *next = self->next;
        self->next = NULL;
        self->listed = false;
        network_ppp_teardown(self, true);
        self = next;
    }
}

static const mp_rom_map_elem_t network_ppp_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&network_ppp___del___obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_ppp_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_ppp_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&network_ppp_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&network_ppp_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_ppp_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig), MP_ROM_PTR(&network_ppp_ifconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipconfig), MP_ROM_PTR(&network_ppp_ipconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll), MP_ROM_PTR(&network_ppp_poll_obj) },

    { MP_ROM_QSTR(MP_QSTR_SEC_NONE), MP_ROM_INT(PPPAUTHTYPE_NONE) },
    { MP_ROM_QSTR(MP_QSTR_SEC_PAP), MP_ROM_INT(PPPAUTHTYPE_PAP) },
    { MP_ROM_QSTR(MP_QSTR_SEC_CHAP), MP_ROM_INT(PPPAUTHTYPE_CHAP) },

    // Deprecated interface for backwards compatibility
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_ppp_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_AUTH_NONE), MP_ROM_INT(PPPAUTHTYPE_NONE) },
    { MP_ROM_QSTR(MP_QSTR_AUTH_PAP), MP_ROM_INT(PPPAUTHTYPE_PAP) },
    { MP_ROM_QSTR(MP_QSTR_AUTH_CHAP), MP_ROM_INT(PPPAUTHTYPE_CHAP) },
};
static MP_DEFINE_CONST_DICT(network_ppp_locals_dict, network_ppp_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    esp_network_ppp_lwip_type,
    MP_QSTR_PPP,
    MP_TYPE_FLAG_NONE,
    make_new, network_ppp_make_new,
    locals_dict, &network_ppp_locals_dict
    );

MP_REGISTER_ROOT_POINTER(struct _network_ppp_obj_t *network_ppp_obj_head);

#endif
