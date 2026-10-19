/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 * and Mnemote Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016, 2017 Nick Moore @mnemote
 *
 * Based on extmod/modlwip.c
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Galen Hazelwood
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "py/gc.h"
#include "py/runtime0.h"
#include "py/nlr.h"
#include "py/objlist.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "shared/netutils/netutils.h"
#include "modnetwork.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/ip4.h"
#include "lwip/igmp.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"
#include "esp_log.h"

// See note at bottom of file about why this isn't MICROPY_PY_SOCKET
#if MICROPY_PY_NETWORK

// How long a blocking socket call waits before coming up for air.
//
// This is not a timeout and it does not decide when data is noticed. The socket
// stays blocking and the slice is imposed with SO_RCVTIMEO, so lwIP returns the
// instant data arrives; the timeout itself is tracked against the clock (see
// socket_timed_out). All this period decides is how promptly a thread sitting
// in recv() notices what arrives through mp_handle_pending(): a
// KeyboardInterrupt, a callback scheduled from an ISR by micropython.schedule(),
// an exception scheduled from C.
//
// It is why Ctrl-C out of a blocked recv() can take up to a tenth of a second
// at the default. A port or board may shorten it: the cost is small, because
// a blocked socket sleeps inside lwIP rather than spinning.
#ifndef MICROPY_PY_SOCKET_POLL_PERIOD_MS
#define MICROPY_PY_SOCKET_POLL_PERIOD_MS (100)
#endif
#define SOCKET_POLL_US (MICROPY_PY_SOCKET_POLL_PERIOD_MS * 1000)
#define MDNS_QUERY_TIMEOUT_MS (5000)
#define MDNS_LOCAL_SUFFIX ".local"

#ifndef NO_QSTR
#include "mdns.h"
#endif

enum {
    SOCKET_STATE_NEW,
    SOCKET_STATE_CONNECTED,
    SOCKET_STATE_PEER_CLOSED,
};

typedef struct _socket_obj_t {
    mp_obj_base_t base;
    int fd;
    uint8_t domain;
    uint8_t type;
    uint8_t proto;
    uint8_t state;
    // 0 = non-blocking, UINT64_MAX = block indefinitely, otherwise the
    // caller's timeout in milliseconds. Kept as the caller gave it rather
    // than as a retry count, so it can actually be honoured.
    uint64_t timeout_ms;
    // Set while SO_RCVTIMEO holds a shortened final slice rather than the full
    // polling period; the next blocking call puts the period back.
    bool slice_trimmed;
    #if MICROPY_PY_SOCKET_EVENTS
    mp_obj_t events_callback;
    struct _socket_obj_t *events_next;
    #endif
} socket_obj_t;

static const char *TAG = "modsocket";

void _socket_settimeout(socket_obj_t *sock, uint64_t timeout_ms);

#if MICROPY_PY_SOCKET_EVENTS
// Support for callbacks on asynchronous socket events (when socket becomes readable)

// This divisor is used to reduce the load on the system, so it doesn't poll sockets too often
#define USOCKET_EVENTS_DIVISOR (8)

static uint8_t socket_events_divisor;
static socket_obj_t *socket_events_head;

void socket_events_deinit(void) {
    socket_events_head = NULL;
}

// Assumes the socket is not already in the linked list, and adds it
static void socket_events_add(socket_obj_t *sock) {
    sock->events_next = socket_events_head;
    socket_events_head = sock;
}

// Assumes the socket is already in the linked list, and removes it
static void socket_events_remove(socket_obj_t *sock) {
    for (socket_obj_t **s = &socket_events_head;; s = &(*s)->events_next) {
        if (*s == sock) {
            *s = (*s)->events_next;
            return;
        }
    }
}

// Polls all registered sockets for readability and calls their callback if they are readable
void socket_events_handler(void) {
    if (socket_events_head == NULL) {
        return;
    }
    if (--socket_events_divisor) {
        return;
    }
    socket_events_divisor = USOCKET_EVENTS_DIVISOR;

    fd_set rfds;
    FD_ZERO(&rfds);
    int max_fd = 0;

    for (socket_obj_t *s = socket_events_head; s != NULL; s = s->events_next) {
        FD_SET(s->fd, &rfds);
        max_fd = MAX(max_fd, s->fd);
    }

    // Poll the sockets
    struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
    int r = lwip_select(max_fd + 1, &rfds, NULL, NULL, &timeout);
    if (r <= 0) {
        return;
    }

    // Call the callbacks
    for (socket_obj_t *s = socket_events_head; s != NULL; s = s->events_next) {
        if (FD_ISSET(s->fd, &rfds)) {
            mp_call_function_1_protected(s->events_callback, s);
        }
    }
}

#endif // MICROPY_PY_SOCKET_EVENTS

static inline void check_for_exceptions(void) {
    mp_handle_pending(MP_HANDLE_PENDING_CALLBACKS_AND_EXCEPTIONS);
}

// Blocking operations poll in SOCKET_POLL_US slices so that a pending
// KeyboardInterrupt is noticed, but the caller's timeout is tracked separately
// against the clock. Deriving a retry count from the polling period, as this
// used to, rounded every timeout up to a multiple of it and made any timeout
// shorter than one period indistinguishable from a non-blocking socket.
static bool socket_timed_out(socket_obj_t *sock, mp_uint_t start_ticks) {
    if (sock->timeout_ms == UINT64_MAX) {
        return false;
    }
    if (sock->timeout_ms == 0) {
        return true;    // non-blocking: one attempt is all it gets
    }
    return (uint64_t)(mp_hal_ticks_ms() - start_ticks) >= sock->timeout_ms;
}

static int socket_timeout_errno(socket_obj_t *sock) {
    return sock->timeout_ms == 0 ? MP_EWOULDBLOCK : MP_ETIMEDOUT;
}

static void socket_set_slice_us(socket_obj_t *sock, uint64_t slice_us) {
    struct timeval tv = { .tv_sec = 0, .tv_usec = slice_us };
    lwip_setsockopt(sock->fd, SOL_SOCKET, SO_SNDTIMEO, (const void *)&tv, sizeof(tv));
    lwip_setsockopt(sock->fd, SOL_SOCKET, SO_RCVTIMEO, (const void *)&tv, sizeof(tv));
}

// Called once at the top of each blocking operation: if the previous one left a
// shortened slice behind, restore the full polling period.
static void socket_slice_reset(socket_obj_t *sock) {
    if (sock->slice_trimmed) {
        sock->slice_trimmed = false;
        socket_set_slice_us(sock, SOCKET_POLL_US);
    }
}

// How long the next wait may last: a full polling period, unless less than that
// is left of the caller's timeout. Without the second half the final slice
// overshoots by up to a whole period, which is why a 250 ms timeout used to
// fire at 301 ms.
//
// Never less than a millisecond: lwIP keeps SO_RCVTIMEO and SO_SNDTIMEO in
// whole milliseconds, and reads zero as "wait for ever".
static uint64_t socket_slice_us(socket_obj_t *sock, mp_uint_t start_ticks) {
    uint64_t slice_us = SOCKET_POLL_US;
    if (sock->timeout_ms == 0 || sock->timeout_ms == UINT64_MAX) {
        return slice_us;
    }
    uint64_t elapsed = (uint64_t)(mp_hal_ticks_ms() - start_ticks);
    if (elapsed >= sock->timeout_ms) {
        // The deadline passed since the caller last looked, as when a pending
        // callback ran longer than what was left: one more short wait, and
        // the loop says so.
        return 1000;
    }
    uint64_t remain_us = (sock->timeout_ms - elapsed) * 1000;
    return remain_us < slice_us ? remain_us : slice_us;
}

// For the loops that wait inside lwIP via SO_RCVTIMEO. connect() does not use
// this: it waits in lwip_select() and passes the slice as that call's timeout.
static void socket_slice_trim(socket_obj_t *sock, mp_uint_t start_ticks) {
    uint64_t slice_us = socket_slice_us(sock, start_ticks);
    if (slice_us < SOCKET_POLL_US) {
        sock->slice_trimmed = true;
        socket_set_slice_us(sock, slice_us);
    }
}

#if MICROPY_HW_ENABLE_MDNS_QUERIES
// This function mimics lwip_getaddrinfo, but makes an mDNS query
static int mdns_getaddrinfo(const char *host_str, const char *port_str,
    const struct addrinfo *hints, struct addrinfo **res) {
    int host_len = strlen(host_str);
    const int local_len = sizeof(MDNS_LOCAL_SUFFIX) - 1;
    if (host_len <= local_len ||
        strcasecmp(host_str + host_len - local_len, MDNS_LOCAL_SUFFIX) != 0) {
        return 0;
    }

    // mDNS query
    char host_no_local[host_len - local_len + 1];
    memcpy(host_no_local, host_str, host_len - local_len);
    host_no_local[host_len - local_len] = '\0';

    esp_ip4_addr_t addr = {0};

    esp_err_t err = mdns_query_a(host_no_local, MDNS_QUERY_TIMEOUT_MS, &addr);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NOT_FOUND) {
            *res = NULL;
            return 0;
        }
        *res = NULL;
        return err;
    }

    struct addrinfo *ai = memp_malloc(MEMP_NETDB);
    if (ai == NULL) {
        *res = NULL;
        return EAI_MEMORY;
    }
    memset(ai, 0, sizeof(struct addrinfo) + sizeof(struct sockaddr_storage));

    struct sockaddr_in *sa = (struct sockaddr_in *)((uint8_t *)ai + sizeof(struct addrinfo));
    inet_addr_from_ip4addr(&sa->sin_addr, &addr);
    sa->sin_family = AF_INET;
    sa->sin_len = sizeof(struct sockaddr_in);
    sa->sin_port = lwip_htons((u16_t)atoi(port_str));
    ai->ai_family = AF_INET;
    ai->ai_canonname = ((char *)sa + sizeof(struct sockaddr_storage));
    memcpy(ai->ai_canonname, host_str, host_len + 1);
    ai->ai_addrlen = sizeof(struct sockaddr_storage);
    ai->ai_addr = (struct sockaddr *)sa;
    ai->ai_socktype = SOCK_STREAM;
    if (hints) {
        ai->ai_socktype = hints->ai_socktype;
        ai->ai_protocol = hints->ai_protocol;
    }

    *res = ai;
    return 0;
}
#endif // MICROPY_HW_ENABLE_MDNS_QUERIES

static void _getaddrinfo_inner(const mp_obj_t host, const mp_obj_t portx,
    struct addrinfo *hints, struct addrinfo **res) {
    int retval = 0;

    *res = NULL;

    mp_obj_t port = portx;
    if (mp_obj_is_integer(port)) {
        // This is perverse, because lwip_getaddrinfo promptly converts it back to an int, but
        // that's the API we have to work with ...
        port = mp_obj_str_binary_op(MP_BINARY_OP_MODULO, mp_obj_new_str_via_qstr("%s", 2), port);
    }

    const char *host_str = mp_obj_str_get_str(host);
    const char *port_str = mp_obj_str_get_str(port);

    if (host_str[0] == '\0') {
        // a host of "" is equivalent to the default/all-local IP address
        host_str = "0.0.0.0";
    }

    MP_THREAD_GIL_EXIT();

    // The ai_canonname field is used below, so set the hint.
    hints->ai_flags |= AI_CANONNAME;

    #if MICROPY_HW_ENABLE_MDNS_QUERIES
    retval = mdns_getaddrinfo(host_str, port_str, hints, res);
    #endif

    if (retval == 0 && *res == NULL) {
        // Normal query
        retval = lwip_getaddrinfo(host_str, port_str, hints, res);
    }

    MP_THREAD_GIL_ENTER();

    // Per docs: instead of raising gaierror getaddrinfo raises negative error number
    if (retval != 0) {
        mp_raise_OSError(retval > 0 ? -retval : retval);
    }
    // Somehow LwIP returns a resolution of 0.0.0.0 for failed lookups, traced it as far back
    // as netconn_gethostbyname_addrtype returning OK instead of error.
    if (*res == NULL ||
        (strcmp(res[0]->ai_canonname, "0.0.0.0") == 0 && strcmp(host_str, "0.0.0.0") != 0)) {
        lwip_freeaddrinfo(*res);
        mp_raise_OSError(-2); // name or service not known
    }

    assert(retval == 0 && *res != NULL);
}

static void _socket_getaddrinfo(const mp_obj_t addrtuple, struct addrinfo **resp) {
    mp_obj_t *elem;
    mp_obj_get_array_fixed_n(addrtuple, 2, &elem);
    struct addrinfo hints = { 0 };
    _getaddrinfo_inner(elem[0], elem[1], &hints, resp);
}

static mp_obj_t socket_make_new(const mp_obj_type_t *type_in, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 3, false);

    socket_obj_t *sock = mp_obj_malloc_with_finaliser(socket_obj_t, type_in);
    sock->domain = AF_INET;
    sock->type = SOCK_STREAM;
    sock->proto = 0;
    if (n_args > 0) {
        sock->domain = mp_obj_get_int(args[0]);
        if (n_args > 1) {
            sock->type = mp_obj_get_int(args[1]);
            if (n_args > 2) {
                sock->proto = mp_obj_get_int(args[2]);
            }
        }
    }

    sock->state = sock->type == SOCK_STREAM ? SOCKET_STATE_NEW : SOCKET_STATE_CONNECTED;

    sock->fd = lwip_socket(sock->domain, sock->type, sock->proto);
    if (sock->fd < 0 && errno == ENFILE) {
        // ESP32 LWIP has a hard socket limit, ENFILE is returned when this is
        // reached. Similar to the logic elsewhere for MemoryError, try running
        // GC before failing outright.
        gc_collect();
        sock->fd = lwip_socket(sock->domain, sock->type, sock->proto);
    }
    if (sock->fd < 0) {
        mp_raise_OSError(errno);
    }
    _socket_settimeout(sock, UINT64_MAX);

    return MP_OBJ_FROM_PTR(sock);
}

static mp_obj_t socket_bind(const mp_obj_t arg0, const mp_obj_t arg1) {
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);
    struct addrinfo *res;
    _socket_getaddrinfo(arg1, &res);
    self->state = SOCKET_STATE_CONNECTED;
    int r = lwip_bind(self->fd, res->ai_addr, res->ai_addrlen);
    lwip_freeaddrinfo(res);
    if (r < 0) {
        mp_raise_OSError(errno);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(socket_bind_obj, socket_bind);

// method socket.listen([backlog])
static mp_obj_t socket_listen(size_t n_args, const mp_obj_t *args) {
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    int backlog = MICROPY_PY_SOCKET_LISTEN_BACKLOG_DEFAULT;
    if (n_args > 1) {
        backlog = mp_obj_get_int(args[1]);
        backlog = (backlog < 0) ? 0 : backlog;
    }

    self->state = SOCKET_STATE_CONNECTED;
    int r = lwip_listen(self->fd, backlog);
    if (r < 0) {
        mp_raise_OSError(errno);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_listen_obj, 1, 2, socket_listen);

static mp_obj_t socket_accept(const mp_obj_t arg0) {
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    struct sockaddr addr;
    socklen_t addr_len = sizeof(addr);

    int new_fd = -1;
    mp_uint_t start_ticks = mp_hal_ticks_ms();
    socket_slice_reset(self);
    for (;;) {
        MP_THREAD_GIL_EXIT();
        new_fd = lwip_accept(self->fd, &addr, &addr_len);
        MP_THREAD_GIL_ENTER();
        if (new_fd >= 0) {
            break;
        }
        if (errno != EAGAIN) {
            mp_raise_OSError(errno);
        }
        if (socket_timed_out(self, start_ticks)) {
            mp_raise_OSError(socket_timeout_errno(self));
        }
        check_for_exceptions();
        socket_slice_trim(self, start_ticks);
    }

    // create new socket object
    socket_obj_t *sock = mp_obj_malloc_with_finaliser(socket_obj_t, self->base.type);
    sock->fd = new_fd;
    sock->domain = self->domain;
    sock->type = self->type;
    sock->proto = self->proto;
    sock->state = SOCKET_STATE_CONNECTED;
    _socket_settimeout(sock, UINT64_MAX);

    // make the return value
    uint8_t *ip = (uint8_t *)&((struct sockaddr_in *)&addr)->sin_addr;
    mp_uint_t port = lwip_ntohs(((struct sockaddr_in *)&addr)->sin_port);
    mp_obj_tuple_t *client = mp_obj_new_tuple(2, NULL);
    client->items[0] = sock;
    client->items[1] = netutils_format_inet_addr(ip, port, NETUTILS_BIG);

    return client;
}
static MP_DEFINE_CONST_FUN_OBJ_1(socket_accept_obj, socket_accept);

static mp_obj_t socket_connect(const mp_obj_t arg0, const mp_obj_t arg1) {
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);
    struct addrinfo *res;
    bool blocking = false;
    int flags;
    int raise_err = 0;

    _socket_getaddrinfo(arg1, &res);
    MP_THREAD_GIL_EXIT();
    self->state = SOCKET_STATE_CONNECTED;

    flags = fcntl(self->fd, F_GETFL);

    blocking = (flags & O_NONBLOCK) == 0;

    if (blocking) {
        // For blocking sockets, make the socket temporarily non-blocking and emulate
        // blocking using select.
        //
        // This has two benefits:
        //
        // - Allows handling external exceptions while waiting for connect.
        //
        // - Allows emulating a connect timeout, which is not supported by LWIP or
        //   required by POSIX but is normal behaviour for CPython.
        if (fcntl(self->fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            ESP_LOGE(TAG, "fcntl set failed %d", errno); // Unexpected internal failure
            raise_err = errno;
        }
    }

    if (raise_err == 0) {
        // Try performing the actual connect. Expected to always return immediately.
        int r = lwip_connect(self->fd, res->ai_addr, res->ai_addrlen);
        if (r != 0) {
            raise_err = errno;
        }
    }

    if (blocking) {
        // Set the socket back to blocking. We can still pass it to select() in this state.
        int r = fcntl(self->fd, F_SETFL, flags);
        if (r != 0 && (raise_err == 0 || raise_err == EINPROGRESS)) {
            ESP_LOGE(TAG, "fcntl restore failed %d", errno); // Unexpected internal failure
            raise_err = errno;
        }
    }

    lwip_freeaddrinfo(res);

    if (blocking && raise_err == EINPROGRESS) {
        // Keep calling select() until the socket is marked writable (i.e. connected),
        // or an error or a timeout occurs

        mp_uint_t start_ticks = mp_hal_ticks_ms();
        socket_slice_reset(self);
        for (;;) {
            // The slice, shortened on the last one so a timeout that is not a
            // multiple of the period does not overshoot.
            struct timeval timeout = {
                .tv_sec = 0,
                .tv_usec = socket_slice_us(self, start_ticks),
            };
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(self->fd, &wfds);

            // lwip_select() for the same reason as in socket_stream_ioctl():
            // select() here is esp_vfs_select(), which walks every fd slot up
            // to this socket's number.
            int r = lwip_select(self->fd + 1, NULL, &wfds, NULL, &timeout);
            if (r < 0) {
                // Error condition
                raise_err = errno;
                break;
            } else if (r > 0) {
                // Select indicated the socket is writable. Check for any error.
                socklen_t socklen = sizeof(raise_err);
                r = getsockopt(self->fd, SOL_SOCKET, SO_ERROR, &raise_err, &socklen);
                if (r < 0) {
                    raise_err = errno;
                }
                break;
            } else {
                // Select timed out for this slice; only give up once the
                // caller's own timeout has actually elapsed.
                MP_THREAD_GIL_ENTER();
                if (socket_timed_out(self, start_ticks)) {
                    raise_err = ETIMEDOUT;
                    MP_THREAD_GIL_EXIT();
                    break;
                }
                check_for_exceptions();
                MP_THREAD_GIL_EXIT();
            }
        }
    }

    MP_THREAD_GIL_ENTER();
    if (raise_err) {
        mp_raise_OSError(raise_err);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(socket_connect_obj, socket_connect);

static mp_obj_t socket_setsockopt(size_t n_args, const mp_obj_t *args) {
    (void)n_args; // always 4
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    int opt = mp_obj_get_int(args[2]);

    switch (opt) {
        // level: SOL_SOCKET
        case SO_REUSEADDR:
        case SO_BROADCAST: {
            int val = mp_obj_get_int(args[3]);
            int ret = lwip_setsockopt(self->fd, SOL_SOCKET, opt, &val, sizeof(int));
            if (ret != 0) {
                mp_raise_OSError(errno);
            }
            break;
        }

        case SO_BINDTODEVICE: {
            size_t len;
            const char *val = mp_obj_str_get_data(args[3], &len);
            char ifname[NETIF_NAMESIZE] = {0};
            memcpy(&ifname, val, len);
            int ret = lwip_setsockopt(self->fd, SOL_SOCKET, opt, &ifname, NETIF_NAMESIZE);
            if (ret != 0) {
                mp_raise_OSError(errno);
            }
            break;
        }

            #if MICROPY_PY_SOCKET_EVENTS
        // level: SOL_SOCKET
        // special "register callback" option
        case 20: {
            if (args[3] == mp_const_none) {
                if (self->events_callback != MP_OBJ_NULL) {
                    socket_events_remove(self);
                    self->events_callback = MP_OBJ_NULL;
                }
            } else {
                if (self->events_callback == MP_OBJ_NULL) {
                    socket_events_add(self);
                }
                self->events_callback = args[3];
            }
            break;
        }
            #endif

        // level: IPPROTO_TCP
        case TCP_NODELAY: {
            int val = mp_obj_get_int(args[3]);
            int ret = lwip_setsockopt(self->fd, IPPROTO_TCP, opt, &val, sizeof(int));
            if (ret != 0) {
                mp_raise_OSError(errno);
            }
            break;
        }

        // level: IPPROTO_IP
        case IP_ADD_MEMBERSHIP: {
            mp_buffer_info_t bufinfo;
            mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_READ);
            if (bufinfo.len != sizeof(ip4_addr_t) * 2) {
                mp_raise_ValueError(NULL);
            }

            // POSIX setsockopt has order: group addr, if addr, lwIP has it vice-versa
            err_t err = igmp_joingroup((const ip4_addr_t *)bufinfo.buf + 1, bufinfo.buf);
            if (err != ERR_OK) {
                mp_raise_OSError(-err);
            }
            break;
        }

        default:
            mp_printf(&mp_plat_print, "Warning: lwip.setsockopt() option not implemented\n");
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_setsockopt_obj, 4, 4, socket_setsockopt);

void _socket_settimeout(socket_obj_t *sock, uint64_t timeout_ms) {
    // Blocking calls are made in slices so a MicroPython interrupt is noticed
    // between them; the slice is the shorter of the polling period and what the
    // caller actually asked for, so a timeout below one period is not rounded up.
    sock->timeout_ms = timeout_ms;
    sock->slice_trimmed = false;

    uint64_t slice_us = SOCKET_POLL_US;
    if (timeout_ms == 0) {
        slice_us = 0;
    } else if (timeout_ms != UINT64_MAX && timeout_ms * 1000 < SOCKET_POLL_US) {
        slice_us = timeout_ms * 1000;
    }
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = slice_us
    };
    lwip_setsockopt(sock->fd, SOL_SOCKET, SO_SNDTIMEO, (const void *)&timeout, sizeof(timeout));
    lwip_setsockopt(sock->fd, SOL_SOCKET, SO_RCVTIMEO, (const void *)&timeout, sizeof(timeout));
    lwip_fcntl(sock->fd, F_SETFL, timeout_ms ? 0 : O_NONBLOCK);
}

static mp_obj_t socket_settimeout(const mp_obj_t arg0, const mp_obj_t arg1) {
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);
    if (arg1 == mp_const_none) {
        _socket_settimeout(self, UINT64_MAX);
    } else {
        #if MICROPY_PY_BUILTINS_FLOAT
        _socket_settimeout(self, (uint64_t)(mp_obj_get_float(arg1) * MICROPY_FLOAT_CONST(1000.0)));
        #else
        _socket_settimeout(self, mp_obj_get_int(arg1) * 1000);
        #endif
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(socket_settimeout_obj, socket_settimeout);

static mp_obj_t socket_setblocking(const mp_obj_t arg0, const mp_obj_t arg1) {
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);
    if (mp_obj_is_true(arg1)) {
        _socket_settimeout(self, UINT64_MAX);
    } else {
        _socket_settimeout(self, 0);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(socket_setblocking_obj, socket_setblocking);

// XXX this can end up waiting a very long time if the content is dribbled in one character
// at a time, as the timeout resets each time a recvfrom succeeds ... this is probably not
// good behaviour.
static mp_uint_t _socket_read_data(mp_obj_t self_in, void *buf, size_t size, mp_int_t flags,
    struct sockaddr *from, socklen_t *from_len, int *errcode) {
    socket_obj_t *sock = MP_OBJ_TO_PTR(self_in);

    // A new socket cannot be read from.
    if (sock->state == SOCKET_STATE_NEW) {
        *errcode = MP_ENOTCONN;
        return MP_STREAM_ERROR;
    }

    // If the peer closed the connection then the lwIP socket API will only return "0" once
    // from lwip_recvfrom and then block on subsequent calls.  To emulate POSIX behaviour,
    // which continues to return "0" for each call on a closed socket, we set a flag when
    // the peer closed the socket.
    if (sock->state == SOCKET_STATE_PEER_CLOSED) {
        return 0;
    }

    mp_uint_t start_ticks = mp_hal_ticks_ms();
    socket_slice_reset(sock);
    for (;;) {
        // Poll the socket to see if it has waiting data and only release the GIL if it doesn't.
        // This ensures higher performance in the case of many small reads, eg for readline.
        // lwip_select() rather than select(): this probe runs before *every*
        // recv, and as esp_vfs_select() it costs more than the GIL release it
        // is trying to avoid.
        bool release_gil;
        {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock->fd, &rfds);
            struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
            int r = lwip_select(sock->fd + 1, &rfds, NULL, NULL, &timeout);
            release_gil = r != 1;
        }
        if (release_gil) {
            MP_THREAD_GIL_EXIT();
        }
        int r = lwip_recvfrom(sock->fd, buf, size, flags, from, from_len);
        if (release_gil) {
            MP_THREAD_GIL_ENTER();
        }
        if (r == 0) {
            sock->state = SOCKET_STATE_PEER_CLOSED;
        }
        if (r >= 0) {
            return r;
        }
        if (errno != EWOULDBLOCK || (flags & MSG_DONTWAIT)) {
            *errcode = errno;
            return MP_STREAM_ERROR;
        }
        if (socket_timed_out(sock, start_ticks)) {
            *errcode = socket_timeout_errno(sock);
            return MP_STREAM_ERROR;
        }
        check_for_exceptions();
        socket_slice_trim(sock, start_ticks);
    }
}

mp_obj_t _socket_recvfrom(size_t n_args, const mp_obj_t *args,
    struct sockaddr *from, socklen_t *from_len) {
    mp_obj_t self_in = args[0];
    size_t len = mp_obj_get_int(args[1]);
    int flags = n_args > 2 ? mp_obj_get_int(args[2]) : 0;

    vstr_t vstr;
    vstr_init_len(&vstr, len);

    int errcode;
    mp_uint_t ret = _socket_read_data(self_in, vstr.buf, len, flags, from, from_len, &errcode);
    if (ret == MP_STREAM_ERROR) {
        mp_raise_OSError(errcode);
    }

    vstr.len = ret;
    return mp_obj_new_bytes_from_vstr(&vstr);
}

static mp_obj_t socket_recv(size_t n_args, const mp_obj_t *args) {
    return _socket_recvfrom(n_args, args, NULL, NULL);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_recv_obj, 2, 3, socket_recv);

static mp_obj_t socket_recvfrom(size_t n_args, const mp_obj_t *args) {
    struct sockaddr from;
    socklen_t fromlen = sizeof(from);

    mp_obj_t tuple[2];
    tuple[0] = _socket_recvfrom(n_args, args, &from, &fromlen);

    uint8_t *ip = (uint8_t *)&((struct sockaddr_in *)&from)->sin_addr;
    mp_uint_t port = lwip_ntohs(((struct sockaddr_in *)&from)->sin_port);
    tuple[1] = netutils_format_inet_addr(ip, port, NETUTILS_BIG);

    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_recvfrom_obj, 2, 3, socket_recvfrom);

int _socket_send(socket_obj_t *sock, const char *data, size_t datalen) {
    int sentlen = 0;
    mp_uint_t start_ticks = mp_hal_ticks_ms();
    socket_slice_reset(sock);
    while (sentlen < datalen) {
        MP_THREAD_GIL_EXIT();
        int r = lwip_write(sock->fd, data + sentlen, datalen - sentlen);
        MP_THREAD_GIL_ENTER();
        // lwip returns EINPROGRESS when trying to send right after a non-blocking connect
        if (r < 0 && errno != EWOULDBLOCK && errno != EINPROGRESS) {
            mp_raise_OSError(errno);
        }
        if (r > 0) {
            sentlen += r;
            continue;
        }
        if (socket_timed_out(sock, start_ticks)) {
            break;
        }
        check_for_exceptions();
        socket_slice_trim(sock, start_ticks);
    }
    if (sentlen == 0) {
        mp_raise_OSError(socket_timeout_errno(sock));
    }
    return sentlen;
}

static mp_obj_t socket_send(const mp_obj_t arg0, const mp_obj_t arg1) {
    socket_obj_t *sock = MP_OBJ_TO_PTR(arg0);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg1, &bufinfo, MP_BUFFER_READ);
    int r = _socket_send(sock, bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(r);
}
static MP_DEFINE_CONST_FUN_OBJ_2(socket_send_obj, socket_send);

static mp_obj_t socket_sendall(const mp_obj_t arg0, const mp_obj_t arg1) {
    // XXX behaviour when nonblocking (see extmod/modlwip.c)
    // XXX also timeout behaviour.
    socket_obj_t *sock = MP_OBJ_TO_PTR(arg0);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg1, &bufinfo, MP_BUFFER_READ);
    int r = _socket_send(sock, bufinfo.buf, bufinfo.len);
    if (r < bufinfo.len) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(socket_sendall_obj, socket_sendall);

static mp_obj_t socket_sendto(mp_obj_t self_in, mp_obj_t data_in, mp_obj_t addr_in) {
    socket_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // get the buffer to send
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);

    // create the destination address
    struct sockaddr_in to;
    to.sin_len = sizeof(to);
    to.sin_family = AF_INET;
    to.sin_port = lwip_htons(netutils_parse_inet_addr(addr_in, (uint8_t *)&to.sin_addr, NETUTILS_BIG));

    // send the data
    mp_uint_t start_ticks = mp_hal_ticks_ms();
    socket_slice_reset(self);
    for (;;) {
        MP_THREAD_GIL_EXIT();
        int ret = lwip_sendto(self->fd, bufinfo.buf, bufinfo.len, 0, (struct sockaddr *)&to, sizeof(to));
        MP_THREAD_GIL_ENTER();
        if (ret > 0) {
            return mp_obj_new_int_from_uint(ret);
        }
        if (ret == -1 && errno != EWOULDBLOCK) {
            mp_raise_OSError(errno);
        }
        if (socket_timed_out(self, start_ticks)) {
            mp_raise_OSError(socket_timeout_errno(self));
        }
        check_for_exceptions();
        socket_slice_trim(self, start_ticks);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_3(socket_sendto_obj, socket_sendto);

static mp_obj_t socket_fileno(const mp_obj_t arg0) {
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);
    return mp_obj_new_int(self->fd);
}
static MP_DEFINE_CONST_FUN_OBJ_1(socket_fileno_obj, socket_fileno);

static mp_obj_t socket_makefile(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    return args[0];
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_makefile_obj, 1, 3, socket_makefile);

static mp_uint_t socket_stream_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode) {
    return _socket_read_data(self_in, buf, size, 0, NULL, NULL, errcode);
}

static mp_uint_t socket_stream_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode) {
    socket_obj_t *sock = self_in;
    mp_uint_t start_ticks = mp_hal_ticks_ms();
    socket_slice_reset(sock);
    for (;;) {
        MP_THREAD_GIL_EXIT();
        int r = lwip_write(sock->fd, buf, size);
        MP_THREAD_GIL_ENTER();
        if (r > 0) {
            return r;
        }
        // lwip returns MP_EINPROGRESS when trying to write right after a non-blocking connect
        if (r < 0 && errno != EWOULDBLOCK && errno != EINPROGRESS) {
            *errcode = errno;
            return MP_STREAM_ERROR;
        }
        if (socket_timed_out(sock, start_ticks)) {
            *errcode = socket_timeout_errno(sock);
            return MP_STREAM_ERROR;
        }
        check_for_exceptions();
        socket_slice_trim(sock, start_ticks);
    }
}

static mp_uint_t socket_stream_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    socket_obj_t *socket = self_in;
    if (request == MP_STREAM_POLL) {
        if (socket->fd == -1) {
            return MP_STREAM_POLL_NVAL;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        fd_set wfds;
        FD_ZERO(&wfds);
        fd_set efds;
        FD_ZERO(&efds);
        struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
        if (arg & MP_STREAM_POLL_RD) {
            FD_SET(socket->fd, &rfds);
        }
        if (arg & MP_STREAM_POLL_WR) {
            FD_SET(socket->fd, &wfds);
        }
        if (arg & MP_STREAM_POLL_HUP) {
            FD_SET(socket->fd, &efds);
        }

        // lwip_select() rather than select(): with LWIP_COMPAT_SOCKETS == 0 the
        // latter resolves to esp_vfs_select(), whose cost is proportional to the
        // fd number, and lwIP places sockets at the top of the fd space.
        int r = lwip_select((socket->fd) + 1, &rfds, &wfds, &efds, &timeout);
        if (r < 0) {
            *errcode = MP_EIO;
            return MP_STREAM_ERROR;
        }

        mp_uint_t ret = 0;
        if (FD_ISSET(socket->fd, &rfds)) {
            ret |= MP_STREAM_POLL_RD;
        }
        if (FD_ISSET(socket->fd, &wfds)) {
            ret |= MP_STREAM_POLL_WR;
        }
        if (FD_ISSET(socket->fd, &efds)) {
            ret |= MP_STREAM_POLL_HUP;
        }

        // New (unconnected) sockets are writable and have HUP set.
        if (socket->state == SOCKET_STATE_NEW) {
            ret |= (arg & MP_STREAM_POLL_WR) | MP_STREAM_POLL_HUP;
        }

        return ret;
    } else if (request == MP_STREAM_CLOSE) {
        if (socket->fd >= 0) {
            #if MICROPY_PY_SOCKET_EVENTS
            if (socket->events_callback != MP_OBJ_NULL) {
                socket_events_remove(socket);
                socket->events_callback = MP_OBJ_NULL;
            }
            #endif
            int ret = lwip_close(socket->fd);
            if (ret != 0) {
                *errcode = errno;
                return MP_STREAM_ERROR;
            }
            socket->fd = -1;
        }
        return 0;
    }

    *errcode = MP_EINVAL;
    return MP_STREAM_ERROR;
}

static const mp_rom_map_elem_t socket_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_bind), MP_ROM_PTR(&socket_bind_obj) },
    { MP_ROM_QSTR(MP_QSTR_listen), MP_ROM_PTR(&socket_listen_obj) },
    { MP_ROM_QSTR(MP_QSTR_accept), MP_ROM_PTR(&socket_accept_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect), MP_ROM_PTR(&socket_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&socket_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendall), MP_ROM_PTR(&socket_sendall_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendto), MP_ROM_PTR(&socket_sendto_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&socket_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_recvfrom), MP_ROM_PTR(&socket_recvfrom_obj) },
    { MP_ROM_QSTR(MP_QSTR_setsockopt), MP_ROM_PTR(&socket_setsockopt_obj) },
    { MP_ROM_QSTR(MP_QSTR_settimeout), MP_ROM_PTR(&socket_settimeout_obj) },
    { MP_ROM_QSTR(MP_QSTR_setblocking), MP_ROM_PTR(&socket_setblocking_obj) },
    { MP_ROM_QSTR(MP_QSTR_makefile), MP_ROM_PTR(&socket_makefile_obj) },
    { MP_ROM_QSTR(MP_QSTR_fileno), MP_ROM_PTR(&socket_fileno_obj) },

    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },
};
static MP_DEFINE_CONST_DICT(socket_locals_dict, socket_locals_dict_table);

static const mp_stream_p_t socket_stream_p = {
    .read = socket_stream_read,
    .write = socket_stream_write,
    .ioctl = socket_stream_ioctl
};

static MP_DEFINE_CONST_OBJ_TYPE(
    socket_type,
    MP_QSTR_socket,
    MP_TYPE_FLAG_NONE,
    make_new, socket_make_new,
    protocol, &socket_stream_p,
    locals_dict, &socket_locals_dict
    );

static mp_obj_t esp_socket_getaddrinfo(size_t n_args, const mp_obj_t *args) {
    struct addrinfo hints = { };
    struct addrinfo *res = NULL;

    // Optional args: family=0, type=0, proto=0, flags=0, where 0 is "least narrow"
    if (n_args > 2) {
        hints.ai_family = mp_obj_get_int(args[2]);
    }
    if (n_args > 3) {
        hints.ai_socktype = mp_obj_get_int(args[3]);
    }
    if (hints.ai_socktype == 0) {
        // This is slightly different to CPython with POSIX getaddrinfo. In
        // CPython, calling socket.getaddrinfo() with socktype=0 returns any/all
        // supported SocketKind values. Here, lwip_getaddrinfo() will echo
        // whatever socktype was supplied to the caller. Rather than returning 0
        // (invalid in a result), make it SOCK_STREAM.
        hints.ai_socktype = SOCK_STREAM;
    }
    if (n_args > 4) {
        hints.ai_protocol = mp_obj_get_int(args[4]);
    }
    if (n_args > 5) {
        hints.ai_flags = mp_obj_get_int(args[5]);
    }

    _getaddrinfo_inner(args[0], args[1], &hints, &res);
    mp_obj_t ret_list = mp_obj_new_list(0, NULL);

    for (struct addrinfo *resi = res; resi; resi = resi->ai_next) {
        mp_obj_t addrinfo_objs[5] = {
            mp_obj_new_int(resi->ai_family),
            mp_obj_new_int(resi->ai_socktype),
            mp_obj_new_int(resi->ai_protocol),
            mp_obj_new_str_from_cstr(resi->ai_canonname),
            mp_const_none
        };

        if (resi->ai_family == AF_INET) {
            struct sockaddr_in *addr = (struct sockaddr_in *)resi->ai_addr;
            // This looks odd, but it's really just a u32_t
            ip4_addr_t ip4_addr = { .addr = addr->sin_addr.s_addr };
            char buf[16];
            ip4addr_ntoa_r(&ip4_addr, buf, sizeof(buf));
            mp_obj_t inaddr_objs[2] = {
                mp_obj_new_str_from_cstr(buf),
                mp_obj_new_int(ntohs(addr->sin_port))
            };
            addrinfo_objs[4] = mp_obj_new_tuple(2, inaddr_objs);
        }
        mp_obj_list_append(ret_list, mp_obj_new_tuple(5, addrinfo_objs));
    }

    lwip_freeaddrinfo(res);
    return ret_list;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp_socket_getaddrinfo_obj, 2, 6, esp_socket_getaddrinfo);

/******************************************************************************/
// Non-blocking name resolution.
//
// socket.getaddrinfo() stays blocking, as it is in CPython. This is the
// primitive underneath asyncio's Loop.getaddrinfo(): it starts a query and
// hands back an object that becomes readable when the answer arrives, so the
// event loop can wait on it through select.poll() like any other stream.
//
// Lifetime is the delicate part. lwIP holds a pointer to the request for the
// whole query and there is no way to cancel one, so the object has to outlive
// a coroutine that gives up early -- asyncio.wait_for() being the obvious case.
// It is therefore held on a GC root list until the answer (or the failure) is
// in, exactly like the PPP session list in network_ppp.c. Putting this state on
// the C stack, as extmod/modlwip.c does, is a use-after-free waiting to happen.

typedef struct _getaddrinfo_req_t {
    mp_obj_base_t base;
    // Written on the lwIP thread, read on the MicroPython thread. `status` is
    // published last, after `addr`, with a barrier in between.
    volatile int status;                // 0 pending, 1 resolved, <0 errno
    ip_addr_t addr;
    // Set up before the request is handed over; read on the lwIP thread.
    const char *name;
    mp_obj_t host;                      // keeps `name`'s storage alive
    uint16_t port;
    uint8_t socktype;
    uint8_t proto;
    uint8_t addrtype;
    struct _getaddrinfo_req_t *next;
    bool listed;
} getaddrinfo_req_t;

static const mp_obj_type_t getaddrinfo_req_type;

static void getaddrinfo_req_unlist(getaddrinfo_req_t *req) {
    getaddrinfo_req_t **link = &MP_STATE_PORT(socket_getaddrinfo_req_head);
    while (*link != NULL) {
        if (*link == req) {
            *link = req->next;
            req->next = NULL;
            req->listed = false;
            return;
        }
        link = &(*link)->next;
    }
}

// Release requests whose answer has arrived but whose caller never collected it
// (a cancelled coroutine). Only ever called from the MicroPython thread.
static void getaddrinfo_req_reap(void) {
    getaddrinfo_req_t **link = &MP_STATE_PORT(socket_getaddrinfo_req_head);
    while (*link != NULL) {
        getaddrinfo_req_t *req = *link;
        if (req->status != 0) {
            *link = req->next;
            req->next = NULL;
            req->listed = false;
        } else {
            link = &req->next;
        }
    }
}

// Both callbacks below run on the lwIP tcpip thread. They must not touch the VM.
static void getaddrinfo_found_cb(const char *name, const ip_addr_t *ipaddr, void *arg) {
    getaddrinfo_req_t *req = arg;
    int status = -2; // name or service not known, as the blocking path reports it
    if (ipaddr != NULL) {
        req->addr = *ipaddr;
        status = 1;
    }
    __sync_synchronize();
    req->status = status;
}

// CONFIG_LWIP_TCPIP_CORE_LOCKING is off in this port, so the DNS API may only be
// called from the tcpip thread itself. tcpip_callback() is how we get there.
static void getaddrinfo_start_cb(void *arg) {
    getaddrinfo_req_t *req = arg;
    ip_addr_t addr;
    err_t err = dns_gethostbyname_addrtype(req->name, &addr, getaddrinfo_found_cb, req, req->addrtype);
    if (err == ERR_INPROGRESS) {
        // getaddrinfo_found_cb will publish the result.
        return;
    }
    int status = -2;
    if (err == ERR_OK) {
        // Answer was already cached.
        req->addr = addr;
        status = 1;
    }
    __sync_synchronize();
    req->status = status;
}

// socket._getaddrinfo_start(host, port, family=0, type=0, proto=0, flags=0)
static mp_obj_t esp_socket_getaddrinfo_start(size_t n_args, const mp_obj_t *args) {
    getaddrinfo_req_reap();

    mp_int_t family = n_args > 2 ? mp_obj_get_int(args[2]) : 0;
    mp_int_t socktype = n_args > 3 ? mp_obj_get_int(args[3]) : 0;
    mp_int_t proto = n_args > 4 ? mp_obj_get_int(args[4]) : 0;
    if (socktype == 0) {
        // Same choice as the blocking getaddrinfo(): 0 is not valid in a result.
        socktype = SOCK_STREAM;
    }

    getaddrinfo_req_t *req = mp_obj_malloc(getaddrinfo_req_t, &getaddrinfo_req_type);
    req->status = 0;
    req->host = args[0];
    req->name = mp_obj_str_get_str(args[0]);
    req->port = (uint16_t)mp_obj_get_int(args[1]);
    req->socktype = (uint8_t)socktype;
    req->proto = (uint8_t)proto;
    req->addrtype = family == AF_INET ? LWIP_DNS_ADDRTYPE_IPV4
        : family == AF_INET6 ? LWIP_DNS_ADDRTYPE_IPV6
        : LWIP_DNS_ADDRTYPE_DEFAULT;
    req->next = NULL;
    req->listed = false;

    if (ipaddr_aton(req->name, &req->addr)) {
        // Already an address literal: nothing to resolve, and nothing to hand
        // to lwIP. CPython short-circuits this case too (_ipaddr_info), but it
        // has to happen here rather than in asyncio, because only the port
        // knows how it spells a sockaddr. Costs ~0.7 ms otherwise, which is
        // most of the event loop's jitter budget.
        req->status = 1;
        return MP_OBJ_FROM_PTR(req);
    }

    // On the list before lwIP is told about it, not after.
    req->next = MP_STATE_PORT(socket_getaddrinfo_req_head);
    MP_STATE_PORT(socket_getaddrinfo_req_head) = req;
    req->listed = true;

    if (tcpip_callback(getaddrinfo_start_cb, req) != ERR_OK) {
        getaddrinfo_req_unlist(req);
        mp_raise_OSError(MP_EAGAIN);
    }

    return MP_OBJ_FROM_PTR(req);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp_socket_getaddrinfo_start_obj, 2, 6, esp_socket_getaddrinfo_start);

// Collect the answer. Raises EAGAIN if called before the object reads readable,
// and the same errno the blocking path uses if the name did not resolve.
static mp_obj_t getaddrinfo_req_result(mp_obj_t self_in) {
    getaddrinfo_req_t *req = MP_OBJ_TO_PTR(self_in);
    int status = req->status;
    __sync_synchronize();
    if (status == 0) {
        mp_raise_OSError(MP_EAGAIN);
    }
    if (req->listed) {
        getaddrinfo_req_unlist(req);
    }
    if (status < 0) {
        mp_raise_OSError(status);
    }

    char buf[IPADDR_STRLEN_MAX];
    ipaddr_ntoa_r(&req->addr, buf, sizeof(buf));
    mp_obj_t inaddr_objs[2] = {
        mp_obj_new_str_from_cstr(buf),
        mp_obj_new_int(req->port),
    };
    mp_obj_t addrinfo_objs[5] = {
        mp_obj_new_int(IP_IS_V6(&req->addr) ? AF_INET6 : AF_INET),
        mp_obj_new_int(req->socktype),
        mp_obj_new_int(req->proto),
        req->host, // canonname: echo what was asked for, lwIP's DNS does not give one
        mp_obj_new_tuple(2, inaddr_objs),
    };
    mp_obj_t entry = mp_obj_new_tuple(5, addrinfo_objs);
    return mp_obj_new_list(1, &entry);
}
static MP_DEFINE_CONST_FUN_OBJ_1(getaddrinfo_req_result_obj, getaddrinfo_req_result);

static mp_uint_t getaddrinfo_req_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    getaddrinfo_req_t *req = MP_OBJ_TO_PTR(self_in);
    if (request != MP_STREAM_POLL) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    // Readable once there is something to collect, success or failure alike.
    return (arg & MP_STREAM_POLL_RD) && req->status != 0 ? MP_STREAM_POLL_RD : 0;
}

static const mp_rom_map_elem_t getaddrinfo_req_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_result), MP_ROM_PTR(&getaddrinfo_req_result_obj) },
};
static MP_DEFINE_CONST_DICT(getaddrinfo_req_locals_dict, getaddrinfo_req_locals_dict_table);

static const mp_stream_p_t getaddrinfo_req_stream_p = {
    .ioctl = getaddrinfo_req_ioctl,
};

static MP_DEFINE_CONST_OBJ_TYPE(
    getaddrinfo_req_type,
    MP_QSTR_getaddrinfo_request,
    MP_TYPE_FLAG_NONE,
    protocol, &getaddrinfo_req_stream_p,
    locals_dict, &getaddrinfo_req_locals_dict
    );

static mp_obj_t esp_socket_initialize() {
    static int initialized = 0;
    if (!initialized) {
        ESP_LOGI(TAG, "Initializing");
        esp_netif_init();
        initialized = 1;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(esp_socket_initialize_obj, esp_socket_initialize);

static const mp_rom_map_elem_t mp_module_socket_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_socket) },
    { MP_ROM_QSTR(MP_QSTR___init__), MP_ROM_PTR(&esp_socket_initialize_obj) },
    { MP_ROM_QSTR(MP_QSTR_socket), MP_ROM_PTR(&socket_type) },
    { MP_ROM_QSTR(MP_QSTR_getaddrinfo), MP_ROM_PTR(&esp_socket_getaddrinfo_obj) },
    // Private: the non-blocking half, used by asyncio's Loop.getaddrinfo().
    { MP_ROM_QSTR(MP_QSTR__getaddrinfo_start), MP_ROM_PTR(&esp_socket_getaddrinfo_start_obj) },

    { MP_ROM_QSTR(MP_QSTR_AF_INET), MP_ROM_INT(AF_INET) },
    { MP_ROM_QSTR(MP_QSTR_AF_INET6), MP_ROM_INT(AF_INET6) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_STREAM), MP_ROM_INT(SOCK_STREAM) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_DGRAM), MP_ROM_INT(SOCK_DGRAM) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_RAW), MP_ROM_INT(SOCK_RAW) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_TCP), MP_ROM_INT(IPPROTO_TCP) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_UDP), MP_ROM_INT(IPPROTO_UDP) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_IP), MP_ROM_INT(IPPROTO_IP) },
    { MP_ROM_QSTR(MP_QSTR_SOL_SOCKET), MP_ROM_INT(SOL_SOCKET) },
    { MP_ROM_QSTR(MP_QSTR_SO_REUSEADDR), MP_ROM_INT(SO_REUSEADDR) },
    { MP_ROM_QSTR(MP_QSTR_SO_BROADCAST), MP_ROM_INT(SO_BROADCAST) },
    { MP_ROM_QSTR(MP_QSTR_SO_BINDTODEVICE), MP_ROM_INT(SO_BINDTODEVICE) },
    { MP_ROM_QSTR(MP_QSTR_IP_ADD_MEMBERSHIP), MP_ROM_INT(IP_ADD_MEMBERSHIP) },
    { MP_ROM_QSTR(MP_QSTR_TCP_NODELAY), MP_ROM_INT(TCP_NODELAY) },
    { MP_ROM_QSTR(MP_QSTR_MSG_PEEK), MP_ROM_INT(MSG_PEEK) },
    { MP_ROM_QSTR(MP_QSTR_MSG_DONTWAIT), MP_ROM_INT(MSG_DONTWAIT) },
};

static MP_DEFINE_CONST_DICT(mp_module_socket_globals, mp_module_socket_globals_table);

MP_REGISTER_ROOT_POINTER(struct _getaddrinfo_req_t *socket_getaddrinfo_req_head);

const mp_obj_module_t mp_module_socket = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_socket_globals,
};

// Note: This port doesn't define MICROPY_PY_SOCKET or MICROPY_PY_LWIP so
// this will not conflict with the common implementation provided by
// extmod/mod{lwip,socket}.c.
MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_socket, mp_module_socket);

#endif // MICROPY_PY_NETWORK
