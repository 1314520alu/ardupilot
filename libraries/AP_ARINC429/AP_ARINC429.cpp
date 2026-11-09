/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The files under src/ are included here so waf's top-level glob does not
 * compile them into builds that leave AP_ARINC429_ENABLED at 0.
 */

#include "AP_ARINC429_config.h"

#if AP_ARINC429_ENABLED

#include "AP_ARINC429.h"

#include <AP_HAL/AP_HAL.h>
#include <cstdio>
#include <cstring>
#include <errno.h>

#if AP_NETWORKING_ENABLED
#include <AP_Networking/AP_Networking.h>
#endif

extern const AP_HAL::HAL &hal;

AP_ARINC429 *AP_ARINC429::_singleton;

const AP_Param::GroupInfo AP_ARINC429::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: CS35 ARINC429 enable
    // @Description: Enable the CS35 ARINC429 codec. This does not command the engine and does not claim Ethernet. Received words are monitoring data only.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_ARINC429, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SDI
    // @DisplayName: CS35 ARINC429 expected SDI
    // @Description: Source/Destination Identifier required on D9..D10 before a word is fresh. Appendix B default is 00.
    // @Values: 0:00,1:01,2:10,3:11
    // @User: Advanced
    AP_GROUPINFO("SDI", 1, AP_ARINC429, _sdi, 0),

    // @Param: NG_TO
    // @DisplayName: Ng freshness timeout
    // @Description: Freshness timeout for label 144 Ng. 0 disables the timeout. The 24 ms ECU nominal refresh is not this value. An invalid word does not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("NG_TO", 2, AP_ARINC429, _to_ng_ms, 0),

    // @Param: NP_TO
    // @DisplayName: Np freshness timeout
    // @Description: Freshness timeout for label 145 Np. 0 disables the timeout. Invalid words do not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("NP_TO", 3, AP_ARINC429, _to_np_ms, 0),

    // @Param: T45_TO
    // @DisplayName: T45 freshness timeout
    // @Description: Freshness timeout for label 147 T45. 0 disables the timeout. Invalid words do not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("T45_TO", 4, AP_ARINC429, _to_t45_ms, 0),

    // @Param: MKP_TO
    // @DisplayName: Torque freshness timeout
    // @Description: Freshness timeout for label 151 torque. 0 disables the timeout. Invalid words do not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("MKP_TO", 5, AP_ARINC429, _to_mkp_ms, 0),

    // @Param: WN1_TO
    // @DisplayName: Warn freshness timeout
    // @Description: Freshness timeout for label 175 warn. 0 disables the timeout. Invalid words do not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("WN1_TO", 6, AP_ARINC429, _to_wn1_ms, 0),

    // @Param: ST_TO
    // @DisplayName: State freshness timeout
    // @Description: Freshness timeout for label 176 state. 0 disables the timeout. Invalid words do not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("ST_TO", 7, AP_ARINC429, _to_st_ms, 0),

    // @Param: WN2_TO
    // @DisplayName: Warn2 freshness timeout
    // @Description: Freshness timeout for label 177 warn2. 0 disables the timeout. Invalid words do not refresh the timer.
    // @Units: ms
    // @Range: 0 60000
    // @User: Advanced
    AP_GROUPINFO("WN2_TO", 8, AP_ARINC429, _to_wn2_ms, 0),

    // @Param: IP0
    // @DisplayName: NT1-M peer address octet 0
    // @Description: Most significant octet of the NT1-M TCP server address. Zero is a legal octet, so 10.0.0.1 and 192.168.0.7 are usable. The socket stays closed only when all four octets are 0 or A429_REM_PORT is 0. This does not change the flight controller address. The manual example 192.168.3.7 is not programmed here.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP0", 9, AP_ARINC429, _ip0, 0),

    // @Param: IP1
    // @DisplayName: NT1-M peer address octet 1
    // @Description: Second octet of the NT1-M TCP server address. Zero is a legal octet. Combined with IP0, IP2 and IP3; a single zero octet does not disable the client.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP1", 10, AP_ARINC429, _ip1, 0),

    // @Param: IP2
    // @DisplayName: NT1-M peer address octet 2
    // @Description: Third octet of the NT1-M TCP server address. Zero is a legal octet. Combined with the other octets; a single zero octet does not disable the client.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP2", 11, AP_ARINC429, _ip2, 0),

    // @Param: IP3
    // @DisplayName: NT1-M peer address octet 3
    // @Description: Least significant octet of the NT1-M TCP server address. Zero is a legal octet. The destination is unconfigured only when the whole address is 0.0.0.0 or the remote port is 0.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP3", 12, AP_ARINC429, _ip3, 0),

    // @Param: REM_PORT
    // @DisplayName: NT1-M TCP server port
    // @Description: Remote TCP port of the NT1-M. 0 does not open a socket. The manual factory example is 8887 and is not programmed here. The flight controller connects as a TCP client and uses an ephemeral local port.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("REM_PORT", 13, AP_ARINC429, _rem_port, 0),

    // @Param: LOC_PORT
    // @DisplayName: Unused local port
    // @Description: Stored only. The TCP client does not bind this port. The shared Ethernet interface is not claimed.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("LOC_PORT", 14, AP_ARINC429, _loc_port, 0),

    // @Param: FRAME
    // @DisplayName: Provisional byte framing
    // @Description: 0 leaves the TCP bytes uninterpreted. 1 opts into the provisional UART429-S profile: C0 plus the codec logical word little-endian, label integer not bit-reversed, and no CD rate command. The RS422-429-1C manual dated 2026-01-18, pages 3 and 4, describes a C0 frame whose label byte is bit-reversed when decoded. This value does not select that wire profile and does not enable transmission by itself.
    // @Values: 0:None,1:UART429-S_C0_provisional
    // @User: Advanced
    AP_GROUPINFO("FRAME", 15, AP_ARINC429, _frame, 0),

    AP_GROUPEND
};

AP_ARINC429::AP_ARINC429()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

static uint32_t timeout_ms_to_us(int16_t ms)
{
    if (ms <= 0) {
        return 0;
    }
    return (uint32_t)ms * 1000u;
}

static uint8_t clamp_octet(int16_t v)
{
    if (v < 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return (uint8_t)v;
}

static uint16_t clamp_port(int32_t v)
{
    if (v < 0) {
        return 0;
    }
    if (v > 65535) {
        return 65535;
    }
    return (uint16_t)v;
}

void AP_ARINC429::apply_params(void)
{
    int16_t sdi = _sdi.get();
    if (sdi < 0 || sdi > 3) {
        sdi = 0;
    }
    a429_link_set_sdi(&_core.link, (uint8_t)sdi);
    a429_link_set_timeout_us(&_core.link, 0, timeout_ms_to_us(_to_ng_ms.get()));
    a429_link_set_timeout_us(&_core.link, 1, timeout_ms_to_us(_to_np_ms.get()));
    a429_link_set_timeout_us(&_core.link, 2, timeout_ms_to_us(_to_t45_ms.get()));
    a429_link_set_timeout_us(&_core.link, 3, timeout_ms_to_us(_to_mkp_ms.get()));
    a429_link_set_timeout_us(&_core.link, 4, timeout_ms_to_us(_to_wn1_ms.get()));
    a429_link_set_timeout_us(&_core.link, 5, timeout_ms_to_us(_to_st_ms.get()));
    a429_link_set_timeout_us(&_core.link, 6, timeout_ms_to_us(_to_wn2_ms.get()));

    _core.endpoint.remote_ipv4 =
        ((uint32_t)clamp_octet(_ip0.get()) << 24) |
        ((uint32_t)clamp_octet(_ip1.get()) << 16) |
        ((uint32_t)clamp_octet(_ip2.get()) << 8) |
        (uint32_t)clamp_octet(_ip3.get());
    _core.endpoint.remote_port = clamp_port(_rem_port.get());
    _core.endpoint.local_port = clamp_port(_loc_port.get());
    if (_core.endpoint.remote_ipv4 != 0 && _core.endpoint.remote_port != 0) {
        _core.endpoint.carrier = A429_CARRIER_TCP_CLIENT;
    } else {
        _core.endpoint.carrier = A429_CARRIER_UNSPECIFIED;
    }
    a429_tcp_configure(&_core.tcp, 1, _core.endpoint.remote_ipv4, _core.endpoint.remote_port);
    a429_core_set_frame(&_core, _frame.get() == (int8_t)A429_FRAME_UART429S_C0 ?
                        A429_FRAME_UART429S_C0 : A429_FRAME_NONE);
}

#if AP_NETWORKING_ENABLED
int AP_ARINC429::tcp_stream_open(void *ctx, uint32_t ipv4_host, uint16_t port)
{
    AP_ARINC429 *self = static_cast<AP_ARINC429 *>(ctx);
    char addr[16];
    unsigned octets[4];
    unsigned i;
    char *p;
    if (self->_tcp_sock != nullptr) {
        delete self->_tcp_sock;
        self->_tcp_sock = nullptr;
    }
    self->_tcp_sock = NEW_NOTHROW SocketAPM(false);
    if (self->_tcp_sock == nullptr) {
        return -1;
    }
    self->_tcp_sock->set_blocking(false);
    octets[0] = (unsigned)((ipv4_host >> 24) & 0xFFu);
    octets[1] = (unsigned)((ipv4_host >> 16) & 0xFFu);
    octets[2] = (unsigned)((ipv4_host >> 8) & 0xFFu);
    octets[3] = (unsigned)(ipv4_host & 0xFFu);
    p = addr;
    for (i = 0; i < 4u; i++) {
        unsigned v = octets[i];
        char tmp[3];
        uint8_t n = 0;
        if (i > 0) {
            *p++ = '.';
        }
        do {
            tmp[n++] = (char)('0' + (v % 10u));
            v /= 10u;
        } while (v > 0u && n < 3u);
        while (n > 0u) {
            *p++ = tmp[--n];
        }
    }
    *p = '\0';
    /* Blocks this worker thread for at most 200 ms. Not the vehicle loop. */
    if (!self->_tcp_sock->connect_timeout(addr, port, 200)) {
        delete self->_tcp_sock;
        self->_tcp_sock = nullptr;
        return -1;
    }
    self->_tcp_sock->set_blocking(false);
    return 0;
}

int AP_ARINC429::tcp_stream_send(void *ctx, const uint8_t *bytes, uint16_t n)
{
    AP_ARINC429 *self = static_cast<AP_ARINC429 *>(ctx);
    ssize_t wrote;
    if (self->_tcp_sock == nullptr) {
        return -1;
    }
    wrote = self->_tcp_sock->send(bytes, n);
    if (wrote > 0) {
        return (int)wrote;
    }
    if (wrote < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return 0;
    }
    return -1;
}

int AP_ARINC429::tcp_stream_recv(void *ctx, uint8_t *bytes, uint16_t n)
{
    AP_ARINC429 *self = static_cast<AP_ARINC429 *>(ctx);
    ssize_t got;
    if (self->_tcp_sock == nullptr) {
        return -1;
    }
    got = self->_tcp_sock->recv(bytes, n, 0);
    if (got > 0) {
        return (int)got;
    }
    if (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return 0;
    }
    return -1;
}

void AP_ARINC429::tcp_stream_close(void *ctx)
{
    AP_ARINC429 *self = static_cast<AP_ARINC429 *>(ctx);
    if (self->_tcp_sock != nullptr) {
        delete self->_tcp_sock;
        self->_tcp_sock = nullptr;
    }
}

void AP_ARINC429::tcp_lock_cb(void *ctx)
{
    static_cast<AP_ARINC429 *>(ctx)->_tcp_sem.take_blocking();
}

void AP_ARINC429::tcp_unlock_cb(void *ctx)
{
    static_cast<AP_ARINC429 *>(ctx)->_tcp_sem.give();
}

void AP_ARINC429::tcp_worker(void)
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    while (true) {
        /* Do not open a socket until the existing stack has an address.
         * This function does not write NET_IP or restart the interface. */
        if (AP::network().get_ip_active() == 0) {
            hal.scheduler->delay(100);
            continue;
        }
        a429_tcp_poll(&_core.tcp, AP_HAL::micros());
        hal.scheduler->delay(2);
    }
}
#endif

void AP_ARINC429::init(void)
{
    a429_core_init(&_core);
#if AP_NETWORKING_ENABLED
    {
        a429_stream_t stream;
        memset(&stream, 0, sizeof(stream));
        stream.open = tcp_stream_open;
        stream.send = tcp_stream_send;
        stream.recv = tcp_stream_recv;
        stream.close = tcp_stream_close;
        stream.ctx = this;
        a429_tcp_set_lock(&_core.tcp, tcp_lock_cb, tcp_unlock_cb, this);
        a429_tcp_set_stream(&_core.tcp, &stream);
        _core.tcp.io_external = 1;
        if (!_tcp_thread_started) {
            _tcp_thread_started = hal.scheduler->thread_create(
                                      FUNCTOR_BIND_MEMBER(&AP_ARINC429::tcp_worker, void),
                                      "A429TCP", 2048, AP_HAL::Scheduler::PRIORITY_NET, 0);
            if (!_tcp_thread_started) {
                _core.tcp.io_external = 0;
                a429_tcp_set_stream(&_core.tcp, nullptr);
            }
        }
    }
#endif
    _inited = true;
}

void AP_ARINC429::update(void)
{
    if (!_inited) {
        return;
    }
    if (_enable.get() == 0) {
        a429_tcp_configure(&_core.tcp, 0, 0, 0);
        a429_core_set_frame(&_core, A429_FRAME_NONE);
        return;
    }
    apply_params();
    /*
     * Scheduling and byte-pipe maintenance. Words are not copied into
     * motors, servos, EFI, or eRob. Unconfigured framing sends no bytes.
     */
    a429_core_update(&_core, AP_HAL::micros());
}

void AP_ARINC429::sim_inject_rx(uint32_t raw)
{
    if (!_inited) {
        return;
    }
    a429_core_inject_rx(&_core, raw, AP_HAL::micros());
}

void AP_ARINC429::sim_set_clp_v(float volts)
{
    if (!_inited) {
        return;
    }
    a429_sim_set_clp(&_core.link, volts);
}

void AP_ARINC429::sim_set_nr_pct(float pct)
{
    if (!_inited) {
        return;
    }
    a429_sim_set_nr(&_core.link, pct);
}

bool AP_ARINC429::sim_set_pcmd(uint8_t index, const a429_pcmd_levels_t &levels)
{
    if (!_inited) {
        return false;
    }
    return a429_sim_set_pcmd(&_core.link, index, &levels) == 1;
}

void AP_ARINC429::sim_clear_tx(void)
{
    if (!_inited) {
        return;
    }
    a429_sim_clear(&_core.link);
}

bool AP_ARINC429::get_fresh_eu(uint8_t rx_index, float &eu) const
{
    if (rx_index >= A429_RX_N) {
        return false;
    }
    const a429_rx_slot_t &slot = _core.link.rx[rx_index];
    if (!slot.fresh || !slot.view.eu_valid) {
        return false;
    }
    eu = slot.view.eu;
    return true;
}

namespace AP
{
AP_ARINC429 *arinc429()
{
    return AP_ARINC429::get_singleton();
}
}

#include "src/arinc429_catalog.c"
#include "src/arinc429_word.c"
#include "src/arinc429_sched.c"
#include "src/arinc429_link.c"
#include "src/arinc429_frame.c"
#include "src/arinc429_tcp.c"
#include "src/arinc429_transport.c"

#endif  // AP_ARINC429_ENABLED
