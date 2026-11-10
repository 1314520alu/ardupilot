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
#include <AP_Arming/AP_Arming.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_RPM/AP_RPM_config.h>
#include <RC_Channel/RC_Channel.h>
#include <cstdio>
#include <cstring>
#include <errno.h>

#if AP_RPM_ENABLED
#include <AP_RPM/AP_RPM.h>
#endif

#if AP_NETWORKING_ENABLED
#include <AP_Networking/AP_Networking.h>
#endif

extern const AP_HAL::HAL &hal;

AP_ARINC429 *AP_ARINC429::_singleton;

const AP_Param::GroupInfo AP_ARINC429::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: CS35 ARINC429 enable
    // @Description: Enable the CS35 ARINC429 codec. Received words are monitoring data. Labels 340, 342 and 344 are filled only when A429_ENG_CH selects an RC channel. This does not move motors, servos, or eRob, and it does not claim Ethernet.
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
    // @Description: 0 leaves the TCP bytes uninterpreted. 1 is the provisional UART429-S profile and still sends the codec logical word with the label integer not bit-reversed. 2 is the RS422-429-1C profile from the 2026-01-18 manual pages 3 and 4: the same C0 frame, with only the label byte bit-reversed. Neither value sends a CD rate command or enables transmission by itself. The NT1-M path is not hardware-validated.
    // @Values: 0:None,1:UART429-S_C0_provisional,2:RS422-429-1C
    // @User: Advanced
    AP_GROUPINFO("FRAME", 15, AP_ARINC429, _frame, 0),

    // @Param: ENG_CH
    // @DisplayName: Engine run RC channel
    // @Description: RC channel that requests engine run. 0 leaves labels 340, 342 and 344 on the simulation snapshot. 1 to 16 select a channel. PWM above 1800 requests run. Low, middle, an invalid sample, or RC failsafe requests stop. There is no separate start label: idle makes the ECU sequence the starter and ignition. Motor interlock is aux function 32, not this channel. This does not move motors, servos, or eRob.
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("ENG_CH", 16, AP_ARINC429, _eng_ch, 0),

    // @Param: NR_INST
    // @DisplayName: Rotor RPM sensor instance
    // @Description: AP_RPM instance for the optional runup check. 0 is the first sensor (RPM1) and 1 is RPM2. A missing sample does not by itself abort flight. The check runs only when NR_REF, NR_CRIT and RUNUP are all non-zero.
    // @Range: 0 7
    // @User: Advanced
    AP_GROUPINFO("NR_INST", 17, AP_ARINC429, _nr_inst, 0),

    // @Param: NR_REF
    // @DisplayName: Rotor rpm at 100 percent Nr
    // @Description: Rotor rpm that is 100 percent Nr. Nr percent is measured rpm times 100 divided by this value. 0 disables the runup check. Flight is still commanded before Nr reaches NR_CRIT.
    // @Units: rpm
    // @Range: 0 100000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("NR_REF", 18, AP_ARINC429, _nr_ref, 0),

    // @Param: NR_CRIT
    // @DisplayName: Runup Nr threshold
    // @Description: Nr percent that ends the runup watch. 0 disables the runup check. Reaching this value latches success, so a later dip does not return the command to idle.
    // @Units: %
    // @Range: 0 150
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("NR_CRIT", 19, AP_ARINC429, _nr_crit, 0),

    // @Param: RUNUP
    // @DisplayName: Runup time limit
    // @Description: Seconds Nr may stay below NR_CRIT after flight is commanded. 0 disables the runup check. If the time elapses while a valid RPM sample is still below NR_CRIT, labels 340, 342 and 344 return to idle until motor interlock is released or the aircraft is disarmed. No RPM sample does not abort. NR_REF, NR_CRIT and RUNUP must all be non-zero.
    // @Units: s
    // @Range: 0 120
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("RUNUP", 20, AP_ARINC429, _runup, 0),

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
    a429_core_set_sdi(&_core, (uint8_t)sdi);
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
    {
        int8_t frame = _frame.get();
        uint8_t profile = A429_FRAME_NONE;
        if (frame == (int8_t)A429_FRAME_UART429S_C0) {
            profile = A429_FRAME_UART429S_C0;
        } else if (frame == (int8_t)A429_FRAME_RS422_429_1C) {
            profile = A429_FRAME_RS422_429_1C;
        }
        a429_core_set_frame(&_core, profile);
    }
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

uint32_t AP_ARINC429::tcp_stream_now(void *ctx)
{
    (void)ctx;
    return AP_HAL::micros();
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
        a429_tcp_set_clock(&_core.tcp, tcp_stream_now, this);
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

void AP_ARINC429::command_engine_mode(void)
{
    a429_mode_in_t in;
    a429_pcmd_levels_t levels;
    int step;
    int16_t ch;

    memset(&in, 0, sizeof(in));
    in.enable = _enable.get() == 0 ? 0u : 1u;
    ch = _eng_ch.get();
    if (ch < 0 || ch > 255) {
        ch = 0;
    }
    in.eng_ch = (uint8_t)ch;
    if (in.enable && in.eng_ch >= 1u && in.eng_ch <= A429_ENG_CH_MAX) {
        const bool failsafe = rc().in_rc_failsafe();
        const bool rc_ok = rc().has_valid_input() && !failsafe;
        RC_Channel *chan = rc().channel((uint8_t)(in.eng_ch - 1u));
        RC_Channel *ilock = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK);
        int16_t pwm = 0;

        in.rc_failsafe = failsafe ? 1u : 0u;
        if (chan != nullptr) {
            in.rc_present = 1u;
            pwm = chan->get_radio_in();
            if (pwm > 0) {
                in.rc_pwm = (uint16_t)pwm;
            }
            if (rc_ok &&
                pwm > (int16_t)RC_Channel::RC_MIN_LIMIT_PWM &&
                pwm < (int16_t)RC_Channel::RC_MAX_LIMIT_PWM) {
                in.rc_valid = 1u;
            }
        }
        if (ilock == nullptr) {
            in.interlock_released = 1u;
        } else if (rc_ok) {
            if (ilock->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
                in.interlock_high = 1u;
            } else {
                in.interlock_released = 1u;
            }
        }
        in.armed = AP::arming().is_armed() ? 1u : 0u;
        in.on_ground = AP_Notify::flags.flying ? 0u : 1u;
#if AP_RPM_ENABLED
        {
            AP_RPM *sensor = AP::rpm();
            float rpm = 0.0f;
            int8_t inst = _nr_inst.get();
            if (sensor != nullptr && inst >= 0 && sensor->get_rpm((uint8_t)inst, rpm)) {
                in.nr_sample = 1u;
                in.nr_rpm = rpm;
            }
        }
#endif
        in.nr_ref = _nr_ref.get();
        in.nr_crit_pct = _nr_crit.get();
        if (is_positive(_runup.get())) {
            float runup_s = _runup.get();
            if (runup_s > 4000000.0f) {
                runup_s = 4000000.0f;
            }
            in.runup_ms = (uint32_t)(runup_s * 1000.0f);
        }
        in.now_ms = AP_HAL::millis();
    }

    step = a429_mode_step(&_mode_state, &in, &levels);
    if (step > 0) {
        uint8_t i;
        for (i = 0; i < A429_PCMD_N; i++) {
            a429_sim_set_pcmd(&_core.link, i, &levels);
        }
    } else if (step < 0) {
        a429_sim_clear_pcmd(&_core.link);
    }
}

void AP_ARINC429::update(void)
{
    if (!_inited) {
        return;
    }
    if (_enable.get() == 0) {
        /* Drop a previous takeover. Default ENG_CH 0 never owned the words,
         * so this does not wipe a simulation snapshot. */
        command_engine_mode();
        a429_tcp_configure(&_core.tcp, 0, 0, 0);
        a429_core_set_frame(&_core, A429_FRAME_NONE);
        a429_link_session_lost(&_core.link);
        return;
    }
    apply_params();
    command_engine_mode();
    /*
     * Scheduling and byte-pipe maintenance. The three command words are
     * not copied into motors, servos, EFI, or eRob. Unconfigured framing
     * sends no bytes, and no CD rate command is generated.
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
    float value;
    if (a429_core_fresh_eu(&_core, rx_index, &value) != 1) {
        return false;
    }
    eu = value;
    return true;
}

bool AP_ARINC429::get_fresh_disc(uint8_t rx_index, int16_t &data) const
{
    int16_t value;
    if (a429_core_fresh_disc(&_core, rx_index, &value) != 1) {
        return false;
    }
    data = value;
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
#include "src/arinc429_mode.c"

#endif  // AP_ARINC429_ENABLED
