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
    // @DisplayName: Future carrier address octet 0
    // @Description: Stored remote IPv4 octet 0 for a future ARINC429 carrier. Unused. Does not open a socket and does not choose UDP or TCP.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP0", 9, AP_ARINC429, _ip0, 0),

    // @Param: IP1
    // @DisplayName: Future carrier address octet 1
    // @Description: Stored remote IPv4 octet 1 for a future ARINC429 carrier. Unused.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP1", 10, AP_ARINC429, _ip1, 0),

    // @Param: IP2
    // @DisplayName: Future carrier address octet 2
    // @Description: Stored remote IPv4 octet 2 for a future ARINC429 carrier. Unused.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP2", 11, AP_ARINC429, _ip2, 0),

    // @Param: IP3
    // @DisplayName: Future carrier address octet 3
    // @Description: Stored remote IPv4 octet 3 for a future ARINC429 carrier. Unused.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IP3", 12, AP_ARINC429, _ip3, 0),

    // @Param: REM_PORT
    // @DisplayName: Future carrier remote port
    // @Description: Stored remote port for a future ARINC429 carrier. Unused. The transport protocol is unspecified.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("REM_PORT", 13, AP_ARINC429, _rem_port, 0),

    // @Param: LOC_PORT
    // @DisplayName: Future carrier local port
    // @Description: Stored local port for a future ARINC429 carrier. Unused. This does not bind the shared Ethernet port.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("LOC_PORT", 14, AP_ARINC429, _loc_port, 0),

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
    _core.endpoint.carrier = A429_CARRIER_UNSPECIFIED;
}

void AP_ARINC429::init(void)
{
    a429_core_init(&_core);
    _inited = true;
}

void AP_ARINC429::update(void)
{
    if (!_inited || _enable.get() == 0) {
        return;
    }
    apply_params();
    /*
     * Scheduling and word assembly only. The null carrier returns no
     * frames. Fresh values are not copied into motors, servos, or EFI.
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
#include "src/arinc429_transport.c"

#endif  // AP_ARINC429_ENABLED
