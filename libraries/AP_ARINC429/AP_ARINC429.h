/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include "AP_ARINC429_config.h"

#if AP_ARINC429_ENABLED

#include "arinc429.h"

#include <AP_Param/AP_Param.h>

/*
 * CS35 ARINC429 codec on the flight-controller side.
 * Received words are monitoring data. Transmit words are built only from
 * an explicit simulation snapshot. Nothing in this class commands an engine,
 * a servo, or a motor, and it does not open the Ethernet port.
 */
class AP_ARINC429
{
public:
    AP_ARINC429();

    static AP_ARINC429 *get_singleton(void)
    {
        return _singleton;
    }

    void init(void);
    void update(void);

    void sim_inject_rx(uint32_t raw);
    void sim_set_clp_v(float volts);
    void sim_set_nr_pct(float pct);
    bool sim_set_pcmd(uint8_t index, const a429_pcmd_levels_t &levels);
    void sim_clear_tx(void);

    bool get_fresh_eu(uint8_t rx_index, float &eu) const;
    uint32_t parity_error_count(void) const
    {
        return _core.link.counters.parity_errors;
    }
    uint32_t tx_not_sent_count(void) const
    {
        return _core.tx_not_sent;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void apply_params(void);

    AP_Int8 _enable;
    AP_Int8 _sdi;
    AP_Int16 _to_ng_ms;
    AP_Int16 _to_np_ms;
    AP_Int16 _to_t45_ms;
    AP_Int16 _to_mkp_ms;
    AP_Int16 _to_wn1_ms;
    AP_Int16 _to_st_ms;
    AP_Int16 _to_wn2_ms;
    AP_Int16 _ip0;
    AP_Int16 _ip1;
    AP_Int16 _ip2;
    AP_Int16 _ip3;
    AP_Int32 _rem_port;
    AP_Int32 _loc_port;

    a429_core_t _core;
    bool _inited;

    static AP_ARINC429 *_singleton;
};

namespace AP
{
AP_ARINC429 *arinc429();
};

#endif  // AP_ARINC429_ENABLED
