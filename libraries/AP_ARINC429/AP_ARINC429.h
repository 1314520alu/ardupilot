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
#include <AP_Networking/AP_Networking_Config.h>
#if AP_NETWORKING_ENABLED
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/Socket.h>
#endif

/*
 * CS35 ARINC429 codec on the flight-controller side.
 * Received words are monitoring data. Label 310 is the helicopter
 * collective output converted to volts. Label 311 is measured rotor speed
 * from AP_RPM when A429_NR_REF is a positive scale. Labels 312, 313, 314
 * and 316 are ambient pressure, baro temperature, absolute altitude and
 * sensor true airspeed when those samples are valid. Labels 321..326 are
 * UTC from GPS or the RTC, all six together. Labels 317, 320 and 331 are
 * not sent. Labels 340/342/344 are filled from the pilot engine switch
 * when A429_ENG_CH is 1..16.
 * Those words do not move a motor, a servo, or eRob, and this class does
 * not claim Ethernet.
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
    bool get_fresh_disc(uint8_t rx_index, int16_t &data) const;
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
    void command_engine_mode(void);
    void publish_measured_nr(void);
    void publish_collective(void);
    void publish_airdata(void);
    bool read_measured_nr(float &rpm) const;
    bool read_collective_pct(float &pct) const;
    bool read_pressure_kpa(float &kpa) const;
    bool read_temperature_c(float &degc) const;
    bool read_altitude_amsl_m(float &alt_m) const;
    bool read_tas_mps(float &tas) const;
    bool read_utc_ymdhms(int &hour, int &minute, int &second,
                         int &year, int &month, int &day) const;
#if AP_NETWORKING_ENABLED
    void tcp_worker(void);
    static int tcp_stream_open(void *ctx, uint32_t ipv4_host, uint16_t port);
    static int tcp_stream_send(void *ctx, const uint8_t *bytes, uint16_t n);
    static int tcp_stream_recv(void *ctx, uint8_t *bytes, uint16_t n);
    static void tcp_stream_close(void *ctx);
    static void tcp_lock_cb(void *ctx);
    static void tcp_unlock_cb(void *ctx);
    static uint32_t tcp_stream_now(void *ctx);
    bool _tcp_thread_started;
    SocketAPM *_tcp_sock;
    HAL_Semaphore _tcp_sem;
#endif

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
    AP_Int8 _frame;
    AP_Int8 _eng_ch;
    AP_Int8 _nr_inst;
    AP_Float _nr_ref;
    AP_Float _nr_crit;
    AP_Float _runup;
    AP_Float _clp_v0;
    AP_Float _clp_v100;

    a429_core_t _core;
    a429_mode_state_t _mode_state;
    bool _inited;

    static AP_ARINC429 *_singleton;
};

namespace AP
{
AP_ARINC429 *arinc429();
};

#endif  // AP_ARINC429_ENABLED
