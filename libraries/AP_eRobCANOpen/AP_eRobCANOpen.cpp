/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/*
 * AP_eRobCANOpen binds the minimal eRob master to CAN2.
 * The files under src/ are included here so waf's top-level glob does not also compile them,
 * and so the C master is absent from builds with AP_EROB_CANOPEN_ENABLED=0.
 */

#include "AP_eRobCANOpen_config.h"

#if AP_EROB_CANOPEN_ENABLED

#include "AP_eRobCANOpen.h"

#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_eRobCANOpen::var_info[] = {

    // @Param: NODE
    // @DisplayName: eRob CAN first node
    // @Description: Node-ID of the first eRob servo selected by the servo bitmask, counting set bits from low to high. The driver owns the whole iface. Only NMT, SYNC and these nodes' PDO, SDO, EMCY and heartbeat are used. Other frames on that port are discarded.
    // @Range: 1 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("NODE", 1, AP_eRobCANOpen, _node_id, 1),

    // @Param: BM
    // @DisplayName: eRob CAN servo bitmask
    // @Description: Servo outputs transmitted as eRob CAN CSP targets. Same bits as CAN_Dx_UC_SRV_BM: bit 0 is Servo 1. Set bits are taken from low to high. The lowest bit uses NODE, and each next set bit uses the next Node-ID. At most 6 axes. 0 sends no servo targets.
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("BM", 6, AP_eRobCANOpen, _servo_bm, 0),

    // index 2 was SRV and index 3 was NUM on this branch

    // @Param: MIN
    // @DisplayName: eRob CAN position min
    // @Description: CSP target in plus when the servo output is at SERVOx_MIN. All axes share this end. 524288 plus is one output revolution.
    // @User: Advanced
    AP_GROUPINFO("MIN", 4, AP_eRobCANOpen, _pos_min, 0),

    // @Param: MAX
    // @DisplayName: eRob CAN position max
    // @Description: CSP target in plus when the servo output is at SERVOx_MAX. All axes share this end. 524288 plus is one output revolution. Per-axis travel is not separate yet.
    // @User: Advanced
    AP_GROUPINFO("MAX", 5, AP_eRobCANOpen, _pos_max, 524288),

    AP_GROUPEND
};

AP_eRobCANOpen::AP_eRobCANOpen()
{
    AP_Param::setup_object_defaults(this, var_info);
    _bus.drv = this;
    _active_node = 1;
    _num_axes = 0;
    _started = false;
}

uint8_t AP_eRobCANOpen::node_id() const
{
    const int16_t n = _node_id.get();
    if (n < 1 || n > 127) {
        return 1;
    }
    return uint8_t(n);
}

AP_eRobCANOpen *AP_eRobCANOpen::get_erob(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::eRobCAN) {
        return nullptr;
    }
    return static_cast<AP_eRobCANOpen*>(AP::can().get_driver(driver_index));
}

bool AP_eRobCANOpen::add_interface(AP_HAL::CANIface* can_iface)
{
    _active_node = node_id();

#if HAL_NUM_CAN_IFACES < 2
    (void)can_iface;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN binds CAN2 only");
    return false;
#else
    if (can_iface == nullptr || can_iface != hal.can[1]) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN binds CAN2 only");
        return false;
    }
    if (_iface != nullptr) {
        return false;
    }
    if (!can_iface->is_initialized()) {
        return false;
    }
    if (!can_iface->set_event_handle(&_rx_sem)) {
        return false;
    }
    _iface = can_iface;
    return true;
#endif
}

void AP_eRobCANOpen::setup_axes()
{
    int16_t node = _node_id.get();
    if (node < 1 || node > 127) {
        node = 1;
    }
    _active_node = uint8_t(node);

    const uint32_t mask = uint32_t(_servo_bm.get());
    uint8_t num = 0;
    bool truncated = false;
    const uint8_t nchan = NUM_SERVO_CHANNELS < 32 ? NUM_SERVO_CHANNELS : 32;
    for (uint8_t chan = 0; chan < nchan; chan++) {
        if ((mask & (1U << chan)) == 0) {
            continue;
        }
        if (num >= EROB_MAX_AXES || node + num > 127) {
            truncated = true;
            break;
        }
        _axis[num].node = uint8_t(node + num);
        _axis[num].channel = chan;
        _axis[num].tried = false;
        _axis[num].reported = false;
        _axis[num].last_try_ms = 0;
        co_master_init(&_axis[num].master, &_bus, _axis[num].node);
        erob_init(&_axis[num].servo, &_axis[num].master);
        num++;
    }
    if (num == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eRob CAN: servo mask is 0");
    } else if (truncated) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: servo mask limited to %u axes", unsigned(num));
    }
    // Publish after the axes exist. push_servos() runs on another thread.
    _num_axes = num;
}

bool AP_eRobCANOpen::try_configure(uint8_t index, uint32_t now_ms)
{
    Axis &axis = _axis[index];
    const erob_state_t st = erob_get_state(&axis.servo);
    if (st == EROB_ST_ENABLED || st == EROB_ST_ENABLE_SEQ || st == EROB_ST_FAULT) {
        return false;
    }
    if (axis.tried && (now_ms - axis.last_try_ms) < 5000) {
        return false;
    }
    axis.tried = true;
    axis.last_try_ms = now_ms;
    if (erob_configure_pdo_csp(&axis.servo) == 0) {
        // One SYNC producer for the group. Slaves do not produce SYNC.
        if (index != 0) {
            co_master_set_sync(&axis.master, false, 0);
        }
        (void)erob_enable(&axis.servo);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: node %u configure failed",
                      unsigned(axis.node));
    }
    return true;
}

void AP_eRobCANOpen::poll_axes()
{
    can_frame_t fr;
    while (hal_recv(&fr, 0) == 0) {
        uint8_t index = 0;
        if (id_for_axis(uint16_t(fr.id), index)) {
            co_master_handle_frame(&_axis[index].master, &fr);
        }
    }
    for (uint8_t i = 0; i < _num_axes; i++) {
        erob_poll_motion(&_axis[i].servo);
        if (!_axis[i].reported && erob_get_state(&_axis[i].servo) == EROB_ST_ENABLED) {
            _axis[i].reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eRob CAN: servo node %u enabled",
                          unsigned(_axis[i].node));
        }
    }
    if (_num_axes > 0) {
        co_master_poll_sync(&_axis[0].master);
    }
}

void AP_eRobCANOpen::init(uint8_t driver_index)
{
    _driver_index = driver_index;
    _active_node = node_id();
    if (_iface == nullptr || _started) {
        return;
    }

    // Name the thread before create. PiccoloCAN sets the name afterwards.
    snprintf(_thread_name, sizeof(_thread_name), "erobcan%u", unsigned(driver_index));
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_eRobCANOpen::loop, void),
                                      _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        return;
    }
    _started = true;
}

void AP_eRobCANOpen::loop()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(50);
    }

    _bus.drv = this;
    setup_axes();

    while (true) {
        const uint32_t now = AP_HAL::millis();
        for (uint8_t i = 0; i < _num_axes; i++) {
            if (try_configure(i, now)) {
                break;
            }
        }
        poll_axes();
        hal.scheduler->delay_microseconds(1000);
    }
}

bool AP_eRobCANOpen::frame_for_master(const AP_HAL::CANFrame &rx, can_frame_t *fr)
{
    uint8_t len;

    if (rx.isExtended() || rx.isErrorFrame() || rx.isRemoteTransmissionRequest() || rx.isCanFDFrame()) {
        return false;
    }
    len = rx.dlc > 8 ? 8 : rx.dlc;
    const uint16_t std_id = uint16_t(rx.id & AP_HAL::CANFrame::MaskStdID);
    bool reserved = false;
    if (_num_axes == 0) {
        reserved = erob_id_is_reserved(std_id, _active_node);
    } else {
        for (uint8_t i = 0; i < _num_axes; i++) {
            if (erob_id_is_reserved(std_id, _axis[i].node)) {
                reserved = true;
                break;
            }
        }
    }
    if (!reserved) {
        return false;
    }
    memset(fr, 0, sizeof(*fr));
    fr->id = std_id;
    fr->dlc = len;
    if (len > 0) {
        memcpy(fr->data, rx.data, len);
    }
    return true;
}

bool AP_eRobCANOpen::id_for_axis(uint16_t std_id, uint8_t &index) const
{
    if (std_id == CO_COB_NMT || std_id == CO_COB_SYNC) {
        return false;
    }
    for (uint8_t i = 0; i < _num_axes; i++) {
        if (erob_id_is_reserved(std_id, _axis[i].node)) {
            index = i;
            return true;
        }
    }
    return false;
}

void AP_eRobCANOpen::push_servos()
{
    if (_num_axes == 0) {
        return;
    }
    const uint32_t disabled = SRV_Channels::get_disabled_channel_mask();
    for (uint8_t i = 0; i < _num_axes; i++) {
        const uint8_t chan = _axis[i].channel;
        if (chan >= NUM_SERVO_CHANNELS) {
            continue;
        }
        if ((disabled & (1U << chan)) != 0) {
            continue;
        }
        SRV_Channel *ch = SRV_Channels::srv_channel(chan);
        if (ch == nullptr) {
            continue;
        }
        const uint16_t pwm = ch->get_output_pwm();
        if (pwm == 0) {
            continue;
        }
        const int32_t plus = erob_pwm_to_plus(pwm, ch->get_output_min(), ch->get_output_max(),
                                              _pos_min.get(), _pos_max.get());
        erob_set_target_pos(&_axis[i].servo, plus);
    }
}

void AP_eRobCANOpen::set_target_pos(int32_t plus)
{
    if (_num_axes == 0) {
        return;
    }
    erob_set_target_pos(&_axis[0].servo, plus);
}

int32_t AP_eRobCANOpen::get_position() const
{
    if (_num_axes == 0) {
        return 0;
    }
    return erob_get_position(&_axis[0].servo);
}

uint16_t AP_eRobCANOpen::get_statusword() const
{
    if (_num_axes == 0) {
        return 0;
    }
    return erob_get_statusword(&_axis[0].servo);
}

int AP_eRobCANOpen::hal_send(const can_frame_t *fr)
{
    if (_iface == nullptr || fr == nullptr || fr->is_ext || fr->is_rtr || fr->dlc > 8) {
        return -1;
    }
    AP_HAL::CANFrame tx(fr->id & AP_HAL::CANFrame::MaskStdID, fr->data, fr->dlc, false);
    if (_iface->send(tx, AP_HAL::micros64() + 5000, 0) <= 0) {
        return -1;
    }
    return 0;
}

int AP_eRobCANOpen::hal_recv(can_frame_t *fr, uint32_t timeout_ms)
{
    if (_iface == nullptr || fr == nullptr) {
        return -1;
    }

    const uint64_t deadline = AP_HAL::micros64() + uint64_t(timeout_ms) * 1000ULL;
    for (;;) {
        AP_HAL::CANFrame rx;
        uint64_t ts = 0;
        AP_HAL::CANIface::CanIOFlags flags = 0;
        if (_iface->receive(rx, ts, flags) > 0) {
            if (frame_for_master(rx, fr)) {
                return 0;
            }
            if (timeout_ms != 0 && AP_HAL::micros64() >= deadline) {
                return -1;
            }
            continue;
        }
        if (timeout_ms == 0 || AP_HAL::micros64() >= deadline) {
            return -1;
        }
        bool read_select = true;
        bool write_select = false;
        const bool ready = _iface->select(read_select, write_select, nullptr, deadline);
        if (!ready || !read_select) {
            if (AP_HAL::micros64() >= deadline) {
                return -1;
            }
            hal.scheduler->delay_microseconds(50);
        }
    }
}

extern "C" int can_bus_send(can_bus_t *bus, const can_frame_t *fr)
{
    if (bus == nullptr || bus->drv == nullptr) {
        return -1;
    }
    return bus->drv->hal_send(fr);
}

extern "C" int can_bus_recv(can_bus_t *bus, can_frame_t *fr, uint32_t timeout_ms)
{
    if (bus == nullptr || bus->drv == nullptr) {
        return -1;
    }
    return bus->drv->hal_recv(fr, timeout_ms);
}

extern "C" uint32_t can_bus_millis(can_bus_t *bus)
{
    (void)bus;
    return AP_HAL::millis();
}

extern "C" {
#include "src/canopen_master.c"
#include "src/erob_servo.c"
}

#endif  // AP_EROB_CANOPEN_ENABLED
