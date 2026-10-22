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

#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_eRobCANOpen::var_info[] = {

    // @Param: NODE
    // @DisplayName: eRob CANopen node id
    // @Description: Node-ID of the eRob on this driver. The driver owns the whole iface. NMT, SYNC and this node's PDO, SDO, EMCY and heartbeat are the only frames it uses. Other frames on that port are discarded.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("NODE", 1, AP_eRobCANOpen, _node_id, 1),

    AP_GROUPEND
};

AP_eRobCANOpen::AP_eRobCANOpen()
{
    AP_Param::setup_object_defaults(this, var_info);
    _bus.drv = this;
    _active_node = 1;
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
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::ERobCANOpen) {
        return nullptr;
    }
    return static_cast<AP_eRobCANOpen*>(AP::can().get_driver(driver_index));
}

bool AP_eRobCANOpen::add_interface(AP_HAL::CANIface* can_iface)
{
    _active_node = node_id();

#if HAL_NUM_CAN_IFACES < 2
    (void)can_iface;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ERobCANopen binds CAN2 only");
    return false;
#else
    if (can_iface == nullptr || can_iface != hal.can[1]) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ERobCANopen binds CAN2 only");
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
    co_master_init(&_master, &_bus, _active_node);
    erob_init(&_servo, &_master);

    bool tried = false;
    uint32_t last_try_ms = 0;
    while (true) {
        const erob_state_t st = erob_get_state(&_servo);
        const uint32_t now = AP_HAL::millis();
        if (st != EROB_ST_ENABLED && st != EROB_ST_ENABLE_SEQ && st != EROB_ST_FAULT) {
            if (!tried || (now - last_try_ms) >= 5000) {
                tried = true;
                last_try_ms = now;
                if (erob_configure_pdo_csp(&_servo) == 0) {
                    (void)erob_enable(&_servo);
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ERobCANopen: node %u configure failed",
                                  unsigned(_active_node));
                }
            }
        }
        erob_poll(&_servo);
        if (!_reported && erob_get_state(&_servo) == EROB_ST_ENABLED) {
            _reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ERobCANopen: node %u enabled", unsigned(_active_node));
        }
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
    if (!erob_id_is_reserved(std_id, _active_node)) {
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

void AP_eRobCANOpen::set_target_pos(int32_t plus)
{
    erob_set_target_pos(&_servo, plus);
}

int32_t AP_eRobCANOpen::get_position() const
{
    return erob_get_position(&_servo);
}

uint16_t AP_eRobCANOpen::get_statusword() const
{
    return erob_get_statusword(&_servo);
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
