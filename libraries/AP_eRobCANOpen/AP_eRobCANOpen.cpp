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
#include <AP_Logger/AP_Logger_config.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#if HAL_LOGGING_ENABLED
#include <AP_Logger/AP_Logger.h>
#endif

#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_eRobCANOpen::var_info[] = {

    // @Param: NODE
    // @DisplayName: eRob CAN first node
    // @Description: Node-ID of the first eRob servo selected by the servo bitmask, counting set bits from low to high. Used only when SV1..SV8 are all 0. If any SVn is non-zero, the SV/ID slots are used and NODE is ignored. The driver owns the whole iface. Only NMT, SYNC and these nodes' PDO, SDO, EMCY and heartbeat are used. Other frames on that port are discarded. Node-IDs are hard-set on the servos.
    // @Range: 1 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("NODE", 1, AP_eRobCANOpen, _node_id, 1),

    // @Param: BM
    // @DisplayName: eRob CAN servo bitmask
    // @Description: Servo outputs transmitted as eRob CAN CSP targets when SV1..SV8 are all 0. Same bits as CAN_Dx_UC_SRV_BM: bit 0 is Servo 1. Set bits are taken from low to high. The lowest bit uses NODE, and each next set bit uses the next Node-ID. At most 8 axes. 0 sends no servo targets. If any SVn is non-zero, this mask is ignored.
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("BM", 6, AP_eRobCANOpen, _servo_bm, 0),

    // index 2 was SRV and index 3 was NUM on this branch

    // @Param: MIN
    // @DisplayName: eRob CAN position min
    // @Description: CSP target in plus when the servo output is at SERVOx_MIN. All axes share this end. One output revolution is RES counts, default 524288. This limit stays in plus.
    // @User: Advanced
    AP_GROUPINFO("MIN", 4, AP_eRobCANOpen, _pos_min, 0),

    // @Param: MAX
    // @DisplayName: eRob CAN position max
    // @Description: CSP target in plus when the servo output is at SERVOx_MAX. All axes share this end. One output revolution is RES counts, default 524288. This limit stays in plus. Per-axis travel is not separate yet.
    // @User: Advanced
    AP_GROUPINFO("MAX", 5, AP_eRobCANOpen, _pos_max, 524288),

    // @Param: GEAR
    // @DisplayName: eRob CAN gear ratio
    // @Description: Motor turns per output revolution. 0 leaves motor rpm invalid. Output rpm and output degrees do not use this. On eRob142 a nameplate ratio of 120 is entered as 121 (ratio + 1). All axes on this driver share it.
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR", 7, AP_eRobCANOpen, _gear, 0),

    // @Param: RES
    // @DisplayName: eRob CAN output counts per rev
    // @Description: Output-encoder counts in one revolution. Converts 0x6064 to degrees and 0x606C to output rpm. Default 524288 is the 19-bit scale. A high-precision single-turn encoder may be 1048576. Turn the output one revolution and read 0x6064 before changing this. 0 uses 524288. MIN and MAX stay in plus and are not rewritten. All axes share it. Manual V1.9 has no 0x608F, so this is not read from the drive.
    // @Range: 0 16777216
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RES", 24, AP_eRobCANOpen, _pos_res, 524288),

    // @Param: SV1
    // @DisplayName: eRob CAN slot 1 servo
    // @Description: Servo output number for slot 1, so SERVO this number drives ID1. 0 leaves the slot unused. If any SVn is non-zero, slots replace NODE and BM. If every SVn is 0, NODE plus BM is used. MIN, MAX, GEAR and RES stay shared. A duplicate servo, a value outside 1..32, or a servo with ID 0 is skipped.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV1", 8, AP_eRobCANOpen, _slot_sv[0], 0),

    // @Param: SV2
    // @DisplayName: eRob CAN slot 2 servo
    // @Description: Servo output number for slot 2. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV2", 9, AP_eRobCANOpen, _slot_sv[1], 0),

    // @Param: SV3
    // @DisplayName: eRob CAN slot 3 servo
    // @Description: Servo output number for slot 3. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV3", 10, AP_eRobCANOpen, _slot_sv[2], 0),

    // @Param: SV4
    // @DisplayName: eRob CAN slot 4 servo
    // @Description: Servo output number for slot 4. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV4", 11, AP_eRobCANOpen, _slot_sv[3], 0),

    // @Param: SV5
    // @DisplayName: eRob CAN slot 5 servo
    // @Description: Servo output number for slot 5. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV5", 12, AP_eRobCANOpen, _slot_sv[4], 0),

    // @Param: SV6
    // @DisplayName: eRob CAN slot 6 servo
    // @Description: Servo output number for slot 6. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV6", 13, AP_eRobCANOpen, _slot_sv[5], 0),

    // @Param: SV7
    // @DisplayName: eRob CAN slot 7 servo
    // @Description: Servo output number for slot 7. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV7", 14, AP_eRobCANOpen, _slot_sv[6], 0),

    // @Param: SV8
    // @DisplayName: eRob CAN slot 8 servo
    // @Description: Servo output number for slot 8. 0 leaves the slot unused. See SV1.
    // @Range: 0 32
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SV8", 15, AP_eRobCANOpen, _slot_sv[7], 0),

    // @Param: ID1
    // @DisplayName: eRob CAN slot 1 Node-ID
    // @Description: CANopen Node-ID driven by the servo in SV1. 1..127. The servo's ID is hard-set on the drive. This driver does not write object 0x100B. SV1 set with ID 0 is skipped. Ignored when SV1 is 0.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID1", 16, AP_eRobCANOpen, _slot_id[0], 0),

    // @Param: ID2
    // @DisplayName: eRob CAN slot 2 Node-ID
    // @Description: CANopen Node-ID for SV2. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID2", 17, AP_eRobCANOpen, _slot_id[1], 0),

    // @Param: ID3
    // @DisplayName: eRob CAN slot 3 Node-ID
    // @Description: CANopen Node-ID for SV3. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID3", 18, AP_eRobCANOpen, _slot_id[2], 0),

    // @Param: ID4
    // @DisplayName: eRob CAN slot 4 Node-ID
    // @Description: CANopen Node-ID for SV4. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID4", 19, AP_eRobCANOpen, _slot_id[3], 0),

    // @Param: ID5
    // @DisplayName: eRob CAN slot 5 Node-ID
    // @Description: CANopen Node-ID for SV5. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID5", 20, AP_eRobCANOpen, _slot_id[4], 0),

    // @Param: ID6
    // @DisplayName: eRob CAN slot 6 Node-ID
    // @Description: CANopen Node-ID for SV6. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID6", 21, AP_eRobCANOpen, _slot_id[5], 0),

    // @Param: ID7
    // @DisplayName: eRob CAN slot 7 Node-ID
    // @Description: CANopen Node-ID for SV7. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID7", 22, AP_eRobCANOpen, _slot_id[6], 0),

    // @Param: ID8
    // @DisplayName: eRob CAN slot 8 Node-ID
    // @Description: CANopen Node-ID for SV8. See ID1.
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ID8", 23, AP_eRobCANOpen, _slot_id[7], 0),

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

static constexpr uint32_t EROB_FAULT_WARN_MS = 5000;

bool AP_eRobCANOpen::slots_configured() const
{
    for (uint8_t i = 0; i < EROB_SLOT_COUNT; i++) {
        if (_slot_sv[i].get() != 0) {
            return true;
        }
    }
    return false;
}

void AP_eRobCANOpen::add_axis(uint8_t &num, uint8_t node, uint8_t channel)
{
    Axis &axis = _axis[num];
    axis.node = node;
    axis.channel = channel;
    axis.tried = false;
    axis.reported = false;
    axis.fault_active = false;
    axis.last_try_ms = 0;
    axis.last_log_ms = 0;
    axis.last_named_ms = 0;
    axis.last_fault_ms = 0;
    co_master_init(&axis.master, &_bus, node);
    erob_init(&axis.servo, &axis.master);
    num++;
}

void AP_eRobCANOpen::setup_axes()
{
    static_assert(EROB_SLOT_COUNT <= EROB_MAX_AXES, "eRob slots exceed the axis table");

    uint8_t num = 0;
    if (slots_configured()) {
        bool seen_id[128] {};
        bool seen_sv[33] {};
        bool truncated = false;
        const uint8_t sv_max = NUM_SERVO_CHANNELS < 32 ? NUM_SERVO_CHANNELS : 32;
        for (uint8_t i = 0; i < EROB_SLOT_COUNT; i++) {
            const int16_t sv = _slot_sv[i].get();
            const int16_t id = _slot_id[i].get();
            const unsigned slot = unsigned(i + 1);
            if (sv == 0) {
                continue;
            }
            if (sv < 1 || sv > sv_max) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: slot %u SERVO %d out of range", slot, int(sv));
                continue;
            }
            if (id == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: slot %u ID 0 skipped", slot);
                continue;
            }
            if (id < 1 || id > 127) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: slot %u ID %d out of range", slot, int(id));
                continue;
            }
            if (seen_id[id]) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: slot %u duplicate ID %d skipped", slot, int(id));
                continue;
            }
            if (seen_sv[sv]) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: slot %u duplicate SERVO %d skipped", slot, int(sv));
                continue;
            }
            if (num >= EROB_MAX_AXES) {
                truncated = true;
                break;
            }
            seen_id[id] = true;
            seen_sv[sv] = true;
            add_axis(num, uint8_t(id), uint8_t(sv - 1));
        }
        if (num == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: no servo slots");
        } else if (truncated) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: slot table limited to %u", unsigned(num));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eRob CAN: %u slots, NODE and BM ignored", unsigned(num));
        }
    } else {
        int16_t node = _node_id.get();
        if (node < 1 || node > 127) {
            node = 1;
        }
        _active_node = uint8_t(node);

        const uint32_t mask = uint32_t(_servo_bm.get());
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
            add_axis(num, uint8_t(node + num), chan);
        }
        if (num == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eRob CAN: servo mask is 0");
        } else if (truncated) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob CAN: servo mask limited to %u axes", unsigned(num));
        }
    }
    if (num > 0) {
        _active_node = _axis[0].node;
    }
    // Publish after the axes exist. push_servos() runs on another thread.
    _num_axes = num;
}

void AP_eRobCANOpen::probe_nodes()
{
    if (_num_axes == 0) {
        return;
    }

    bool alive[EROB_MAX_AXES] {};
    // One broadcast reset so a powered servo emits boot-up. Configure repeats
    // a per-node reset afterwards. This does not write 0x100B.
    (void)co_nmt(&_axis[0].master, CO_NMT_RESET_COMM, 0);

    const uint32_t t0 = AP_HAL::millis();
    while ((AP_HAL::millis() - t0) < EROB_BOOT_TIMEOUT_MS) {
        bool all = true;
        for (uint8_t i = 0; i < _num_axes; i++) {
            if (!alive[i]) {
                all = false;
                break;
            }
        }
        if (all) {
            break;
        }
        can_frame_t fr;
        if (hal_recv(&fr, 20) != 0) {
            continue;
        }
        if (fr.dlc < 1 || fr.data[0] != 0) {
            continue;
        }
        for (uint8_t i = 0; i < _num_axes; i++) {
            if (fr.id == uint32_t(CO_COB_HB_BASE) + _axis[i].node) {
                alive[i] = true;
            }
        }
    }

    for (uint8_t i = 0; i < _num_axes; i++) {
        if (alive[i]) {
            continue;
        }
        uint32_t device_type = 0;
        if (co_sdo_read(&_axis[i].master, 0x1000, 0x00, &device_type, 4) == 0 ||
            _axis[i].master.boot_seen) {
            alive[i] = true;
        }
        (void)device_type;
    }

    for (uint8_t i = 0; i < _num_axes; i++) {
        const unsigned servo_num = unsigned(_axis[i].channel + 1);
        const unsigned id = unsigned(_axis[i].node);
        if (alive[i]) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eRob: SERVO%u id %u OK", servo_num, id);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "eRob: SERVO%u id %u no response", servo_num, id);
        }
    }
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

int32_t AP_eRobCANOpen::counts_per_rev() const
{
    const int32_t res = _pos_res.get();
    if (res <= 0) {
        return EROB_PLUS_PER_REV;
    }
    return res;
}

bool AP_eRobCANOpen::axis_faulted(uint8_t index) const
{
    const erob_servo_t &servo = _axis[index].servo;
    return (erob_get_statusword(&servo) & EROB_SW_FAULT_BIT) != 0 ||
           erob_get_state(&servo) == EROB_ST_FAULT;
}

void AP_eRobCANOpen::warn_statusword_fault(uint8_t index, uint32_t now_ms)
{
    Axis &axis = _axis[index];
    if (!axis_faulted(index)) {
        axis.fault_active = false;
        return;
    }
    if (axis.fault_active && (now_ms - axis.last_fault_ms) < EROB_FAULT_WARN_MS) {
        return;
    }
    axis.fault_active = true;
    axis.last_fault_ms = now_ms;
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "eRob: SERVO%u id %u statusword fault",
                  unsigned(axis.channel + 1), unsigned(axis.node));
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
        const uint32_t now_ms = AP_HAL::millis();
        // Queue the critical text before poll_motion's one-shot fault reset.
        warn_statusword_fault(i, now_ms);
        erob_poll_motion(&_axis[i].servo);
        erob_poll_diag(&_axis[i].servo);
        if (!_axis[i].reported && erob_get_state(&_axis[i].servo) == EROB_ST_ENABLED) {
            _axis[i].reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "eRob CAN: servo node %u enabled",
                          unsigned(_axis[i].node));
        }
        publish_feedback(i, now_ms);
    }
    if (_num_axes > 0) {
        co_master_poll_sync(&_axis[0].master);
    }
}

static void erob_named(char *out, size_t len, uint8_t servo_num, char tag)
{
    snprintf(out, len, "ER%u%c", unsigned(servo_num), tag);
}

void AP_eRobCANOpen::publish_feedback(uint8_t index, uint32_t now_ms)
{
    Axis &axis = _axis[index];
    const erob_feedback_t *fb = erob_feedback(&axis.servo);
    const uint8_t servo_num = uint8_t(axis.channel + 1);
    char name[8];

    if (fb == nullptr || (fb->valid & EROB_FB_TPDO1) == 0) {
        return;
    }
#if HAL_LOGGING_ENABLED || HAL_GCS_ENABLED
    const int32_t res = counts_per_rev();
#endif

#if HAL_LOGGING_ENABLED
    if (AP::logger().logging_enabled() && (now_ms - axis.last_log_ms) >= 100) {
        axis.last_log_ms = now_ms;
        // @LoggerMessage: ERB1
        // @Description: eRob CAN cyclic feedback for one mapped servo
        // @Field: TimeUS: Time since system startup
        // @Field: Ax: axis index on this driver, slot order or servo-mask order
        // @Field: Nd: CANopen Node-ID
        // @Field: Sv: servo output number, SERVO1 is 1
        // @Field: SW: statusword 0x6041
        // @Field: Md: mode display 0x6061, 8 is CSP
        // @Field: En: 1 when (statusword & 0x006F) == 0x0027
        // @Field: Pos: actual position 0x6064, plus
        // @Field: Mot: motor encoder 0x2240, plus
        // @Field: Ld: load encoder 0x20A0, plus
        // @Field: Vel: velocity 0x606C, plus/s
        // @Field: Tq: torque 0x6077, permille of rated
        // @Field: Iq: current 0x6078, permille of rated
        // @Field: mV: DC bus 0x6079, millivolts
        // @Field: FE: following error 0x60F4, plus
        AP::logger().WriteStreaming(
            "ERB1",
            "TimeUS,Ax,Nd,Sv,SW,Md,En,Pos,Mot,Ld,Vel,Tq,Iq,mV,FE",
            "s###-------%%v-",
            "F0000000000AAC0",
            "QBBBHbBiiiihhIi",
            AP_HAL::micros64(),
            unsigned(index),
            unsigned(axis.node),
            unsigned(servo_num),
            unsigned(fb->statusword),
            int(fb->mode_disp),
            unsigned(fb->op_enabled ? 1 : 0),
            int(fb->pos_actual),
            int(fb->motor_enc),
            int(fb->load_enc),
            int(fb->vel_plus_s),
            int(fb->torque_permille),
            int(fb->current_permille),
            uint32_t(fb->dc_mv),
            int(fb->follow_err));

        {
            const float nan = AP::logger().quiet_nanf();
            const float gear = _gear.get();
            const bool have_vel = (fb->valid & EROB_FB_TPDO3) != 0;
            const float out_rpm = have_vel ? erob_output_rpm(fb->vel_plus_s, res) : nan;
            const float motor_rpm = (have_vel && is_positive(gear)) ? out_rpm * gear : nan;
            const float amps = (have_vel && (fb->valid & EROB_FB_IRATED) != 0) ?
                               erob_current_a(fb->current_permille, fb->rated_current_mA) : nan;
            const float nm = (have_vel && (fb->valid & EROB_FB_TRATED) != 0) ?
                             erob_torque_nm(fb->torque_permille, fb->rated_torque_mNm) : nan;
            const float volts = (fb->valid & EROB_FB_TPDO4) != 0 ? fb->dc_mv * 0.001f : nan;
            // @LoggerMessage: ERB3
            // @Description: eRob CAN physical units for one mapped servo. ERB1 keeps the raw objects. A DataFlash format string holds 16 fields and ERB1 is already full.
            // @Field: TimeUS: Time since system startup
            // @Field: Ax: axis index on this driver
            // @Field: Sv: servo output number, SERVO1 is 1
            // @Field: Deg: output position from 0x6064, degrees, using RES
            // @Field: Rpm: output speed from 0x606C, rpm, using RES
            // @Field: Mrpm: motor rpm, output rpm times GEAR, invalid when GEAR is 0
            // @Field: Amp: current from 0x6078 and 0x6075, ampere
            // @Field: Nm: torque from 0x6077 and 0x6076, newton metre
            // @Field: V: DC bus from 0x6079, volt
            AP::logger().WriteStreaming(
                "ERB3",
                "TimeUS,Ax,Sv,Deg,Rpm,Mrpm,Amp,Nm,V",
                "s##dqqAtv",
                "F00000000",
                "QBBffffff",
                AP_HAL::micros64(),
                unsigned(index),
                unsigned(servo_num),
                erob_position_deg(fb->pos_actual, res),
                out_rpm,
                motor_rpm,
                amps,
                nm,
                volts);
        }

        if ((fb->valid & (EROB_FB_ERR | EROB_FB_WARN | EROB_FB_TEMP | EROB_FB_DIFF)) != 0) {
            // @LoggerMessage: ERB2
            // @Description: eRob CAN slow diagnostics for one mapped servo
            // @Field: TimeUS: Time since system startup
            // @Field: Ax: axis index on this driver
            // @Field: Nd: CANopen Node-ID
            // @Field: Err: error code 0x603F
            // @Field: Warn: warning code 0x3B68
            // @Field: Tmp: drive temperature 0x22A2, degC
            // @Field: Diff: dual-encoder difference 0x2241, plus
            AP::logger().WriteStreaming(
                "ERB2",
                "TimeUS,Ax,Nd,Err,Warn,Tmp,Diff",
                "s##--O-",
                "F000000",
                "QBBHIhi",
                AP_HAL::micros64(),
                unsigned(index),
                unsigned(axis.node),
                unsigned(fb->error_code),
                uint32_t(fb->warning_code),
                int(fb->drive_temp_c),
                int(fb->enc_diff));
        }
    }
#else
    (void)now_ms;
#endif

#if HAL_GCS_ENABLED
    if ((now_ms - axis.last_named_ms) < 1000) {
        return;
    }
    axis.last_named_ms = now_ms;
    erob_named(name, sizeof(name), servo_num, 'P');
    gcs().send_named_int(name, fb->pos_actual);
    erob_named(name, sizeof(name), servo_num, 'G');
    gcs().send_named_float(name, erob_position_deg(fb->pos_actual, res));
    if ((fb->valid & EROB_FB_TPDO2) != 0) {
        erob_named(name, sizeof(name), servo_num, 'M');
        gcs().send_named_int(name, fb->motor_enc);
        erob_named(name, sizeof(name), servo_num, 'L');
        gcs().send_named_int(name, fb->load_enc);
    }
    if ((fb->valid & EROB_FB_TPDO3) != 0) {
        const float out_rpm = erob_output_rpm(fb->vel_plus_s, res);
        const float gear = _gear.get();
        erob_named(name, sizeof(name), servo_num, 'S');
        gcs().send_named_int(name, fb->vel_plus_s);
        erob_named(name, sizeof(name), servo_num, 'I');
        gcs().send_named_int(name, fb->current_permille);
        erob_named(name, sizeof(name), servo_num, 'R');
        gcs().send_named_float(name, out_rpm);
        if (is_positive(gear)) {
            erob_named(name, sizeof(name), servo_num, 'N');
            gcs().send_named_float(name, out_rpm * gear);
        }
        if ((fb->valid & EROB_FB_IRATED) != 0) {
            erob_named(name, sizeof(name), servo_num, 'C');
            gcs().send_named_float(name, erob_current_a(fb->current_permille, fb->rated_current_mA));
        }
        if ((fb->valid & EROB_FB_TRATED) != 0) {
            erob_named(name, sizeof(name), servo_num, 'Q');
            gcs().send_named_float(name, erob_torque_nm(fb->torque_permille, fb->rated_torque_mNm));
        }
    }
    if ((fb->valid & EROB_FB_TPDO4) != 0) {
        erob_named(name, sizeof(name), servo_num, 'U');
        gcs().send_named_float(name, fb->dc_mv * 0.001f);
        erob_named(name, sizeof(name), servo_num, 'F');
        gcs().send_named_int(name, fb->follow_err);
    }
    if ((fb->valid & EROB_FB_TEMP) != 0) {
        erob_named(name, sizeof(name), servo_num, 'T');
        gcs().send_named_float(name, fb->drive_temp_c);
    }
    if ((fb->valid & EROB_FB_ERR) != 0) {
        erob_named(name, sizeof(name), servo_num, 'E');
        gcs().send_named_int(name, fb->error_code);
    }
    if ((fb->valid & EROB_FB_WARN) != 0) {
        erob_named(name, sizeof(name), servo_num, 'W');
        gcs().send_named_int(name, int32_t(fb->warning_code));
    }
    if ((fb->valid & EROB_FB_MODE) != 0) {
        erob_named(name, sizeof(name), servo_num, 'O');
        gcs().send_named_int(name, fb->mode_disp);
    }
    if ((fb->valid & EROB_FB_DIFF) != 0) {
        erob_named(name, sizeof(name), servo_num, 'D');
        gcs().send_named_int(name, fb->enc_diff);
    }
    erob_named(name, sizeof(name), servo_num, 'A');
    gcs().send_named_int(name, fb->op_enabled ? 1 : 0);
#else
    (void)servo_num;
    (void)name;
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
    // Logging the feedback snapshot needs more than the PiccoloCAN 4k stack.
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_eRobCANOpen::loop, void),
                                      _thread_name, 8192, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
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
    probe_nodes();

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
        // A faulted slot must not keep taking CSP targets. The critical
        // text is raised on the CAN thread before any further write.
        if (axis_faulted(i)) {
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
