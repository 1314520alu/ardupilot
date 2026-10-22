#pragma once

#include "AP_eRobCANOpen_config.h"

#if AP_EROB_CANOPEN_ENABLED

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "erob_config.h"
#include "erob_servo.h"

class AP_eRobCANOpen;

/*
 * Completed here so the firmware TU can pass this object into the C master.
 * The host stub defines its own can_bus in a separate binary.
 */
struct can_bus {
    AP_eRobCANOpen *drv;
};

/*
 * eRob CAN servo backend for one HAL iface.
 * Selecting CAN_Dx_PROTOCOL = eRobCAN gives this driver exclusive use of
 * that iface. It binds CAN2 only. Each SV/ID slot maps one SERVO output to
 * one CANopen Node-ID as a CiA402 CSP target. If every SVn is 0, NODE plus
 * the servo bitmask is the fallback. Frames that are not those nodes' eRob
 * CAN traffic are discarded. This is not a general CANopen tool. Node-IDs
 * are hard-set on the servos; this driver does not write 0x100B.
 */
class AP_eRobCANOpen : public AP_CANDriver
{
public:
    AP_eRobCANOpen();

    CLASS_NO_COPY(AP_eRobCANOpen);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_eRobCANOpen *get_erob(uint8_t driver_index);

    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // Copy SERVO outputs into CSP targets. Called from SRV_Channels::push().
    void push_servos();

    void set_target_pos(int32_t plus);
    int32_t get_position() const;
    uint16_t get_statusword() const;

private:
    struct Axis {
        canopen_master_t master;
        erob_servo_t servo;
        uint8_t node;
        uint8_t channel;
        bool tried;
        bool reported;
        bool fault_active;
        uint32_t last_try_ms;
        uint32_t last_log_ms;
        uint32_t last_named_ms;
        uint32_t last_fault_ms;
        uint32_t last_silent_ms;
    };

    void loop();
    void setup_axes();
    bool slots_configured() const;
    void add_axis(uint8_t &num, uint8_t node, uint8_t channel);
    void probe_nodes();
    void poll_axes();
    void warn_statusword_fault(uint8_t index, uint32_t now_ms);
    void warn_node_silent(uint8_t index, uint32_t now_ms);
    bool axis_faulted(uint8_t index) const;
    int32_t counts_per_rev() const;
    void publish_feedback(uint8_t index, uint32_t now_ms);
    bool try_configure(uint8_t index, uint32_t now_ms);
    bool frame_for_master(const AP_HAL::CANFrame &rx, can_frame_t *fr);
    bool id_for_axis(uint16_t std_id, uint8_t &index) const;
    int hal_send(const can_frame_t *fr);
    int hal_recv(can_frame_t *fr, uint32_t timeout_ms);
    uint8_t node_id() const;

    friend int can_bus_send(can_bus_t *bus, const can_frame_t *fr);
    friend int can_bus_recv(can_bus_t *bus, can_frame_t *fr, uint32_t timeout_ms);
    friend uint32_t can_bus_millis(can_bus_t *bus);

    AP_Int8 _node_id;
    AP_Int32 _servo_bm;
    AP_Int32 _pos_min;
    AP_Int32 _pos_max;
    AP_Float _gear;
    AP_Int32 _pos_res;
    AP_Int32 _pos_ctr;
    AP_Int8 _slot_sv[EROB_SLOT_COUNT];
    AP_Int8 _slot_id[EROB_SLOT_COUNT];

    uint8_t _active_node;
    uint8_t _num_axes;
    uint8_t _driver_index;
    bool _started;
    AP_HAL::CANIface* _iface;
    HAL_BinarySemaphore _rx_sem;
    char _thread_name[12];

    can_bus_t _bus;
    Axis _axis[EROB_MAX_AXES];
};

#endif  // AP_EROB_CANOPEN_ENABLED
