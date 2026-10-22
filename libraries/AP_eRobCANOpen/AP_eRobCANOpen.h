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
 * that iface. It binds CAN2 only. Servo channels map to consecutive node
 * IDs as CiA402 CSP targets. Frames that are not those nodes' eRob CAN
 * traffic are discarded. This is not a general CANopen tool.
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
        bool tried;
        bool reported;
        uint32_t last_try_ms;
    };

    void loop();
    void setup_axes();
    void poll_axes();
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
    AP_Int8 _srv_first;
    AP_Int8 _axis_count;
    AP_Int32 _pos_min;
    AP_Int32 _pos_max;

    uint8_t _active_node;
    uint8_t _num_axes;
    uint8_t _srv_base;
    uint8_t _driver_index;
    bool _started;
    AP_HAL::CANIface* _iface;
    HAL_BinarySemaphore _rx_sem;
    char _thread_name[12];

    can_bus_t _bus;
    Axis _axis[EROB_MAX_AXES];
};

#endif  // AP_EROB_CANOPEN_ENABLED
