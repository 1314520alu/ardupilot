#pragma once

#include "AP_eRobCANOpen_config.h"

#if AP_EROB_CANOPEN_ENABLED

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

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
 * Minimal eRob CANopen master for one HAL iface.
 * Selecting CAN_Dx_PROTOCOL = ERobCANOpen gives this driver exclusive use of
 * that iface. It binds CAN2 only. Frames that are not this node's CANopen
 * traffic are discarded.
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

    void set_target_pos(int32_t plus);
    int32_t get_position() const;
    uint16_t get_statusword() const;

private:
    void loop();
    bool frame_for_master(const AP_HAL::CANFrame &rx, can_frame_t *fr);
    int hal_send(const can_frame_t *fr);
    int hal_recv(can_frame_t *fr, uint32_t timeout_ms);
    uint8_t node_id() const;

    friend int can_bus_send(can_bus_t *bus, const can_frame_t *fr);
    friend int can_bus_recv(can_bus_t *bus, can_frame_t *fr, uint32_t timeout_ms);
    friend uint32_t can_bus_millis(can_bus_t *bus);

    AP_Int8 _node_id;
    uint8_t _active_node;
    uint8_t _driver_index;
    bool _started;
    bool _reported;
    AP_HAL::CANIface* _iface;
    HAL_BinarySemaphore _rx_sem;
    char _thread_name[12];

    can_bus_t _bus;
    canopen_master_t _master;
    erob_servo_t _servo;
};

#endif  // AP_EROB_CANOPEN_ENABLED
