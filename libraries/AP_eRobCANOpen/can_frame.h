#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Classic CAN frame. CANopen on CAN2 uses 11-bit ids (is_ext = false). */
typedef struct {
    uint32_t id;
    bool is_ext;
    bool is_rtr;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

/*
 * Platform hook. The host stub implements these symbols.
 * AP_eRobCANOpen.cpp implements them for the HAL iface it accepted (CAN2),
 * standard frames only, when AP_EROB_CANOPEN_ENABLED is set.
 */
typedef struct can_bus can_bus_t;

int can_bus_send(can_bus_t *bus, const can_frame_t *fr);
int can_bus_recv(can_bus_t *bus, can_frame_t *fr, uint32_t timeout_ms);
uint32_t can_bus_millis(can_bus_t *bus);

#ifdef __cplusplus
}
#endif
