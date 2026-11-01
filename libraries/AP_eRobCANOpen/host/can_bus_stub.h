#pragma once

#include "can_frame.h"

can_bus_t *can_bus_stub_get(void);
void can_bus_stub_advance(uint32_t ms);
uint16_t can_bus_stub_tx_count(void);
const can_frame_t *can_bus_stub_tx(uint16_t index);
int can_bus_stub_overflow(void);
void can_bus_stub_tx_reset(void);

/* While set, every RPDO reply keeps statusword bit3 set. Drops queued RX. */
void can_bus_stub_set_fault(int fault);

/* While set, RPDO replies carry no TPDO. Drops queued RX. */
void can_bus_stub_set_silent(int silent);

void can_bus_stub_push_status(uint16_t sw, int32_t pos);
void can_bus_stub_push_heartbeat(uint8_t nmt_state);
