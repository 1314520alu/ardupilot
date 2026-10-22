#pragma once

#include "can_frame.h"

can_bus_t *can_bus_stub_get(void);
void can_bus_stub_advance(uint32_t ms);
uint16_t can_bus_stub_tx_count(void);
const can_frame_t *can_bus_stub_tx(uint16_t index);
int can_bus_stub_overflow(void);
