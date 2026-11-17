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

/* ENABLE_OP replies with 0x020F. Other control words follow CiA402. */
void can_bus_stub_set_fault_on_enable(int enable);

/* RPDO replies use sw and ignore the control word. lock=0 clears it. */
void can_bus_stub_lock_status(int lock, uint16_t sw);

/* While set, RPDO replies carry no TPDO. Drops queued RX. */
void can_bus_stub_set_silent(int silent);

void can_bus_stub_push_status(uint16_t sw, int32_t pos);
void can_bus_stub_push_heartbeat(uint8_t nmt_state);

/* 0x6064 value returned to the next SDO upload. Does not emit a TPDO. */
void can_bus_stub_set_position(int32_t pos);

/* Non-zero: SDO uploads of this index are aborted. 0 answers normally. */
void can_bus_stub_fail_read(uint16_t index);
