#pragma once

/*
 * Physical ports on ZeroOne X6 / X6 Ultra:
 *   CAN1 = DroneCAN. This module does not claim it.
 *   CAN2 = AP_CAN::Protocol::ERobCANOpen (15) when the user selects it.
 * Other 11-bit IDs on that port stay classic CAN; AP_eRobCANOpen demuxes them.
 * PROTOCOL2 is not used: its 11-bit aux path transmits on every iface of the
 * primary driver, and ZeroOneX6 defaults put CAN1 and CAN2 on one DroneCAN driver.
 * HAL iface index is zero-based, so CAN2 is iface 1.
 */

#ifndef EROB_CAN_BUS_INDEX
#define EROB_CAN_BUS_INDEX 2u
#endif

#ifndef EROB_CAN_BITRATE
#define EROB_CAN_BITRATE 1000000u
#endif

/* Manual §5.5.1 writes 0x1006 = 1000 for 1000 us, so the unit is microseconds. */
#ifndef EROB_SYNC_PERIOD_US
#define EROB_SYNC_PERIOD_US 4000u
#endif

/* Placeholder until the customer supplies the real node list. */
#ifndef EROB_DEFAULT_NODE_ID
#define EROB_DEFAULT_NODE_ID 1u
#endif

/* Manual §5.1.1 note 1: wait at least 500 ms after enable before a move. */
#ifndef EROB_POST_ENABLE_HOLDOFF_MS
#define EROB_POST_ENABLE_HOLDOFF_MS 500u
#endif

/* Manual §5.1.2: gap between enable and disable. */
#ifndef EROB_ENABLE_DISABLE_GAP_MS
#define EROB_ENABLE_DISABLE_GAP_MS 300u
#endif

#ifndef EROB_BOOT_TIMEOUT_MS
#define EROB_BOOT_TIMEOUT_MS 2000u
#endif

/* HAL iface for EROB_CAN_BUS_INDEX. CAN2 -> 1. Do not use 0 (that is CAN1). */
static inline unsigned erob_hal_can_iface(void)
{
    return (unsigned)(EROB_CAN_BUS_INDEX - 1u);
}
