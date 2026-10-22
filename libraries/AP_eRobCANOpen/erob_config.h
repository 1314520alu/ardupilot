#pragma once

/*
 * Physical ports on ZeroOne X6 / X6 Ultra:
 *   CAN1 = DroneCAN when CAN_D1_PROTOCOL is 1. This module does not claim it.
 *   CAN2 = eRob CAN when that driver is CAN_Dx_PROTOCOL 15 (eRobCAN).
 * One protocol owns the iface. This driver discards every frame it does not use.
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

/* Manual §5.2.1: output encoder counts per revolution. */
#ifndef EROB_PLUS_PER_REV
#define EROB_PLUS_PER_REV 524288
#endif

/*
 * Explicit SV/ID slots, and the NODE+BM fallback, share this axis table.
 * One SYNC producer. Eight slots match SV1..SV8.
 */
#ifndef EROB_MAX_AXES
#define EROB_MAX_AXES 8
#endif

#ifndef EROB_SLOT_COUNT
#define EROB_SLOT_COUNT 8
#endif

/* One diagnostic SDO per axis. Not sent on every SYNC. */
#ifndef EROB_DIAG_PERIOD_MS
#define EROB_DIAG_PERIOD_MS 250u
#endif

/* HAL iface for EROB_CAN_BUS_INDEX. CAN2 -> 1. Do not use 0 (that is CAN1). */
static inline unsigned erob_hal_can_iface(void)
{
    return (unsigned)(EROB_CAN_BUS_INDEX - 1u);
}
