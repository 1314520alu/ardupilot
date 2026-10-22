/*
 * Host check for the CAN2 eRob master. No hardware and no ArduPilot HAL.
 * Returns 0 only when the §5.5.1 configure/enable sequence matches.
 */

#include "erob_servo.h"
#include "erob_config.h"
#include "can_bus_stub.h"

#include <stdio.h>
#include <string.h>

static int g_fails;

static void expect_true(int cond, const char *msg)
{
    if (!cond) {
        fprintf(stderr, "FAIL: %s\n", msg);
        g_fails++;
    }
}

static int find_sdo_u32(uint8_t node, uint16_t index, uint8_t sub, uint32_t val)
{
    uint16_t i;
    uint8_t raw[4];
    raw[0] = (uint8_t)val;
    raw[1] = (uint8_t)(val >> 8);
    raw[2] = (uint8_t)(val >> 16);
    raw[3] = (uint8_t)(val >> 24);
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        if (fr->is_ext || fr->id != (uint32_t)(0x600u + node) || fr->dlc < 8) {
            continue;
        }
        if (fr->data[0] != 0x23) {
            continue;
        }
        if (fr->data[1] != (uint8_t)(index & 0xff) || fr->data[2] != (uint8_t)(index >> 8)) {
            continue;
        }
        if (fr->data[3] != sub) {
            continue;
        }
        if (memcmp(&fr->data[4], raw, 4) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static int find_nmt(uint8_t cmd, uint8_t node)
{
    uint16_t i;
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        if (!fr->is_ext && fr->id == 0 && fr->dlc >= 2 &&
            fr->data[0] == cmd && fr->data[1] == node) {
            return (int)i;
        }
    }
    return -1;
}

static int find_sync_after(int after)
{
    uint16_t i;
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        if ((int)i <= after) {
            continue;
        }
        if (!fr->is_ext && fr->id == CO_COB_SYNC && fr->dlc == 0) {
            return (int)i;
        }
    }
    return -1;
}

static int last_rpdo(uint8_t node, uint16_t *cw, int32_t *pos)
{
    uint16_t i;
    int found = 0;
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        if (fr->is_ext || fr->id != (uint32_t)(0x200u + node) || fr->dlc < 6) {
            continue;
        }
        *cw = (uint16_t)(fr->data[0] | (fr->data[1] << 8));
        *pos = (int32_t)((uint32_t)fr->data[2] | ((uint32_t)fr->data[3] << 8) |
                         ((uint32_t)fr->data[4] << 16) | ((uint32_t)fr->data[5] << 24));
        found = 1;
    }
    return found;
}

static void dump_tx(void)
{
    uint16_t i;
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        uint8_t b;
        fprintf(stderr, "  [%u] id=0x%03X dlc=%u", i, fr->id & 0x7ffu, fr->dlc);
        for (b = 0; b < fr->dlc; b++) {
            fprintf(stderr, " %02X", fr->data[b]);
        }
        fprintf(stderr, "\n");
    }
}

int main(void)
{
    can_bus_t *bus = can_bus_stub_get();
    canopen_master_t master;
    erob_servo_t servo;
    const uint8_t node = (uint8_t)EROB_DEFAULT_NODE_ID;
    int i_1006;
    int i_start;
    int i_sync;
    int loops;
    uint16_t cw = 0;
    int32_t pos = 0;
    uint16_t tx_at_enable;

    expect_true(EROB_CAN_BUS_INDEX == 2u, "default bus is CAN2");
    expect_true(EROB_CAN_BITRATE == 1000000u, "default bitrate is 1 Mbps");
    expect_true(EROB_SYNC_PERIOD_US == 4000u, "default SYNC period is 4000 us");
    expect_true(erob_hal_can_iface() == 1u, "HAL iface for CAN2 is 1, not CAN1");
    expect_true(erob_id_is_reserved(0x000, node), "NMT is reserved");
    expect_true(erob_id_is_reserved(0x080, node), "SYNC is reserved");
    expect_true(erob_id_is_reserved(0x081, node), "EMCY for node 1 is reserved");
    expect_true(erob_id_is_reserved(0x181, node), "TPDO1 is reserved");
    expect_true(erob_id_is_reserved(0x201, node), "RPDO1 is reserved");
    expect_true(erob_id_is_reserved(0x281, node), "TPDO2 is reserved");
    expect_true(erob_id_is_reserved(0x581, node), "SDO tx is reserved");
    expect_true(erob_id_is_reserved(0x601, node), "SDO rx is reserved");
    expect_true(erob_id_is_reserved(0x701, node), "heartbeat is reserved");
    expect_true(!erob_id_is_reserved(0x123, node), "0x123 is not reserved");
    expect_true(!erob_id_is_reserved(0x181, 2), "another node's TPDO is not reserved");
    expect_true(erob_id_is_reserved(0x080, 2), "SYNC stays reserved for any node");
    expect_true(erob_pwm_to_plus(1000, 1000, 2000, 0, EROB_PLUS_PER_REV) == 0,
                "servo min maps to plus min");
    expect_true(erob_pwm_to_plus(2000, 1000, 2000, 0, EROB_PLUS_PER_REV) == EROB_PLUS_PER_REV,
                "servo max maps to plus max");
    expect_true(erob_pwm_to_plus(1500, 1000, 2000, 0, EROB_PLUS_PER_REV) == EROB_PLUS_PER_REV / 2,
                "servo mid maps to half a revolution");
    expect_true(erob_pwm_to_plus(1500, 1000, 2000, -1000, 1000) == 0,
                "servo mid maps to the middle of a signed span");
    expect_true(erob_pwm_to_plus(500, 1000, 2000, 0, 100) == 0,
                "pwm below servo min clamps");
    expect_true(erob_pwm_to_plus(1500, 1500, 1500, 10, 20) == 10,
                "equal pwm ends do not divide");

    co_master_init(&master, bus, node);
    erob_init(&servo, &master);
    expect_true(erob_configure_pdo_csp(&servo) == 0, "CSP PDO configure");
    expect_true(erob_get_state(&servo) == EROB_ST_CONFIG, "state after configure");
    expect_true(!can_bus_stub_overflow(), "no queue overflow during configure");

    expect_true(find_nmt(CO_NMT_STOP, node) >= 0, "NMT stop");
    expect_true(find_nmt(CO_NMT_RESET_COMM, node) >= 0, "NMT reset communication");
    i_start = find_nmt(CO_NMT_START, node);
    expect_true(i_start >= 0, "NMT start");
    i_1006 = find_sdo_u32(node, 0x1006, 0x00, EROB_SYNC_PERIOD_US);
    expect_true(i_1006 >= 0, "0x1006 written as 4000 us");
    expect_true(i_start > i_1006, "cycle period is set before NMT start");
    expect_true(find_sdo_u32(node, 0x1005, 0x00, 0x00000080u) >= 0, "slave SYNC producer off");
    expect_true(find_sdo_u32(node, 0x1800, 0x01, 0x80000000u | (0x180u + node)) >= 0,
                "TxPDO1 disabled with node COB-ID");
    expect_true(find_sdo_u32(node, 0x1800, 0x01, 0x180u + node) >= 0, "TxPDO1 enabled");
    expect_true(find_sdo_u32(node, 0x1A00, 0x01, 0x60410010u) >= 0, "TxPDO maps statusword");
    expect_true(find_sdo_u32(node, 0x1A00, 0x02, 0x60640020u) >= 0, "TxPDO maps actual position");
    expect_true(find_sdo_u32(node, 0x1400, 0x01, 0x80000000u | (0x200u + node)) >= 0,
                "RxPDO1 disabled with node COB-ID");
    expect_true(find_sdo_u32(node, 0x1400, 0x01, 0x200u + node) >= 0, "RxPDO1 enabled");
    expect_true(find_sdo_u32(node, 0x1600, 0x01, 0x60400010u) >= 0, "RxPDO maps controlword");
    expect_true(find_sdo_u32(node, 0x1600, 0x02, 0x607A0020u) >= 0, "RxPDO maps target position");
    expect_true(find_sync_after(-1) < 0, "no SYNC before the cyclic phase");

    expect_true(erob_enable(&servo) == 0, "enable starts");
    for (loops = 0; loops < 20 && erob_get_state(&servo) != EROB_ST_ENABLED; loops++) {
        can_bus_stub_advance(50);
        erob_poll(&servo);
    }
    expect_true(erob_get_state(&servo) == EROB_ST_ENABLED, "CiA402 reached operation enabled");
    expect_true((erob_get_statusword(&servo) & EROB_SW_MASK) == EROB_SW_OP_ENABLED,
                "statusword mask is operation enabled");
    expect_true(erob_disable(&servo) == -1, "disable is refused inside the 300 ms gap");
    expect_true(erob_get_state(&servo) == EROB_ST_ENABLED, "state stays enabled after refused disable");

    tx_at_enable = can_bus_stub_tx_count();
    erob_set_target_pos(&servo, 10);
    can_bus_stub_advance(100);
    erob_poll(&servo);
    expect_true(last_rpdo(node, &cw, &pos), "RPDO present during holdoff");
    expect_true(cw == EROB_CW_ENABLE_OP, "holdoff controlword is 0x000F");
    expect_true(pos == 0, "target is held at actual position for 500 ms");

    can_bus_stub_advance(EROB_POST_ENABLE_HOLDOFF_MS);
    erob_poll(&servo);
    expect_true(last_rpdo(node, &cw, &pos), "RPDO present after holdoff");
    expect_true(pos == 10, "target position 10 plus is sent after holdoff");
    i_sync = find_sync_after((int)tx_at_enable - 1);
    expect_true(i_sync >= 0, "SYNC 0x080 is produced on the cyclic poll");
    expect_true(!can_bus_stub_overflow(), "no queue overflow");

    if (g_fails != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_fails);
        dump_tx();
        return 1;
    }
    printf("OK CAN%u iface %u @ %u bps, SYNC %u us, node %u, state %d\n",
           (unsigned)EROB_CAN_BUS_INDEX, erob_hal_can_iface(),
           (unsigned)EROB_CAN_BITRATE, (unsigned)EROB_SYNC_PERIOD_US,
           (unsigned)node, (int)erob_get_state(&servo));
    return 0;
}
