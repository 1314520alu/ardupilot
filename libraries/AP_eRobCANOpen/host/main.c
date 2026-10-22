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

static int near_f(float a, float b)
{
    float d = a - b;
    if (d < 0.0f) {
        d = -d;
    }
    return d < 0.0001f;
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
    expect_true(erob_id_is_reserved(0x381, node), "TPDO3 is reserved");
    expect_true(erob_id_is_reserved(0x481, node), "TPDO4 is reserved");
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
    expect_true(erob_target_plus(0, 0) == 0, "center 0 leaves the mapped target unchanged");
    expect_true(erob_target_plus(-43691, 262144) == 218453, "19-bit minus 30 deg is 218453");
    expect_true(erob_target_plus(43691, 262144) == 305835, "19-bit plus 30 deg is 305835");
    expect_true(erob_target_plus(0, 262144) == 262144, "symmetric MIN/MAX midpoint is CTR");
    expect_true(erob_pwm_to_plus(1500, 1000, 2000, -43691, 43691) == 0,
                "PWM midpoint maps to the MIN/MAX midpoint");
    expect_true(erob_plus_rel_center(262144, 262144) == 0, "center count is zero relative");
    expect_true(erob_plus_rel_center(218453, 262144) == -43691, "absolute low end is -30 deg");
    expect_true(erob_plus_rel_center(305835, 262144) == 43691, "absolute high end is +30 deg");
    expect_true(near_f(erob_position_deg(erob_plus_rel_center(262144, 262144), EROB_PLUS_PER_REV), 0.0f),
                "center count is 0 degrees");
    expect_true(near_f(erob_position_deg(0, EROB_PLUS_PER_REV), 0.0f), "zero plus is zero degrees");
    expect_true(near_f(erob_position_deg(EROB_PLUS_PER_REV, EROB_PLUS_PER_REV), 360.0f),
                "one output revolution is 360 degrees");
    expect_true(near_f(erob_position_deg(EROB_PLUS_PER_REV / 2, EROB_PLUS_PER_REV), 180.0f),
                "half a revolution is 180 degrees");
    expect_true(near_f(erob_output_rpm(EROB_PLUS_PER_REV, EROB_PLUS_PER_REV), 60.0f),
                "one rev per second is 60 rpm");
    expect_true(near_f(erob_position_deg(EROB_PLUS_PER_REV, 0), 360.0f),
                "non-positive resolution uses 524288");
    expect_true(near_f(erob_position_deg(1048576, 1048576), 360.0f),
                "20-bit revolution is 360 degrees");
    expect_true(near_f(erob_output_rpm(1048576, 1048576), 60.0f),
                "20-bit rev per second is 60 rpm");
    {
        float d19 = erob_position_deg(43691, EROB_PLUS_PER_REV);
        float d20 = erob_position_deg(87381, 1048576);
        float e19 = d19 - 30.0f;
        float e20 = d20 - 30.0f;
        if (e19 < 0.0f) {
            e19 = -e19;
        }
        if (e20 < 0.0f) {
            e20 = -e20;
        }
        expect_true(e19 < 0.01f, "19-bit 30 degrees is 43691 plus");
        expect_true(e20 < 0.01f, "20-bit 30 degrees is 87381 plus");
    }
    expect_true(near_f(erob_current_a(9, 5000), 0.045f), "current uses rated mA");
    expect_true(near_f(erob_torque_nm(-7, 2000), -0.014f), "torque uses rated mNm");
    {
        erob_feedback_t blank;
        memset(&blank, 0, sizeof(blank));
        erob_cache_rating(&blank, 0x6075, 0);
        expect_true((blank.valid & EROB_FB_IRATED) == 0, "zero rated current stays invalid");
        erob_cache_rating(&blank, 0x6076, 0);
        expect_true((blank.valid & EROB_FB_TRATED) == 0, "zero rated torque stays invalid");
        erob_cache_rating(&blank, 0x6075, 5000);
        expect_true(blank.rated_current_mA == 5000 && (blank.valid & EROB_FB_IRATED) != 0,
                    "non-zero rated current is cached");
    }

    co_master_init(&master, bus, node);
    erob_init(&servo, &master);
    expect_true(erob_configure_pdo_csp(&servo) == 0, "CSP PDO configure");
    expect_true(erob_get_state(&servo) == EROB_ST_CONFIG, "state after configure");
    {
        const erob_feedback_t *rated = erob_feedback(&servo);
        expect_true(rated != NULL && (rated->valid & EROB_FB_IRATED) != 0 &&
                    rated->rated_current_mA == 5000u, "configure reads 0x6075");
        expect_true((rated->valid & EROB_FB_TRATED) != 0 &&
                    rated->rated_torque_mNm == 2000u, "configure reads 0x6076");
    }
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
    expect_true(find_sdo_u32(node, 0x1A00, 0x01, 0x60410010u) >= 0, "TxPDO1 maps statusword");
    expect_true(find_sdo_u32(node, 0x1A00, 0x02, 0x60640020u) >= 0, "TxPDO1 maps actual position");
    expect_true(find_sdo_u32(node, 0x1801, 0x01, 0x80000000u | (0x280u + node)) >= 0,
                "TxPDO2 disabled");
    expect_true(find_sdo_u32(node, 0x1801, 0x01, 0x280u + node) >= 0, "TxPDO2 enabled");
    expect_true(find_sdo_u32(node, 0x1A01, 0x01, 0x22400020u) >= 0, "TxPDO2 maps motor encoder");
    expect_true(find_sdo_u32(node, 0x1A01, 0x02, 0x20A00020u) >= 0, "TxPDO2 maps load encoder");
    expect_true(find_sdo_u32(node, 0x1A02, 0x01, 0x606C0020u) >= 0, "TxPDO3 maps velocity");
    expect_true(find_sdo_u32(node, 0x1A02, 0x02, 0x60770010u) >= 0, "TxPDO3 maps torque");
    expect_true(find_sdo_u32(node, 0x1A02, 0x03, 0x60780010u) >= 0, "TxPDO3 maps current");
    expect_true(find_sdo_u32(node, 0x1802, 0x01, 0x380u + node) >= 0, "TxPDO3 enabled");
    expect_true(find_sdo_u32(node, 0x1A03, 0x01, 0x60790020u) >= 0, "TxPDO4 maps DC voltage");
    expect_true(find_sdo_u32(node, 0x1A03, 0x02, 0x60F40020u) >= 0, "TxPDO4 maps following error");
    expect_true(find_sdo_u32(node, 0x1803, 0x01, 0x480u + node) >= 0, "TxPDO4 enabled");
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

    {
        const erob_feedback_t *fb = erob_feedback(&servo);
        int d;
        expect_true(fb != NULL && (fb->valid & EROB_FB_TPDO1) != 0, "TPDO1 cached");
        expect_true(fb->op_enabled, "enable bit is statusword 0x0027");
        expect_true((fb->valid & EROB_FB_TPDO2) != 0, "TPDO2 cached");
        expect_true(fb->motor_enc == 111 && fb->load_enc == 222, "encoder pair");
        expect_true((fb->valid & EROB_FB_TPDO3) != 0, "TPDO3 cached");
        expect_true(fb->vel_plus_s == 333, "velocity plus/s");
        expect_true(fb->torque_permille == -7 && fb->current_permille == 9, "torque and current");
        expect_true((fb->valid & EROB_FB_TPDO4) != 0, "TPDO4 cached");
        expect_true(fb->dc_mv == 48000u && fb->follow_err == 15, "voltage and following error");
        expect_true(near_f(erob_current_a(fb->current_permille, fb->rated_current_mA), 0.045f),
                    "TPDO current converts to amps");
        expect_true(near_f(erob_torque_nm(fb->torque_permille, fb->rated_torque_mNm), -0.014f),
                    "TPDO torque converts to Nm");
        expect_true(near_f(erob_output_rpm(fb->vel_plus_s, EROB_PLUS_PER_REV),
                           erob_output_rpm(333, EROB_PLUS_PER_REV)),
                    "TPDO velocity converts to output rpm");
        for (d = 0; d < 40; d++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
        fb = erob_feedback(&servo);
        expect_true((fb->valid & EROB_FB_ERR) != 0 && fb->error_code == 0x1234, "SDO error code");
        expect_true((fb->valid & EROB_FB_WARN) != 0 && fb->warning_code == 0x0000FF01u, "SDO warning");
        expect_true((fb->valid & EROB_FB_TEMP) != 0 && fb->drive_temp_c == -5, "SDO drive temp");
        expect_true((fb->valid & EROB_FB_MODE) != 0 && fb->mode_disp == EROB_MODE_CSP, "SDO mode display");
        expect_true((fb->valid & EROB_FB_DIFF) != 0 && fb->enc_diff == 42, "SDO dual-encoder difference");
        expect_true(erob_get_position(&servo) == 0, "CSP position still tracks TPDO1");
    }
    expect_true(!can_bus_stub_overflow(), "no queue overflow after diagnostics");

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
