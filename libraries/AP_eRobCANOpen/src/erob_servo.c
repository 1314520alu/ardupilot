/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CiA402 enable plus the fixed CSP PDO map from eRob manual §5.5.1.
 */

#include "../erob_servo.h"
#include "../erob_config.h"

#include <string.h>

static void on_tpdo(void *user, uint8_t pdo_num, const uint8_t *data, uint8_t dlc)
{
    erob_servo_t *s = (erob_servo_t *)user;
    (void)pdo_num;
    if (dlc < 6) {
        return;
    }
    s->statusword = (uint16_t)(data[0] | (data[1] << 8));
    s->pos_actual = (int32_t)((uint32_t)data[2] | ((uint32_t)data[3] << 8) |
                              ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 24));
    s->feedback_valid = true;
}

void erob_init(erob_servo_t *s, canopen_master_t *co)
{
    memset(s, 0, sizeof(*s));
    s->co = co;
    s->mode = EROB_MODE_CSP;
    s->state = EROB_ST_IDLE;
    co_master_set_hooks(co, on_tpdo, NULL, NULL, s);
}

static int sdo_u8(canopen_master_t *m, uint16_t index, uint8_t sub, uint8_t val)
{
    return co_sdo_write(m, index, sub, &val, 1);
}

static int sdo_u32(canopen_master_t *m, uint16_t index, uint8_t sub, uint32_t val)
{
    return co_sdo_write(m, index, sub, &val, 4);
}

static int fail(erob_servo_t *s, int code)
{
    co_master_set_sync(s->co, false, 0);
    s->state = EROB_ST_ERROR;
    return code;
}

int erob_configure_pdo_csp(erob_servo_t *s)
{
    canopen_master_t *m = s->co;
    uint8_t n = m->node_id;
    uint32_t tpdo_cob;
    uint32_t rpdo_cob;
    uint8_t mode_disp = 0;
    int32_t pos = 0;

    if (n < 1 || n > 127) {
        return fail(s, -1);
    }

    co_master_set_sync(m, false, 0);
    s->mode = EROB_MODE_CSP;
    co_master_use_default_cob(m);

    /* §5.5.1: stop, reset communication, wait for boot-up, then SDO in pre-op. */
    if (co_nmt(m, CO_NMT_STOP, n) != 0) {
        return fail(s, -1);
    }
    m->boot_seen = false;
    if (co_nmt(m, CO_NMT_RESET_COMM, n) != 0) {
        return fail(s, -1);
    }
    if (co_master_wait_boot(m, EROB_BOOT_TIMEOUT_MS) != 0) {
        return fail(s, -1);
    }

    if (sdo_u8(m, 0x6060, 0x00, EROB_MODE_CSP) != 0) {
        return fail(s, -1);
    }
    if (co_sdo_read(m, 0x6061, 0x00, &mode_disp, 1) != 0 || mode_disp != EROB_MODE_CSP) {
        return fail(s, -2);
    }

    /* bit30 clear: this slave must not produce SYNC. COB-ID stays 0x080. */
    if (sdo_u32(m, 0x1005, 0x00, 0x00000080u) != 0) {
        return fail(s, -1);
    }
    if (sdo_u32(m, 0x1006, 0x00, m->sync_period_us) != 0) {
        return fail(s, -1);
    }

    tpdo_cob = (uint32_t)(CO_COB_TPDO1_BASE + n);
    rpdo_cob = (uint32_t)(CO_COB_RPDO1_BASE + n);

    /* TxPDO1: disable, type 1, map 6041:10 + 6064:20, two entries, enable. */
    if (sdo_u32(m, 0x1800, 0x01, tpdo_cob | 0x80000000u) != 0 ||
        sdo_u8(m, 0x1800, 0x02, 0x01) != 0 ||
        sdo_u8(m, 0x1A00, 0x00, 0x00) != 0 ||
        sdo_u32(m, 0x1A00, 0x01, 0x60410010u) != 0 ||
        sdo_u32(m, 0x1A00, 0x02, 0x60640020u) != 0 ||
        sdo_u8(m, 0x1A00, 0x00, 0x02) != 0 ||
        sdo_u32(m, 0x1800, 0x01, tpdo_cob) != 0) {
        return fail(s, -1);
    }

    /* RxPDO1: disable, type 1, map 6040:10 + 607A:20, two entries, enable. */
    if (sdo_u32(m, 0x1400, 0x01, rpdo_cob | 0x80000000u) != 0 ||
        sdo_u8(m, 0x1400, 0x02, 0x01) != 0 ||
        sdo_u8(m, 0x1600, 0x00, 0x00) != 0 ||
        sdo_u32(m, 0x1600, 0x01, 0x60400010u) != 0 ||
        sdo_u32(m, 0x1600, 0x02, 0x607A0020u) != 0 ||
        sdo_u8(m, 0x1600, 0x00, 0x02) != 0 ||
        sdo_u32(m, 0x1400, 0x01, rpdo_cob) != 0) {
        return fail(s, -1);
    }

    if (co_nmt(m, CO_NMT_START, n) != 0) {
        return fail(s, -1);
    }

    /* §5.3.2: copy actual position into the target before position mode runs. */
    if (co_sdo_read(m, 0x6064, 0x00, &pos, 4) != 0) {
        return fail(s, -1);
    }
    s->pos_actual = pos;
    s->target_pos = pos;
    s->hold_pos = pos;
    if (co_sdo_write(m, 0x607A, 0x00, &pos, 4) != 0) {
        return fail(s, -1);
    }

    co_master_set_sync(m, true, m->sync_period_us);
    s->state = EROB_ST_CONFIG;
    return 0;
}

int erob_set_mode(erob_servo_t *s, uint8_t mode)
{
    s->mode = mode;
    return co_sdo_write(s->co, 0x6060, 0x00, &mode, 1);
}

static int send_cw_pos(erob_servo_t *s, uint16_t cw, int32_t pos)
{
    uint8_t buf[6];
    buf[0] = (uint8_t)(cw & 0xff);
    buf[1] = (uint8_t)(cw >> 8);
    buf[2] = (uint8_t)((uint32_t)pos);
    buf[3] = (uint8_t)((uint32_t)pos >> 8);
    buf[4] = (uint8_t)((uint32_t)pos >> 16);
    buf[5] = (uint8_t)((uint32_t)pos >> 24);
    return co_rpdo_send(s->co, 0, buf, 6);
}

int erob_enable(erob_servo_t *s)
{
    int32_t pos = 0;
    uint32_t now = can_bus_millis(s->co->bus);

    if (s->state == EROB_ST_CONFIG && s->enable_done_ms != 0 &&
        (now - s->enable_done_ms) < EROB_ENABLE_DISABLE_GAP_MS) {
        return -1;
    }
    if (co_sdo_read(s->co, 0x6064, 0x00, &pos, 4) == 0) {
        s->pos_actual = pos;
        s->target_pos = pos;
        s->hold_pos = pos;
    }
    s->enable_step = 1;
    s->enable_ts_ms = now;
    s->state = EROB_ST_ENABLE_SEQ;
    return send_cw_pos(s, EROB_CW_FAULT_RESET, s->hold_pos);
}

int erob_disable(erob_servo_t *s)
{
    uint32_t now = can_bus_millis(s->co->bus);
    if (s->state == EROB_ST_ENABLED &&
        (now - s->enable_done_ms) < EROB_ENABLE_DISABLE_GAP_MS) {
        return -1;
    }
    s->state = EROB_ST_CONFIG;
    s->enable_done_ms = now;
    return send_cw_pos(s, EROB_CW_SHUTDOWN, s->hold_pos);
}

void erob_set_target_pos(erob_servo_t *s, int32_t plus)
{
    s->target_pos = plus;
}

void erob_set_target_vel(erob_servo_t *s, int32_t plus_per_s)
{
    /* Not on the CSP PDO. Kept so a later CSV map can use the same setter. */
    s->target_vel = plus_per_s;
}

static const uint16_t enable_cw[] = {
    EROB_CW_FAULT_RESET,
    EROB_CW_SHUTDOWN,
    EROB_CW_SWITCH_ON,
    EROB_CW_ENABLE_OP
};

void erob_poll(erob_servo_t *s)
{
    uint32_t now;
    uint16_t sw;

    co_master_poll_rx(s->co);
    now = can_bus_millis(s->co->bus);
    sw = s->statusword;

    if (s->state == EROB_ST_ENABLE_SEQ) {
        if ((now - s->enable_ts_ms) >= 50) {
            s->enable_ts_ms = now;
            if (s->enable_step < 4) {
                (void)send_cw_pos(s, enable_cw[s->enable_step], s->hold_pos);
                s->enable_step++;
            } else if ((sw & EROB_SW_FAULT_BIT) != 0) {
                s->state = EROB_ST_FAULT;
                (void)send_cw_pos(s, EROB_CW_FAULT_RESET, s->hold_pos);
            } else if ((sw & EROB_SW_MASK) == EROB_SW_OP_ENABLED) {
                s->state = EROB_ST_ENABLED;
                s->enable_done_ms = now;
                s->hold_pos = s->pos_actual;
                (void)send_cw_pos(s, EROB_CW_ENABLE_OP, s->hold_pos);
            } else {
                (void)send_cw_pos(s, EROB_CW_ENABLE_OP, s->hold_pos);
            }
        }
        co_master_poll_sync(s->co);
        return;
    }

    if (s->state == EROB_ST_ENABLED) {
        int32_t cmd = s->target_pos;
        if ((sw & EROB_SW_FAULT_BIT) != 0) {
            s->state = EROB_ST_FAULT;
            (void)send_cw_pos(s, EROB_CW_FAULT_RESET, s->hold_pos);
            co_master_poll_sync(s->co);
            return;
        }
        if ((now - s->enable_done_ms) < EROB_POST_ENABLE_HOLDOFF_MS) {
            cmd = s->hold_pos;
        }
        (void)send_cw_pos(s, EROB_CW_ENABLE_OP, cmd);
    }

    co_master_poll_sync(s->co);
}

erob_state_t erob_get_state(const erob_servo_t *s)
{
    return s->state;
}

uint16_t erob_get_statusword(const erob_servo_t *s)
{
    return s->statusword;
}

int32_t erob_get_position(const erob_servo_t *s)
{
    return s->pos_actual;
}
