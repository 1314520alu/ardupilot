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

#include <stdint.h>
#include <string.h>

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)rd_u16(p);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)rd_u32(p);
}

static void note_status(erob_servo_t *s, uint16_t sw)
{
    s->statusword = sw;
    s->fb.statusword = sw;
    s->fb.op_enabled = (sw & EROB_SW_MASK) == EROB_SW_OP_ENABLED;
}

static void on_tpdo(void *user, uint8_t pdo_num, const uint8_t *data, uint8_t dlc)
{
    erob_servo_t *s = (erob_servo_t *)user;

    if (pdo_num == 0 && dlc >= 6) {
        note_status(s, rd_u16(data));
        s->pos_actual = rd_i32(&data[2]);
        s->fb.pos_actual = s->pos_actual;
        s->fb.valid |= EROB_FB_TPDO1;
        s->feedback_valid = true;
        return;
    }
    if (pdo_num == 1 && dlc >= 8) {
        s->fb.motor_enc = rd_i32(&data[0]);
        s->fb.load_enc = rd_i32(&data[4]);
        s->fb.valid |= EROB_FB_TPDO2;
        return;
    }
    if (pdo_num == 2 && dlc >= 8) {
        s->fb.vel_plus_s = rd_i32(&data[0]);
        s->fb.torque_permille = rd_i16(&data[4]);
        s->fb.current_permille = rd_i16(&data[6]);
        s->fb.valid |= EROB_FB_TPDO3;
        return;
    }
    if (pdo_num == 3 && dlc >= 8) {
        s->fb.dc_mv = rd_u32(&data[0]);
        s->fb.follow_err = rd_i32(&data[4]);
        s->fb.valid |= EROB_FB_TPDO4;
    }
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

static void read_ratings(erob_servo_t *s);

static int fail(erob_servo_t *s, int code)
{
    co_master_set_sync(s->co, false, 0);
    s->state = EROB_ST_ERROR;
    return code;
}

/* Disable, transmission type 1, map, enable. entries are index<<16 | sub<<8 | bits. */
static int map_tpdo(canopen_master_t *m, uint16_t comm, uint16_t map,
                    uint32_t cob, const uint32_t *entries, uint8_t nent)
{
    uint8_t i;

    if (sdo_u32(m, comm, 0x01, cob | 0x80000000u) != 0 ||
        sdo_u8(m, comm, 0x02, 0x01) != 0 ||
        sdo_u8(m, map, 0x00, 0x00) != 0) {
        return -1;
    }
    for (i = 0; i < nent; i++) {
        if (sdo_u32(m, map, (uint8_t)(i + 1), entries[i]) != 0) {
            return -1;
        }
    }
    if (sdo_u8(m, map, 0x00, nent) != 0 ||
        sdo_u32(m, comm, 0x01, cob) != 0) {
        return -1;
    }
    return 0;
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

    /*
     * TxPDO1 is 6 bytes: statusword + actual position.
     * TxPDO2 is 8 bytes: motor encoder + load encoder.
     * TxPDO3 is 8 bytes: velocity + torque + current.
     * TxPDO4 is 8 bytes: DC bus + following error.
     * 0x6061 is 8 bits and would need a pad on EtherCAT; it is SDO instead.
     */
    {
        static const uint32_t tpdo1[] = { 0x60410010u, 0x60640020u };
        static const uint32_t tpdo2[] = { 0x22400020u, 0x20A00020u };
        static const uint32_t tpdo3[] = { 0x606C0020u, 0x60770010u, 0x60780010u };
        static const uint32_t tpdo4[] = { 0x60790020u, 0x60F40020u };
        if (map_tpdo(m, 0x1800, 0x1A00, tpdo_cob, tpdo1, 2) != 0 ||
            map_tpdo(m, 0x1801, 0x1A01, (uint32_t)(CO_COB_TPDO2_BASE + n), tpdo2, 2) != 0 ||
            map_tpdo(m, 0x1802, 0x1A02, (uint32_t)(CO_COB_TPDO3_BASE + n), tpdo3, 3) != 0 ||
            map_tpdo(m, 0x1803, 0x1A03, (uint32_t)(CO_COB_TPDO4_BASE + n), tpdo4, 2) != 0) {
            return fail(s, -1);
        }
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

    /*
     * Rated current and torque are not part of CSP. A miss or a zero
     * stays invalid and is retried from the slow SDO poll.
     */
    read_ratings(s);

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

void erob_poll_motion(erob_servo_t *s)
{
    uint32_t now;
    uint16_t sw;

    now = can_bus_millis(s->co->bus);
    sw = s->statusword;

    if (s->state == EROB_ST_ENABLE_SEQ) {
        /* Stop the enable sequence once the drive reports a fault.
         * One fault-reset follows the GCS critical text. No further writes.
         */
        if ((sw & EROB_SW_FAULT_BIT) != 0) {
            s->state = EROB_ST_FAULT;
            (void)send_cw_pos(s, EROB_CW_FAULT_RESET, s->hold_pos);
            return;
        }
        if ((now - s->enable_ts_ms) >= 50) {
            s->enable_ts_ms = now;
            if (s->enable_step < 4) {
                (void)send_cw_pos(s, enable_cw[s->enable_step], s->hold_pos);
                s->enable_step++;
            } else if ((sw & EROB_SW_MASK) == EROB_SW_OP_ENABLED) {
                s->state = EROB_ST_ENABLED;
                s->enable_done_ms = now;
                s->hold_pos = s->pos_actual;
                (void)send_cw_pos(s, EROB_CW_ENABLE_OP, s->hold_pos);
            } else {
                (void)send_cw_pos(s, EROB_CW_ENABLE_OP, s->hold_pos);
            }
        }
        return;
    }

    if (s->state == EROB_ST_ENABLED) {
        int32_t cmd = s->target_pos;
        if ((sw & EROB_SW_FAULT_BIT) != 0) {
            s->state = EROB_ST_FAULT;
            (void)send_cw_pos(s, EROB_CW_FAULT_RESET, s->hold_pos);
            return;
        }
        if ((now - s->enable_done_ms) < EROB_POST_ENABLE_HOLDOFF_MS) {
            cmd = s->hold_pos;
        }
        (void)send_cw_pos(s, EROB_CW_ENABLE_OP, cmd);
    }
}

static void apply_diag(erob_servo_t *s, uint16_t index, const uint8_t *data)
{
    switch (index) {
    case 0x603F:
        s->fb.error_code = rd_u16(data);
        s->fb.valid |= EROB_FB_ERR;
        break;
    case 0x3B68:
        s->fb.warning_code = rd_u32(data);
        s->fb.valid |= EROB_FB_WARN;
        break;
    case 0x22A2:
        /* UINT in the dictionary, but the range is -40..150 C. */
        s->fb.drive_temp_c = rd_i16(data);
        s->fb.valid |= EROB_FB_TEMP;
        break;
    case 0x6061:
        s->fb.mode_disp = (int8_t)data[0];
        s->fb.valid |= EROB_FB_MODE;
        break;
    case 0x2241:
        s->fb.enc_diff = rd_i32(data);
        s->fb.valid |= EROB_FB_DIFF;
        break;
    case 0x6075:
    case 0x6076:
        erob_cache_rating(&s->fb, index, rd_u32(data));
        break;
    default:
        break;
    }
}

static void read_ratings(erob_servo_t *s)
{
    uint32_t v;

    if ((s->fb.valid & EROB_FB_IRATED) == 0) {
        v = 0;
        if (co_sdo_read(s->co, 0x6075, 0x00, &v, 4) == 0) {
            erob_cache_rating(&s->fb, 0x6075, v);
        }
    }
    if ((s->fb.valid & EROB_FB_TRATED) == 0) {
        v = 0;
        if (co_sdo_read(s->co, 0x6076, 0x00, &v, 4) == 0) {
            erob_cache_rating(&s->fb, 0x6076, v);
        }
    }
}

void erob_poll_diag(erob_servo_t *s)
{
    canopen_master_t *m = s->co;
    uint32_t now;
    uint32_t phase;
    static const uint16_t idx[] = {
        0x603F, 0x3B68, 0x22A2, 0x6061, 0x2241, 0x6075, 0x6076
    };
    static const uint8_t sz[] = { 2, 4, 2, 1, 4, 4, 4 };
    const uint8_t nslot = 7;

    if (s->state != EROB_ST_ENABLED) {
        return;
    }
    now = can_bus_millis(m->bus);
    if (s->diag_wait) {
        if (m->sdo_reply_ready &&
            m->sdo_reply_index == s->diag_index &&
            m->sdo_reply_sub == 0) {
            if ((m->sdo_reply_cs & 0xe0) == 0x40) {
                apply_diag(s, s->diag_index, m->sdo_reply_data);
            }
            m->sdo_reply_ready = false;
            s->diag_wait = false;
            s->diag_slot = (uint8_t)((s->diag_slot + 1) % nslot);
        } else if ((now - s->diag_tx_ms) < 80u) {
            return;
        } else {
            s->diag_wait = false;
            s->diag_slot = (uint8_t)((s->diag_slot + 1) % nslot);
        }
    }

    /* Spread the axes so they do not upload on the same tick. */
    phase = (uint32_t)(m->node_id % EROB_MAX_AXES) * 40u;
    if (s->diag_tx_ms == 0) {
        if ((now - s->enable_done_ms) < phase) {
            return;
        }
    } else if ((now - s->diag_tx_ms) < EROB_DIAG_PERIOD_MS) {
        return;
    }

    {
        uint8_t guard = 0;
        while (guard < nslot) {
            const uint16_t cand = idx[s->diag_slot];
            if ((cand == 0x6075 && (s->fb.valid & EROB_FB_IRATED) != 0) ||
                (cand == 0x6076 && (s->fb.valid & EROB_FB_TRATED) != 0)) {
                s->diag_slot = (uint8_t)((s->diag_slot + 1) % nslot);
                guard++;
                continue;
            }
            break;
        }
        if (guard >= nslot) {
            return;
        }
    }
    s->diag_index = idx[s->diag_slot];
    s->diag_size = sz[s->diag_slot];
    (void)s->diag_size;
    if (co_sdo_read_send(m, s->diag_index, 0) != 0) {
        return;
    }
    s->diag_wait = true;
    s->diag_tx_ms = now;
}

void erob_poll(erob_servo_t *s)
{
    co_master_poll_rx(s->co);
    erob_poll_motion(s);
    erob_poll_diag(s);
    co_master_poll_sync(s->co);
}

int32_t erob_pwm_to_plus(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max,
                         int32_t plus_min, int32_t plus_max)
{
    int32_t span;
    int32_t delta;

    if (pwm_max <= pwm_min) {
        return plus_min;
    }
    if (pwm < pwm_min) {
        pwm = pwm_min;
    } else if (pwm > pwm_max) {
        pwm = pwm_max;
    }
    span = (int32_t)(pwm_max - pwm_min);
    delta = (int32_t)(pwm - pwm_min);
    return plus_min + (int32_t)(((int64_t)(plus_max - plus_min) * (int64_t)delta) / (int64_t)span);
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

const erob_feedback_t *erob_feedback(const erob_servo_t *s)
{
    return &s->fb;
}

void erob_cache_rating(erob_feedback_t *fb, uint16_t index, uint32_t value)
{
    if (fb == NULL || value == 0) {
        return;
    }
    if (index == 0x6075) {
        fb->rated_current_mA = value;
        fb->valid |= EROB_FB_IRATED;
    } else if (index == 0x6076) {
        fb->rated_torque_mNm = value;
        fb->valid |= EROB_FB_TRATED;
    }
}

static int32_t counts_or_default(int32_t counts_per_rev)
{
    if (counts_per_rev <= 0) {
        return EROB_PLUS_PER_REV;
    }
    return counts_per_rev;
}

float erob_position_deg(int32_t plus, int32_t counts_per_rev)
{
    return (float)((double)plus * 360.0 / (double)counts_or_default(counts_per_rev));
}

float erob_output_rpm(int32_t plus_per_s, int32_t counts_per_rev)
{
    return (float)((double)plus_per_s * 60.0 / (double)counts_or_default(counts_per_rev));
}

float erob_current_a(int16_t permille, uint32_t rated_mA)
{
    return (float)((double)permille * (double)rated_mA / 1000000.0);
}

float erob_torque_nm(int16_t permille, uint32_t rated_mNm)
{
    return (float)((double)permille * (double)rated_mNm / 1000000.0);
}
