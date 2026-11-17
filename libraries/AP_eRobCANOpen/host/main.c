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

static int count_cw(uint8_t node, uint16_t want)
{
    int n = 0;
    uint16_t i;
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        uint16_t cw;
        if (fr->is_ext || fr->id != (uint32_t)(0x200u + node) || fr->dlc < 2) {
            continue;
        }
        cw = (uint16_t)(fr->data[0] | (fr->data[1] << 8));
        if (cw == want) {
            n++;
        }
    }
    return n;
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

static void check_prearm(void)
{
    erob_servo_t s;
    const int32_t ctr = 262144;
    const int32_t pmin = -43691;
    const int32_t pmax = 43691;
    const int32_t res = EROB_PLUS_PER_REV;
    const int32_t margin = (int32_t)(((int64_t)res * 5) / 360);
    const int32_t hi = ctr + pmax + margin;
    const int32_t lo = ctr + pmin - margin;
    memset(&s, 0, sizeof(s));
    expect_true(margin == 7281, "5 deg at 524288 counts is 7281 plus");
    s.last_tpdo_ms = 1000;
    s.state = EROB_ST_ENABLED;
    s.statusword = EROB_SW_OP_ENABLED;
    s.pos_actual = ctr;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_OK,
                "center position may arm");
    s.pos_actual = hi;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_OK,
                "5 deg outside MAX may arm");
    s.pos_actual = hi + 1;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_POS,
                "more than 5 deg outside MAX blocks arming");
    s.pos_actual = lo;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_OK,
                "5 deg outside MIN may arm");
    s.pos_actual = lo - 1;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_POS,
                "more than 5 deg outside MIN blocks arming");
    s.pos_actual = ctr;
    s.statusword = (uint16_t)(EROB_SW_OP_ENABLED | EROB_SW_FAULT_BIT);
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_FAULT,
                "fault bit blocks arming");
    s.statusword = EROB_SW_READY_TO_SW;
    s.state = EROB_ST_ENABLE_SEQ;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_NOT_ENABLED,
                "enable sequence is not operation enabled");
    s.state = EROB_ST_ENABLED;
    s.statusword = EROB_SW_OP_ENABLED;
    expect_true(erob_prearm_status(&s, 1000 + EROB_NODE_SILENT_MS, ctr, pmin, pmax, res) == EROB_PREARM_OK,
                "TPDO age of 200 ms may still arm");
    expect_true(erob_prearm_status(&s, 1000 + EROB_NODE_SILENT_MS + 1, ctr, pmin, pmax, res) == EROB_PREARM_NO_RESPONSE,
                "TPDO age above 200 ms blocks arming");
    s.last_tpdo_ms = 0;
    expect_true(erob_prearm_status(&s, 1200, ctr, pmin, pmax, res) == EROB_PREARM_NO_RESPONSE,
                "no TPDO1 is no response");
}

static int find_sdo_cmd(uint8_t node, uint8_t cs, uint16_t index)
{
    uint16_t i;
    for (i = 0; i < can_bus_stub_tx_count(); i++) {
        const can_frame_t *fr = can_bus_stub_tx(i);
        uint16_t got;
        if (fr->is_ext || fr->id != (uint32_t)(0x600u + node) || fr->dlc < 4) {
            continue;
        }
        if (fr->data[0] != cs) {
            continue;
        }
        got = (uint16_t)(fr->data[1] | (fr->data[2] << 8));
        if (got == index) {
            return (int)i;
        }
    }
    return -1;
}

static int32_t rpdo_pos(const can_frame_t *fr)
{
    return (int32_t)((uint32_t)fr->data[2] | ((uint32_t)fr->data[3] << 8) |
                     ((uint32_t)fr->data[4] << 16) | ((uint32_t)fr->data[5] << 24));
}

#define HOME_SEQ_MAX 480

struct home_case {
    const char *name;
    int32_t actual;
    int32_t mid;
    int32_t lo;
    int32_t hi;
    int32_t rate;
    int32_t pwm_during;
    int32_t pwm_after;
    int hold;
    int far_ok;
};

static void check_home_case(const struct home_case *c)
{
    can_bus_t *bus;
    canopen_master_t master;
    erob_servo_t servo;
    const uint8_t node = (uint8_t)EROB_DEFAULT_NODE_ID;
    int32_t seq[HOME_SEQ_MAX];
    int n = 0;
    int i;
    int read_at;
    int write_at;
    int32_t lo;
    int32_t hi;
    int32_t use_rate;
    int32_t step;
    int32_t first_move = 0;
    int moved = 0;
    int segment_bad = 0;
    int guard;
    uint16_t cursor;

    bus = can_bus_stub_get();
    can_bus_stub_set_position(c->actual);
    co_master_init(&master, bus, node);
    erob_init(&servo, &master);
    erob_set_counts_per_rev(&servo, EROB_PLUS_PER_REV);
    /* Rate comes up before 0x6064 is known, which used to park the ramp at 0. */
    erob_set_rate_deg_s(&servo, 0);
    erob_set_rate_deg_s(&servo, c->rate);
    erob_set_acc_deg_s2(&servo, 0);
    erob_set_dec_deg_s2(&servo, 0);
    erob_set_home_window(&servo, c->mid, c->lo, c->hi);
    erob_set_home_far_ok(&servo, c->far_ok);

    expect_true(erob_configure_pdo_csp(&servo) == 0, c->name);
    read_at = find_sdo_cmd(node, 0x40, 0x6064);
    write_at = find_sdo_cmd(node, 0x23, 0x607A);
    expect_true(read_at >= 0 && write_at > read_at, "0x6064 is read before 0x607A is written");
    expect_true(find_sdo_u32(node, 0x607A, 0x00, (uint32_t)c->actual) >= 0,
                "configure 0x607A equals actual");

    can_bus_stub_tx_reset();
    /* Stick/mixer command is already present and is not on the way to mid. */
    erob_set_target_pos(&servo, c->pwm_during);
    expect_true(erob_enable(&servo) == 0, c->name);
    cursor = 0;
    for (guard = 0; guard < 160; guard++) {
        while (cursor < can_bus_stub_tx_count()) {
            const can_frame_t *fr = can_bus_stub_tx(cursor);
            cursor++;
            if (fr->is_ext || fr->id != (uint32_t)(0x200u + node) || fr->dlc < 6) {
                continue;
            }
            if (n < HOME_SEQ_MAX) {
                seq[n++] = rpdo_pos(fr);
            }
        }
        if (c->hold) {
            if (guard > 40 && erob_get_state(&servo) == EROB_ST_ENABLED) {
                break;
            }
        } else if (n > 0 && seq[n - 1] == c->mid &&
                   erob_get_state(&servo) == EROB_ST_ENABLED) {
            break;
        }
        /* push_servos() keeps rewriting the CSP target while the servo comes up. */
        erob_set_target_pos(&servo, c->pwm_during);
        can_bus_stub_advance(50);
        erob_poll(&servo);
    }
    expect_true(!can_bus_stub_overflow(), "home trace did not overflow the stub log");
    expect_true(n > 0 && n < HOME_SEQ_MAX, c->name);
    expect_true(n > 0 && seq[0] == c->actual, "first 0x607A equals actual");

    lo = c->actual < c->mid ? c->actual : c->mid;
    hi = c->actual < c->mid ? c->mid : c->actual;
    for (i = 0; i < n; i++) {
        int inside = seq[i] >= lo && seq[i] <= hi;
        int ordered = 1;
        if (i > 0) {
            if (c->mid >= c->actual) {
                ordered = seq[i] >= seq[i - 1];
            } else {
                ordered = seq[i] <= seq[i - 1];
            }
        }
        if ((!inside || !ordered) && !segment_bad) {
            fprintf(stderr, "FAIL: %s target %d -> %d leaves the straight path [%d, %d] to %d\n",
                    c->name, i > 0 ? seq[i - 1] : seq[i], seq[i], lo, hi, c->mid);
            g_fails++;
            segment_bad = 1;
        }
        if (seq[i] != c->actual && !moved) {
            first_move = seq[i];
            moved = 1;
        }
    }

    use_rate = c->rate > 0 ? c->rate : EROB_HOME_FALLBACK_DEG_PER_S;
    step = (int32_t)((int64_t)EROB_PLUS_PER_REV * (int64_t)use_rate * 50 / 360 / 1000);
    if (step < 1) {
        step = 1;
    }
    if (c->hold) {
        {
            uint16_t cw = 0;
            int32_t pos = 0;
            expect_true(!moved, "out-of-travel return does not leave actual");
            expect_true(erob_take_home_hold(&servo), "out-of-travel return warns once");
            expect_true(!erob_take_home_hold(&servo), "out-of-travel warning does not repeat");
            erob_set_target_pos(&servo, c->mid + 1000);
            can_bus_stub_advance(50);
            erob_poll(&servo);
            expect_true(last_rpdo(node, &cw, &pos) && pos == c->actual,
                        "out-of-travel hold does not follow PWM");
        }
    } else {
        int32_t want = c->mid >= c->actual ? c->actual + step : c->actual - step;
        expect_true(moved && first_move == want, c->name);
        if (!(moved && first_move == want)) {
            fprintf(stderr, "FAIL: %s first step %d want %d (moved %d)\n",
                    c->name, first_move, want, moved);
        }
        expect_true(n > 0 && seq[n - 1] == c->mid, "return reaches mid");
        if (c->far_ok) {
            expect_true(!erob_take_home_hold(&servo),
                        "allowed long return does not warn");
        }
        erob_set_target_pos(&servo, c->pwm_after);
        can_bus_stub_advance(50);
        erob_poll(&servo);
        {
            uint16_t cw = 0;
            int32_t pos = 0;
            expect_true(last_rpdo(node, &cw, &pos), "follow RPDO present");
            if (c->rate <= 0) {
                expect_true(pos == c->pwm_after, "after mid, RATE 0 follows the PWM target");
            } else if (c->pwm_after > c->mid) {
                expect_true(pos > c->mid && pos <= c->pwm_after,
                            "after mid, the target follows PWM upward");
            } else if (c->pwm_after < c->mid) {
                expect_true(pos < c->mid && pos >= c->pwm_after,
                            "after mid, the target follows PWM downward");
            } else {
                expect_true(pos == c->mid, "after mid, a mid PWM target stays at mid");
            }
        }
    }
}

static void check_return_to_mid(void)
{
    const int32_t ctr = 262144;
    const int32_t travel = 43691;
    const int32_t lo = ctr - travel;
    const int32_t hi = ctr + travel;
    const int32_t span = hi - lo;
    const int32_t margin = (int32_t)(((int64_t)EROB_PLUS_PER_REV * EROB_PREARM_MARGIN_DEG) / 360);
    const int32_t limit = span + margin;
    struct home_case cases[10];
    int i;
    can_bus_t *bus;
    canopen_master_t master;
    erob_servo_t servo;
    const uint8_t node = (uint8_t)EROB_DEFAULT_NODE_ID;

    expect_true(margin == 7281, "home margin matches the 5 deg pre-arm margin");
    expect_true(erob_mid_plus(-travel, travel, ctr) == ctr, "symmetric mid is CTR");
    expect_true(erob_mid_plus(-40000, 40000, 300000) == 300000, "mid at 300000");
    expect_true(erob_mid_plus(0, 0, 300000) == 300000, "zero MIN/MAX mid is CTR");

    memset(cases, 0, sizeof(cases));
    cases[0] = (struct home_case){
        "mid 262144 actual below moves up", 200000, ctr, lo, hi, 20, 100000, 300000, 0, 0
    };
    cases[1] = (struct home_case){
        "mid 262144 actual above moves down", 300000, ctr, lo, hi, 20, 400000, 200000, 0, 0
    };
    cases[2] = (struct home_case){
        "mid 300000 actual below moves up", 280000, 300000, 250000, 350000, 20, 100000, 340000, 0, 0
    };
    cases[3] = (struct home_case){
        "mid 300000 actual above moves down", 320000, 300000, 250000, 350000, 20, 400000, 260000, 0, 0
    };
    cases[4] = (struct home_case){
        "RATE 0 still slews to mid", 200000, ctr, lo, hi, 0, ctr, 300000, 0, 0
    };
    cases[5] = (struct home_case){
        "distance inside travel plus margin still moves",
        ctr - limit, ctr, lo, hi, 20, 100000, ctr + 1000, 0, 0
    };
    cases[6] = (struct home_case){
        "distance past travel plus margin holds",
        ctr - limit - 1, ctr, lo, hi, 20, ctr, ctr, 1, 0
    };
    cases[7] = (struct home_case){
        "actual right of mid and outside travel holds",
        400000, ctr, lo, hi, 20, ctr, ctr, 1, 0
    };
    cases[8] = (struct home_case){
        "FAR allows a straight return below mid past the margin",
        ctr - limit - 1, ctr, lo, hi, 20, 100000, ctr + 1000, 0, 1
    };
    cases[9] = (struct home_case){
        "FAR allows a straight return above mid past the margin",
        400000, ctr, lo, hi, 20, 500000, 200000, 0, 1
    };

    for (i = 0; i < 10; i++) {
        check_home_case(&cases[i]);
    }

    /*
     * Servo power-cycle after it had already reached mid. The stick stays
     * off to one side. The next enable must read the new actual and walk
     * back to mid, not continue toward the stick and not reuse the old count.
     */
    {
        can_bus_t *bus2;
        canopen_master_t master2;
        erob_servo_t servo2;
        int32_t seq[HOME_SEQ_MAX];
        int n = 0;
        int guard;
        uint16_t cursor = 0;
        const int32_t again = 200000;
        bus2 = can_bus_stub_get();
        can_bus_stub_set_position(300000);
        co_master_init(&master2, bus2, node);
        erob_init(&servo2, &master2);
        erob_set_counts_per_rev(&servo2, EROB_PLUS_PER_REV);
        erob_set_rate_deg_s(&servo2, 20);
        erob_set_acc_deg_s2(&servo2, 0);
        erob_set_dec_deg_s2(&servo2, 0);
        erob_set_home_window(&servo2, ctr, lo, hi);
        expect_true(erob_configure_pdo_csp(&servo2) == 0, "rehome configure");
        expect_true(erob_enable(&servo2) == 0, "rehome enable");
        for (guard = 0; guard < 160 && erob_get_state(&servo2) != EROB_ST_ENABLED; guard++) {
            can_bus_stub_advance(50);
            erob_set_target_pos(&servo2, 400000);
            erob_poll(&servo2);
        }
        for (guard = 0; guard < 160; guard++) {
            uint16_t cw = 0;
            int32_t pos = 0;
            can_bus_stub_advance(50);
            erob_set_target_pos(&servo2, 400000);
            erob_poll(&servo2);
            if (last_rpdo(node, &cw, &pos) && pos == ctr) {
                break;
            }
        }
        {
            uint16_t cw = 0;
            int32_t pos = 0;
            expect_true(last_rpdo(node, &cw, &pos) && pos == ctr, "rehome setup reaches mid");
        }
        can_bus_stub_set_position(again);
        can_bus_stub_set_silent(1);
        can_bus_stub_advance(EROB_NODE_SILENT_MS);
        erob_poll(&servo2);
        expect_true(erob_is_silent(&servo2), "power cycle is silent");
        can_bus_stub_set_silent(0);
        can_bus_stub_tx_reset();
        can_bus_stub_push_heartbeat(0x05);
        for (guard = 0; guard < 80 && erob_get_state(&servo2) != EROB_ST_ENABLED; guard++) {
            can_bus_stub_advance(50);
            erob_set_target_pos(&servo2, 100000);
            erob_poll(&servo2);
        }
        expect_true(erob_get_state(&servo2) == EROB_ST_ENABLED, "power cycle enables again");
        n = 0;
        cursor = 0;
        for (guard = 0; guard < 160; guard++) {
            while (cursor < can_bus_stub_tx_count()) {
                const can_frame_t *fr = can_bus_stub_tx(cursor);
                int32_t pos;
                cursor++;
                if (fr->is_ext || fr->id != (uint32_t)(0x200u + node) || fr->dlc < 6) {
                    continue;
                }
                pos = rpdo_pos(fr);
                if (n < HOME_SEQ_MAX) {
                    seq[n++] = pos;
                }
            }
            if (n > 0 && seq[n - 1] == ctr) {
                break;
            }
            can_bus_stub_advance(50);
            erob_set_target_pos(&servo2, 100000);
            erob_poll(&servo2);
        }
        expect_true(n > 0 && seq[0] == again, "power-cycle first 0x607A is the new actual");
        {
            int i;
            int bad = 0;
            for (i = 0; i < n; i++) {
                int ordered = i == 0 || seq[i] >= seq[i - 1];
                int inside = seq[i] >= again && seq[i] <= ctr;
                if ((!ordered || !inside) && !bad) {
                    fprintf(stderr, "FAIL: power cycle %d -> %d is not a straight rise to mid\n",
                            i > 0 ? seq[i - 1] : seq[i], seq[i]);
                    g_fails++;
                    bad = 1;
                }
            }
        }
        expect_true(n > 0 && seq[n - 1] == ctr, "power cycle returns to mid before the stick");
    }

    /* No 0x607A at all until 0x6064 has been read. */
    bus = can_bus_stub_get();
    can_bus_stub_fail_read(0x6064);
    co_master_init(&master, bus, node);
    erob_init(&servo, &master);
    erob_set_home_window(&servo, ctr, lo, hi);
    expect_true(erob_configure_pdo_csp(&servo) != 0, "configure fails when 0x6064 cannot be read");
    expect_true(find_sdo_cmd(node, 0x23, 0x607A) < 0, "failed 0x6064 read does not write 0x607A");
    can_bus_stub_tx_reset();
    expect_true(erob_enable(&servo) == -1, "enable waits until actual position is known");
    {
        uint16_t k;
        int rpdo = 0;
        for (k = 0; k < can_bus_stub_tx_count(); k++) {
            const can_frame_t *fr = can_bus_stub_tx(k);
            if (!fr->is_ext && fr->id == (uint32_t)(0x200u + node)) {
                rpdo++;
            }
        }
        expect_true(rpdo == 0, "enable does not send 0x607A before actual is known");
    }
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
    check_prearm();

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
    expect_true(erob_prearm_status(&servo, can_bus_millis(bus), 0, 0, EROB_PLUS_PER_REV, EROB_PLUS_PER_REV) == EROB_PREARM_OK,
                "live enabled servo may arm");
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

    can_bus_stub_tx_reset();
    can_bus_stub_set_fault(1);
    can_bus_stub_push_status((uint16_t)(EROB_SW_OP_ENABLED | EROB_SW_FAULT_BIT), 0);
    erob_poll(&servo);
    expect_true(erob_get_state(&servo) == EROB_ST_FAULT, "fault starts recovery");
    expect_true(erob_prearm_status(&servo, can_bus_millis(bus), 0, 0, EROB_PLUS_PER_REV, EROB_PLUS_PER_REV) == EROB_PREARM_FAULT,
                "live fault blocks arming");
    expect_true(erob_fault_count(&servo) == 1, "fault entry counts once");
    {
        int k;
        for (k = 0; k < 12; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
    }
    expect_true(count_cw(node, 0x0000) >= 2, "fault-reset edge is re-armed");
    expect_true(count_cw(node, EROB_CW_FAULT_RESET) >= 2, "fault reset retries while bit3 stays set");
    expect_true(count_cw(node, EROB_CW_SHUTDOWN) == 0, "no shutdown while the fault bit stays set");
    expect_true(count_cw(node, EROB_CW_SWITCH_ON) == 0, "no switch-on while the fault bit stays set");
    expect_true(count_cw(node, EROB_CW_ENABLE_OP) == 0, "no enable while the fault bit stays set");
    expect_true(erob_get_state(&servo) == EROB_ST_FAULT, "fault does not latch off");
    expect_true(erob_fault_count(&servo) == 1, "retries do not recount the same fault");

    can_bus_stub_set_fault(0);
    {
        int k;
        for (k = 0; k < 20 && erob_get_state(&servo) != EROB_ST_ENABLED; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
    }
    expect_true(erob_get_state(&servo) == EROB_ST_ENABLED, "fault recovery reaches operation enabled");
    expect_true(erob_take_recovered(&servo), "recovery is reported once");
    expect_true(!erob_take_recovered(&servo), "recovery report does not repeat");
    expect_true(last_rpdo(node, &cw, &pos), "bumpless RPDO present");
    expect_true(cw == EROB_CW_ENABLE_OP && pos == 0, "first recovered 0x607A is current 0x6064");

    erob_set_counts_per_rev(&servo, EROB_PLUS_PER_REV);
    erob_set_target_pos(&servo, 30000);
    can_bus_stub_advance(50);
    erob_poll(&servo);
    expect_true(last_rpdo(node, &cw, &pos), "ramp RPDO present");
    expect_true(pos == 7281, "50 ms ramp step is 100 deg/s at 524288 counts");
    can_bus_stub_advance(EROB_RECOVER_RAMP_MS);
    erob_poll(&servo);
    expect_true(last_rpdo(node, &cw, &pos) && pos == 30000, "ramp reaches the command within 0.6 s");

    can_bus_stub_set_fault(1);
    can_bus_stub_push_status((uint16_t)(EROB_SW_OP_ENABLED | EROB_SW_FAULT_BIT), 0);
    erob_poll(&servo);
    expect_true(erob_fault_count(&servo) == 2, "a later fault increments the count");
    can_bus_stub_set_fault(0);
    {
        int k;
        for (k = 0; k < 20 && erob_get_state(&servo) != EROB_ST_ENABLED; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
    }
    expect_true(erob_get_state(&servo) == EROB_ST_ENABLED, "a second fault also recovers");
    (void)erob_take_recovered(&servo);

    can_bus_stub_tx_reset();
    can_bus_stub_set_silent(1);
    can_bus_stub_advance(EROB_NODE_SILENT_MS);
    erob_poll(&servo);
    expect_true(erob_is_silent(&servo), "no heartbeat or PDO is silent");
    can_bus_stub_advance(1);
    expect_true(erob_prearm_status(&servo, can_bus_millis(bus), 0, 0, EROB_PLUS_PER_REV, EROB_PLUS_PER_REV) == EROB_PREARM_NO_RESPONSE,
                "live silence blocks arming");
    expect_true(erob_fault_count(&servo) == 2, "silence is not counted as a CiA402 fault");
    can_bus_stub_set_silent(0);
    can_bus_stub_push_heartbeat(0x05);
    erob_poll(&servo);
    expect_true(find_nmt(CO_NMT_START, node) >= 0, "reappearance sends NMT start");
    {
        int k;
        for (k = 0; k < 20 && erob_get_state(&servo) != EROB_ST_ENABLED; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
    }
    expect_true(erob_get_state(&servo) == EROB_ST_ENABLED, "silence recovery reaches operation enabled");
    expect_true(last_rpdo(node, &cw, &pos) && cw == EROB_CW_ENABLE_OP && pos == 0,
                "silence recovery writes 0x6064 before the ramp");
    expect_true(!can_bus_stub_overflow(), "no queue overflow after recovery");

    erob_set_counts_per_rev(&servo, EROB_PLUS_PER_REV);
    erob_set_rate_deg_s(&servo, 20);
    erob_set_acc_deg_s2(&servo, 0);
    erob_set_dec_deg_s2(&servo, 0);
    erob_set_target_pos(&servo, 100000);
    can_bus_stub_advance(50);
    erob_poll(&servo);
    expect_true(last_rpdo(node, &cw, &pos), "slew RPDO present");
    expect_true(cw == EROB_CW_ENABLE_OP && pos == 1456, "50 ms at RATE 20 deg/s is 1456 plus");

    erob_set_rate_deg_s(&servo, 0);
    erob_set_rate_deg_s(&servo, 100);
    erob_set_acc_deg_s2(&servo, 100);
    erob_set_dec_deg_s2(&servo, 100);
    erob_set_target_pos(&servo, 100000);
    can_bus_stub_advance(50);
    erob_poll(&servo);
    expect_true(last_rpdo(node, &cw, &pos) && pos > 0 && pos < 1000,
                "ACC 100 deg/s^2 first step is below the RATE cap");

    can_bus_stub_tx_reset();
    erob_set_target_pos(&servo, 43691);
    can_bus_stub_lock_status(1, 0x0240);
    can_bus_stub_push_status(0x0240, 0);
    {
        int k;
        for (k = 0; k < 8; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
    }
    expect_true(count_cw(node, EROB_CW_ENABLE_OP) == 0, "0x0240 does not repeat enable operation");
    expect_true(last_rpdo(node, &cw, &pos) && pos != 43691, "0x0240 does not hold the flight target");
    expect_true(cw == EROB_CW_SHUTDOWN || cw == 0x0000, "0x0240 commands shutdown at the actual position");
    can_bus_stub_lock_status(0, 0);
    {
        int k;
        for (k = 0; k < 30 && erob_get_state(&servo) != EROB_ST_ENABLED; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
    }
    expect_true(erob_get_state(&servo) == EROB_ST_ENABLED, "dropout recovery reaches operation enabled");
    (void)erob_take_recovered(&servo);

    can_bus_stub_tx_reset();
    can_bus_stub_set_fault_on_enable(1);
    erob_set_rate_deg_s(&servo, 0);
    erob_set_target_pos(&servo, 43691);
    {
        int k;
        int enable_at_limit = 0;
        int seen_reset = 0;
        for (k = 0; k < 40; k++) {
            can_bus_stub_advance(50);
            erob_poll(&servo);
        }
        for (k = 0; k < (int)can_bus_stub_tx_count(); k++) {
            const can_frame_t *fr = can_bus_stub_tx((uint16_t)k);
            uint16_t frame_cw;
            int32_t frame_pos;
            if (fr->is_ext || fr->id != (uint32_t)(0x200u + node) || fr->dlc < 6) {
                continue;
            }
            frame_cw = (uint16_t)(fr->data[0] | (fr->data[1] << 8));
            frame_pos = (int32_t)((uint32_t)fr->data[2] | ((uint32_t)fr->data[3] << 8) |
                                  ((uint32_t)fr->data[4] << 16) | ((uint32_t)fr->data[5] << 24));
            if (frame_cw == 0x0000) {
                seen_reset = 1;
            }
            if (seen_reset && frame_cw == EROB_CW_ENABLE_OP && frame_pos == 43691) {
                enable_at_limit++;
            }
        }
        expect_true(count_cw(node, EROB_CW_ENABLE_OP) >= 1, "persistent fault still attempts one enable");
        expect_true(count_cw(node, EROB_CW_ENABLE_OP) <= 4, "persistent fault does not re-enable every 0.2 s");
        expect_true(enable_at_limit == 0, "fault re-enable does not command the +30 deg target");
        expect_true(erob_get_state(&servo) == EROB_ST_FAULT, "persistent fault stays in recovery");
    }
    can_bus_stub_set_fault_on_enable(0);

    check_return_to_mid();

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
