#include "../arinc429.h"

#include <string.h>

static int a429_float_finite(float v)
{
    union {
        float f;
        uint32_t u;
    } x;
    x.f = v;
    return ((x.u >> 23) & 0xffu) != 0xffu;
}

static int a429_float_positive(float v)
{
    return a429_float_finite(v) && v > 0.0f;
}

static void a429_mode_clear_run(a429_mode_state_t *st)
{
    st->abort_latch = 0;
    st->flight_timing = 0;
    st->nr_reached = 0;
    st->flight_since_ms = 0;
}

int a429_mode_step(a429_mode_state_t *st, const a429_mode_in_t *in, a429_pcmd_levels_t *levels)
{
    uint8_t engine_run;
    uint8_t want_flight;
    uint8_t mode;

    if (st == 0 || in == 0) {
        return 0;
    }
    if (in->enable == 0 || in->eng_ch < 1u || in->eng_ch > A429_ENG_CH_MAX) {
        if (st->owned) {
            st->owned = 0;
            a429_mode_clear_run(st);
            return -1;
        }
        return 0;
    }
    if (levels == 0) {
        return 0;
    }

    st->owned = 1;
    engine_run = 0;
    if (in->rc_present && in->rc_valid && in->rc_failsafe == 0 &&
        in->rc_pwm > A429_ENG_PWM_HIGH) {
        engine_run = 1;
    }

    /* A released interlock or a disarm ends the runup latch. An absent
     * RC sample is not a release: the caller leaves both interlock flags
     * clear, and the latch stays until a real release or a disarm. */
    if (in->interlock_released || in->armed == 0) {
        a429_mode_clear_run(st);
    }

    want_flight = 0;
    if (engine_run && in->armed && in->interlock_high && in->interlock_released == 0 &&
        st->abort_latch == 0) {
        want_flight = 1;
    }

    if (want_flight) {
        mode = A429_MODE_FLIGHT;
        if (st->flight_timing == 0) {
            st->flight_timing = 1;
            st->flight_since_ms = in->now_ms;
            st->nr_reached = 0;
        }
        /* The first flight word is sent on this step. Nr is not required
         * before that word. The timer is unsigned time since the command. */
        if (a429_float_positive(in->nr_ref) && a429_float_positive(in->nr_crit_pct) &&
            in->runup_ms != 0u && in->nr_sample && a429_float_finite(in->nr_rpm) &&
            st->nr_reached == 0) {
            float pct = in->nr_rpm * 100.0f / in->nr_ref;
            if (a429_float_finite(pct) && pct >= in->nr_crit_pct) {
                st->nr_reached = 1;
            } else if ((uint32_t)(in->now_ms - st->flight_since_ms) >= in->runup_ms) {
                st->abort_latch = 1;
                st->flight_timing = 0;
                st->nr_reached = 0;
                mode = A429_MODE_IDLE;
            }
        }
    } else {
        st->flight_timing = 0;
        st->nr_reached = 0;
        mode = engine_run ? A429_MODE_IDLE : A429_MODE_STOP;
    }

    memset(levels, 0, sizeof(*levels));
    levels->mode = mode;
    levels->on_ground = in->on_ground ? 1u : 0u;
    return 1;
}
