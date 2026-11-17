#include "../arinc429.h"

#include <string.h>

void a429_sched_init(a429_sched_t *s)
{
    if (s == 0) {
        return;
    }
    memset(s, 0, sizeof(*s));
}

int a429_sched_poll(a429_sched_t *s, uint32_t now_us)
{
    int32_t late;

    if (s == 0) {
        return 0;
    }
    if (!s->started) {
        s->started = 1;
        s->next_tx_us = now_us + A429_FC_PERIOD_US;
        return 0;
    }
    late = (int32_t)(now_us - s->next_tx_us);
    if (late < 0) {
        return 0;
    }
    s->tx_slots++;
    if ((uint32_t)late >= A429_FC_PERIOD_US) {
        s->tx_slots_skipped += (uint32_t)late / A429_FC_PERIOD_US;
        s->next_tx_us = now_us + A429_FC_PERIOD_US;
    } else {
        s->next_tx_us += A429_FC_PERIOD_US;
    }
    return 1;
}
