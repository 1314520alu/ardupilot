#include "../arinc429.h"

#include <string.h>

static const uint8_t pcmd_labels[A429_PCMD_N] = {0340, 0342, 0344};

static void note_counters(a429_counters_t *c, const a429_word_view_t *view)
{
    const a429_label_info_t *info;

    c->rx_count++;
    if (!view->parity_ok) {
        c->parity_errors++;
        return;
    }
    if (!view->reserved_ok) {
        c->reserved_errors++;
        return;
    }
    if (!view->sdi_ok) {
        c->sdi_errors++;
        return;
    }
    if (!view->known) {
        return;
    }
    if (view->sm == A429_SM_FAIL) {
        c->sm_fail++;
    } else if (view->sm == A429_SM_NCD) {
        c->sm_ncd++;
    } else if (view->sm == A429_SM_FT) {
        c->sm_ft++;
    } else if (view->sm == A429_SM_NORMAL) {
        c->sm_normal++;
    }
    info = a429_catalog_find(view->label);
    if (info != 0 && info->has_range && view->sm == A429_SM_NORMAL && !view->eu_valid) {
        c->range_errors++;
    }
    if (view->eu_valid || view->disc_valid) {
        c->valid_count++;
    }
}

void a429_edge_reset(a429_edge_state_t *st)
{
    if (st == 0) {
        return;
    }
    memset(st, 0, sizeof(*st));
}

void a429_edge_update(a429_edge_state_t *st, const a429_pcmd_levels_t *now,
                      int sample_valid, a429_pcmd_edges_t *edges)
{
    if (edges != 0) {
        memset(edges, 0, sizeof(*edges));
    }
    if (st == 0 || edges == 0 || !sample_valid || now == 0) {
        return;
    }
    if (!st->have) {
        st->have = 1;
        st->prev = *now;
        return;
    }
    edges->cold_run = (uint8_t)(st->prev.cold_run == 0 && now->cold_run != 0);
    edges->false_start = (uint8_t)(st->prev.false_start == 0 && now->false_start != 0);
    edges->channel_switch = (uint8_t)(st->prev.channel_switch == 0 && now->channel_switch != 0);
    st->prev = *now;
}

void a429_link_init(a429_link_t *link)
{
    if (link == 0) {
        return;
    }
    memset(link, 0, sizeof(*link));
}

void a429_link_set_sdi(a429_link_t *link, uint8_t sdi)
{
    if (link == 0 || sdi > 3u) {
        return;
    }
    link->expected_sdi = sdi;
}

void a429_link_set_timeout_us(a429_link_t *link, uint8_t rx_index, uint32_t timeout_us)
{
    if (link == 0 || rx_index >= A429_RX_N) {
        return;
    }
    link->rx[rx_index].timeout_us = timeout_us;
}

void a429_link_ingest(a429_link_t *link, uint32_t raw, uint32_t now_us)
{
    a429_word_view_t view;
    int rx;
    int pcmd;
    int valid;

    if (link == 0) {
        return;
    }
    a429_inspect(raw, link->expected_sdi, &view);
    note_counters(&link->counters, &view);
    rx = a429_phase1_rx_index(view.label);
    if (rx >= 0) {
        a429_rx_slot_t *slot = &link->rx[rx];
        valid = view.eu_valid || view.disc_valid;
        slot->raw = raw;
        slot->rx_us = now_us;
        slot->have_raw = 1;
        if (valid) {
            slot->view = view;
            slot->have_valid = 1;
            slot->valid_us = now_us;
            slot->fresh = 1;
            if (view.eu_valid) {
                slot->last_valid_eu = view.eu;
            }
        } else if (!slot->have_valid) {
            slot->view = view;
        }
        return;
    }
    pcmd = a429_pcmd_index(view.label);
    if (pcmd >= 0) {
        a429_pcmd_slot_t *slot = &link->pcmd[pcmd];
        a429_pcmd_levels_t levels;
        memset(&levels, 0, sizeof(levels));
        slot->raw = raw;
        slot->rx_us = now_us;
        slot->have_raw = 1;
        slot->view = view;
        if (view.disc_valid) {
            a429_pcmd_decode((uint16_t)view.data, &levels);
            slot->levels = levels;
        }
        a429_edge_update(&slot->edge, &levels, view.disc_valid, &slot->edges);
        return;
    }
    link->rx_untracked++;
    link->last_untracked_raw = raw;
    link->last_untracked_us = now_us;
}

void a429_link_poll_timeouts(a429_link_t *link, uint32_t now_us)
{
    uint8_t i;
    if (link == 0) {
        return;
    }
    for (i = 0; i < A429_RX_N; i++) {
        a429_rx_slot_t *slot = &link->rx[i];
        int32_t age;
        if (slot->timeout_us == 0u || !slot->have_valid || !slot->fresh) {
            continue;
        }
        age = (int32_t)(now_us - slot->valid_us);
        if (age >= (int32_t)slot->timeout_us) {
            slot->fresh = 0;
            slot->timeout_count++;
        }
    }
}

void a429_sim_clear(a429_link_t *link)
{
    if (link == 0) {
        return;
    }
    memset(&link->sim, 0, sizeof(link->sim));
}

void a429_sim_set_clp(a429_link_t *link, float volts)
{
    if (link == 0) {
        return;
    }
    link->sim.clp_set = 1;
    link->sim.clp_v = volts;
}

void a429_sim_set_nr(a429_link_t *link, float pct)
{
    if (link == 0) {
        return;
    }
    link->sim.nr_set = 1;
    link->sim.nr_pct = pct;
}

int a429_sim_set_pcmd(a429_link_t *link, uint8_t index, const a429_pcmd_levels_t *levels)
{
    if (link == 0 || levels == 0 || index >= A429_PCMD_N) {
        return 0;
    }
    link->sim.pcmd_set[index] = 1;
    link->sim.pcmd[index] = *levels;
    return 1;
}

uint8_t a429_link_build_tx(a429_link_t *link, uint32_t *words, uint8_t max_n)
{
    uint8_t n = 0;
    uint8_t i;
    uint32_t w;

    if (link == 0) {
        return 0;
    }
    link->last_tx_count = 0;
    if (words == 0 || max_n == 0) {
        return 0;
    }
    if (link->sim.clp_set) {
        if (a429_encode_bnr(0310, link->expected_sdi, A429_SM_NORMAL, link->sim.clp_v, &w) && n < max_n) {
            words[n++] = w;
        } else {
            link->tx_rejected++;
        }
    }
    if (link->sim.nr_set) {
        if (a429_encode_bnr(0311, link->expected_sdi, A429_SM_NORMAL, link->sim.nr_pct, &w) && n < max_n) {
            words[n++] = w;
        } else {
            link->tx_rejected++;
        }
    }
    for (i = 0; i < A429_PCMD_N; i++) {
        if (!link->sim.pcmd_set[i]) {
            continue;
        }
        if (a429_encode_pcmd(pcmd_labels[i], link->expected_sdi, A429_SM_NORMAL, &link->sim.pcmd[i], &w) && n < max_n) {
            words[n++] = w;
        } else {
            link->tx_rejected++;
        }
    }
    link->last_tx_count = n;
    for (i = 0; i < n && i < A429_TX_MAX; i++) {
        link->last_tx[i] = words[i];
    }
    link->tx_words_built += n;
    return n;
}
