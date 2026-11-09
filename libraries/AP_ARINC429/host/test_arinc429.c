/*
 * Host checks for the CS35 ARINC429 logical word.
 * Expected words below were calculated from appendix B bit positions,
 * not by calling the codec. Bit 1 is the uint32 LSB.
 *
 * Ng 50.00% label 0144 SDI 00 SM 11 data 5000:
 *   0x64 | (5000<<13) | (3<<29) = 0x62710064, 10 data bits set, so
 *   odd parity sets bit 32 -> 0xE2710064.
 */
#include "../arinc429.h"

#include <stdio.h>
#include <string.h>

static int g_fails;

static void expect_true(int cond, const char *msg)
{
    if (!cond) {
        printf("FAIL: %s\n", msg);
        g_fails++;
    }
}

static int bit_count(uint32_t w)
{
    int n = 0;
    int i;
    for (i = 0; i < 32; i++) {
        n += (int)((w >> i) & 1u);
    }
    return n;
}

static uint32_t oracle_word(uint8_t label, uint8_t sdi, uint16_t data, uint8_t sm)
{
    uint32_t w = (uint32_t)label;
    int n;
    int i;
    w |= ((uint32_t)(sdi & 3u)) << 8;
    w |= ((uint32_t)data) << 13;
    w |= ((uint32_t)(sm & 3u)) << 29;
    n = 0;
    for (i = 0; i < 31; i++) {
        n += (int)((w >> i) & 1u);
    }
    if ((n & 1) == 0) {
        w |= 1u << 31;
    }
    return w;
}

static int is_nan_f(float v)
{
    union {
        float f;
        uint32_t u;
    } x;
    x.f = v;
    return ((x.u >> 23) & 0xffu) == 0xffu && (x.u & 0x7fffffu) != 0u;
}

static void test_known_words(void)
{
    uint32_t w = 0xDEADBEEFu;
    const uint32_t ng50 = 0xE2710064u;
    const uint32_t t45_m65 = 0x7FAEC067u;
    const uint32_t mkp_m30 = 0x7FDA8069u;
    const uint32_t mkp_800 = 0xE3E80069u;
    const uint32_t ng0 = 0x60000064u;
    const uint32_t ng_max = 0xE5E24064u;

    expect_true(oracle_word(0144, 0, 5000, A429_SM_NORMAL) == ng50, "oracle Ng 50");
    expect_true(bit_count(ng50) % 2 == 1, "Ng 50 odd population");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &w) == 1, "encode Ng 50");
    expect_true(w == ng50, "Ng 50 matches hardcoded word");
    expect_true(((w >> 13) & 0xFFFFu) == 5000u, "Ng data in D14..D29");
    expect_true(((w >> 10) & 0x7u) == 0u, "reserved D11..D13 clear");
    expect_true(((w >> 8) & 0x3u) == 0u, "SDI D9..D10 is 00");
    expect_true(((w >> 29) & 0x3u) == A429_SM_NORMAL, "SM D31..D30 normal");
    expect_true((w & 0xFFu) == 0144, "label 0144 in D1..D8");

    w = 0xDEADBEEFu;
    expect_true(a429_encode_bnr(0147, 0, A429_SM_NORMAL, -65.0f, &w) == 1, "encode T45 -65");
    expect_true(w == t45_m65, "T45 -65 hardcoded");
    expect_true((int16_t)((w >> 13) & 0xFFFFu) == (int16_t)-650, "T45 sign in D29");

    expect_true(a429_encode_bnr(0151, 0, A429_SM_NORMAL, -30.0f, &w) == 1, "encode torque -30");
    expect_true(w == mkp_m30, "torque -30 hardcoded");
    expect_true(a429_encode_bnr(0151, 0, A429_SM_NORMAL, 800.0f, &w) == 1, "encode torque 800");
    expect_true(w == mkp_800, "torque 800 hardcoded");

    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 0.0f, &w) == 1, "encode Ng 0");
    expect_true(w == ng0, "Ng 0 hardcoded");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 120.5f, &w) == 1, "encode Ng 120.5");
    expect_true(w == ng_max, "Ng 120.5 hardcoded");
    expect_true(a429_encode_bnr(0145, 0, A429_SM_NORMAL, 150.0f, &w) == 1, "encode Np 150");
    expect_true(w == oracle_word(0145, 0, 15000, A429_SM_NORMAL), "Np 150 oracle");
    expect_true(a429_encode_bnr(0147, 0, A429_SM_NORMAL, 1060.0f, &w) == 1, "encode T45 1060");
    expect_true(w == oracle_word(0147, 0, 10600, A429_SM_NORMAL), "T45 1060 oracle");
}

static void test_reject_bad_numbers(void)
{
    uint32_t w = 0xDEADBEEFu;
    float nan_v = 0.0f;
    nan_v = nan_v / nan_v;

    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, nan_v, &w) == 0, "reject NaN");
    expect_true(w == 0xDEADBEEFu, "NaN leaves output unchanged");
    {
        union { float f; uint32_t u; } inf;
        inf.u = 0x7f800000u;
        expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, inf.f, &w) == 0, "reject +Inf");
        inf.u = 0xff800000u;
        expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, inf.f, &w) == 0, "reject -Inf");
        expect_true(w == 0xDEADBEEFu, "non-finite leaves output unchanged");
    }
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 1.0e30f, &w) == 0, "reject huge EU");
    expect_true(w == 0xDEADBEEFu, "huge EU leaves output unchanged");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 120.51f, &w) == 0, "reject Ng above 120.5");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, -0.01f, &w) == 0, "reject Ng below 0");
    expect_true(a429_encode_bnr(0147, 0, A429_SM_NORMAL, -65.1f, &w) == 0, "reject T45 below -65");
    expect_true(a429_encode_bnr(0147, 0, A429_SM_NORMAL, 1060.1f, &w) == 0, "reject T45 above 1060");
    expect_true(a429_encode_bnr(0151, 0, A429_SM_NORMAL, -30.1f, &w) == 0, "reject torque below -30");
    expect_true(a429_encode_bnr(0151, 0, A429_SM_NORMAL, 800.1f, &w) == 0, "reject torque above 800");
    expect_true(a429_encode_bnr(0144, 4, A429_SM_NORMAL, 1.0f, &w) == 0, "reject SDI > 3");
    expect_true(a429_encode_bnr(0144, 0, 4, 1.0f, &w) == 0, "reject SM > 3");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 1.0f, 0) == 0, "reject null output");
    expect_true(a429_encode_bnr(0146, 0, A429_SM_NORMAL, 10.0f, &w) == 1, "catalog Ngc still encodes");
    expect_true(a429_encode_bnr(0331, 0, A429_SM_NORMAL, 1.0f, &w) == 0, "label 331 is not a BNR encode");
}

static void test_all_sm_and_faults(void)
{
    uint32_t base = 0;
    uint32_t word;
    a429_word_view_t view;
    a429_link_t link;
    const uint32_t fail_w = 0x82710064u;
    const uint32_t ncd_w = 0x22710064u;
    const uint32_t ft_w = 0x42710064u;
    uint8_t sm;

    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &base) == 1, "base Ng");
    for (sm = 0; sm < 4; sm++) {
        expect_true(a429_encode_bnr(0144, 0, sm, 50.0f, &word) == 1, "encode each SM");
        expect_true(a429_parity_ok(word), "each SM keeps odd parity");
        a429_inspect(word, 0, &view);
        expect_true(view.sm == sm, "inspect SM");
        if (sm == A429_SM_NORMAL) {
            expect_true(view.eu_valid == 1, "only normal SM is valid EU");
            expect_true(view.eu > 49.9f && view.eu < 50.1f, "normal EU 50");
        } else {
            expect_true(view.eu_valid == 0, "non-normal SM is not valid EU");
            expect_true(is_nan_f(view.eu), "invalid EU is NaN");
        }
    }
    expect_true(a429_encode_bnr(0144, 0, A429_SM_FAIL, 50.0f, &word) == 1 && word == fail_w, "SM fail word");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_NCD, 50.0f, &word) == 1 && word == ncd_w, "SM ncd word");
    expect_true(a429_encode_bnr(0144, 0, A429_SM_FT, 50.0f, &word) == 1 && word == ft_w, "SM ft word");

    a429_inspect(base ^ (1u << 31), 0, &view);
    expect_true(view.parity_ok == 0 && view.eu_valid == 0, "parity flip invalid");
    expect_true(view.raw == (base ^ (1u << 31)), "parity flip keeps raw");
    expect_true(view.label == 0144, "parity flip keeps the label");

    word = (base & 0x7FFFFFFFu) | (1u << 10);
    {
        uint32_t fixed = word & 0x7FFFFFFFu;
        int n = 0;
        int i;
        for (i = 0; i < 31; i++) {
            n += (int)((fixed >> i) & 1u);
        }
        if ((n & 1) == 0) {
            fixed |= 1u << 31;
        }
        word = fixed;
    }
    a429_inspect(word, 0, &view);
    expect_true(view.parity_ok == 1 && view.reserved_ok == 0 && view.eu_valid == 0, "reserved bit invalid");

    word = oracle_word(0144, 1, 5000, A429_SM_NORMAL);
    a429_inspect(word, 0, &view);
    expect_true(view.sdi_ok == 0 && view.eu_valid == 0, "SDI 01 is not the default");

    word = oracle_word(0144, 0, 13000, A429_SM_NORMAL);
    a429_inspect(word, 0, &view);
    expect_true(view.parity_ok == 1 && view.in_range == 0 && view.eu_valid == 0, "over-range Ng invalid");

    a429_link_init(&link);
    a429_link_ingest(&link, base ^ (1u << 31), 1000u);
    expect_true(link.counters.parity_errors == 1u, "parity counter increments");
    expect_true(link.counters.rx_count == 1u, "rx counter increments");
    expect_true(link.rx[0].have_raw == 1 && link.rx[0].raw == (base ^ (1u << 31)), "raw kept on bad parity");
    expect_true(link.rx[0].have_valid == 0 && link.rx[0].fresh == 0, "bad parity is not fresh");
    expect_true(link.counters.valid_count == 0u, "bad parity is not valid");
    a429_link_ingest(&link, word, 2000u); /* over-range, still label 144 */
    expect_true(link.counters.range_errors == 1u, "range counter increments");
    expect_true(link.rx[0].raw == word, "over-range raw kept");
    expect_true(link.rx[0].fresh == 0, "over-range is not fresh");
}

static void test_edges(void)
{
    a429_edge_state_t a;
    a429_edge_state_t b;
    a429_pcmd_levels_t hi;
    a429_pcmd_levels_t lo;
    a429_pcmd_edges_t edges;
    a429_link_t link;
    uint32_t word = 0;
    uint16_t bits;

    memset(&hi, 0, sizeof(hi));
    memset(&lo, 0, sizeof(lo));
    hi.mode = A429_MODE_FLIGHT;
    hi.cold_run = 1;
    hi.false_start = 1;
    hi.channel_switch = 1;
    hi.rotor_brake = 1;
    hi.on_ground = 1;
    hi.power_assurance = 1;
    hi.overspeed_a = 1;
    hi.overspeed_b = 1;
    lo.mode = A429_MODE_STOP;

    a429_edge_reset(&a);
    a429_edge_reset(&b);
    a429_edge_update(&a, &hi, 0, &edges);
    expect_true(edges.cold_run == 0 && edges.false_start == 0 && edges.channel_switch == 0,
                "invalid sample emits no edge");
    expect_true(a.have == 0, "invalid sample does not arm the detector");

    a429_edge_update(&a, &hi, 1, &edges);
    expect_true(a.have == 1, "first valid sample arms");
    expect_true(edges.cold_run == 0 && edges.false_start == 0 && edges.channel_switch == 0,
                "startup high is not an edge");
    a429_edge_update(&a, &hi, 1, &edges);
    expect_true(edges.cold_run == 0, "held high is not an edge");
    a429_edge_update(&a, &lo, 1, &edges);
    expect_true(edges.cold_run == 0 && edges.channel_switch == 0, "falling level is not an edge");
    a429_edge_update(&a, &hi, 0, &edges);
    expect_true(edges.cold_run == 0 && a.prev.cold_run == 0, "invalid high does not move state");
    a429_edge_update(&a, &hi, 1, &edges);
    expect_true(edges.cold_run == 1 && edges.false_start == 1 && edges.channel_switch == 1,
                "0 to 1 is an edge");
    expect_true(edges.cold_run == 1, "edge result is separate from the level snapshot");

    a429_edge_update(&b, &hi, 1, &edges);
    expect_true(edges.cold_run == 0, "a second label starts clean");

    bits = (uint16_t)(4u | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 10));
    a429_pcmd_decode(bits, &hi);
    expect_true(hi.mode == A429_MODE_FLIGHT, "flight code 100");
    expect_true(hi.cold_run == 1 && hi.false_start == 1 && hi.rotor_brake == 1, "level bits D17..D19");
    expect_true(hi.on_ground == 1 && hi.power_assurance == 1, "ground and power-assurance levels");
    expect_true(hi.channel_switch == 1 && hi.overspeed_a == 1 && hi.overspeed_b == 1, "D22..D24 levels");

    memset(&lo, 0, sizeof(lo));
    lo.mode = A429_MODE_STOP;
    expect_true(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &lo, &word) == 1, "encode stop");
    expect_true(((word >> 13) & 7u) == 1u, "D16..D14 is 001");
    expect_true(a429_parity_ok(word), "stop word has odd parity");
    lo.mode = A429_MODE_IDLE;
    expect_true(a429_encode_pcmd(0342, 0, A429_SM_NORMAL, &lo, &word) == 1, "encode idle");
    expect_true(((word >> 13) & 7u) == 3u && (word & 0xFFu) == 0342, "idle code 011 on 342");
    lo.mode = A429_MODE_FLIGHT;
    expect_true(a429_encode_pcmd(0344, 0, A429_SM_NORMAL, &lo, &word) == 1, "encode flight");
    expect_true(((word >> 13) & 7u) == 4u && (word & 0xFFu) == 0344, "flight code 100 on 344");
    lo.mode = A429_MODE_UNKNOWN;
    expect_true(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &lo, &word) == 0, "unknown mode is not encoded");

    a429_link_init(&link);
    lo.mode = A429_MODE_FLIGHT;
    lo.cold_run = 1;
    lo.false_start = 1;
    lo.channel_switch = 1;
    expect_true(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &lo, &word) == 1, "pcmd 340 high");
    a429_link_ingest(&link, word, 10u);
    expect_true(link.pcmd[0].edges.cold_run == 0, "link startup does not edge 340");
    expect_true(link.pcmd[0].levels.cold_run == 1, "link snapshot is a level");
    lo.cold_run = 0;
    lo.false_start = 0;
    lo.channel_switch = 0;
    expect_true(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &lo, &word) == 1, "pcmd 340 low");
    a429_link_ingest(&link, word, 20u);
    lo.cold_run = 1;
    word ^= 1u << 31; /* parity error on the same label; level bits stay low */
    a429_link_ingest(&link, word, 30u);
    expect_true(link.pcmd[0].edges.cold_run == 0, "bad parity does not edge");
    expect_true(link.pcmd[0].levels.cold_run == 0, "bad parity keeps previous level");
    expect_true(a429_encode_pcmd(0342, 0, A429_SM_NORMAL, &lo, &word) == 1, "pcmd 342 high");
    a429_link_ingest(&link, word, 40u);
    expect_true(link.pcmd[1].edges.cold_run == 0, "342 has its own detector");
    expect_true(link.pcmd[0].edge.have == 1 && link.pcmd[0].levels.cold_run == 0, "340 state unchanged by 342");
}

static void test_schedule_and_timeout(void)
{
    a429_sched_t sched;
    a429_link_t link;
    uint32_t ng = 0;
    uint32_t bad;
    uint32_t t0 = 0xFFFFF000u;

    a429_sched_init(&sched);
    expect_true(a429_sched_poll(&sched, 0) == 0, "first poll only arms");
    expect_true(a429_sched_poll(&sched, 12000u) == 0, "12 ms is not the FC period");
    expect_true(a429_sched_poll(&sched, 12500u) == 1, "due at 12500 us");
    expect_true(a429_sched_poll(&sched, 12500u) == 0, "same time does not repeat");
    expect_true(a429_sched_poll(&sched, 24500u) == 0, "13 ms alternate is not used");
    expect_true(a429_sched_poll(&sched, 25000u) == 1, "second slot at +12500");
    expect_true(a429_sched_poll(&sched, 37500u) == 1, "third slot at +12500");
    expect_true(sched.tx_slots == 3u && sched.tx_slots_skipped == 0u, "on-time slots are not skips");

    a429_sched_init(&sched);
    a429_sched_poll(&sched, 0);
    expect_true(a429_sched_poll(&sched, 62500u) == 1, "late tick emits one slot");
    expect_true(sched.tx_slots_skipped == 4u, "late tick records skipped slots");
    expect_true(a429_sched_poll(&sched, 62500u) == 0, "late tick does not burst");
    expect_true(a429_sched_poll(&sched, 74999u) == 0, "reschedule is a full period ahead");
    expect_true(a429_sched_poll(&sched, 75000u) == 1, "next slot after the reschedule");

    a429_sched_init(&sched);
    expect_true(a429_sched_poll(&sched, t0) == 0, "arm near wrap");
    expect_true(a429_sched_poll(&sched, t0 + 12499u) == 0, "still early across the wrap");
    expect_true(a429_sched_poll(&sched, t0 + 12500u) == 1, "due across the microsecond wrap");
    expect_true(a429_sched_poll(&sched, t0 + 12500u) == 0, "wrap slot does not repeat");

    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 10.0f, &ng) == 1, "Ng for timeout");
    bad = ng ^ (1u << 31);

    a429_link_init(&link);
    a429_link_ingest(&link, ng, 0);
    a429_link_poll_timeouts(&link, 1000000u);
    expect_true(link.rx[0].fresh == 1 && link.rx[0].timeout_count == 0u,
                "24 ms nominal refresh is not a timeout");

    a429_link_set_timeout_us(&link, 0, 50000u);
    a429_link_ingest(&link, bad, 40000u);
    expect_true(link.rx[0].valid_us == 0u, "invalid noise does not move the valid time");
    expect_true(link.rx[0].fresh == 1, "noise does not by itself clear a still-fresh sample");
    a429_link_poll_timeouts(&link, 50000u);
    expect_true(link.rx[0].fresh == 0 && link.rx[0].timeout_count == 1u, "timeout once at the policy");
    a429_link_poll_timeouts(&link, 200000u);
    expect_true(link.rx[0].timeout_count == 1u, "timeout does not count every poll");
    a429_link_ingest(&link, bad, 200000u);
    expect_true(link.rx[0].fresh == 0 && link.rx[0].valid_us == 0u, "more noise does not revive data");
    expect_true(link.rx[0].raw == bad && link.rx[0].rx_us == 200000u, "noise still updates raw and rx time");
    a429_link_ingest(&link, ng, 210000u);
    expect_true(link.rx[0].fresh == 1 && link.rx[0].valid_us == 210000u, "a valid word is fresh again");

    a429_link_poll_timeouts(&link, 260000u);
    expect_true(link.rx[0].fresh == 0 && link.rx[0].timeout_count == 2u, "second timeout is a new event");
}

static void test_catalog_and_tx_authority(void)
{
    uint16_t n = 0;
    uint16_t i;
    uint16_t phase1 = 0;
    const a429_label_info_t *rows;
    const a429_label_info_t *life;
    const a429_label_info_t *ngc;
    const a429_label_info_t *state331;
    a429_core_t core;
    a429_pcmd_levels_t levels;
    uint32_t buf[4];
    uint32_t ng = 0;

    rows = a429_catalog(&n);
    expect_true(rows != 0 && n > 20u, "catalog lists the appendix");
    for (i = 0; i < n; i++) {
        phase1 = (uint16_t)(phase1 + rows[i].phase1);
    }
    expect_true(phase1 == 12u, "phase one is the approved minimal set");
    ngc = a429_catalog_find(0146);
    expect_true(ngc != 0 && ngc->phase1 == 0 && ngc->has_range == 1, "Ngc is documented, not phase one");
    state331 = a429_catalog_find(0331);
    expect_true(state331 != 0 && state331->phase1 == 0 && state331->kind == A429_KIND_DISC,
                "label 331 stays documented");
    life = a429_catalog_find(0210);
    expect_true(life != 0 && life->phase1 == 0, "life labels 0200..0220 are documented");
    expect_true(a429_catalog_find(0200) != 0 && a429_catalog_find(0220) != 0, "life span endpoints");
    expect_true(a429_catalog_find(0221) == 0, "label past the life span is absent");
    expect_true(a429_phase1_rx_index(0144) == 0, "Ng slot");
    expect_true(a429_phase1_rx_index(0145) == 1, "Np slot");
    expect_true(a429_phase1_rx_index(0147) == 2, "T45 slot");
    expect_true(a429_phase1_rx_index(0151) == 3, "torque slot");
    expect_true(a429_phase1_rx_index(0175) == 4, "warn slot");
    expect_true(a429_phase1_rx_index(0176) == 5, "state slot");
    expect_true(a429_phase1_rx_index(0177) == 6, "warn2 slot");
    expect_true(a429_phase1_rx_index(0146) < 0, "Ngc is not a tracked receive slot");

    memset(buf, 0xA5, sizeof(buf));
    expect_true(a429_transport_null()->recv_words(0, 0, buf, 4) == 0, "null recv yields no words");
    expect_true(buf[0] == 0xA5A5A5A5u && buf[1] == 0xA5A5A5A5u, "null recv writes no frame");
    expect_true(a429_transport_null()->send_words(0, 0, buf, 1) == -1, "null send refuses");

    a429_core_init(&core);
    expect_true(core.endpoint.carrier == A429_CARRIER_UNSPECIFIED, "carrier is unspecified");
    core.endpoint.remote_ipv4 = 0x0A000002u;
    core.endpoint.remote_port = 4000;
    core.endpoint.local_port = 4001;
    a429_core_update(&core, 0);
    a429_core_update(&core, 12500u);
    expect_true(core.link.last_tx_count == 0, "no sim input means no command words");
    expect_true(core.tx_not_sent == 0u, "an empty set is not presented as a send");
    expect_true(core.sched.tx_slots == 1u, "the slot still elapses");

    memset(&levels, 0, sizeof(levels));
    levels.mode = A429_MODE_FLIGHT;
    levels.rotor_brake = 1;
    expect_true(a429_sim_set_pcmd(&core.link, 0, &levels) == 1, "sim may supply a level snapshot");
    a429_sim_set_clp(&core.link, 2.5f);
    a429_sim_set_nr(&core.link, 100.0f);
    a429_core_update(&core, 25000u);
    expect_true(core.link.last_tx_count == 3u, "sim builds CLP, Nr and one PCMD");
    expect_true((core.link.last_tx[0] & 0xFFu) == 0310, "first word is CLP");
    expect_true((core.link.last_tx[1] & 0xFFu) == 0311, "second word is Nr");
    expect_true((core.link.last_tx[2] & 0xFFu) == 0340, "third word is PCMD1");
    expect_true(((core.link.last_tx[2] >> 13) & 7u) == A429_MODE_FLIGHT, "PCMD carries the supplied level");
    expect_true(core.tx_not_sent == 3u, "null carrier did not accept the words");
    a429_core_update(&core, 25000u);
    expect_true(core.tx_not_sent == 3u, "no catch-up send on the same tick");

    expect_true(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 80.0f, &ng) == 1, "Ng inject");
    a429_core_inject_rx(&core, ng, 25000u);
    expect_true(core.link.rx[0].fresh == 1, "injected Ng is fresh in simulation");
    a429_core_inject_rx(&core, oracle_word(0146, 0, 1000, A429_SM_NORMAL), 26000u);
    expect_true(core.link.rx_untracked == 1u, "Ngc is not a phase-one channel");
    expect_true(core.link.rx[0].fresh == 1, "an untracked label does not disturb Ng");
    a429_core_inject_rx(&core, oracle_word(0310, 0, 100, A429_SM_NORMAL), 27000u);
    expect_true(core.link.rx_untracked == 2u, "a TX label does not keep RX data as a receive channel");
}

/* The carrier test double lives only in this file. Firmware ships the null carrier. */
static uint32_t g_seen_ip;
static uint16_t g_seen_rp;
static uint16_t g_seen_lp;
static uint8_t g_seen_carrier;
static int g_send_calls;
static uint32_t g_rx_word;
static int g_rx_left;

static int fake_send2(void *ctx, const a429_endpoint_t *ep, const uint32_t *words, uint8_t n)
{
    (void)ctx;
    (void)words;
    g_send_calls++;
    if (ep != 0) {
        g_seen_ip = ep->remote_ipv4;
        g_seen_rp = ep->remote_port;
        g_seen_lp = ep->local_port;
        g_seen_carrier = ep->carrier;
    }
    expect_true(n > 0, "send is not called with an empty set");
    return -1;
}

static int fake_recv2(void *ctx, const a429_endpoint_t *ep, uint32_t *words, uint8_t max_n)
{
    (void)ctx;
    (void)ep;
    if (g_rx_left <= 0 || max_n == 0 || words == 0) {
        return 0;
    }
    words[0] = g_rx_word;
    g_rx_left = 0;
    return 1;
}

static void test_carrier_interface(void)
{
    a429_core_t core;
    a429_transport_t fake;
    a429_pcmd_levels_t levels;
    uint32_t ng;

    memset(&fake, 0, sizeof(fake));
    fake.send_words = fake_send2;
    fake.recv_words = fake_recv2;
    a429_core_init(&core);
    core.endpoint.remote_ipv4 = 0xC0A8010Au;
    core.endpoint.remote_port = 5000;
    core.endpoint.local_port = 5001;
    core.endpoint.carrier = A429_CARRIER_UNSPECIFIED;
    a429_core_set_transport(&core, &fake);
    memset(&levels, 0, sizeof(levels));
    levels.mode = A429_MODE_IDLE;
    a429_sim_set_pcmd(&core.link, 1, &levels);
    g_send_calls = 0;
    a429_core_update(&core, 0);
    expect_true(g_send_calls == 0, "arming the schedule does not send");
    a429_core_update(&core, 12500u);
    expect_true(g_send_calls == 1, "one due slot sends once");
    expect_true(g_seen_ip == 0xC0A8010Au && g_seen_rp == 5000 && g_seen_lp == 5001, "endpoint is passed through");
    expect_true(g_seen_carrier == A429_CARRIER_UNSPECIFIED, "send does not invent a UDP or TCP choice");
    expect_true((core.link.last_tx[0] & 0xFFu) == 0342, "the word is PCMD2");
    expect_true(core.tx_not_sent == 1u, "a refusing carrier is counted");

    expect_true(a429_encode_bnr(0175, 0, A429_SM_NORMAL, 0.0f, &ng) == 0, "warn is not a BNR value");
    ng = oracle_word(0175, 0, 0x1555, A429_SM_NORMAL);
    g_rx_word = ng;
    g_rx_left = 1;
    a429_core_update(&core, 25000u);
    expect_true(core.link.rx[4].have_raw == 1 && core.link.rx[4].fresh == 1, "warn word received via the carrier");
    expect_true(core.link.rx[4].view.disc_valid == 1, "warn is a discrete word");
    expect_true(core.link.rx[4].view.data == (int16_t)0x1555, "warn keeps the data field");
    expect_true(core.link.rx[4].view.eu_valid == 0, "warn has no engineering value");

    ng = oracle_word(0176, 0, 0x0001, A429_SM_FAIL);
    a429_core_inject_rx(&core, ng, 26000u);
    expect_true(core.link.rx[5].fresh == 0 && core.link.rx[5].have_raw == 1, "failed state word is stored and not fresh");
    expect_true(core.link.counters.sm_fail >= 1u, "SM fail is a counter");
}

static void test_disc_sm_matrix(void)
{
    a429_link_t link;
    uint32_t word;
    uint8_t sm;
    a429_link_init(&link);
    for (sm = 0; sm < 4; sm++) {
        word = oracle_word(0177, 0, 0x00AAu, sm);
        a429_link_ingest(&link, word, 1000u + sm);
    }
    expect_true(link.counters.sm_fail == 1u, "one fail");
    expect_true(link.counters.sm_ncd == 1u, "one non-computed");
    expect_true(link.counters.sm_ft == 1u, "one functional test");
    expect_true(link.counters.sm_normal == 1u, "one normal");
    expect_true(link.rx[6].fresh == 1, "only the normal warn2 word is fresh");
    expect_true(link.rx[6].view.data == (int16_t)0x00AA, "warn2 data kept");
    word = oracle_word(0177, 0, 0x0001, A429_SM_NCD);
    a429_link_ingest(&link, word, 2000u);
    expect_true(link.rx[6].fresh == 1, "NCD does not extend or replace the valid sample's freshness time");
    expect_true(link.rx[6].valid_us == 1003u, "valid time stays on the normal word");
    expect_true(link.rx[6].raw == word, "latest raw is the NCD word");
}

int a429_host_tests(void);

int a429_host_tests(void)
{
    g_fails = 0;
    test_known_words();
    test_reject_bad_numbers();
    test_all_sm_and_faults();
    test_edges();
    test_schedule_and_timeout();
    test_catalog_and_tx_authority();
    test_carrier_interface();
    test_disc_sm_matrix();
    if (g_fails == 0) {
        printf("RESULT: all passed\n");
    } else {
        printf("RESULT: %d failed\n", g_fails);
    }
    return g_fails;
}

#ifndef A429_NO_MAIN
int a429_nt1m_tests(void);
int main(void)
{
    int fails = a429_host_tests();
    fails += a429_nt1m_tests();
    return fails == 0 ? 0 : 1;
}
#endif
