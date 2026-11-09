#include "../arinc429.h"

#include <string.h>

static int null_send(void *ctx, const a429_endpoint_t *ep, const uint32_t *words, uint8_t n)
{
    (void)ctx;
    (void)ep;
    (void)words;
    (void)n;
    return -1;
}

static int null_recv(void *ctx, const a429_endpoint_t *ep, uint32_t *words, uint8_t max_n)
{
    (void)ctx;
    (void)ep;
    (void)words;
    (void)max_n;
    return 0;
}

static const a429_transport_t null_transport = {null_send, null_recv, 0};

const a429_transport_t *a429_transport_null(void)
{
    return &null_transport;
}

void a429_core_init(a429_core_t *core)
{
    if (core == 0) {
        return;
    }
    memset(core, 0, sizeof(*core));
    a429_link_init(&core->link);
    a429_sched_init(&core->sched);
    a429_tcp_init(&core->tcp);
    a429_frame_dec_init(&core->frame_dec, A429_FRAME_NONE);
    core->endpoint.carrier = A429_CARRIER_UNSPECIFIED;
    core->transport = a429_transport_null();
}

static void core_drop_wire_state(a429_core_t *core)
{
    a429_frame_dec_resync(&core->frame_dec);
    a429_tcp_abandon_hold(&core->tcp);
    a429_link_session_lost(&core->link);
}

void a429_core_set_frame(a429_core_t *core, uint8_t profile)
{
    if (core == 0) {
        return;
    }
    if (!a429_frame_profile_active(profile)) {
        profile = A429_FRAME_NONE;
    }
    if (core->frame_profile != profile) {
        core->frame_profile = profile;
        a429_frame_dec_init(&core->frame_dec, profile);
        core_drop_wire_state(core);
    }
}

void a429_core_set_sdi(a429_core_t *core, uint8_t sdi)
{
    uint8_t prev;
    if (core == 0 || sdi > 3u) {
        return;
    }
    prev = core->link.expected_sdi;
    a429_link_set_sdi(&core->link, sdi);
    if (prev != core->link.expected_sdi) {
        a429_frame_dec_resync(&core->frame_dec);
        a429_tcp_abandon_hold(&core->tcp);
    }
}

int a429_core_fresh_eu(const a429_core_t *core, uint8_t rx_index, float *eu)
{
    if (core == 0 || core->tcp_epoch_seen != core->tcp.epoch) {
        return 0;
    }
    return a429_link_fresh_eu(&core->link, rx_index, eu);
}

int a429_core_fresh_disc(const a429_core_t *core, uint8_t rx_index, int16_t *data)
{
    if (core == 0 || core->tcp_epoch_seen != core->tcp.epoch) {
        return 0;
    }
    return a429_link_fresh_disc(&core->link, rx_index, data);
}

void a429_core_set_transport(a429_core_t *core, const a429_transport_t *transport)
{
    if (core == 0) {
        return;
    }
    core->transport = (transport != 0) ? transport : a429_transport_null();
}

void a429_core_update(a429_core_t *core, uint32_t now_us)
{
    const a429_transport_t *transport;
    uint32_t tx[A429_TX_MAX];
    uint32_t rx[8];
    uint8_t n;
    int got;
    int i;

    if (core == 0) {
        return;
    }
    if (!core->tcp.io_external) {
        a429_tcp_poll(&core->tcp, now_us);
    }
    transport = (core->transport != 0) ? core->transport : a429_transport_null();
    if (a429_sched_poll(&core->sched, now_us)) {
        n = a429_link_build_tx(&core->link, tx, A429_TX_MAX);
        if (n > 0 && a429_frame_profile_active(core->frame_profile)) {
            uint8_t framed[A429_TCP_TX_CAP];
            uint8_t framed_n = 0;
            if (a429_frame_encode(core->frame_profile, tx, n, framed, (uint8_t)sizeof(framed), &framed_n) &&
                a429_tcp_submit(&core->tcp, framed, framed_n, now_us)) {
                /* one current slot is in the byte pipe; nothing is queued behind it */
            } else {
                core->tx_not_sent += n;
            }
        } else if (n > 0 && core->tcp.stream.open != 0) {
            /* TCP is present and framing is not opted in. Do not invent bytes. */
            core->tx_unframed += n;
        } else if (n > 0 && transport->send_words != 0) {
            if (transport->send_words(transport->ctx, &core->endpoint, tx, n) != 0) {
                core->tx_not_sent += n;
            }
        }
    }
    {
        uint8_t raw[32];
        uint8_t gotb;
        for (;;) {
            uint32_t epoch = core->tcp_epoch_seen;
            gotb = a429_tcp_drain_epoch(&core->tcp, raw, (uint8_t)sizeof(raw), &epoch);
            if (epoch != core->tcp_epoch_seen) {
                a429_frame_dec_resync(&core->frame_dec);
                a429_link_session_lost(&core->link);
                core->tcp_epoch_seen = epoch;
            }
            if (gotb == 0) {
                break;
            }
            if (a429_frame_profile_active(core->frame_profile)) {
                uint32_t words[8];
                uint8_t nw = a429_frame_push(&core->frame_dec, raw, gotb, words, 8);
                uint8_t wi;
                for (wi = 0; wi < nw; wi++) {
                    a429_link_ingest(&core->link, words[wi], now_us);
                }
            } else {
                core->rx_unframed_bytes += gotb;
            }
        }
    }
    if (transport->recv_words != 0) {
        got = transport->recv_words(transport->ctx, &core->endpoint, rx, 8);
        if (got > 8) {
            got = 8;
        }
        for (i = 0; i < got; i++) {
            a429_link_ingest(&core->link, rx[i], now_us);
        }
    }
    a429_link_poll_timeouts(&core->link, now_us);
}

void a429_core_inject_rx(a429_core_t *core, uint32_t raw, uint32_t now_us)
{
    if (core == 0) {
        return;
    }
    a429_link_ingest(&core->link, raw, now_us);
}
