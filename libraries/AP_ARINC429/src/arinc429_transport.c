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
    core->endpoint.carrier = A429_CARRIER_UNSPECIFIED;
    core->transport = a429_transport_null();
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
    transport = (core->transport != 0) ? core->transport : a429_transport_null();
    if (a429_sched_poll(&core->sched, now_us)) {
        n = a429_link_build_tx(&core->link, tx, A429_TX_MAX);
        if (n > 0 && transport->send_words != 0) {
            if (transport->send_words(transport->ctx, &core->endpoint, tx, n) != 0) {
                core->tx_not_sent += n;
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
