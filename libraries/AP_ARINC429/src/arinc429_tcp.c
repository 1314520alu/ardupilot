#include "../arinc429.h"

#include <string.h>

static void tcp_lock(a429_tcp_t *tcp)
{
    if (tcp->lock != 0) {
        tcp->lock(tcp->lock_ctx);
    }
}

static void tcp_unlock(a429_tcp_t *tcp)
{
    if (tcp->unlock != 0) {
        tcp->unlock(tcp->lock_ctx);
    }
}

static int peer_ok(const a429_tcp_t *tcp)
{
    return tcp->enabled && tcp->peer_ip != 0u && tcp->peer_port != 0u;
}

static void tcp_drop_tx(a429_tcp_t *tcp)
{
    tcp->tx_len = 0;
    tcp->tx_off = 0;
}

static void tcp_drop_rx(a429_tcp_t *tcp)
{
    tcp->rx_count = 0;
}

void a429_tcp_init(a429_tcp_t *tcp)
{
    if (tcp == 0) {
        return;
    }
    memset(tcp, 0, sizeof(*tcp));
    tcp->state = A429_TCP_OFF;
    tcp->backoff_us = A429_TCP_BACKOFF_MIN_US;
}

void a429_tcp_set_stream(a429_tcp_t *tcp, const a429_stream_t *stream)
{
    if (tcp == 0) {
        return;
    }
    tcp_lock(tcp);
    if (tcp->stream.close != 0) {
        void (*close_fn)(void *) = tcp->stream.close;
        void *ctx = tcp->stream.ctx;
        tcp_unlock(tcp);
        close_fn(ctx);
        tcp_lock(tcp);
    }
    memset(&tcp->stream, 0, sizeof(tcp->stream));
    if (stream != 0) {
        tcp->stream = *stream;
    }
    tcp_drop_tx(tcp);
    tcp->state = A429_TCP_OFF;
    tcp->epoch++;
    tcp_unlock(tcp);
}

void a429_tcp_set_clock(a429_tcp_t *tcp, uint32_t (*now_us)(void *ctx), void *ctx)
{
    if (tcp == 0) {
        return;
    }
    tcp_lock(tcp);
    tcp->now_us_fn = now_us;
    tcp->now_us_ctx = ctx;
    tcp_unlock(tcp);
}

void a429_tcp_set_lock(a429_tcp_t *tcp, void (*lock)(void *ctx), void (*unlock)(void *ctx), void *ctx)
{
    if (tcp == 0) {
        return;
    }
    tcp->lock = lock;
    tcp->unlock = unlock;
    tcp->lock_ctx = ctx;
}

void a429_tcp_configure(a429_tcp_t *tcp, uint8_t enabled, uint32_t peer_ip, uint16_t peer_port)
{
    uint8_t changed;
    if (tcp == 0) {
        return;
    }
    tcp_lock(tcp);
    changed = (uint8_t)(tcp->enabled != (enabled ? 1u : 0u) || tcp->peer_ip != peer_ip || tcp->peer_port != peer_port);
    tcp->enabled = enabled ? 1u : 0u;
    tcp->peer_ip = peer_ip;
    tcp->peer_port = peer_port;
    if (changed) {
        /* The owner thread closes the socket. Closing here would run on
         * the vehicle main thread. */
        tcp_drop_tx(tcp);
        tcp_drop_rx(tcp);
        tcp->force_close = 1;
        tcp->backoff_us = A429_TCP_BACKOFF_MIN_US;
        tcp->epoch++;
    }
    tcp_unlock(tcp);
}

static void tcp_fail(a429_tcp_t *tcp, uint32_t now_us, int established)
{
    void (*close_fn)(void *) = tcp->stream.close;
    void *ctx = tcp->stream.ctx;
    tcp_drop_tx(tcp);
    tcp_drop_rx(tcp);
    tcp->state = A429_TCP_BACKOFF;
    if (established) {
        tcp->disconnects++;
        /* Only a completed submitted slot returns the wait to the minimum.
         * A slow handshake or an idle connection keeps the grown wait. */
        if (tcp->session_stable) {
            tcp->backoff_us = A429_TCP_BACKOFF_MIN_US;
        }
    } else {
        tcp->connect_fail++;
    }
    tcp->session_stable = 0;
    tcp->next_attempt_us = now_us + tcp->backoff_us;
    if (tcp->backoff_us > (A429_TCP_BACKOFF_MAX_US / 2u)) {
        tcp->backoff_us = A429_TCP_BACKOFF_MAX_US;
    } else {
        tcp->backoff_us *= 2u;
    }
    tcp->epoch++;
    tcp_unlock(tcp);
    if (close_fn != 0) {
        close_fn(ctx);
    }
    tcp_lock(tcp);
}

void a429_tcp_poll(a429_tcp_t *tcp, uint32_t now_us)
{
    uint8_t state;
    uint8_t allowed;
    uint32_t epoch;
    uint32_t ip;
    uint16_t port;
    uint8_t tx_copy[A429_TCP_TX_CAP];
    uint8_t tx_len;
    uint8_t tx_off;
    uint32_t tx_queued_us;
    uint32_t (*now_fn)(void *);
    void *now_ctx;
    uint8_t rx_space;
    int ns = 0;
    int nr = 0;
    uint8_t rbuf[32];
    uint8_t want;
    uint8_t held = 1;

    if (tcp == 0 || tcp->stream.open == 0) {
        return;
    }
    tcp_lock(tcp);
    allowed = (uint8_t)peer_ok(tcp);
    state = tcp->state;
    epoch = tcp->epoch;
    ip = tcp->peer_ip;
    port = tcp->peer_port;
    tx_len = tcp->tx_len;
    tx_off = tcp->tx_off;
    tx_queued_us = tcp->tx_queued_us;
    now_fn = tcp->now_us_fn;
    now_ctx = tcp->now_us_ctx;
    if (tx_off < tx_len) {
        memcpy(tx_copy, tcp->tx, tx_len);
    }
    rx_space = (uint8_t)(A429_TCP_RX_CAP - tcp->rx_count);
    if (tcp->force_close || !allowed) {
        void (*close_fn)(void *) = tcp->stream.close;
        void *ctx = tcp->stream.ctx;
        uint8_t was_up = (uint8_t)(state != A429_TCP_OFF || tcp->force_close);
        tcp_drop_tx(tcp);
        tcp_drop_rx(tcp);
        tcp->state = A429_TCP_OFF;
        tcp->force_close = 0;
        tcp->epoch++;
        state = A429_TCP_OFF;
        epoch = tcp->epoch;
        tcp_unlock(tcp);
        held = 0;
        if (was_up && close_fn != 0) {
            close_fn(ctx);
        }
        if (!allowed) {
            return;
        }
        tcp_lock(tcp);
        if (!peer_ok(tcp) || tcp->force_close) {
            tcp_unlock(tcp);
            return;
        }
        tcp_unlock(tcp);
    }
    if (state == A429_TCP_BACKOFF && (int32_t)(now_us - tcp->next_attempt_us) < 0) {
        if (held) {
            tcp_unlock(tcp);
        }
        return;
    }
    if (held) {
        tcp_unlock(tcp);
    }

    if (state == A429_TCP_UP) {
        if (tx_off < tx_len &&
            (int32_t)(now_us - tx_queued_us) >= (int32_t)A429_TCP_TX_MAX_AGE_US) {
            tcp_lock(tcp);
            if (tcp->epoch == epoch && tcp->state == A429_TCP_UP && tcp->tx_off < tcp->tx_len) {
                tcp->tx_expired++;
                if (tcp->tx_off > 0) {
                    /* Prefix already accepted by send(). Reset instead of
                     * appending a newer frame onto that prefix. */
                    tcp_fail(tcp, now_us, 1);
                    tcp_unlock(tcp);
                    return;
                }
                tcp_drop_tx(tcp);
            }
            tcp_unlock(tcp);
            tx_len = 0;
            tx_off = 0;
        }
        if (tx_off < tx_len && tcp->stream.send != 0) {
            ns = tcp->stream.send(tcp->stream.ctx, tx_copy + tx_off, (uint16_t)(tx_len - tx_off));
        }
        want = rx_space;
        if (want > sizeof(rbuf)) {
            want = (uint8_t)sizeof(rbuf);
        }
        if (want > 0 && tcp->stream.recv != 0) {
            nr = tcp->stream.recv(tcp->stream.ctx, rbuf, want);
        } else if (rx_space == 0) {
            tcp_lock(tcp);
            if (tcp->epoch == epoch) {
                tcp->rx_backpressure++;
            }
            tcp_unlock(tcp);
        }
        tcp_lock(tcp);
        if (tcp->epoch != epoch || tcp->state != A429_TCP_UP) {
            tcp_unlock(tcp);
            return;
        }
        if (ns < 0) {
            tcp_fail(tcp, now_us, 1);
            tcp_unlock(tcp);
            return;
        }
        if (ns > 0) {
            uint8_t step = (uint8_t)ns;
            if ((uint16_t)tx_off + step > tx_len) {
                step = (uint8_t)(tx_len - tx_off);
            }
            tcp->tx_off = (uint8_t)(tcp->tx_off + step);
            tcp->tx_bytes += step;
            if (step < (uint8_t)(tx_len - tx_off)) {
                tcp->partial_sends++;
            }
            if (tcp->tx_off >= tcp->tx_len) {
                tcp_drop_tx(tcp);
                tcp->session_stable = 1;
            }
        }
        if (nr < 0) {
            tcp_fail(tcp, now_us, 1);
            tcp_unlock(tcp);
            return;
        }
        if (nr > 0) {
            uint8_t take = (uint8_t)nr;
            uint8_t room = (uint8_t)(A429_TCP_RX_CAP - tcp->rx_count);
            if (take > room) {
                take = room;
            }
            if (take > 0) {
                memcpy(tcp->rx + tcp->rx_count, rbuf, take);
                tcp->rx_count = (uint8_t)(tcp->rx_count + take);
                tcp->rx_bytes += take;
            }
            if ((uint8_t)nr > take) {
                tcp->rx_backpressure++;
            }
        }
        tcp_unlock(tcp);
        return;
    }

    {
        int opened;
        uint32_t opened_us = now_us;
        opened = tcp->stream.open(tcp->stream.ctx, ip, port);
        if (now_fn != 0) {
            /* After open returns, outside the state lock. A blocking
             * connect must not be timed from the poll's entry stamp. */
            opened_us = now_fn(now_ctx);
        }
        tcp_lock(tcp);
        if (tcp->epoch != epoch || !peer_ok(tcp)) {
            tcp_unlock(tcp);
            return;
        }
        if (opened == 0) {
            tcp_drop_tx(tcp);
            tcp_drop_rx(tcp);
            tcp->state = A429_TCP_UP;
            tcp->connect_ok++;
            tcp->session_stable = 0;
            tcp->up_since_us = opened_us;
            tcp->epoch++;
        } else if (opened > 0) {
            tcp->state = A429_TCP_CONNECTING;
        } else {
            tcp_fail(tcp, now_us, 0);
        }
        tcp_unlock(tcp);
    }
}

int a429_tcp_submit(a429_tcp_t *tcp, const uint8_t *bytes, uint8_t n, uint32_t now_us)
{
    if (tcp == 0 || bytes == 0 || n == 0 || n > A429_TCP_TX_CAP) {
        return 0;
    }
    tcp_lock(tcp);
    if (tcp->state != A429_TCP_UP || tcp->tx_off < tcp->tx_len) {
        tcp->tx_dropped++;
        tcp_unlock(tcp);
        return 0;
    }
    memcpy(tcp->tx, bytes, n);
    tcp->tx_len = n;
    tcp->tx_off = 0;
    tcp->tx_queued_us = now_us;
    tcp_unlock(tcp);
    return 1;
}

uint8_t a429_tcp_drain_epoch(a429_tcp_t *tcp, uint8_t *out, uint8_t max_n, uint32_t *epoch)
{
    uint8_t n = 0;
    if (tcp == 0) {
        return 0;
    }
    tcp_lock(tcp);
    if (epoch != 0) {
        *epoch = tcp->epoch;
    }
    if (out != 0 && max_n > 0) {
        n = tcp->rx_count;
        if (n > max_n) {
            n = max_n;
        }
        if (n > 0) {
            memcpy(out, tcp->rx, n);
            memmove(tcp->rx, tcp->rx + n, (size_t)(tcp->rx_count - n));
            tcp->rx_count = (uint8_t)(tcp->rx_count - n);
        }
    }
    tcp_unlock(tcp);
    return n;
}

uint8_t a429_tcp_drain(a429_tcp_t *tcp, uint8_t *out, uint8_t max_n)
{
    return a429_tcp_drain_epoch(tcp, out, max_n, 0);
}
