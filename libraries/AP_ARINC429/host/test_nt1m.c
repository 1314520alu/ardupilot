/*
 * NT1-M TCP client session against a scripted pipe and a local mock server.
 * The mock is not an NT1-M and does not prove wire timing.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "../arinc429.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#if !defined(_WIN32)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

static int nt_fails;

static void nt_expect(int cond, const char *msg)
{
    if (!cond) {
        printf("FAIL: %s\n", msg);
        nt_fails++;
    }
}

typedef struct {
    int connected;
    int fail_open;
    int progress_left;
    int send_max;
    int fail_send;
    int block_send;
    uint32_t open_delay_us;
    uint8_t sent[256];
    int sent_n;
    uint8_t inbox[256];
    int inbox_n;
    int inbox_r;
    int recv_max;
    int recv_calls;
    int open_calls;
} script_io_t;

static script_io_t g_io;
static uint32_t g_clock;

static uint32_t test_clock(void *ctx)
{
    (void)ctx;
    return g_clock;
}

static int script_open(void *ctx, uint32_t ip, uint16_t port)
{
    script_io_t *io = (script_io_t *)ctx;
    (void)ip;
    (void)port;
    io->open_calls++;
    if (io->fail_open) {
        return -1;
    }
    if (io->open_delay_us != 0u) {
        g_clock += io->open_delay_us;
    }
    if (io->progress_left > 0) {
        io->progress_left--;
        return 1;
    }
    io->connected = 1;
    return 0;
}

static int script_send(void *ctx, const uint8_t *bytes, uint16_t n)
{
    script_io_t *io = (script_io_t *)ctx;
    uint16_t i;
    uint16_t allow;
    if (!io->connected) {
        return -1;
    }
    if (io->block_send) {
        return 0;
    }
    if (io->fail_send) {
        /* Leave the script connected so this poll can still read. The
         * client close, not this return, ends the session. */
        return -1;
    }
    if (n == 0) {
        return 0;
    }
    allow = n;
    if (io->send_max > 0 && allow > (uint16_t)io->send_max) {
        allow = (uint16_t)io->send_max;
    }
    for (i = 0; i < allow && io->sent_n < (int)sizeof(io->sent); i++) {
        io->sent[io->sent_n++] = bytes[i];
    }
    return (int)allow;
}

static int script_recv(void *ctx, uint8_t *bytes, uint16_t n)
{
    script_io_t *io = (script_io_t *)ctx;
    int avail;
    int take;
    int i;
    io->recv_calls++;
    if (!io->connected) {
        return -1;
    }
    avail = io->inbox_n - io->inbox_r;
    if (avail <= 0) {
        return 0;
    }
    take = avail;
    if (take > (int)n) {
        take = (int)n;
    }
    if (io->recv_max > 0 && take > io->recv_max) {
        take = io->recv_max;
    }
    for (i = 0; i < take; i++) {
        bytes[i] = io->inbox[io->inbox_r++];
    }
    return take;
}

static void script_close(void *ctx)
{
    script_io_t *io = (script_io_t *)ctx;
    io->connected = 0;
}

static void script_reset(a429_tcp_t *tcp)
{
    a429_stream_t stream;
    memset(&g_io, 0, sizeof(g_io));
    g_io.send_max = 1;
    g_io.recv_max = 1;
    a429_tcp_init(tcp);
    memset(&stream, 0, sizeof(stream));
    stream.open = script_open;
    stream.send = script_send;
    stream.recv = script_recv;
    stream.close = script_close;
    stream.ctx = &g_io;
    a429_tcp_set_stream(tcp, &stream);
}

static void test_label_and_provisional_frame(void)
{
    uint32_t word = 0;
    uint8_t bytes[8];
    uint8_t n = 0;
    uint32_t out = 0;
    a429_frame_dec_t dec;
    uint8_t logical = 036;

    nt_expect(a429_label_bit_reverse(logical) == 0x78u, "octal 036 reverses to 0x78");
    nt_expect(a429_label_bit_reverse(0x64u) != 0x64u, "codec label byte is not its reverse");
    nt_expect(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &word) == 1, "Ng word");
    nt_expect((word & 0xFFu) == 0x64u, "codec stores label 0144 as 0x64");
    nt_expect(a429_frame_encode(A429_FRAME_NONE, &word, 1, bytes, sizeof(bytes), &n) == 0, "unconfigured framing encodes nothing");
    nt_expect(a429_frame_encode(A429_FRAME_UART429S_C0, &word, 1, bytes, sizeof(bytes), &n) == 1, "provisional C0 encodes");
    nt_expect(n == 5u && bytes[0] == 0xC0u && bytes[1] == 0x64u, "C0 payload keeps the logical label byte");
    nt_expect(bytes[1] != a429_label_bit_reverse(0x64u), "C0 payload is not the reversed label");

    a429_frame_dec_init(&dec, A429_FRAME_NONE);
    nt_expect(a429_frame_push(&dec, bytes, n, &out, 1) == 0, "unconfigured decoder emits no word");
    a429_frame_dec_init(&dec, A429_FRAME_UART429S_C0);
    nt_expect(a429_frame_push(&dec, bytes, 1, &out, 1) == 0, "split header waits");
    nt_expect(a429_frame_push(&dec, bytes + 1, 4, &out, 1) == 1 && out == word, "C0 round trip");
    {
        uint8_t junk[3] = {0xCDu, 0x11u, 0xC0u};
        uint8_t rest[4];
        rest[0] = (uint8_t)(word & 0xFFu);
        rest[1] = (uint8_t)((word >> 8) & 0xFFu);
        rest[2] = (uint8_t)((word >> 16) & 0xFFu);
        rest[3] = (uint8_t)((word >> 24) & 0xFFu);
        a429_frame_dec_init(&dec, A429_FRAME_UART429S_C0);
        nt_expect(a429_frame_push(&dec, junk, 3, &out, 1) == 0, "CD is not a word");
        nt_expect(dec.skipped == 2u, "non-C0 bytes are skipped");
        nt_expect(a429_frame_push(&dec, rest, 4, &out, 1) == 1 && out == word, "resync on the next C0");
    }
}

static void test_scripted_session(void)
{
    a429_tcp_t tcp;
    uint8_t frame[5] = {0xC0u, 0x64u, 0x01u, 0x02u, 0x03u};
    uint8_t other[5] = {0xC0u, 0x65u, 0x04u, 0x05u, 0x06u};
    uint8_t drained[64];
    int i;

    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0, 8887);
    a429_tcp_poll(&tcp, 0);
    nt_expect(g_io.open_calls == 0, "port without an address does not connect");
    a429_tcp_configure(&tcp, 1, 0xC0A80307u, 0);
    a429_tcp_poll(&tcp, 1000);
    nt_expect(g_io.open_calls == 0, "address without a port does not connect");

    a429_tcp_configure(&tcp, 1, 0xC0A80307u, 8887);
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 0) == 0, "submit before connect is dropped");
    nt_expect(tcp.tx_dropped == 1u, "pre-connect submit counted");
    g_io.progress_left = 1;
    a429_tcp_poll(&tcp, 2000);
    nt_expect(tcp.state == A429_TCP_CONNECTING && g_io.connected == 0, "connect can stay in progress");
    a429_tcp_poll(&tcp, 3000);
    nt_expect(tcp.state == A429_TCP_UP && tcp.connect_ok == 1u, "second open completes");
    nt_expect(g_io.sent_n == 0, "bytes queued before connect are not replayed");

    nt_expect(a429_tcp_submit(&tcp, frame, 5, 4000u) == 1, "submit one frame");
    a429_tcp_poll(&tcp, 4000);
    nt_expect(g_io.sent_n == 1 && g_io.sent[0] == 0xC0u, "partial write sends one byte");
    nt_expect(tcp.partial_sends == 1u, "partial write is counted");
    a429_tcp_submit(&tcp, other, 5, 4000u);
    nt_expect(tcp.tx_dropped == 2u, "a second frame is not queued behind the partial");
    for (i = 0; i < 4; i++) {
        a429_tcp_poll(&tcp, 5000u + (uint32_t)i);
    }
    nt_expect(g_io.sent_n == 5, "the current frame finishes without a burst");
    nt_expect(memcmp(g_io.sent, frame, 5) == 0, "finished bytes are the first frame only");

    g_io.fail_send = 1;
    nt_expect(a429_tcp_submit(&tcp, other, 5, 6000u) == 1, "submit after the pipe is idle");
    a429_tcp_poll(&tcp, 6000);
    nt_expect(tcp.state == A429_TCP_BACKOFF && tcp.disconnects == 1u, "send failure disconnects");
    nt_expect(tcp.tx_len == 0, "unsent tail is discarded");
    g_io.fail_send = 0;
    g_io.sent_n = 0;
    a429_tcp_poll(&tcp, 6000);
    nt_expect(tcp.state == A429_TCP_BACKOFF, "backoff holds the retry");
    a429_tcp_poll(&tcp, 6000u + A429_TCP_BACKOFF_MIN_US);
    nt_expect(tcp.state == A429_TCP_UP && tcp.connect_ok == 2u, "reconnects after backoff");
    nt_expect(g_io.sent_n == 0, "reconnect does not replay the discarded frame");
    nt_expect(a429_tcp_submit(&tcp, other, 5, 7000u + A429_TCP_BACKOFF_MIN_US) == 1, "a new slot can be sent");
    for (i = 0; i < 5; i++) {
        a429_tcp_poll(&tcp, 7000u + A429_TCP_BACKOFF_MIN_US + (uint32_t)i);
    }
    nt_expect(g_io.sent_n == 5 && memcmp(g_io.sent, other, 5) == 0, "only the post-reconnect frame is sent");

    memcpy(g_io.inbox, frame, 5);
    memcpy(g_io.inbox + 5, other, 5);
    g_io.inbox_n = 10;
    g_io.inbox_r = 0;
    g_io.recv_max = 1;
    {
        a429_frame_dec_t dec;
        uint32_t words[4];
        uint8_t produced = 0;
        uint8_t chunk[8];
        uint8_t got;
        a429_frame_dec_init(&dec, A429_FRAME_UART429S_C0);
        for (i = 0; i < 12 && produced < 2; i++) {
            a429_tcp_poll(&tcp, 8000u + (uint32_t)i);
            got = a429_tcp_drain(&tcp, chunk, sizeof(chunk));
            produced = (uint8_t)(produced + a429_frame_push(&dec, chunk, got, words + produced, (uint8_t)(4u - produced)));
        }
        nt_expect(produced == 2u, "fragmented reads become two words");
        nt_expect(words[0] == 0x03020164u && words[1] == 0x06050465u, "fragmented little-endian words");
    }

    g_io.recv_max = 16;
    memcpy(g_io.inbox, frame, 5);
    memcpy(g_io.inbox + 5, other, 5);
    g_io.inbox_n = 10;
    g_io.inbox_r = 0;
    a429_tcp_poll(&tcp, 9000);
    {
        uint8_t chunk[16];
        uint8_t got = a429_tcp_drain(&tcp, chunk, sizeof(chunk));
        a429_frame_dec_t dec;
        uint32_t words[2];
        a429_frame_dec_init(&dec, A429_FRAME_UART429S_C0);
        nt_expect(got == 10u, "coalesced read returns both frames");
        nt_expect(a429_frame_push(&dec, chunk, got, words, 2) == 2u, "coalesced bytes decode");
    }

    g_io.recv_max = 32;
    g_io.inbox_r = 0;
    g_io.inbox_n = 0;
    for (i = 0; i < 200; i++) {
        g_io.inbox[g_io.inbox_n++] = 0xA5u;
    }
    while (tcp.rx_count < A429_TCP_RX_CAP) {
        uint8_t before = tcp.rx_count;
        a429_tcp_poll(&tcp, 10000);
        if (tcp.rx_count == before) {
            break;
        }
    }
    {
        int calls = g_io.recv_calls;
        nt_expect(tcp.rx_count == A429_TCP_RX_CAP, "rx storage fills to the cap");
        a429_tcp_poll(&tcp, 10001);
        nt_expect(tcp.rx_count == A429_TCP_RX_CAP, "a full buffer does not grow");
        nt_expect(g_io.recv_calls == calls, "a full buffer does not read the socket");
        nt_expect(tcp.rx_backpressure > 0u, "backpressure is counted");
    }
    nt_expect(a429_tcp_drain(&tcp, drained, sizeof(drained)) > 0u, "drain releases buffer space");

    g_io.inbox_n = g_io.inbox_r;
    {
        uint32_t rx_before = tcp.rx_bytes;
        uint32_t slots;
        a429_sched_t sched;
        a429_sched_init(&sched);
        a429_sched_poll(&sched, 0);
        slots = 0;
        a429_tcp_poll(&tcp, 11000);
        nt_expect(a429_sched_poll(&sched, 62500u) == 1u, "late scheduler emits one slot");
        slots++;
        a429_tcp_poll(&tcp, 12000);
        nt_expect(a429_sched_poll(&sched, 62500u) == 0u, "the late slot does not repeat");
        nt_expect(tcp.rx_bytes == rx_before, "peer silence adds no bytes");
        nt_expect(slots == 1u, "no scheduling burst");
    }
}

#if !defined(_WIN32)
typedef struct {
    int fd;
} posix_io_t;

static posix_io_t g_pos;

static int posix_open(void *ctx, uint32_t ip, uint16_t port)
{
    posix_io_t *io = (posix_io_t *)ctx;
    struct sockaddr_in addr;
    int fd;
    (void)ip;
    if (io->fd >= 0) {
        close(io->fd);
        io->fd = -1;
    }
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    io->fd = fd;
    return 0;
}

static int posix_send(void *ctx, const uint8_t *bytes, uint16_t n)
{
    posix_io_t *io = (posix_io_t *)ctx;
    ssize_t w;
    if (io->fd < 0) {
        return -1;
    }
    w = send(io->fd, bytes, n, MSG_NOSIGNAL);
    if (w < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }
    if (w == 0) {
        return -1;
    }
    return (int)w;
}

static int posix_recv(void *ctx, uint8_t *bytes, uint16_t n)
{
    posix_io_t *io = (posix_io_t *)ctx;
    ssize_t r;
    if (io->fd < 0) {
        return -1;
    }
    r = recv(io->fd, bytes, n, MSG_DONTWAIT);
    if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }
    if (r == 0) {
        return -1;
    }
    return (int)r;
}

static void posix_close(void *ctx)
{
    posix_io_t *io = (posix_io_t *)ctx;
    if (io->fd >= 0) {
        close(io->fd);
        io->fd = -1;
    }
}

static int listen_local(uint16_t *port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);
    int one = 1;
    if (fd < 0) {
        return -1;
    }
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(fd, 1) != 0 ||
        getsockname(fd, (struct sockaddr *)&addr, &len) != 0) {
        close(fd);
        return -1;
    }
    *port = ntohs(addr.sin_port);
    return fd;
}

static int accept_once(int srv)
{
    int i;
    int flags = fcntl(srv, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(srv, F_SETFL, flags | O_NONBLOCK);
    }
    for (i = 0; i < 50; i++) {
        int fd = accept(srv, 0, 0);
        if (fd >= 0) {
            return fd;
        }
        usleep(10000);
    }
    return -1;
}

static void test_connection_framing(void)
{
    a429_core_t core;
    uint32_t ng50 = 0;
    uint8_t frame[5];
    uint8_t n = 0;
    uint8_t i;

    nt_expect(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &ng50) == 1, "Ng50 word");
    nt_expect(a429_frame_encode(A429_FRAME_UART429S_C0, &ng50, 1, frame, 5, &n) == 1 && n == 5u,
              "Ng50 provisional frame");

    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0xC0A80307u, 8887);
    a429_core_update(&core, 0);
    nt_expect(core.tcp.state == A429_TCP_UP, "framing session is up");
    g_io.recv_max = 8;
    g_io.inbox[0] = 0xC0u;
    g_io.inbox[1] = 0x64u;
    g_io.inbox[2] = 0x20u;
    g_io.inbox_n = 3;
    g_io.inbox_r = 0;
    a429_core_update(&core, 1000u);
    nt_expect(core.link.counters.rx_count == 0u, "a partial C0 frame is not a word");
    g_io.connected = 0;
    a429_core_update(&core, 2000u);
    nt_expect(core.tcp.state == A429_TCP_BACKOFF, "partial frame is followed by disconnect");
    a429_core_update(&core, 2000u + A429_TCP_BACKOFF_MIN_US);
    nt_expect(core.tcp.state == A429_TCP_UP, "session reconnects");
    for (i = 0; i < 5u; i++) {
        g_io.inbox[i] = frame[i];
    }
    g_io.inbox_n = 5;
    g_io.inbox_r = 0;
    a429_core_update(&core, 3000u + A429_TCP_BACKOFF_MIN_US);
    nt_expect(core.link.rx[0].fresh == 1, "Ng50 after reconnect is fresh");
    nt_expect(core.link.rx[0].view.eu > 49.9f && core.link.rx[0].view.eu < 50.1f,
              "reconnect does not assemble Ng97.29 from the previous connection");
    nt_expect(core.link.rx[0].view.raw == ng50, "reconnect delivers the new word only");

    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0xC0A80307u, 8887);
    a429_core_update(&core, 0);
    g_io.recv_max = 2;
    g_io.inbox[0] = frame[0];
    g_io.inbox[1] = frame[1];
    g_io.inbox_n = 2;
    g_io.inbox_r = 0;
    a429_core_update(&core, 1000u);
    nt_expect(core.link.counters.rx_count == 0u && core.frame_dec.primed == 1u,
              "same connection keeps a split header");
    g_io.recv_max = 3;
    g_io.inbox[0] = frame[2];
    g_io.inbox[1] = frame[3];
    g_io.inbox[2] = frame[4];
    g_io.inbox_n = 3;
    g_io.inbox_r = 0;
    a429_core_update(&core, 2000u);
    nt_expect(core.link.rx[0].fresh == 1 && core.link.rx[0].view.raw == ng50,
              "same connection accepts a split frame");

    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0xC0A80307u, 8887);
    a429_core_update(&core, 0);
    g_io.recv_max = 16;
    for (i = 0; i < 5u; i++) {
        g_io.inbox[i] = frame[i];
        g_io.inbox[5u + i] = frame[i];
    }
    g_io.inbox_n = 10;
    g_io.inbox_r = 0;
    a429_core_update(&core, 1000u);
    nt_expect(core.link.counters.rx_count == 2u, "same connection accepts two coalesced frames");
    nt_expect(core.link.rx[0].view.raw == ng50, "coalesced frames stay Ng50");
}

static void test_zero_octet_peer(void)
{
    a429_tcp_t tcp;
    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0x0A000001u, 8887);
    a429_tcp_poll(&tcp, 0);
    nt_expect(g_io.open_calls == 1 && tcp.state == A429_TCP_UP, "10.0.0.1 is a configured peer");
    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0xC0A80007u, 8887);
    a429_tcp_poll(&tcp, 0);
    nt_expect(g_io.open_calls == 1 && tcp.state == A429_TCP_UP, "192.168.0.7 keeps its zero octet");
    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0, 8887);
    a429_tcp_poll(&tcp, 0);
    nt_expect(g_io.open_calls == 0, "0.0.0.0 is an unconfigured destination");
}

static void test_tx_age_and_write_fail(void)
{
    a429_tcp_t tcp;
    a429_core_t core;
    uint8_t frame[5] = {0xC0u, 0x64u, 0x00u, 0x71u, 0xE2u};
    uint8_t newer[5] = {0xC0u, 0x65u, 0x01u, 0x02u, 0x03u};
    uint32_t ng50 = 0;
    uint8_t ngframe[5];
    uint8_t n = 0;
    int calls_before;

    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0x0A000001u, 8887);
    a429_tcp_poll(&tcp, 0);
    g_io.block_send = 1;
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 1000u) == 1, "queue a frame that cannot be written");
    a429_tcp_poll(&tcp, 1000u + A429_TCP_TX_MAX_AGE_US - 1u);
    nt_expect(tcp.tx_len == 5u && g_io.sent_n == 0, "a young would-block keeps the current frame");
    a429_tcp_submit(&tcp, newer, 5, 1000u + A429_TCP_TX_MAX_AGE_US - 1u);
    nt_expect(tcp.tx_dropped >= 1u, "a newer snapshot is not queued behind it");
    a429_tcp_poll(&tcp, 1000u + A429_TCP_TX_MAX_AGE_US);
    g_io.block_send = 0;
    a429_tcp_poll(&tcp, 1000u + A429_TCP_TX_MAX_AGE_US + 1u);
    nt_expect(g_io.sent_n == 0 && tcp.tx_expired >= 1u, "a 50 ms would-block drops the unsent frame");
    nt_expect(tcp.state == A429_TCP_UP, "no prefix was sent, so the connection stays up");

    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0x0A000001u, 8887);
    a429_tcp_poll(&tcp, 0);
    g_io.send_max = 2;
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 1000u) == 1, "queue a frame that sends a prefix");
    a429_tcp_poll(&tcp, 1000u);
    nt_expect(g_io.sent_n == 2, "two bytes were accepted by send");
    g_io.block_send = 1;
    a429_tcp_poll(&tcp, 1000u + A429_TCP_TX_MAX_AGE_US);
    nt_expect(tcp.state == A429_TCP_BACKOFF, "a stale partial frame resets the connection");
    nt_expect(g_io.sent_n == 2, "the suffix is not written after the age limit");
    /* Bytes already returned by send() can remain in the OS or NT1-M cache.
     * This assertion only covers bytes our session has not handed to send(). */
    g_io.block_send = 0;
    g_io.sent_n = 0;
    a429_tcp_poll(&tcp, 1000u + A429_TCP_TX_MAX_AGE_US + A429_TCP_BACKOFF_MIN_US);
    nt_expect(tcp.state == A429_TCP_UP, "reconnects after the partial-frame reset");
    nt_expect(a429_tcp_submit(&tcp, newer, 5, 1000u + A429_TCP_TX_MAX_AGE_US + A429_TCP_BACKOFF_MIN_US) == 1,
              "the post-reset snapshot is a whole frame");
    g_io.send_max = 5;
    a429_tcp_poll(&tcp, 1000u + A429_TCP_TX_MAX_AGE_US + A429_TCP_BACKOFF_MIN_US);
    nt_expect(g_io.sent_n == 5 && memcmp(g_io.sent, newer, 5) == 0,
              "the new frame is not concatenated onto the old suffix");

    nt_expect(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &ng50) == 1, "Ng50 for write-fail");
    nt_expect(a429_frame_encode(A429_FRAME_UART429S_C0, &ng50, 1, ngframe, 5, &n) == 1, "Ng50 frame");
    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 8887);
    a429_core_update(&core, 0);
    g_io.recv_max = 2;
    g_io.inbox[0] = 0xC0u;
    g_io.inbox[1] = 0x64u;
    g_io.inbox_n = 2;
    g_io.inbox_r = 0;
    a429_core_update(&core, 1000u);
    nt_expect(core.frame_dec.primed == 1u, "parser holds C0 64");
    g_io.inbox[0] = 0x20u;
    g_io.inbox_n = 1;
    g_io.inbox_r = 0;
    g_io.recv_max = 1;
    g_io.fail_send = 1;
    nt_expect(a429_tcp_submit(&core.tcp, frame, 5, 2000u) == 1, "a write is in flight");
    calls_before = g_io.recv_calls;
    a429_core_update(&core, 2000u);
    nt_expect(g_io.recv_calls > calls_before && g_io.inbox_r == 1,
              "the same poll consumed a received byte and the write then failed");
    nt_expect(core.tcp.disconnects >= 1u, "the write failure drops the connection");
    nt_expect(core.frame_dec.primed == 0u && core.tcp.rx_count == 0u,
              "write failure clears staged RX and the decoder");
    g_io.fail_send = 0;
    a429_core_update(&core, 2000u + A429_TCP_BACKOFF_MIN_US);
    g_io.recv_max = 5;
    memcpy(g_io.inbox, ngframe, 5);
    g_io.inbox_n = 5;
    g_io.inbox_r = 0;
    a429_core_update(&core, 3000u + A429_TCP_BACKOFF_MIN_US);
    nt_expect(core.link.rx[0].fresh == 1 && core.link.rx[0].view.raw == ng50,
              "bytes read beside the failed write do not continue into the next session");
}

static uint32_t disc_word(uint8_t label, uint16_t data, uint8_t sm)
{
    uint32_t w = label;
    w |= (uint32_t)data << 13;
    w |= ((uint32_t)(sm & 3u)) << 29;
    return a429_apply_odd_parity(w);
}

static void feed_words(a429_core_t *core, const uint32_t *words, uint8_t nwords, uint32_t *now)
{
    uint8_t buf[64];
    uint8_t n = 0;
    int spins = 0;
    nt_expect(a429_frame_encode(A429_FRAME_UART429S_C0, words, nwords, buf, (uint8_t)sizeof(buf), &n) == 1,
              "phase-one frames fit one read buffer pair");
    memcpy(g_io.inbox, buf, n);
    g_io.inbox_n = n;
    g_io.inbox_r = 0;
    g_io.recv_max = 32;
    while (g_io.inbox_r < g_io.inbox_n && spins < 4) {
        *now += 1000u;
        a429_core_update(core, *now);
        spins++;
    }
    nt_expect(g_io.inbox_r == g_io.inbox_n, "the script delivered every frame byte");
}

static void test_disconnect_clears_fresh(void)
{
    a429_core_t core;
    uint32_t words[A429_RX_N];
    uint32_t saved_us[A429_RX_N];
    uint32_t saved_raw[A429_RX_N];
    float saved_eu[4];
    uint32_t rx_count;
    uint32_t valid_count;
    uint32_t timeout_count;
    uint32_t now;
    uint32_t bad = 0;
    uint32_t again = 0;
    float eu = -1.0f;
    int16_t data = 0;
    uint8_t i;
    const float expect_eu[4] = {50.0f, 10.0f, 0.0f, 10.0f};

    nt_expect(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &words[0]) == 1, "Ng50");
    nt_expect(a429_encode_bnr(0145, 0, A429_SM_NORMAL, 10.0f, &words[1]) == 1, "Np10");
    nt_expect(a429_encode_bnr(0147, 0, A429_SM_NORMAL, 0.0f, &words[2]) == 1, "T45 0");
    nt_expect(a429_encode_bnr(0151, 0, A429_SM_NORMAL, 10.0f, &words[3]) == 1, "torque 10");
    words[4] = disc_word(0175, 0x1111u, A429_SM_NORMAL);
    words[5] = disc_word(0176, 0x2222u, A429_SM_NORMAL);
    words[6] = disc_word(0177, 0x3333u, A429_SM_NORMAL);

    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 8887);
    now = 0;
    a429_core_update(&core, now);
    nt_expect(core.tcp.state == A429_TCP_UP, "freshness session is up");
    nt_expect(core.link.rx[0].timeout_us == 0u, "Ng timeout stays at the default 0");
    feed_words(&core, words, A429_RX_N, &now);
    for (i = 0; i < A429_RX_N; i++) {
        saved_us[i] = core.link.rx[i].valid_us;
        saved_raw[i] = core.link.rx[i].last_valid.raw;
        nt_expect(core.link.rx[i].fresh == 1u, "every phase-one label starts fresh");
        nt_expect(core.link.rx[i].timeout_count == 0u, "timeout 0 did not fire");
    }
    nt_expect(a429_link_fresh_eu(&core.link, 0, &eu) == 1 && eu > 49.9f && eu < 50.1f, "Ng getter is live");
    nt_expect(a429_link_fresh_eu(&core.link, 1, &eu) == 1 && eu > 9.9f && eu < 10.1f, "Np getter is live");
    nt_expect(a429_link_fresh_eu(&core.link, 2, &eu) == 1 && eu > -0.1f && eu < 0.1f, "T45 getter is live");
    nt_expect(a429_link_fresh_eu(&core.link, 3, &eu) == 1 && eu > 9.9f && eu < 10.1f, "torque getter is live");
    nt_expect(a429_link_fresh_eu(&core.link, 4, &eu) == 0, "a discrete label has no EU getter");
    nt_expect(a429_link_fresh_disc(&core.link, 4, &data) == 1 && data == (int16_t)0x1111, "warn getter");
    nt_expect(a429_link_fresh_disc(&core.link, 5, &data) == 1 && data == (int16_t)0x2222, "state getter");
    nt_expect(a429_link_fresh_disc(&core.link, 6, &data) == 1 && data == (int16_t)0x3333, "warn2 getter");
    for (i = 0; i < 4u; i++) {
        saved_eu[i] = core.link.rx[i].last_valid_eu;
        nt_expect(saved_eu[i] > expect_eu[i] - 0.1f && saved_eu[i] < expect_eu[i] + 0.1f,
                  "historical EU matches the live sample");
    }
    rx_count = core.link.counters.rx_count;
    valid_count = core.link.counters.valid_count;
    timeout_count = 0;

    g_io.connected = 0;
    g_io.inbox_n = 0;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    for (i = 0; i < A429_RX_N; i++) {
        nt_expect(core.link.rx[i].fresh == 0u, "disconnect clears freshness at timeout 0");
        nt_expect(core.link.rx[i].valid_us == saved_us[i], "disconnect keeps valid_us");
        nt_expect(core.link.rx[i].last_valid.raw == saved_raw[i], "disconnect keeps last_valid");
        nt_expect(core.link.rx[i].timeout_count == 0u, "disconnect is not a timeout");
    }
    nt_expect(a429_link_fresh_eu(&core.link, 0, &eu) == 0, "Ng getter is false after disconnect");
    nt_expect(a429_link_fresh_disc(&core.link, 4, &data) == 0, "warn getter is false after disconnect");
    nt_expect(core.link.counters.rx_count == rx_count && core.link.counters.valid_count == valid_count,
              "disconnect does not invent counter events");
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.tcp.disconnects == 1u, "a second poll during backoff is not another disconnect");
    nt_expect(core.link.rx[0].fresh == 0u && core.link.rx[0].timeout_count == timeout_count,
              "repeated session loss is idempotent");

    now = core.tcp.next_attempt_us;
    g_io.inbox_n = 0;
    g_io.inbox_r = 0;
    a429_core_update(&core, now);
    nt_expect(core.tcp.state == A429_TCP_UP, "reconnects with an empty stream");
    for (i = 0; i < A429_RX_N; i++) {
        nt_expect(core.link.rx[i].fresh == 0u, "reconnect does not restore a stale sample");
        nt_expect(core.link.rx[i].valid_us == saved_us[i] && core.link.rx[i].last_valid.raw == saved_raw[i],
                  "history survives the empty reconnect");
    }
    nt_expect(a429_link_fresh_eu(&core.link, 0, &eu) == 0, "Ng getter stays false with no new sample");

    nt_expect(a429_encode_bnr(0144, 0, A429_SM_FAIL, 50.0f, &bad) == 1, "invalid Ng");
    feed_words(&core, &bad, 1, &now);
    nt_expect(core.link.rx[0].fresh == 0u && core.link.rx[0].valid_us == saved_us[0],
              "an invalid first sample stays invalid");
    nt_expect(core.link.rx[0].last_valid.raw == saved_raw[0], "the invalid word does not replace history");
    nt_expect(a429_link_fresh_eu(&core.link, 0, &eu) == 0, "Ng getter rejects the invalid word");

    nt_expect(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 60.0f, &again) == 1, "Ng60");
    feed_words(&core, &again, 1, &now);
    nt_expect(a429_link_fresh_eu(&core.link, 0, &eu) == 1 && eu > 59.9f && eu < 60.1f,
              "a new valid sample is fresh");
    nt_expect(core.link.rx[0].valid_us == now && core.link.rx[0].last_valid.raw == again,
              "the new sample updates history");
    nt_expect(core.link.rx[1].fresh == 0u && core.link.rx[1].last_valid.raw == saved_raw[1],
              "other labels stay historical");

    saved_us[0] = core.link.rx[0].valid_us;
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 9000);
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.link.rx[0].fresh == 0u && core.link.rx[0].valid_us == saved_us[0],
              "a peer change clears freshness and keeps history");
    a429_tcp_configure(&core.tcp, 0, 0, 0);
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.link.rx[0].fresh == 0u && core.link.rx[0].last_valid.raw == again,
              "disabling the client leaves history and stays unfresh");
    a429_link_session_lost(&core.link);
    nt_expect(core.link.rx[0].fresh == 0u && core.link.rx[0].valid_us == saved_us[0] &&
              core.link.rx[0].timeout_count == 0u, "a repeated loss does not move history or counters");
}

static void grow_to_800ms(a429_tcp_t *tcp)
{
    a429_tcp_configure(tcp, 1, 0x0A000001u, 8887);
    g_io.fail_open = 1;
    a429_tcp_poll(tcp, 0);
    a429_tcp_poll(tcp, A429_TCP_BACKOFF_MIN_US);
    g_io.fail_open = 0;
    nt_expect(tcp->backoff_us == 800000u, "two refusals grow the wait to 800 ms");
    nt_expect(tcp->next_attempt_us == 600000u, "the next attempt is at 600 ms");
}

static void test_backoff_needs_a_completed_frame(void)
{
    a429_tcp_t tcp;
    uint8_t frame[5] = {0xC0u, 0x01u, 0x02u, 0x03u, 0x04u};
    uint32_t t;
    int i;

    script_reset(&tcp);
    grow_to_800ms(&tcp);
    g_clock = 600000u;
    g_io.open_delay_us = 200000u;
    a429_tcp_set_clock(&tcp, test_clock, 0);
    a429_tcp_poll(&tcp, 600000u);
    nt_expect(tcp.state == A429_TCP_UP, "slow open still connects");
    nt_expect(g_clock == 800000u && tcp.up_since_us == g_clock,
              "up_since is the clock after open returns, not the poll entry");
    nt_expect(tcp.backoff_us == 800000u, "the slow open does not shrink backoff");
    g_io.block_send = 1;
    a429_tcp_poll(&tcp, g_clock);
    nt_expect(tcp.session_stable == 0u && tcp.state == A429_TCP_UP,
              "the first poll after a slow open is not a stable session");
    g_io.block_send = 0;
    g_io.fail_send = 1;
    nt_expect(a429_tcp_submit(&tcp, frame, 5, g_clock) == 1, "queue a slot that will fail");
    t = g_clock;
    a429_tcp_poll(&tcp, t);
    nt_expect(tcp.state == A429_TCP_BACKOFF, "the slow session drops");
    nt_expect(tcp.next_attempt_us == t + 800000u, "slow open/drop keeps the 800 ms wait");

    script_reset(&tcp);
    grow_to_800ms(&tcp);
    a429_tcp_poll(&tcp, 600000u);
    nt_expect(tcp.state == A429_TCP_UP, "zero-progress session connects");
    g_io.block_send = 1;
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 600000u) == 1, "queue a slot that would-blocks");
    a429_tcp_poll(&tcp, 600000u + 1000u);
    nt_expect(tcp.session_stable == 0u && tcp.state == A429_TCP_UP && g_io.sent_n == 0,
              "a would-block accepts no progress");
    a429_tcp_poll(&tcp, 600000u + 200000u);
    nt_expect(tcp.session_stable == 0u && tcp.state == A429_TCP_UP,
              "200 ms online with no completed slot is not a stable session");
    g_io.connected = 0;
    a429_tcp_poll(&tcp, 800000u);
    nt_expect(tcp.next_attempt_us == 800000u + 800000u, "zero-TX drop keeps the 800 ms wait");

    script_reset(&tcp);
    grow_to_800ms(&tcp);
    a429_tcp_poll(&tcp, 600000u);
    g_io.send_max = 5;
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 600000u) == 1, "queue a whole frame");
    a429_tcp_poll(&tcp, 600000u);
    nt_expect(tcp.session_stable == 1u && g_io.sent_n == 5, "a completed frame is the stable session");
    g_io.fail_send = 1;
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 600001u) == 1, "queue the slot that will fail");
    a429_tcp_poll(&tcp, 600001u);
    nt_expect(tcp.next_attempt_us == 600001u + A429_TCP_BACKOFF_MIN_US,
              "meaningful progress returns the next wait to 200 ms");
    g_io.fail_send = 0;
    a429_tcp_poll(&tcp, 600001u + A429_TCP_BACKOFF_MIN_US);
    nt_expect(tcp.state == A429_TCP_UP, "the minimum wait reconnects after a completed frame");

    script_reset(&tcp);
    a429_tcp_configure(&tcp, 1, 0x0A000001u, 8887);
    g_io.fail_open = 1;
    t = 0;
    for (i = 0; i < 8; i++) {
        a429_tcp_poll(&tcp, t);
        t = tcp.next_attempt_us;
    }
    nt_expect(tcp.backoff_us == A429_TCP_BACKOFF_MAX_US, "backoff stops at 5 s");
}

static void test_unstable_backoff(void)
{
    a429_tcp_t tcp;
    uint32_t connected_at;
    uint32_t dropped_at;

    script_reset(&tcp);
    g_io.fail_open = 1;
    a429_tcp_configure(&tcp, 1, 0x0A000001u, 8887);
    a429_tcp_poll(&tcp, 0);
    nt_expect(tcp.connect_fail == 1u && tcp.next_attempt_us == A429_TCP_BACKOFF_MIN_US,
              "the first refusal waits the minimum");
    g_io.fail_open = 0;
    a429_tcp_poll(&tcp, A429_TCP_BACKOFF_MIN_US);
    connected_at = A429_TCP_BACKOFF_MIN_US;
    nt_expect(tcp.state == A429_TCP_UP && tcp.backoff_us != A429_TCP_BACKOFF_MIN_US,
              "a successful connect does not shrink the grown backoff");
    g_io.fail_send = 1;
    nt_expect(a429_tcp_submit(&tcp, (const uint8_t *)"\xC0\x01\x02\x03\x04", 5, connected_at + 1u) == 1,
              "unstable session still accepts one attempt");
    a429_tcp_poll(&tcp, connected_at + 1u);
    dropped_at = connected_at + 1u;
    nt_expect(tcp.state == A429_TCP_BACKOFF, "immediate drop disconnects");
    nt_expect(tcp.next_attempt_us == dropped_at + (2u * A429_TCP_BACKOFF_MIN_US),
              "an unstable drop waits the grown backoff, not 200 ms again");
    g_io.fail_send = 0;
    a429_tcp_poll(&tcp, dropped_at + A429_TCP_BACKOFF_MIN_US);
    nt_expect(tcp.state == A429_TCP_BACKOFF, "200 ms is too soon after an unstable drop");
}

static int buffer_has_cd(const uint8_t *bytes, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        if (bytes[i] == 0xCDu) {
            return 1;
        }
    }
    return 0;
}

static void test_rs422_session(void)
{
    a429_core_t core;
    a429_pcmd_levels_t levels;
    uint32_t ng = 0;
    uint32_t nr = 0;
    uint32_t pcmd = 0;
    uint32_t sdi1 = 0;
    uint8_t wire[5];
    uint8_t n = 0;
    uint8_t logical_b;
    uint8_t wire_b;
    uint32_t now;
    float eu = -1.0f;
    int i;

    nt_expect(a429_encode_bnr(0144, 0, A429_SM_NORMAL, 50.0f, &ng) == 1, "Ng50 for RS422 session");
    nt_expect(a429_frame_encode(A429_FRAME_RS422_429_1C, &ng, 1, wire, 5, &n) == 1, "wire Ng frame");
    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_RS422_429_1C);
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 8887);
    now = 0;
    a429_core_update(&core, now);
    g_io.recv_max = 2;
    g_io.inbox[0] = wire[0];
    g_io.inbox[1] = wire[1];
    g_io.inbox_n = 2;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.frame_dec.primed == 1u && core.link.rx[0].fresh == 0u, "a split RS422 frame waits");
    g_io.recv_max = 3;
    g_io.inbox[0] = wire[2];
    g_io.inbox[1] = wire[3];
    g_io.inbox[2] = wire[4];
    g_io.inbox_n = 3;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(a429_core_fresh_eu(&core, 0, &eu) == 1 && core.link.rx[0].view.raw == ng,
              "the split RS422 frame is logical Ng50");

    g_io.recv_max = 16;
    for (i = 0; i < 5; i++) {
        g_io.inbox[i] = wire[i];
        g_io.inbox[5 + i] = wire[i];
    }
    g_io.inbox_n = 10;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.link.counters.rx_count >= 3u && core.link.rx[0].view.raw == ng,
              "two coalesced RS422 frames stay Ng50");

    g_io.recv_max = 3;
    g_io.inbox[0] = wire[0];
    g_io.inbox[1] = wire[1];
    g_io.inbox[2] = wire[2];
    g_io.inbox_n = 3;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.frame_dec.primed == 1u, "a later partial RS422 frame is held");
    g_io.connected = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.frame_dec.primed == 0u && core.link.rx[0].fresh == 0u, "session loss drops the partial and freshness");
    nt_expect(core.link.rx[0].last_valid.raw == ng, "Ng history remains");
    now = core.tcp.next_attempt_us;
    a429_core_update(&core, now);
    g_io.recv_max = 5;
    memcpy(g_io.inbox, wire, 5);
    g_io.inbox_n = 5;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.link.rx[0].fresh == 1u && core.link.rx[0].view.raw == ng,
              "the new session accepts a whole RS422 frame");

    memset(&levels, 0, sizeof(levels));
    levels.mode = A429_MODE_FLIGHT;
    levels.cold_run = 1;
    nt_expect(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &levels, &pcmd) == 1, "pcmd high");
    a429_link_ingest(&core.link, pcmd, now);
    nt_expect(core.link.pcmd[0].edge.have == 1u && core.link.pcmd[0].edges.cold_run == 0u,
              "the first command sample does not edge");
    levels.cold_run = 0;
    nt_expect(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &levels, &pcmd) == 1, "pcmd low");
    a429_link_ingest(&core.link, pcmd, now + 1u);
    levels.cold_run = 1;
    nt_expect(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &levels, &pcmd) == 1, "pcmd rise");
    a429_link_ingest(&core.link, pcmd, now + 2u);
    nt_expect(core.link.pcmd[0].edges.cold_run == 1u, "a real rising edge is latched");

    g_io.recv_max = 2;
    g_io.inbox[0] = 0xC0u;
    g_io.inbox[1] = 0x64u;
    g_io.inbox_n = 2;
    g_io.inbox_r = 0;
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.frame_dec.primed == 1u, "FRAME 1 holds a logical partial");
    a429_core_set_frame(&core, A429_FRAME_RS422_429_1C);
    nt_expect(core.frame_dec.primed == 0u && core.link.rx[0].fresh == 0u, "the profile switch clears the half-frame and freshness");
    nt_expect(core.link.pcmd[0].edge.have == 0u && core.link.pcmd[0].edges.cold_run == 0u,
              "the profile switch re-arms command edges");
    g_io.recv_max = 5;
    memcpy(g_io.inbox, wire, 5);
    g_io.inbox_n = 5;
    g_io.inbox_r = 0;
    now += 1000u;
    a429_core_update(&core, now);
    nt_expect(core.link.rx[0].view.raw == ng, "old FRAME 1 bytes are not stitched onto the RS422 frame");
    levels.mode = A429_MODE_FLIGHT;
    levels.cold_run = 1;
    nt_expect(a429_encode_pcmd(0340, 0, A429_SM_NORMAL, &levels, &pcmd) == 1, "pcmd after switch");
    a429_link_ingest(&core.link, pcmd, now);
    nt_expect(core.link.pcmd[0].edges.cold_run == 0u && core.link.pcmd[0].edge.have == 1u,
              "the first sample after the switch does not synthesize an edge");

    script_reset(&core.tcp);
    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 8887);
    a429_sim_set_nr(&core.link, 100.0f);
    nt_expect(a429_encode_bnr(0311, 0, A429_SM_NORMAL, 100.0f, &nr) == 1, "Nr word");
    logical_b = (uint8_t)nr;
    wire_b = a429_label_bit_reverse(logical_b);
    g_io.block_send = 1;
    a429_core_update(&core, 0);
    a429_core_update(&core, 12500u);
    a429_core_update(&core, 25000u);
    nt_expect(core.tcp.tx_len == 5u && core.tcp.tx[0] == 0xC0u && core.tcp.tx[1] == logical_b,
              "FRAME 1 queues the logical label");
    a429_core_set_frame(&core, A429_FRAME_RS422_429_1C);
    nt_expect(core.tcp.tx_len == 0u && core.frame_dec.primed == 0u, "the unsent logical slot is dropped on the switch");
    g_io.block_send = 0;
    g_io.send_max = 5;
    g_io.sent_n = 0;
    a429_core_update(&core, 37500u);
    a429_core_update(&core, 50000u);
    nt_expect(g_io.sent_n == 5 && g_io.sent[0] == 0xC0u && g_io.sent[1] == wire_b,
              "the next slot is a whole RS422 frame");
    nt_expect(!buffer_has_cd(g_io.sent, g_io.sent_n), "the scheduler does not emit a CD rate command");

    script_reset(&core.tcp);
    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_UART429S_C0);
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 8887);
    a429_sim_set_nr(&core.link, 100.0f);
    g_io.send_max = 2;
    a429_core_update(&core, 0);
    a429_core_update(&core, 12500u);
    a429_core_update(&core, 25000u);
    nt_expect(g_io.sent_n == 2 && core.tcp.tx_off == 2u, "two bytes of the logical frame were accepted");
    a429_core_set_frame(&core, A429_FRAME_RS422_429_1C);
    nt_expect(core.tcp.force_close == 1u && core.tcp.tx_len == 0u, "a partial old frame requests a connection reset");
    g_io.send_max = 5;
    g_io.sent_n = 0;
    a429_core_update(&core, 37500u);
    a429_core_update(&core, 50000u);
    nt_expect(core.tcp.state == A429_TCP_UP && g_io.sent_n == 5 && g_io.sent[0] == 0xC0u &&
              g_io.sent[1] == wire_b, "the reset session sends one whole RS422 frame");

    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_RS422_429_1C);
    a429_tcp_configure(&core.tcp, 1, 0x0A000001u, 8887);
    a429_core_update(&core, 0);
    g_io.recv_max = 5;
    memcpy(g_io.inbox, wire, 5);
    g_io.inbox_n = 5;
    g_io.inbox_r = 0;
    a429_core_update(&core, 1000u);
    nt_expect(a429_core_fresh_eu(&core, 0, &eu) == 1, "Ng is fresh before the epoch moves");
    core.tcp.io_external = 1;
    a429_tcp_abandon_hold(&core.tcp);
    nt_expect(core.link.rx[0].fresh == 1u, "the stored flag waits for the vehicle update");
    nt_expect(a429_core_fresh_eu(&core, 0, &eu) == 0, "the getter sees the new epoch immediately");
    a429_core_update(&core, 2000u);
    nt_expect(core.link.rx[0].fresh == 0u && core.tcp_epoch_seen == core.tcp.epoch,
              "the vehicle update then clears the stored flag");

    nt_expect(a429_encode_bnr(0144, 1, A429_SM_NORMAL, 40.0f, &sdi1) == 1, "Ng SDI 1");
    a429_link_ingest(&core.link, ng, 3000u);
    nt_expect(core.link.rx[0].fresh == 1u, "SDI 0 sample is fresh again");
    a429_core_set_sdi(&core, 1);
    nt_expect(core.link.rx[0].fresh == 0u && core.link.pcmd[0].edge.have == 0u,
              "an SDI change clears freshness and re-arms edges");
    a429_link_ingest(&core.link, ng, 4000u);
    nt_expect(core.link.rx[0].fresh == 0u, "the old SDI is no longer current");
    a429_link_ingest(&core.link, sdi1, 5000u);
    nt_expect(core.link.rx[0].fresh == 1u && core.link.rx[0].view.raw == sdi1, "the new SDI becomes fresh");
}

static void test_unframed_does_not_send(void)
{
    a429_core_t core;
    a429_core_init(&core);
    script_reset(&core.tcp);
    a429_core_set_frame(&core, A429_FRAME_NONE);
    a429_tcp_configure(&core.tcp, 1, 0xC0A80307u, 8887);
    a429_sim_set_nr(&core.link, 100.0f);
    a429_core_update(&core, 0);
    a429_core_update(&core, 12500u);
    nt_expect(core.tcp.state == A429_TCP_UP, "byte pipe connects with framing off");
    nt_expect(core.tx_unframed == 1u, "a due slot is not turned into bytes");
    nt_expect(g_io.sent_n == 0, "the mock peer receives no command");
    nt_expect(core.link.counters.rx_count == 0u, "unframed bytes are not ingested as words");
}

static void test_mock_server(void)
{
    a429_tcp_t tcp;
    a429_stream_t stream;
    uint16_t port = 0;
    int srv = listen_local(&port);
    int cli = -1;
    uint8_t frame[5] = {0xC0u, 0x11u, 0x22u, 0x33u, 0x44u};
    uint8_t got[16];
    int i;
    struct timeval tv;

    nt_expect(srv >= 0 && port != 0, "mock server listens");
    if (srv < 0) {
        return;
    }
    g_pos.fd = -1;
    a429_tcp_init(&tcp);
    memset(&stream, 0, sizeof(stream));
    stream.open = posix_open;
    stream.send = posix_send;
    stream.recv = posix_recv;
    stream.close = posix_close;
    stream.ctx = &g_pos;
    a429_tcp_set_stream(&tcp, &stream);
    a429_tcp_configure(&tcp, 1, 0x7F000001u, port);
    a429_tcp_poll(&tcp, 0);
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    cli = accept_once(srv);
    nt_expect(cli >= 0 && tcp.state == A429_TCP_UP, "mock server accepts the client");
    if (cli < 0) {
        posix_close(&g_pos);
        close(srv);
        return;
    }
    {
        int one = 1;
        setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }
    for (i = 0; i < 5; i++) {
        nt_expect(send(cli, frame + i, 1, 0) == 1, "server writes one byte");
        a429_tcp_poll(&tcp, 1000u + (uint32_t)i);
    }
    {
        uint8_t chunk[8];
        uint8_t n = 0;
        uint8_t tmp[8];
        int spins = 0;
        while (n < 5u && spins < 20) {
            uint8_t gotn = a429_tcp_drain(&tcp, tmp, (uint8_t)(sizeof(tmp)));
            uint8_t k;
            for (k = 0; k < gotn && n < sizeof(chunk); k++) {
                chunk[n++] = tmp[k];
            }
            a429_tcp_poll(&tcp, 2000u + (uint32_t)spins);
            spins++;
        }
        nt_expect(n == 5u && memcmp(chunk, frame, 5) == 0, "fragmented socket reads reassemble");
    }
    nt_expect(send(cli, frame, 5, 0) == 5, "server writes a coalesced frame");
    nt_expect(send(cli, frame, 5, 0) == 5, "server writes a second coalesced frame");
    a429_tcp_poll(&tcp, 3000);
    {
        uint8_t chunk[16];
        uint8_t n = a429_tcp_drain(&tcp, chunk, sizeof(chunk));
        nt_expect(n == 10u, "coalesced socket read");
    }

    nt_expect(a429_tcp_submit(&tcp, frame, 5, 4000u) == 1, "client submits one frame");
    for (i = 0; i < 5; i++) {
        a429_tcp_poll(&tcp, 4000u + (uint32_t)i);
    }
    {
        int total = 0;
        while (total < 5) {
            ssize_t r = recv(cli, got + total, (size_t)(5 - total), 0);
            if (r <= 0) {
                break;
            }
            total += (int)r;
        }
        nt_expect(total == 5 && memcmp(got, frame, 5) == 0, "mock server receives the frame");
    }

    close(cli);
    cli = -1;
    a429_tcp_poll(&tcp, 5000);
    nt_expect(tcp.state == A429_TCP_BACKOFF, "peer close is a disconnect");
    nt_expect(a429_tcp_submit(&tcp, frame, 5, 5000u) == 0, "submit while down is dropped");
    a429_tcp_poll(&tcp, 5000u + A429_TCP_BACKOFF_MIN_US);
    cli = accept_once(srv);
    nt_expect(cli >= 0 && tcp.state == A429_TCP_UP, "client reconnects");
    if (cli >= 0) {
        ssize_t r = recv(cli, got, sizeof(got), MSG_DONTWAIT);
        nt_expect(r < 0, "reconnect does not replay the dropped frame");
        close(cli);
    }

    posix_close(&g_pos);
    close(srv);
}
#endif

int a429_nt1m_tests(void);

int a429_nt1m_tests(void)
{
    nt_fails = 0;
    test_label_and_provisional_frame();
    test_scripted_session();
    test_unframed_does_not_send();
    test_connection_framing();
    test_zero_octet_peer();
    test_tx_age_and_write_fail();
    test_unstable_backoff();
    test_disconnect_clears_fresh();
    test_backoff_needs_a_completed_frame();
    test_rs422_session();
#if !defined(_WIN32)
    test_mock_server();
#endif
    if (nt_fails == 0) {
        printf("NT1M RESULT: all passed\n");
    } else {
        printf("NT1M RESULT: %d failed\n", nt_fails);
    }
    return nt_fails;
}
