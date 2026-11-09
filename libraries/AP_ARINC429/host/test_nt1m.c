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

static int script_open(void *ctx, uint32_t ip, uint16_t port)
{
    script_io_t *io = (script_io_t *)ctx;
    (void)ip;
    (void)port;
    io->open_calls++;
    if (io->fail_open) {
        return -1;
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
    if (io->fail_send) {
        io->connected = 0;
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
    nt_expect(a429_tcp_submit(&tcp, frame, 5) == 0, "submit before connect is dropped");
    nt_expect(tcp.tx_dropped == 1u, "pre-connect submit counted");
    g_io.progress_left = 1;
    a429_tcp_poll(&tcp, 2000);
    nt_expect(tcp.state == A429_TCP_CONNECTING && g_io.connected == 0, "connect can stay in progress");
    a429_tcp_poll(&tcp, 3000);
    nt_expect(tcp.state == A429_TCP_UP && tcp.connect_ok == 1u, "second open completes");
    nt_expect(g_io.sent_n == 0, "bytes queued before connect are not replayed");

    nt_expect(a429_tcp_submit(&tcp, frame, 5) == 1, "submit one frame");
    a429_tcp_poll(&tcp, 4000);
    nt_expect(g_io.sent_n == 1 && g_io.sent[0] == 0xC0u, "partial write sends one byte");
    nt_expect(tcp.partial_sends == 1u, "partial write is counted");
    a429_tcp_submit(&tcp, other, 5);
    nt_expect(tcp.tx_dropped == 2u, "a second frame is not queued behind the partial");
    for (i = 0; i < 4; i++) {
        a429_tcp_poll(&tcp, 5000u + (uint32_t)i);
    }
    nt_expect(g_io.sent_n == 5, "the current frame finishes without a burst");
    nt_expect(memcmp(g_io.sent, frame, 5) == 0, "finished bytes are the first frame only");

    g_io.fail_send = 1;
    nt_expect(a429_tcp_submit(&tcp, other, 5) == 1, "submit after the pipe is idle");
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
    nt_expect(a429_tcp_submit(&tcp, other, 5) == 1, "a new slot can be sent");
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

    nt_expect(a429_tcp_submit(&tcp, frame, 5) == 1, "client submits one frame");
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
    nt_expect(a429_tcp_submit(&tcp, frame, 5) == 0, "submit while down is dropped");
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
