/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CS35 appendix B word layout. Bit numbers are the ICD's 1-based D1..D32.
 * Bit 1 is the least significant bit of the uint32. The octal label is
 * stored in D1..D8 as that integer (0144 -> 0x64). This is the logical
 * word, not a PHY bit reversal. A future converter owns any wire reversal.
 */
#pragma once

#include <stdint.h>

#define A429_FC_PERIOD_US 12500u
#define A429_ECU_NOMINAL_REFRESH_US 24000u
#define A429_RX_N 7
#define A429_PCMD_N 3
#define A429_TX_MAX 5

/* Appendix B SM, D31..D30. */
#define A429_SM_FAIL 0u
#define A429_SM_NCD 1u
#define A429_SM_FT 2u
#define A429_SM_NORMAL 3u

#define A429_DIR_ECU_TO_FC 0u
#define A429_DIR_FC_TO_ECU 1u

#define A429_KIND_BNR 0u
#define A429_KIND_DISC 1u

/* 3-bit D16..D14 code. Values match the bit field. 0 is not a command. */
#define A429_MODE_UNKNOWN 0u
#define A429_MODE_STOP 1u
#define A429_MODE_IDLE 3u
#define A429_MODE_FLIGHT 4u

/* Word transport used by host tests. The NT1-M path is a TCP client. */
#define A429_CARRIER_UNSPECIFIED 0u
#define A429_CARRIER_TCP_CLIENT 1u

/* Byte interpretation. 0 does not turn socket bytes into words. */
#define A429_FRAME_NONE 0u
/* Provisional UART429-S profile: 0xC0 plus the logical codec word,
 * little-endian. The label integer is not bit-reversed and no CD rate
 * command is generated. Persisted value 1 keeps this meaning. */
#define A429_FRAME_UART429S_C0 1u
/* RS422-429-1C manual, pages 3 and 4. Same C0 plus four little-endian
 * bytes, but the low byte is the label with its eight bits reversed.
 * Not selected by value 1. Not a claim that the NT1-M board matches it. */
#define A429_FRAME_RS422_429_1C 2u
/* Pure encoders for the three page-3 rate commands. Nothing schedules them. */
#define A429_RS422_RATE_100K 0u
#define A429_RS422_RATE_50K 1u
#define A429_RS422_RATE_12K5 2u

#define A429_TCP_TX_CAP 32u
#define A429_TCP_RX_CAP 64u
#define A429_TCP_BACKOFF_MIN_US 200000u
#define A429_TCP_BACKOFF_MAX_US 5000000u
/* Abandon an application frame that send() has not finished within 50 ms
 * (four 12.5 ms slots). A prefix already accepted by send() resets the
 * connection instead of being spliced to a newer frame. Closing the socket
 * does not purge bytes the OS or the NT1-M has already accepted. */
#define A429_TCP_TX_MAX_AGE_US 50000u
/* A session is stable only after send() has accepted one complete submitted
 * slot. Handshake time and time spent up with no completed slot do not
 * qualify. The backoff cap below is unchanged. */

#define A429_TCP_OFF 0u
#define A429_TCP_BACKOFF 1u
#define A429_TCP_CONNECTING 2u
#define A429_TCP_UP 3u

/* Octal label span from the appendix, not individually decoded. */
#define A429_LIFE_LABEL_FIRST 0200
#define A429_LIFE_LABEL_LAST 0220

typedef struct {
    uint16_t label_oct;
    uint8_t dir;
    uint8_t phase1;
    uint8_t kind;
    uint8_t has_range;
    int16_t denom;
    int16_t min_raw;
    int16_t max_raw;
    uint16_t span_last_oct;
    const char *name;
} a429_label_info_t;

typedef struct {
    uint32_t raw;
    uint8_t label;
    uint8_t sdi;
    uint8_t sm;
    uint8_t parity_ok;
    uint8_t reserved_ok;
    uint8_t sdi_ok;
    uint8_t known;
    uint8_t kind;
    uint8_t in_range;
    uint8_t eu_valid;
    uint8_t disc_valid;
    int16_t data;
    float eu;
} a429_word_view_t;

/* Levels from one word. These are not edges. */
typedef struct {
    uint8_t mode;
    uint8_t cold_run;
    uint8_t false_start;
    uint8_t rotor_brake;
    uint8_t on_ground;
    uint8_t power_assurance;
    uint8_t channel_switch;
    uint8_t overspeed_a;
    uint8_t overspeed_b;
} a429_pcmd_levels_t;

typedef struct {
    uint8_t cold_run;
    uint8_t false_start;
    uint8_t channel_switch;
} a429_pcmd_edges_t;

typedef struct {
    uint8_t have;
    a429_pcmd_levels_t prev;
} a429_edge_state_t;

typedef struct {
    uint32_t rx_count;
    uint32_t valid_count;
    uint32_t parity_errors;
    uint32_t reserved_errors;
    uint32_t sdi_errors;
    uint32_t sm_fail;
    uint32_t sm_ncd;
    uint32_t sm_ft;
    uint32_t sm_normal;
    uint32_t range_errors;
} a429_counters_t;

typedef struct {
    uint32_t raw;
    uint32_t rx_us;
    uint32_t valid_us;
    uint8_t have_raw;
    uint8_t have_valid;
    uint8_t fresh;
    float last_valid_eu;
    a429_word_view_t last_valid;
    uint32_t timeout_us;
    uint32_t timeout_count;
    a429_word_view_t view;
} a429_rx_slot_t;

typedef struct {
    uint32_t raw;
    uint32_t rx_us;
    uint8_t have_raw;
    a429_word_view_t view;
    a429_pcmd_levels_t levels;
    a429_pcmd_edges_t edges;
    a429_edge_state_t edge;
} a429_pcmd_slot_t;

typedef struct {
    uint8_t clp_set;
    float clp_v;
    uint8_t nr_set;
    float nr_pct;
    uint8_t pcmd_set[A429_PCMD_N];
    a429_pcmd_levels_t pcmd[A429_PCMD_N];
} a429_sim_tx_t;

typedef struct {
    uint8_t expected_sdi;
    a429_counters_t counters;
    a429_rx_slot_t rx[A429_RX_N];
    a429_pcmd_slot_t pcmd[A429_PCMD_N];
    a429_sim_tx_t sim;
    uint32_t rx_untracked;
    uint32_t last_untracked_raw;
    uint32_t last_untracked_us;
    uint32_t tx_words_built;
    uint32_t tx_rejected;
    uint8_t last_tx_count;
    uint32_t last_tx[A429_TX_MAX];
} a429_link_t;

typedef struct {
    uint8_t started;
    uint32_t next_tx_us;
    uint32_t tx_slots;
    uint32_t tx_slots_skipped;
} a429_sched_t;

typedef struct {
    uint32_t remote_ipv4;
    uint16_t remote_port;
    uint16_t local_port;
    uint8_t carrier;
} a429_endpoint_t;

/*
 * Explicit carrier. It must not open or claim the single X6 Ethernet
 * port (that port is shared with a future PROFINET gateway).
 * send returns 0 only if a real carrier accepted the words.
 * The null carrier returns -1 and writes nothing.
 * recv returns the number of words copied, or a negative error.
 * The null carrier returns 0 and does not modify the buffer.
 */
typedef struct a429_transport {
    int (*send_words)(void *ctx, const a429_endpoint_t *ep, const uint32_t *words, uint8_t n);
    int (*recv_words)(void *ctx, const a429_endpoint_t *ep, uint32_t *words, uint8_t max_n);
    void *ctx;
} a429_transport_t;

/*
 * Byte pipe under the TCP session. connect/send/recv must be usable from
 * one owner thread. connect may block that thread; it must not be the
 * vehicle main loop. send/recv return a byte count, 0 for would-block,
 * and -1 for a hard failure or disconnect.
 */
typedef struct a429_stream {
    int (*open)(void *ctx, uint32_t ipv4_host, uint16_t port);
    int (*send)(void *ctx, const uint8_t *bytes, uint16_t n);
    int (*recv)(void *ctx, uint8_t *bytes, uint16_t n);
    void (*close)(void *ctx);
    void *ctx;
} a429_stream_t;

typedef struct {
    a429_stream_t stream;
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);
    void *lock_ctx;
    uint8_t enabled;
    uint8_t io_external;
    uint8_t force_close;
    uint8_t state;
    uint32_t peer_ip;
    uint16_t peer_port;
    uint32_t epoch;
    uint32_t next_attempt_us;
    uint32_t backoff_us;
    uint32_t connect_ok;
    uint32_t connect_fail;
    uint32_t disconnects;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_dropped;
    uint32_t rx_backpressure;
    uint32_t partial_sends;
    uint32_t tx_expired;
    uint32_t tx_queued_us;
    uint32_t up_since_us;
    uint8_t session_stable;
    uint32_t (*now_us_fn)(void *ctx);
    void *now_us_ctx;
    uint8_t tx[A429_TCP_TX_CAP];
    uint8_t tx_len;
    uint8_t tx_off;
    uint8_t rx[A429_TCP_RX_CAP];
    uint8_t rx_count;
} a429_tcp_t;

typedef struct {
    uint8_t profile;
    uint8_t primed;
    uint8_t hold[4];
    uint8_t hold_n;
    uint32_t skipped;
    uint32_t frames;
} a429_frame_dec_t;

typedef struct {
    a429_sched_t sched;
    a429_link_t link;
    a429_endpoint_t endpoint;
    const a429_transport_t *transport;
    uint32_t tx_not_sent;
    uint8_t frame_profile;
    a429_tcp_t tcp;
    a429_frame_dec_t frame_dec;
    uint32_t rx_unframed_bytes;
    uint32_t tx_unframed;
    uint32_t tcp_epoch_seen;
} a429_core_t;
