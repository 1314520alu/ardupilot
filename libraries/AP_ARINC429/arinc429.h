/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include "arinc429_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

const a429_label_info_t *a429_catalog(uint16_t *count);
const a429_label_info_t *a429_catalog_find(uint16_t label_oct);

uint32_t a429_apply_odd_parity(uint32_t word);
int a429_parity_ok(uint32_t word);

int a429_encode_bnr(uint8_t label_oct, uint8_t sdi, uint8_t sm, float eu, uint32_t *out);
int a429_encode_pcmd(uint8_t label_oct, uint8_t sdi, uint8_t sm,
                     const a429_pcmd_levels_t *levels, uint32_t *out);
void a429_inspect(uint32_t raw, uint8_t expected_sdi, a429_word_view_t *view);
void a429_pcmd_decode(uint16_t data, a429_pcmd_levels_t *levels);

void a429_edge_reset(a429_edge_state_t *st);
void a429_edge_update(a429_edge_state_t *st, const a429_pcmd_levels_t *now,
                      int sample_valid, a429_pcmd_edges_t *edges);

void a429_sched_init(a429_sched_t *s);
int a429_sched_poll(a429_sched_t *s, uint32_t now_us);

void a429_link_init(a429_link_t *link);
void a429_link_set_sdi(a429_link_t *link, uint8_t sdi);
void a429_link_set_timeout_us(a429_link_t *link, uint8_t rx_index, uint32_t timeout_us);
void a429_link_ingest(a429_link_t *link, uint32_t raw, uint32_t now_us);
void a429_link_poll_timeouts(a429_link_t *link, uint32_t now_us);
int a429_phase1_rx_index(uint16_t label_oct);
int a429_pcmd_index(uint16_t label_oct);

void a429_sim_clear(a429_link_t *link);
void a429_sim_set_clp(a429_link_t *link, float volts);
void a429_sim_set_nr(a429_link_t *link, float pct);
int a429_sim_set_pcmd(a429_link_t *link, uint8_t index, const a429_pcmd_levels_t *levels);
uint8_t a429_link_build_tx(a429_link_t *link, uint32_t *words, uint8_t max_n);

uint8_t a429_label_bit_reverse(uint8_t logical_label);
int a429_frame_encode(uint8_t profile, const uint32_t *words, uint8_t nwords,
                      uint8_t *out, uint8_t cap, uint8_t *out_n);
void a429_frame_dec_init(a429_frame_dec_t *dec, uint8_t profile);
uint8_t a429_frame_push(a429_frame_dec_t *dec, const uint8_t *bytes, uint8_t n,
                        uint32_t *words, uint8_t max_words);

void a429_tcp_init(a429_tcp_t *tcp);
void a429_tcp_set_stream(a429_tcp_t *tcp, const a429_stream_t *stream);
void a429_tcp_set_lock(a429_tcp_t *tcp, void (*lock)(void *ctx), void (*unlock)(void *ctx), void *ctx);
void a429_tcp_configure(a429_tcp_t *tcp, uint8_t enabled, uint32_t peer_ip, uint16_t peer_port);
void a429_tcp_poll(a429_tcp_t *tcp, uint32_t now_us);
int a429_tcp_submit(a429_tcp_t *tcp, const uint8_t *bytes, uint8_t n);
uint8_t a429_tcp_drain(a429_tcp_t *tcp, uint8_t *out, uint8_t max_n);

const a429_transport_t *a429_transport_null(void);
void a429_core_init(a429_core_t *core);
void a429_core_set_frame(a429_core_t *core, uint8_t profile);
void a429_core_set_transport(a429_core_t *core, const a429_transport_t *transport);
void a429_core_update(a429_core_t *core, uint32_t now_us);
void a429_core_inject_rx(a429_core_t *core, uint32_t raw, uint32_t now_us);

#ifdef __cplusplus
}
#endif
