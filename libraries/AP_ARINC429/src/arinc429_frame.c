#include "../arinc429.h"

uint8_t a429_label_bit_reverse(uint8_t logical_label)
{
    uint8_t out = 0;
    uint8_t i;
    for (i = 0; i < 8u; i++) {
        out = (uint8_t)((out << 1) | ((logical_label >> i) & 1u));
    }
    return out;
}

uint32_t a429_label_wire_swap(uint32_t word)
{
    /* Page 4 reverses the label byte only. Bits 8..32, including parity, stay. */
    return (word & 0xFFFFFF00u) | a429_label_bit_reverse((uint8_t)word);
}

int a429_frame_profile_active(uint8_t profile)
{
    return profile == A429_FRAME_UART429S_C0 || profile == A429_FRAME_RS422_429_1C;
}

int a429_rs422_rate_encode(uint8_t rate, uint8_t *out, uint8_t cap, uint8_t *out_n)
{
    uint8_t mid;
    if (out_n != 0) {
        *out_n = 0;
    }
    if (out == 0 || cap < 5u) {
        return 0;
    }
    if (rate == A429_RS422_RATE_100K) {
        mid = 0x00u;
    } else if (rate == A429_RS422_RATE_50K) {
        mid = 0x03u;
    } else if (rate == A429_RS422_RATE_12K5) {
        mid = 0x07u;
    } else {
        return 0;
    }
    out[0] = 0xCDu;
    out[1] = mid;
    out[2] = 0x00u;
    out[3] = 0x55u;
    out[4] = 0x55u;
    if (out_n != 0) {
        *out_n = 5u;
    }
    return 1;
}

int a429_frame_encode(uint8_t profile, const uint32_t *words, uint8_t nwords,
                      uint8_t *out, uint8_t cap, uint8_t *out_n)
{
    uint8_t i;
    uint8_t need;
    if (out_n != 0) {
        *out_n = 0;
    }
    if (!a429_frame_profile_active(profile) || words == 0 || out == 0 || nwords == 0) {
        return 0;
    }
    if (nwords > 255u / 5u) {
        return 0;
    }
    need = (uint8_t)(nwords * 5u);
    if (need > cap) {
        return 0;
    }
    for (i = 0; i < nwords; i++) {
        uint32_t w = words[i];
        uint8_t *p = out + (i * 5u);
        if (profile == A429_FRAME_RS422_429_1C) {
            w = a429_label_wire_swap(w);
        }
        p[0] = 0xC0u;
        p[1] = (uint8_t)(w & 0xFFu);
        p[2] = (uint8_t)((w >> 8) & 0xFFu);
        p[3] = (uint8_t)((w >> 16) & 0xFFu);
        p[4] = (uint8_t)((w >> 24) & 0xFFu);
    }
    if (out_n != 0) {
        *out_n = need;
    }
    return 1;
}

void a429_frame_dec_init(a429_frame_dec_t *dec, uint8_t profile)
{
    if (dec == 0) {
        return;
    }
    dec->profile = profile;
    dec->primed = 0;
    dec->hold_n = 0;
    dec->skipped = 0;
    dec->frames = 0;
}

void a429_frame_dec_resync(a429_frame_dec_t *dec)
{
    if (dec == 0) {
        return;
    }
    dec->primed = 0;
    dec->hold_n = 0;
}

uint8_t a429_frame_push(a429_frame_dec_t *dec, const uint8_t *bytes, uint8_t n,
                        uint32_t *words, uint8_t max_words)
{
    uint8_t i;
    uint8_t produced = 0;
    if (dec == 0 || bytes == 0 || !a429_frame_profile_active(dec->profile)) {
        return 0;
    }
    for (i = 0; i < n; i++) {
        uint8_t b = bytes[i];
        if (!dec->primed) {
            if (b == 0xC0u) {
                dec->primed = 1;
                dec->hold_n = 0;
            } else {
                dec->skipped++;
            }
            continue;
        }
        dec->hold[dec->hold_n++] = b;
        if (dec->hold_n < 4u) {
            continue;
        }
        if (produced < max_words && words != 0) {
            uint32_t w = (uint32_t)dec->hold[0] |
                         ((uint32_t)dec->hold[1] << 8) |
                         ((uint32_t)dec->hold[2] << 16) |
                         ((uint32_t)dec->hold[3] << 24);
            if (dec->profile == A429_FRAME_RS422_429_1C) {
                /* Inverse of the TX swap. Parity is not recomputed. */
                w = a429_label_wire_swap(w);
            }
            words[produced] = w;
            produced++;
            dec->frames++;
        } else {
            dec->skipped += 5u;
        }
        dec->primed = 0;
        dec->hold_n = 0;
    }
    return produced;
}
