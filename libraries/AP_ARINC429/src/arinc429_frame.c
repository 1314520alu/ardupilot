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

int a429_frame_encode(uint8_t profile, const uint32_t *words, uint8_t nwords,
                      uint8_t *out, uint8_t cap, uint8_t *out_n)
{
    uint8_t i;
    uint8_t need;
    if (out_n != 0) {
        *out_n = 0;
    }
    if (profile != A429_FRAME_UART429S_C0 || words == 0 || out == 0 || nwords == 0) {
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
    if (dec == 0 || bytes == 0 || dec->profile != A429_FRAME_UART429S_C0) {
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
            words[produced] = (uint32_t)dec->hold[0] |
                              ((uint32_t)dec->hold[1] << 8) |
                              ((uint32_t)dec->hold[2] << 16) |
                              ((uint32_t)dec->hold[3] << 24);
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
