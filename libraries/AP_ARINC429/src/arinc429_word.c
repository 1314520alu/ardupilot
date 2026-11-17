#include "../arinc429.h"

#include <string.h>

static int a429_finite(float v)
{
    union {
        float f;
        uint32_t u;
    } x;
    x.f = v;
    /* Exponent 0xff is IEEE-754 infinity or NaN. */
    return ((x.u >> 23) & 0xffu) != 0xffu;
}

static int32_t a429_round_i32(double v)
{
    if (v >= 0.0) {
        return (int32_t)(v + 0.5);
    }
    return (int32_t)(v - 0.5);
}

static float a429_nan(void)
{
    union {
        uint32_t u;
        float f;
    } v;
    v.u = 0x7fc00000u;
    return v.f;
}

uint32_t a429_apply_odd_parity(uint32_t word)
{
    uint32_t x;

    word &= 0x7FFFFFFFu;
    x = word;
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    if ((x & 1u) == 0u) {
        word |= 1u << 31;
    }
    return word;
}

int a429_parity_ok(uint32_t word)
{
    uint32_t x = word;
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (x & 1u) == 1u;
}

static uint32_t a429_pack(uint8_t label, uint8_t sdi, uint8_t sm, uint16_t data)
{
    uint32_t w = label;
    w |= ((uint32_t)(sdi & 3u)) << 8;
    w |= ((uint32_t)data) << 13;
    w |= ((uint32_t)(sm & 3u)) << 29;
    return a429_apply_odd_parity(w);
}

int a429_encode_bnr(uint8_t label_oct, uint8_t sdi, uint8_t sm, float eu, uint32_t *out)
{
    const a429_label_info_t *info;
    double scaled;
    int32_t raw;

    if (out == 0 || sdi > 3u || sm > 3u || !a429_finite(eu)) {
        return 0;
    }
    info = a429_catalog_find(label_oct);
    if (info == 0 || info->kind != A429_KIND_BNR || info->denom <= 0) {
        return 0;
    }
    scaled = (double)eu * (double)info->denom;
    if (scaled >= 32768.0 || scaled < -32768.5) {
        return 0;
    }
    raw = a429_round_i32(scaled);
    if (raw > 32767 || raw < -32768) {
        return 0;
    }
    if (info->has_range && (raw < info->min_raw || raw > info->max_raw)) {
        return 0;
    }
    *out = a429_pack(label_oct, sdi, sm, (uint16_t)(int16_t)raw);
    return 1;
}

void a429_pcmd_decode(uint16_t data, a429_pcmd_levels_t *levels)
{
    uint8_t mode;
    if (levels == 0) {
        return;
    }
    memset(levels, 0, sizeof(*levels));
    mode = (uint8_t)(data & 7u);
    if (mode == A429_MODE_STOP || mode == A429_MODE_IDLE || mode == A429_MODE_FLIGHT) {
        levels->mode = mode;
    } else {
        levels->mode = A429_MODE_UNKNOWN;
    }
    levels->cold_run = (uint8_t)((data >> 3) & 1u);
    levels->false_start = (uint8_t)((data >> 4) & 1u);
    levels->rotor_brake = (uint8_t)((data >> 5) & 1u);
    levels->on_ground = (uint8_t)((data >> 6) & 1u);
    levels->power_assurance = (uint8_t)((data >> 7) & 1u);
    levels->channel_switch = (uint8_t)((data >> 8) & 1u);
    levels->overspeed_a = (uint8_t)((data >> 9) & 1u);
    levels->overspeed_b = (uint8_t)((data >> 10) & 1u);
}

int a429_encode_pcmd(uint8_t label_oct, uint8_t sdi, uint8_t sm,
                     const a429_pcmd_levels_t *levels, uint32_t *out)
{
    uint16_t data;
    if (out == 0 || levels == 0 || sdi > 3u || sm > 3u) {
        return 0;
    }
    if (a429_pcmd_index(label_oct) < 0) {
        return 0;
    }
    if (levels->mode == A429_MODE_STOP) {
        data = 1u;
    } else if (levels->mode == A429_MODE_IDLE) {
        data = 3u;
    } else if (levels->mode == A429_MODE_FLIGHT) {
        data = 4u;
    } else {
        return 0;
    }
    if (levels->cold_run) {
        data |= (uint16_t)(1u << 3);
    }
    if (levels->false_start) {
        data |= (uint16_t)(1u << 4);
    }
    if (levels->rotor_brake) {
        data |= (uint16_t)(1u << 5);
    }
    if (levels->on_ground) {
        data |= (uint16_t)(1u << 6);
    }
    if (levels->power_assurance) {
        data |= (uint16_t)(1u << 7);
    }
    if (levels->channel_switch) {
        data |= (uint16_t)(1u << 8);
    }
    if (levels->overspeed_a) {
        data |= (uint16_t)(1u << 9);
    }
    if (levels->overspeed_b) {
        data |= (uint16_t)(1u << 10);
    }
    *out = a429_pack(label_oct, sdi, sm, data);
    return 1;
}

void a429_inspect(uint32_t raw, uint8_t expected_sdi, a429_word_view_t *view)
{
    const a429_label_info_t *info;

    if (view == 0) {
        return;
    }
    memset(view, 0, sizeof(*view));
    view->raw = raw;
    view->eu = a429_nan();
    view->label = (uint8_t)(raw & 0xFFu);
    view->sdi = (uint8_t)((raw >> 8) & 0x3u);
    view->sm = (uint8_t)((raw >> 29) & 0x3u);
    view->data = (int16_t)((raw >> 13) & 0xFFFFu);
    view->parity_ok = (uint8_t)a429_parity_ok(raw);
    view->reserved_ok = (uint8_t)(((raw >> 10) & 0x7u) == 0u);
    view->sdi_ok = (uint8_t)(view->sdi == (expected_sdi & 3u));
    info = a429_catalog_find(view->label);
    if (info != 0) {
        view->known = 1;
        view->kind = info->kind;
    }
    if (!view->parity_ok || !view->reserved_ok || !view->sdi_ok || info == 0) {
        return;
    }
    if (view->sm != A429_SM_NORMAL) {
        return;
    }
    if (info->kind == A429_KIND_DISC) {
        view->disc_valid = 1;
        return;
    }
    if (info->kind != A429_KIND_BNR || info->denom <= 0) {
        return;
    }
    if (info->has_range && (view->data < info->min_raw || view->data > info->max_raw)) {
        view->in_range = 0;
        return;
    }
    view->in_range = 1;
    view->eu = (float)((double)view->data / (double)info->denom);
    view->eu_valid = 1;
}
