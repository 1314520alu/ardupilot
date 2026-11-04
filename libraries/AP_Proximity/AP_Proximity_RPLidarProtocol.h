#pragma once

/*
 * Slamtec RPLIDAR wire helpers shared by the serial driver and the S3E UDP
 * backend. Dense Express capsules are 84 bytes, answer type 0x85:
 *   sync nibbles 0xA / 0x5, checksum in the low nibbles, then start angle
 *   (q6) and 40 little-endian distances in millimetres.
 * Checksum is the XOR of every byte from start_angle onward (Slamtec SDK
 * UnpackerHandler_DenseCapsuleNode). A failed checksum is not a valid range.
 */

#include <stdint.h>
#include <string.h>

namespace AP_RPLidar
{

static constexpr uint8_t PREAMBLE = 0xA5;
static constexpr uint8_t CMDFLAG_HAS_PAYLOAD = 0x80;
static constexpr uint16_t EXPRESS_SCAN_FLAG_BOOST = 0x0001;
static constexpr uint8_t ANS_TYPE_DENSE = 0x85;
static constexpr uint8_t DENSE_BYTES = 84;
static constexpr uint8_t SYNC1 = 0x0A;
static constexpr uint8_t SYNC2 = 0x05;
static constexpr uint16_t SYNCBIT = uint16_t(1U << 15);
static constexpr uint8_t DENSE_POINTS = 40;

struct Point {
    float angle_deg;     // sensor frame, 0..360, before orientation/yaw correction
    float distance_m;
};

struct DenseParse {
    bool have_prev;
    uint8_t prev[DENSE_BYTES];
    int8_t last_sync;
    uint16_t express_sample_us;
};

inline uint16_t rd_le16(const uint8_t *p)
{
    return uint16_t(p[0]) | uint16_t(uint16_t(p[1]) << 8);
}

inline bool dense_sync_ok(const uint8_t *b)
{
    return b != nullptr && (b[0] >> 4) == SYNC1 && (b[1] >> 4) == SYNC2;
}

// True only when sync nibbles match and the XOR checksum matches.
inline bool dense_checksum_ok(const uint8_t *b)
{
    if (!dense_sync_ok(b)) {
        return false;
    }
    const uint8_t recv = uint8_t((b[0] & 0x0FU) | ((b[1] & 0x0FU) << 4));
    uint8_t calc = 0;
    for (uint8_t i = 2; i < DENSE_BYTES; i++) {
        calc ^= b[i];
    }
    return recv == calc;
}

inline void dense_reset(DenseParse &st)
{
    st.have_prev = false;
    st.last_sync = 0;
    st.express_sample_us = 0;
    memset(st.prev, 0, sizeof(st.prev));
}

/*
 * Emit points from the previous dense capsule, using curr as the end angle.
 * curr must already have passed dense_checksum_ok(). Returns the number of
 * non-zero ranges written to out. A new-revolution sync bit drops the
 * previous capsule (Slamtec does the same) and stores curr for next time.
 * Angle-step outliers are not emitted, but curr still becomes the new prev.
 */
inline uint8_t dense_emit(DenseParse &st, const uint8_t *curr, Point *out, uint8_t max_out)
{
    const uint16_t start_cur = rd_le16(curr + 2);
    if ((start_cur & SYNCBIT) != 0U) {
        st.have_prev = false;
        st.last_sync = 0;
    }

    uint8_t nout = 0;
    if (st.have_prev) {
        const uint16_t start_prev = rd_le16(st.prev + 2);
        const int32_t prev_q8 = int32_t((start_prev & 0x7FFFU) << 2);
        const int32_t curr_q8 = int32_t((start_cur & 0x7FFFU) << 2);
        int32_t diff_q8 = curr_q8 - prev_q8;
        if (prev_q8 > curr_q8) {
            diff_q8 += int32_t(360 << 8);
        }

        uint32_t sus = st.express_sample_us;
        if (sus < 20U) {
            sus = 55U;
        }
        const int64_t max_diff_q8 = (int64_t(360) * 100LL * 40LL / (1000000LL / int64_t(sus))) << 8;
        if (int64_t(diff_q8) <= max_diff_q8) {
            const int32_t angle_inc_q16 = (diff_q8 << 8) / int32_t(DENSE_POINTS);
            int32_t angle_raw_q16 = prev_q8 << 8;
            for (uint8_t pos = 0; pos < DENSE_POINTS; pos++) {
                const uint16_t dist_mm = rd_le16(st.prev + 4U + uint16_t(pos) * 2U);
                const int32_t angle_q6 = angle_raw_q16 >> 10;
                int32_t sync_bit = (((angle_raw_q16 + angle_inc_q16) % (360 << 16)) < (angle_inc_q16 << 1)) ? 1 : 0;
                sync_bit = (sync_bit ^ st.last_sync) & sync_bit;
                st.last_sync = int8_t(sync_bit);
                angle_raw_q16 += angle_inc_q16;

                int32_t aq = angle_q6;
                if (aq < 0) {
                    aq += (360 << 6);
                }
                if (aq >= (360 << 6)) {
                    aq -= (360 << 6);
                }
                if (dist_mm != 0U && out != nullptr && nout < max_out) {
                    out[nout].angle_deg = float(aq) / 64.0f;
                    out[nout].distance_m = float(dist_mm) * 0.001f;
                    nout++;
                }
            }
        }
    }

    memcpy(st.prev, curr, DENSE_BYTES);
    st.have_prev = true;
    return nout;
}

// A5 | cmd | len | payload | XOR of all preceding bytes. cmd must have bit7 set.
inline uint8_t build_payload_cmd(uint8_t *out, uint8_t out_len, uint8_t cmd,
                                 const uint8_t *payload, uint8_t payload_len)
{
    if (out == nullptr || payload == nullptr || payload_len > 32U ||
        (cmd & CMDFLAG_HAS_PAYLOAD) == 0U) {
        return 0;
    }
    const uint8_t n = uint8_t(3U + payload_len + 1U);
    if (out_len < n) {
        return 0;
    }
    out[0] = PREAMBLE;
    out[1] = cmd;
    out[2] = payload_len;
    memcpy(&out[3], payload, payload_len);
    uint8_t xsum = 0;
    for (uint8_t i = 0; i < uint8_t(3U + payload_len); i++) {
        xsum ^= out[i];
    }
    out[3U + payload_len] = xsum;
    return n;
}

} // namespace AP_RPLidar
