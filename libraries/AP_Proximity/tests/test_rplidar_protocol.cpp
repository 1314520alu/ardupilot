#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Proximity/AP_Proximity_RPLidarProtocol.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void stamp_dense_checksum(uint8_t *b)
{
    uint8_t calc = 0;
    for (uint8_t i = 2; i < AP_RPLidar::DENSE_BYTES; i++) {
        calc ^= b[i];
    }
    b[0] = uint8_t((AP_RPLidar::SYNC1 << 4) | (calc & 0x0FU));
    b[1] = uint8_t((AP_RPLidar::SYNC2 << 4) | ((calc >> 4) & 0x0FU));
}

TEST(RPLidarProtocol, DenseChecksum)
{
    uint8_t cap[AP_RPLidar::DENSE_BYTES] {};
    cap[2] = 0x10;
    cap[3] = 0x00;
    cap[4] = 0xE8;
    cap[5] = 0x03; // 1000 mm
    stamp_dense_checksum(cap);
    EXPECT_TRUE(AP_RPLidar::dense_checksum_ok(cap));

    cap[4] ^= 0x01;
    EXPECT_FALSE(AP_RPLidar::dense_checksum_ok(cap));

    stamp_dense_checksum(cap);
    cap[0] = uint8_t((cap[0] & 0x0FU) | 0x10); // break sync nibble
    EXPECT_FALSE(AP_RPLidar::dense_sync_ok(cap));
    EXPECT_FALSE(AP_RPLidar::dense_checksum_ok(cap));
}

TEST(RPLidarProtocol, PayloadCommandXor)
{
    const uint8_t payload[5] { 0x00, 0x01, 0x00, 0x00, 0x00 };
    uint8_t out[16] {};
    const uint8_t n = AP_RPLidar::build_payload_cmd(out, sizeof(out), 0x82, payload, sizeof(payload));
    EXPECT_EQ(9U, n);
    EXPECT_EQ(0xA5, out[0]);
    EXPECT_EQ(0x82, out[1]);
    EXPECT_EQ(5U, out[2]);
    EXPECT_EQ(0x01, out[4]);
    // A5 ^ 82 ^ 05 ^ 00 ^ 01 ^ 00 ^ 00 ^ 00
    EXPECT_EQ(0x23, out[8]);
    EXPECT_EQ(0U, AP_RPLidar::build_payload_cmd(out, sizeof(out), 0x25, payload, sizeof(payload)));
}

TEST(RPLidarProtocol, DenseEmitInterpolatesPreviousCapsule)
{
    uint8_t prev[AP_RPLidar::DENSE_BYTES] {};
    uint8_t curr[AP_RPLidar::DENSE_BYTES] {};
    // previous start angle 0, first cabin 1000 mm
    prev[4] = 0xE8;
    prev[5] = 0x03;
    // current start angle 10 deg in q6 = 640
    curr[2] = uint8_t(640 & 0xFF);
    curr[3] = uint8_t((640 >> 8) & 0xFF);

    AP_RPLidar::DenseParse st{};
    AP_RPLidar::Point pts[AP_RPLidar::DENSE_POINTS];
    EXPECT_EQ(0U, AP_RPLidar::dense_emit(st, prev, pts, AP_RPLidar::DENSE_POINTS));
    EXPECT_TRUE(st.have_prev);

    const uint8_t n = AP_RPLidar::dense_emit(st, curr, pts, AP_RPLidar::DENSE_POINTS);
    EXPECT_EQ(1U, n);
    EXPECT_NEAR(0.0f, pts[0].angle_deg, 0.02f);
    EXPECT_NEAR(1.0f, pts[0].distance_m, 0.0001f);
}

TEST(RPLidarProtocol, DenseSyncBitDropsPrevious)
{
    uint8_t prev[AP_RPLidar::DENSE_BYTES] {};
    uint8_t curr[AP_RPLidar::DENSE_BYTES] {};
    prev[4] = 0xE8;
    prev[5] = 0x03;
    curr[2] = 0x00;
    curr[3] = 0x80; // sync bit set, angle 0

    AP_RPLidar::DenseParse st{};
    AP_RPLidar::Point pts[4];
    EXPECT_EQ(0U, AP_RPLidar::dense_emit(st, prev, pts, 4));
    EXPECT_EQ(0U, AP_RPLidar::dense_emit(st, curr, pts, 4));
    EXPECT_TRUE(st.have_prev);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
