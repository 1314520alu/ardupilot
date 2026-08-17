/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ArduPilot device driver for SLAMTEC RPLIDAR A2 / A-series and compatible
 * models (A1, A2, C1, S1, S3). A-series use legacy SCAN; S3 uses EXPRESS_SCAN dense (0x85) after bring-up.
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 * S-series (incl. S3) model ID layout: high nibble = major (S3 = 8), low nibble = sub-model; see SLAMTEC S-series protocol manual.
 *
 * ----- RPLIDAR S3 在本驱动中的工作流程（概要） -----
 * 1) 上电 / RESET 后 UART 按型号配置波特率（多数 S3 为 1000000；model==0x82 为 460800）。
 * 2) 等待启动串口输出；必要时在 RESET 态超时后主动发 GET_DEVICE_INFO(0x50)。
 * 3) S3 专用引导（Slamtec S 系列手册 / 公开 SDK）：
 *    a. GET_HEALTH(0x52) 读健康字。
 *    b. GET_SAMPLERATE(0x59) 读标准/Express 采样间隔（µs），用于密实帧角度插值门限。
 *    c. HQ_MOTOR_SPEED_CTRL(0xA8) 设置电机转速（默认 10Hz => 600 RPM）。
 *    d. 短延时后发 EXPRESS_SCAN：working_mode=0（legacy express），working_flags=BOOST(0x0001)
 *       即 DenseBoost 高密度模式；雷达以 SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED(0x85)
 *       的 84 字节密实胶囊包输出测距（每包 40 点，距离单位 mm）。
 * 4) 避障量程：传感器最大量程由 distance_max() 提供；请在参数中将 PRXn_MAX 设为 40（米）
 *    以在 ArduPilot 侧将障碍物报告限制在 40m（0 表示使用厂家默认/本驱动返回值）。
 * 5) CUAV-X7：任选支持 1Mbps 的串口（如 TELEM2），SERIALn_PROTOCOL=11 (Lidar360)，SERIALn_BAUD=1000000，
 *    PRX1_TYPE=5 (RPLidarA2 后端，含 S3)。
 * 6) 再次 RESET 前若已识别为 S3，先发 STOP(0x25) 并 flush 串口。
 * 7) Mission Planner: (a) STATUSTEXT is queued only when a MAVLink channel is active (see try_send_s3_gcs_pending).
 *    (b) Long strings are chunked; UTF-8 split mid-character shows as garbled text in MP — keep lines short ASCII.
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_RPLidarA2.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define RP_DEBUG_LEVEL 0

#include <GCS_MAVLink/GCS.h>
#if RP_DEBUG_LEVEL
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

#define COMM_ACTIVITY_TIMEOUT_MS        200

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_PREAMBLE               0xA5
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40

// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52

#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82
#endif

// S3 / Slamtec unified payload commands (cmd byte has bit7 => wire format includes size + XOR checksum)
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD    0x80
#define RPLIDAR_CMD_GET_SAMPLERATE     0x59
#define RPLIDAR_CMD_HQ_MOTOR_SPEED     0xA8

// 10 Hz mechanical spin => 600 RPM (Slamtec convention in public SDK examples).
#define RPLIDAR_S3_TARGET_MOTOR_RPM    600U

#define RPLIDAR_S3_MOTOR_SETTLE_MS    30

// Dense express capsule (Slamtec rplidar_sdk UnpackerHandler_DenseCapsuleNode)
#define RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED  0x85
#define RPLIDAR_DENSE_CAPSULE_BYTES                  84U
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1        0x0AU
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2        0x05U
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT       (1U<<15)

// SL_LIDAR_EXPRESS_SCAN_FLAG_BOOST — request DenseBoost / dense capsuled express stream
#define RPLIDAR_EXPRESS_SCAN_FLAG_BOOST              0x0001U

extern const AP_HAL::HAL& hal;

void AP_Proximity_RPLidarA2::try_send_s3_gcs_pending()
{
#if HAL_GCS_ENABLED && AP_HAVE_GCS_SEND_TEXT
    if (model != Model::S3) {
        return;
    }
    if (gcs().statustext_send_channel_mask() == 0U) {
        return;
    }
    if (_s3_pending_gcs_connected_msg) {
        /* Short ASCII: MP splits STATUSTEXT; UTF-8 mid-chunk breaks Chinese. */
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PRX%u: RPLidar S3 connected", (unsigned)state.instance + 1U);
        _s3_pending_gcs_connected_msg = false;
    }
    if (_s3_pending_gcs_express_msg) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PRX%u: S3 EXPRESS dense 40m", (unsigned)state.instance + 1U);
        _s3_pending_gcs_express_msg = false;
    }
#endif
}

void AP_Proximity_RPLidarA2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    try_send_s3_gcs_pending();

    // request device info 3sec after reset
    // required for S1 support that sends only 9 bytes after a reset (A1,A2 send 63)
    uint32_t now_ms = AP_HAL::millis();
    if ((_state == State::RESET) && (now_ms - _last_reset_ms > 3000)) {
        send_request_for_device_info();
        _state = State::AWAITING_RESPONSE;
        _byte_count = 0;
    }

    if (_state == State::S3_WAIT_MOTOR_SETTLE) {
        _last_distance_received_ms = now_ms;
        if (now_ms >= _s3_deadline_ms) {
            if (model == Model::S3) {
                send_s3_express_scan_dense();
            } else {
                send_scan_mode_request();
            }
            _state = State::AWAITING_RESPONSE;
            _s3_bootstrap = S3Bootstrap::None;
        }
    }

    get_readings();

    // check for timeout and set health status
    if (AP_HAL::millis() - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::NoData);
        Debug(1, "LIDAR NO DATA");
        if (AP_HAL::millis() - _last_reset_ms > 10000) {
            reset_rplidar();
        }
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_max_m() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
        return 8.0f;
    case Model::A2:
        return 16.0f;
    case Model::A2M12:
    case Model::C1:
        return 12.0f;
    case Model::S1:
        return 40.0f;
    case Model::S2:
        return 50.0f;
    case Model::S3:
        return 40.0f;
    }
    return 0.0f;
}

// get minimum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_min_m() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
    case Model::A2:
    case Model::A2M12:
    case Model::C1:
    case Model::S1:
    case Model::S3:
        return 0.2f;
    case Model::S2:
        return 0.05f;
    }
    return 0.0f;
}

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    if (_uart != nullptr && model == Model::S3) {
        send_stop();
        _uart->flush();
    }
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    Debug(1, "LIDAR reset");
    // To-Do: ensure delay of 8m after sending reset request
    _last_reset_ms =  AP_HAL::millis();
    reset();
}

// set Lidar into legacy SCAN mode (non-S3, or S3 fallback path if device_info used legacy scan)
void AP_Proximity_RPLidarA2::send_scan_mode_request()
{
    const uint8_t cmd = RPLIDAR_CMD_SCAN;
    const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, cmd};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent SCAN mode request");
}

void AP_Proximity_RPLidarA2::send_s3_express_scan_dense()
{
    // Legacy express (working_mode=0) + BOOST flag => dense capsuled measurements (see Slamtec S-series protocol).
    const uint8_t payload[5] {
        0x00,
        uint8_t(RPLIDAR_EXPRESS_SCAN_FLAG_BOOST & 0xFFU),
        uint8_t((RPLIDAR_EXPRESS_SCAN_FLAG_BOOST >> 8U) & 0xFFU),
        0x00,
        0x00
    };
    send_command_with_payload_xor(RPLIDAR_CMD_EXPRESS_SCAN, payload, sizeof(payload));
    Debug(1, "Sent S3 EXPRESS_SCAN dense (BOOST)");
    _s3_pending_gcs_express_msg = true;
}

void AP_Proximity_RPLidarA2::send_stop()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_STOP};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent STOP");
}

void AP_Proximity_RPLidarA2::send_command_with_payload_xor(const uint8_t cmd, const void *payload, const uint8_t payload_len)
{
    if (_uart == nullptr || payload_len > 32U || ((cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) == 0U)) {
        return;
    }
    uint8_t buf[3U + 32U + 1U];
    buf[0] = RPLIDAR_PREAMBLE;
    buf[1] = cmd;
    buf[2] = payload_len;
    memcpy(&buf[3], payload, payload_len);
    uint8_t xsum = 0;
    for (uint8_t i = 0; i < 3U + payload_len; i++) {
        xsum ^= buf[i];
    }
    buf[3U + payload_len] = xsum;
    _uart->write(buf, 3U + payload_len + 1U);
}

void AP_Proximity_RPLidarA2::send_s3_motor_rpm(const uint16_t rpm)
{
    send_command_with_payload_xor(RPLIDAR_CMD_HQ_MOTOR_SPEED, &rpm, sizeof(rpm));
    Debug(1, "Sent S3 motor RPM %u", unsigned(rpm));
}

#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
// send EXPRESS_SCAN request for Dense mode
void AP_Proximity_RPLidarA2::send_express_scan_request()
{
    const uint8_t cmd = RPLIDAR_CMD_EXPRESS_SCAN;
    const uint8_t payload_size = 5;
    uint8_t payload[payload_size] {};

    uint8_t checksum = 0;
    checksum ^= RPLIDAR_PREAMBLE;
    checksum ^= cmd;
    checksum ^= payload_size;
    for (uint8_t i = 0; i < sizeof(payload); i++) {
        checksum ^= payload[i];
    }

    // 2 (preamble + cmd) + 1 (payload_size field) + payload_size + 1 (checksum)
    uint8_t tx_buffer[2 + 1 + payload_size + 1];
    tx_buffer[0] = RPLIDAR_PREAMBLE;
    tx_buffer[1] = cmd;
    tx_buffer[2] = payload_size;
    memcpy(&tx_buffer[3], payload, sizeof(payload));
    tx_buffer[3 + sizeof(payload)] = checksum;

    _uart->write(tx_buffer, sizeof(tx_buffer));
    Debug(1, "Sent EXPRESS (Dense) scan request");
}
#endif // AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED

// send request for sensor health
void AP_Proximity_RPLidarA2::send_request_for_health()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent health request");
}

void AP_Proximity_RPLidarA2::send_request_for_sample_rate()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_SAMPLERATE};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent sample rate request");
}

// send request for device information
void AP_Proximity_RPLidarA2::send_request_for_device_info()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_INFO};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent device information request");
}

void AP_Proximity_RPLidarA2::consume_bytes(uint16_t count)
{
    if (count > _byte_count) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        _byte_count = 0;
        return;
    }
    _byte_count -= count;
    if (_byte_count) {
        memmove((void*)&_payload[0], (void*)&_payload[count], _byte_count);
    }
}

void AP_Proximity_RPLidarA2::reset()
{
    _state = State::RESET;
    _byte_count = 0;
    _sync_error = 0;
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
    _use_dense_express = false;
    _express_stream_len = 0;
#endif
    _s3_bootstrap = S3Bootstrap::None;
    _s3_dense_have_prev = false;
    _s3_dense_last_sync = 0;
    _s3_express_sample_us = 0;
    _s3_pending_gcs_connected_msg = false;
    _s3_pending_gcs_express_msg = false;
    memset(_s3_dense_prev_payload, 0, sizeof(_s3_dense_prev_payload));
}

bool AP_Proximity_RPLidarA2::make_first_byte_in_payload(uint8_t desired_byte)
{
    if (_byte_count == 0) {
        return false;
    }
    if (_payload[0] == desired_byte) {
        return true;
    }
    for (auto i=1; i<_byte_count; i++) {
        if (_payload[i] == desired_byte) {
            consume_bytes(i);
            return true;
        }
    }
    // just not in our buffer.  Throw everything away:
    _byte_count = 0;
    return false;
}

void AP_Proximity_RPLidarA2::get_readings()
{
    Debug(2, "             CURRENT STATE: %u ", (unsigned)_state);

#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
    // Express mode reads directly into _express_stream, bypassing _payload
    if (_state == State::AWAITING_EXPRESS_DATA) {
        handle_express_data();
        return;
    }
#endif
    if (_state == State::S3_WAIT_MOTOR_SETTLE) {
        uint8_t dump[128];
        while (_uart->available() > 0) {
            const uint32_t n = MIN((uint32_t)_uart->available(), (uint32_t)sizeof(dump));
            (void)_uart->read(dump, n);
        }
        return;
    }

    const uint32_t nbytes = _uart->available();
    if (nbytes == 0) {
        return;
    }
    const uint32_t bytes_to_read = MIN(nbytes, sizeof(_payload)-_byte_count);
    if (bytes_to_read == 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    const uint32_t bytes_read = _uart->read(&_payload[_byte_count], bytes_to_read);
    if (bytes_read == 0) {
        // this is bad; we were told there were bytes available
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    _byte_count += bytes_read;

    uint32_t previous_loop_byte_count = UINT32_MAX;
    while (_byte_count) {
        if (_byte_count >= previous_loop_byte_count) {
            // this is a serious error, we should always consume some
            // bytes.  Avoid looping forever.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            _uart = nullptr;
            return;
        }
        previous_loop_byte_count = _byte_count;

        switch(_state){
        case State::RESET: {
            // looking for 0x52 at start of buffer; the 62 following
            // bytes are "information"
            if (!make_first_byte_in_payload('R')) { // that's 'R' as in RPiLidar
                return;
            }
            if (_byte_count < 63) {
                return;
            }
#if RP_DEBUG_LEVEL
            // optionally spit out via mavlink the 63-bytes of cruft
            // that is spat out on device reset
            Debug(1, "Got RPLidar Information");
            char xbuffer[64]{};
            memcpy((void*)xbuffer, (void*)&_payload.information, 63);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar: (%s)", xbuffer);
#endif
            // 63 is the magic number of bytes in the spewed-out
            // reset data ... so now we'll just drop that stuff on
            // the floor.
            consume_bytes(63);
            send_request_for_device_info();
            _state = State::AWAITING_RESPONSE;
            continue;
        }
        case State::AWAITING_RESPONSE:
            if (_payload[0] != RPLIDAR_PREAMBLE) {
                // this is a protocol error.  Reset.
                reset();
                return;
            }

            // descriptor packet has 7 byte in total
            if (_byte_count < sizeof(_descriptor)) {
                return;
            }
            // identify the payload data after the descriptor
            static const _descriptor SCAN_DATA_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81 }
            };
            static const _descriptor HEALTH_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06 }
            };
            static const _descriptor DEVICE_INFO_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04 }
            };
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
            static const _descriptor EXPRESS_DATA_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 }
            };
#endif
            static const _descriptor SAMPLERATE_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x15 }
            };
            static const _descriptor DENSE_CAPSULE_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x54, 0x00, 0x00, 0x40, RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED }
            };
            Debug(2,"LIDAR descriptor found");
            if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
            } else if (memcmp((void*)&_payload[0], DENSE_CAPSULE_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DENSE_CAPSULE;
            } else if (memcmp((void*)&_payload[0], DEVICE_INFO_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DEVICE_INFO;
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
            } else if (_use_dense_express && memcmp((void*)&_payload[0], EXPRESS_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_EXPRESS_DATA;
                _express_stream_len = 0;
                _sync_error = 0;
                const uint16_t express_bytes = _byte_count - sizeof(_descriptor);
                if (express_bytes != 0) {
                    memcpy(_express_stream, &_payload[sizeof(_descriptor)], express_bytes);
                    _express_stream_len = express_bytes;
                }
                _byte_count = 0;
                break;
#endif
            } else if (memcmp((void*)&_payload[0], SAMPLERATE_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SAMPLE_RATE;
            } else {
                // unknown descriptor.  Ignore it.
            }
            consume_bytes(sizeof(_descriptor));
            break;

        case State::AWAITING_DEVICE_INFO:
            if (_byte_count < sizeof(_payload.device_info)) {
                return;
            }
            parse_response_device_info();
            consume_bytes(sizeof(_payload.device_info));
            break;

        case State::AWAITING_SCAN_DATA:
            if (_byte_count < sizeof(_payload.sensor_scan)) {
                return;
            }
            parse_response_data();
            consume_bytes(sizeof(_payload.sensor_scan));
            break;

        case State::AWAITING_DENSE_CAPSULE:
            if (_byte_count < RPLIDAR_DENSE_CAPSULE_BYTES) {
                return;
            }
            parse_response_dense_capsule();
            consume_bytes(RPLIDAR_DENSE_CAPSULE_BYTES);
            break;

        case State::AWAITING_HEALTH:
            if (_byte_count < sizeof(_payload.sensor_health)) {
                return;
            }
            parse_response_health();
            consume_bytes(sizeof(_payload.sensor_health));
            break;

#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
        case State::AWAITING_EXPRESS_DATA:
            break;  // handled above, before _payload read
#endif
        case State::AWAITING_SAMPLE_RATE:
            if (_byte_count < sizeof(_payload.sample_rate)) {
                return;
            }
            parse_response_sample_rate();
            consume_bytes(sizeof(_payload.sample_rate));
            break;

        case State::S3_WAIT_MOTOR_SETTLE:
            // Drained at function entry; state advances from update() after motor settle delay.
            break;
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_device_info()
{
    Debug(1, "Received DEVICE_INFO");
    const char *device_type = "UNKNOWN";
    const uint8_t model_id = _payload.device_info.model;
    switch (model_id) {
    case 0x18:
        model = Model::A1;
        device_type = "A1";
        break;
    case 0x28:
        model = Model::A2;
        device_type = "A2";
        break;
    case 0x2C:
        model = Model::A2M12;
        device_type = "A2M12";
        break;
    case 0x41:
        model=Model::C1;
        device_type="C1";
        break;
    case 0x61:
        model = Model::S1;
        device_type = "S1";
        break;
    case 0x71:
        model = Model::S2;
        device_type = "S2";
        break;
    case 0x81:
        model = Model::S3;
        device_type = "S3";
        break;
    default:
        // S3 and other S-series units encode major type in the high nibble (S3 major = 8 per Slamtec SDK / S-series protocol).
        if ((model_id >> 4U) == 8U) {
            model = Model::S3;
            device_type = "S3";
        } else if ((model_id >> 4U) == 6U) {
            model = Model::S1;
            device_type = "S1";
        } else {
            Debug(1, "Unknown device (%u)", model_id);
        }
        break;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar %s hw=%u fw=%u.%u", device_type, _payload.device_info.hardware, _payload.device_info.firmware_minor, _payload.device_info.firmware_major);
#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
    // enable Dense EXPRESS path
    _use_dense_express = (model == Model::S2);
    if (_use_dense_express) {
        _express_stream_len = 0;
        send_express_scan_request();
    } else if (model == Model::S3) {
        _s3_pending_gcs_connected_msg = true;
        if (model_id == 0x82) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: set this serial port to 460800 baud");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: set this serial port to 1000000 baud");
        }
        _s3_bootstrap = S3Bootstrap::SentHealth;
        send_request_for_health();
        _state = State::AWAITING_RESPONSE;
        return;
    } else {
        send_scan_mode_request();
    }
#else
    if (model == Model::S2) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar S2 ExpressScan disabled");
        return;
    }
    // S2/S3 UART default is 1 Mbps in Slamtec SDK; model 0x82 uses 460800.
    if (model == Model::S3) {
        _s3_pending_gcs_connected_msg = true;
        if (model_id == 0x82) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: set this serial port to 460800 baud");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: set this serial port to 1000000 baud");
        }
        _s3_bootstrap = S3Bootstrap::SentHealth;
        send_request_for_health();
        _state = State::AWAITING_RESPONSE;
        return;
    }

    send_scan_mode_request();
#endif
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::handle_proximity_sample(const float angle_deg, const float distance_m)
{
    const float dm = MIN(distance_m, distance_max_m());
    _last_distance_received_ms = AP_HAL::millis();
    if (!ignore_reading(angle_deg, dm)) {
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

        if (face != _last_face) {
            if (_last_distance_valid) {
                frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
            } else {
                frontend.boundary.reset_face(face, state.instance);
            }

            _last_face = face;
            _last_distance_valid = false;
        }
        if (dm > distance_min()) {
            if (!_last_distance_valid || (dm < _last_distance_m)) {
                _last_distance_m = dm;
                _last_distance_valid = true;
                _last_angle_deg = angle_deg;
            }
            database_push(_last_angle_deg, _last_distance_m);
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_data()
{
    if (_sync_error) {
        // out of 5-byte sync mask -> catch new revolution
        Debug(1, "       OUT OF SYNC");
        // on first revolution bit 1 = 1, bit 2 = 0 of the first byte
        if ((_payload[0] & 0x03) == 0x01) {
            _sync_error = 0;
            Debug(1, "                  RESYNC");
        } else {
            return;
        }
    }
    Debug(2, "UART %02x %02x%02x %02x%02x", _payload[0], _payload[2], _payload[1], _payload[4], _payload[3]); //show HEX values
    // check if valid SCAN packet: a valid packet starts with startbits which are complementary plus a checkbit in byte+1
    if (!((_payload.sensor_scan.startbit == !_payload.sensor_scan.not_startbit) && _payload.sensor_scan.checkbit)) {
        Debug(1, "Invalid Payload");
        _sync_error++;
        return;
    }

    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;
    const float angle_deg = wrap_360(_payload.sensor_scan.angle_q6/64.0f * angle_sign + params.yaw_correction);
    const float distance_m = (_payload.sensor_scan.distance_q2/4000.0f);
#if RP_DEBUG_LEVEL >= 2
    const float quality = _payload.sensor_scan.quality;
    Debug(2, "   D%02.2f A%03.1f Q%0.2f", distance_m, angle_deg, quality);
#endif
    handle_proximity_sample(angle_deg, distance_m);
}

void AP_Proximity_RPLidarA2::parse_response_dense_capsule()
{
    const uint8_t *b = &_payload[0];

    if ((b[0] >> 4) != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ||
        (b[1] >> 4) != RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
        _s3_dense_have_prev = false;
        return;
    }

    const uint8_t recv_chksum = (b[0] & 0x0FU) | ((b[1] & 0x0FU) << 4);
    uint8_t calc = 0;
    for (uint16_t i = 2; i < RPLIDAR_DENSE_CAPSULE_BYTES; i++) {
        calc ^= b[i];
    }
    if (recv_chksum != calc) {
        _s3_dense_have_prev = false;
        Debug(1, "dense capsule checksum err");
        return;
    }

    const _dense_capsule_meas * const curr = reinterpret_cast<const _dense_capsule_meas *>(b);
    const uint16_t start_cur = curr->start_angle_sync_q6;

    if ((start_cur & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) != 0U) {
        _s3_dense_have_prev = false;
        _s3_dense_last_sync = 0;
    }

    if (_s3_dense_have_prev) {
        const _dense_capsule_meas * const prev = reinterpret_cast<const _dense_capsule_meas *>(_s3_dense_prev_payload);

        const int32_t prevStart_q8 = int32_t((prev->start_angle_sync_q6 & 0x7FFFU) << 2);
        const int32_t currStart_q8 = int32_t((start_cur & 0x7FFFU) << 2);

        int32_t diff_q8 = currStart_q8 - prevStart_q8;
        if (prevStart_q8 > currStart_q8) {
            diff_q8 += int32_t(360 << 8);
        }
        uint32_t sus = _s3_express_sample_us;
        if (sus < 20U) {
            sus = 55U;
        }
        const int64_t maxDiff_q8 = (int64_t(360) * 100LL * 40LL / (1000000LL / int64_t(sus))) << 8;
        if (int64_t(diff_q8) > maxDiff_q8) {
            memcpy(_s3_dense_prev_payload, b, RPLIDAR_DENSE_CAPSULE_BYTES);
            _s3_dense_have_prev = true;
            return;
        }

        const int32_t angleInc_q16 = (diff_q8 << 8) / 40;
        int32_t currentAngle_raw_q16 = (prevStart_q8 << 8);

        const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;

        for (unsigned pos = 0; pos < 40U; pos++) {
            const uint16_t dist_mm = prev->cabins[pos].distance_mm;

            const int32_t angle_q6 = (currentAngle_raw_q16 >> 10);
            int32_t syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16 << 1)) ? 1 : 0;
            syncBit = (syncBit ^ _s3_dense_last_sync) & syncBit;
            _s3_dense_last_sync = (int8_t)syncBit;

            currentAngle_raw_q16 += angleInc_q16;

            int32_t aq = angle_q6;
            if (aq < 0) {
                aq += (360 << 6);
            }
            if (aq >= (360 << 6)) {
                aq -= (360 << 6);
            }

            if (dist_mm != 0U) {
                const float angle_deg = wrap_360((aq / 64.0f) * angle_sign + params.yaw_correction);
                const float dist_m = float(dist_mm) * 0.001f;
                handle_proximity_sample(angle_deg, dist_m);
            }
        }
    }

    memcpy(_s3_dense_prev_payload, b, RPLIDAR_DENSE_CAPSULE_BYTES);
    _s3_dense_have_prev = true;
}

void AP_Proximity_RPLidarA2::parse_response_health()
{
    const uint8_t st = _payload.sensor_health.status;
    _last_distance_received_ms = AP_HAL::millis();

    if (st == 0U) {
        Debug(1, "LIDAR Healthy");
    } else if (st == 1U) {
        if (model == Model::S3) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: GET_HEALTH warning ec=%u", (unsigned)_payload.sensor_health.error_code);
        }
        Debug(1, "LIDAR Warning");
    } else if (st >= 2U) {
        if (model == Model::S3) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar S3: GET_HEALTH err st=%u ec=%u", (unsigned)st, (unsigned)_payload.sensor_health.error_code);
        } else if (st == 3U) {
            Debug(1, "LIDAR Error");
        }
    }

    if (model == Model::S3 && _s3_bootstrap == S3Bootstrap::SentHealth) {
        send_request_for_sample_rate();
        _s3_bootstrap = S3Bootstrap::SentSampleRate;
        _state = State::AWAITING_RESPONSE;
    }
}

void AP_Proximity_RPLidarA2::parse_response_sample_rate()
{
    _last_distance_received_ms = AP_HAL::millis();
    _s3_express_sample_us = _payload.sample_rate.express_sample_duration_us;

    if (model == Model::S3) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar S3: sample std=%uus expr=%uus",
                      (unsigned)_payload.sample_rate.std_sample_duration_us,
                      (unsigned)_payload.sample_rate.express_sample_duration_us);
    }

    if (model == Model::S3 && _s3_bootstrap == S3Bootstrap::SentSampleRate) {
        send_s3_motor_rpm(RPLIDAR_S3_TARGET_MOTOR_RPM);
        _s3_bootstrap = S3Bootstrap::SentMotor;
        _state = State::S3_WAIT_MOTOR_SETTLE;
        _s3_deadline_ms = AP_HAL::millis() + RPLIDAR_S3_MOTOR_SETTLE_MS;
    }
}

#if AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED
// verify Dense capsulated (Express) block checksum
bool AP_Proximity_RPLidarA2::verify_cabin_checksum(const uint8_t *buf, size_t len)
{
    if (buf == nullptr) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = sizeof(_express_header); i < len; i++) {
        checksum ^= buf[i];
    }

    const uint8_t expected_b0 = uint8_t((EXPRESS_SYNC1 << 4) | (checksum & 0x0F));
    const uint8_t expected_b1 = uint8_t((EXPRESS_SYNC2 << 4) | ((checksum >> 4) & 0x0F));

    return (buf[0] == expected_b0) && (buf[1] == expected_b1);
}


// parse Dense capsulated Express block
void AP_Proximity_RPLidarA2::parse_response_express(const uint8_t *buf)
{
    if (buf == nullptr) {
        return;
    }

    _last_distance_received_ms = AP_HAL::millis();

    // start_angle_q6: 16-bit, Q6 format at bytes [2..3]
    const uint16_t start_angle_q6 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    const float start_angle_deg_raw = start_angle_q6 / 64.0f; // Q6 -> degrees

    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;

    // cabins: 40 cabins * 2-byte distance (mm) starting at buf[4]
    const uint8_t *cab = buf + 4;
    static constexpr int NUM_CABINS = 40;
    // S2 Dense Express default: 32000 samples/s at 10Hz = 0.1125 deg/cabin
    static constexpr float DENSE_CABIN_ANGLE_STEP_DEG = 0.1125f;

    // find the shortest valid cabin; rescan only if that cabin is ignored
    const float dist_min_m_local = distance_min_m();
    const uint8_t *cab_start = cab;
    bool have_min = false;
    float min_distance_m = 0.0f;
    int min_i = 0;

    for (int i = 0; i < NUM_CABINS; i++) {
        const uint16_t dist_mm = (uint16_t)cab[0] | ((uint16_t)cab[1] << 8);
        cab += 2;
        if (dist_mm == 0) {
            continue;
        }
        const float distance_m = dist_mm * 0.001f;
        if (distance_m <= dist_min_m_local) {
            continue;
        }
        if (!have_min || distance_m < min_distance_m) {
            have_min = true;
            min_distance_m = distance_m;
            min_i = i;
        }
    }

    if (!have_min) {
        return;
    }

    const float min_cab_angle_raw = start_angle_deg_raw + min_i * DENSE_CABIN_ANGLE_STEP_DEG;
    float min_angle_deg = wrap_360(min_cab_angle_raw * angle_sign + params.yaw_correction);

    if (ignore_reading(min_angle_deg, min_distance_m)) {
        // fallback: rescan with per-cabin ignore filter
        have_min = false;
        cab = cab_start;

        for (int i = 0; i < NUM_CABINS; i++) {
            const uint16_t dist_mm = (uint16_t)cab[0] | ((uint16_t)cab[1] << 8);
            cab += 2;
            if (dist_mm == 0) {
                continue;
            }
            const float distance_m = dist_mm * 0.001f;
            if (distance_m <= dist_min_m_local) {
                continue;
            }
            const float cab_angle_raw = start_angle_deg_raw + i * DENSE_CABIN_ANGLE_STEP_DEG;
            const float angle_deg = wrap_360(cab_angle_raw * angle_sign + params.yaw_correction);
            if (ignore_reading(angle_deg, distance_m)) {
                continue;
            }
            if (!have_min || distance_m < min_distance_m) {
                have_min = true;
                min_distance_m = distance_m;
                min_angle_deg = angle_deg;
            }
        }

        if (!have_min) {
            return;
        }
    }

    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(min_angle_deg);

    if (face != _last_face) {
        // distance is for a new face, the previous one can be updated now
        if (_last_distance_valid) {
            frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
        } else {
            // reset distance from last face
            frontend.boundary.reset_face(face, state.instance);
        }

        // initialize the new face
        _last_face = face;
        _last_distance_valid = false;
    }

    // update shortest distance for this face
    if (!_last_distance_valid || (min_distance_m < _last_distance_m)) {
        _last_distance_m = min_distance_m;
        _last_distance_valid = true;
        _last_angle_deg = min_angle_deg;
    }
    // update OA database
    database_push(_last_angle_deg, _last_distance_m);
}

bool AP_Proximity_RPLidarA2::express_time_exceeded(uint32_t start_us) const
{
    static constexpr uint32_t MAX_US = 1000;
    return (AP_HAL::micros() - start_us) > MAX_US;
}

void AP_Proximity_RPLidarA2::ensure_express_stream_space(uint16_t need)
{
    if (_express_stream_len + need <= EXPRESS_STREAM_BUFFER_SIZE) {
        return;
    }

    uint16_t keep = EXPRESS_STREAM_BUFFER_SIZE / 2;
    keep = MIN(keep, _express_stream_len);

    if (keep == 0) {
        _express_stream_len = 0;
        _sync_error = 0;
        Debug(1, "EXPRESS stream overflow, dropping all data");
        return;
    }

    const uint16_t src = _express_stream_len - keep;
    if (src != 0) {
        memmove(_express_stream, _express_stream + src, keep);
    }
    _express_stream_len = keep;
    _sync_error = 0;
    Debug(1, "EXPRESS stream overflow, dropping old data");
}

void AP_Proximity_RPLidarA2::handle_express_data()
{
    const uint32_t start_us = AP_HAL::micros();

    uint16_t bytes_read_total = 0;

    while (_uart->available() && bytes_read_total < EXPRESS_MAX_BYTES_CONSUME) {
        if (express_time_exceeded(start_us)) {
            break;
        }

        const uint16_t remaining = uint16_t(EXPRESS_MAX_BYTES_CONSUME - bytes_read_total);
        uint16_t space = EXPRESS_STREAM_BUFFER_SIZE - _express_stream_len;
        if (space == 0) {
            ensure_express_stream_space(1);
            space = EXPRESS_STREAM_BUFFER_SIZE - _express_stream_len;
            if (space == 0) {
                break;
            }
        }
        const uint16_t to_read = MIN(space, remaining);

        if (to_read == 0) {
            break;
        }

        const uint16_t n = _uart->read(_express_stream + _express_stream_len, to_read);
        if (n == 0) {
            break;
        }

        _express_stream_len += n;
        bytes_read_total += n;
    }

    // scan the stream buffer for valid headers and process full blocks
    uint16_t idx = 0;

    while (_express_stream_len >= idx + 2) {
        if (express_time_exceeded(start_us)) {
            break;
        }

        // search for a header candidate (upper nibbles 0xA / 0x5)
        while (idx + 1 < _express_stream_len) {
            if (express_time_exceeded(start_us)) {
                break;
            }
            const uint8_t b0 = _express_stream[idx];
            const uint8_t b1 = _express_stream[idx + 1];
            if ((b0 >> 4) == EXPRESS_SYNC1 && (b1 >> 4) == EXPRESS_SYNC2) {
                // header candidate found
                break;
            }
            idx++;
        }

        if (express_time_exceeded(start_us)) {
            break;
        }

        if (idx + 1 >= _express_stream_len) {
            // no header candidate found, drop all accumulated data
            if (_express_stream_len > 0) {
                Debug(1, "EXPRESS: no header candidate, dropping %u bytes", unsigned(_express_stream_len));
                _express_stream[0] = _express_stream[_express_stream_len - 1];
                _express_stream_len = 1;
            } else {
                _express_stream_len = 0;
            }
            _sync_error = 0;
            return;
        }

        // idx now points to a header candidate
        if (_express_stream_len - idx < EXPRESS_BLOCK_SIZE) {
            // not enough bytes for a full block
            // move remaining bytes to the front and wait for more data
            if (idx > 0) {
                memmove(_express_stream,
                        _express_stream + idx,
                        _express_stream_len - idx);
                _express_stream_len -= idx;
            }
            return;
        }

        // pointer to the candidate block
        uint8_t *blk = _express_stream + idx;

        if (!verify_cabin_checksum(blk, EXPRESS_BLOCK_SIZE)) {
            // upper nibbles match but checksum is invalid
            // advance by one byte and try to resynchronise
            Debug(1, "EXPRESS/DENSE checksum error");
            _sync_error++;

            if (_sync_error > 10) {
                reset_rplidar();
                _express_stream_len = 0;
                return;
            }

            idx++;
            continue;
        }

        // valid block found
        _sync_error = 0;
        parse_response_express(blk);

        // consume this block and look for the next one
        idx += EXPRESS_BLOCK_SIZE;
    }

    // compact any remaining unprocessed bytes to the front of the buffer
    if (idx > 0 && idx <= _express_stream_len) {
        const uint16_t remaining = _express_stream_len - idx;
        if (remaining > 0) {
            memmove(_express_stream,
                    _express_stream + idx,
                    remaining);
        }
        _express_stream_len = remaining;
    }
}
#endif // AP_PROXIMITY_RPLIDAR_EXPRESSSCAN_ENABLED

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
