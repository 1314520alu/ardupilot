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
 * 6) 再次 RESET 前若已识别为 S3，先发 STOP(0x25) 并丢掉 RX 里残留的扫描字节。
 * 7) Mission Planner: (a) STATUSTEXT is queued only when a MAVLink channel is active (see try_send_s3_gcs_pending).
 *    (b) Long strings are chunked; UTF-8 split mid-character shows as garbled text in MP — keep lines short ASCII.
 * 8) Dense 帧按字节重同步：sync 0xA/0x5 对上但 XOR 校验失败时只滑动 1 字节，不把整机 RESET。
 *    校验通过的胶囊（含全 0 距离）会刷新链路时间，避免空场约 10s 被 COMM 超时复位掐断扫描。
 *    健康/采样率/扫描描述符 500ms 无应答则重发，连续失败才 STOP+RESET。
 * 9) 以太网 S3M1-R2E 走 AP_Proximity_RPLidarS3UDP（PRXn_TYPE=19），协议状态机与本文件相同。
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
#include <AP_SerialManager/AP_SerialManager.h>

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

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82

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

AP_Proximity_RPLidarA2::AP_Proximity_RPLidarA2(AP_Proximity &_frontend,
        AP_Proximity::Proximity_State &prox_state,
        AP_Proximity_Params &_params,
        uint8_t serial_instance) :
    AP_Proximity_Backend_Serial(_frontend, prox_state, _params, serial_instance)
{
    // Backend_Serial::begin() runs inside the base constructor, where this
    // override of rxspace() is not yet active. Re-begin so S3's ~70 KB/s
    // dense stream has an 8 KB RX buffer. A1/A2/S1 use the same port and
    // only gain spare RAM.
    if (_uart != nullptr) {
        const uint32_t baud = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, serial_instance);
        _uart->begin(baud, rxspace(), 0);
    }
}

AP_Proximity_RPLidarA2::AP_Proximity_RPLidarA2(AP_Proximity &_frontend,
        AP_Proximity::Proximity_State &prox_state,
        AP_Proximity_Params &_params) :
    AP_Proximity_Backend_Serial(_frontend, prox_state, _params)
{
}

uint32_t AP_Proximity_RPLidarA2::transport_available() const
{
    if (_uart == nullptr) {
        return 0;
    }
    return _uart->available();
}

uint32_t AP_Proximity_RPLidarA2::transport_read(uint8_t *buf, uint32_t maxlen)
{
    if (_uart == nullptr || buf == nullptr || maxlen == 0U) {
        return 0;
    }
    const uint16_t n = uint16_t(MIN(maxlen, uint32_t(0xFFFFU)));
    const ssize_t got = _uart->read(buf, n);
    if (got <= 0) {
        return 0;
    }
    return uint32_t(got);
}

void AP_Proximity_RPLidarA2::transport_write(const uint8_t *buf, uint32_t len)
{
    if (_uart == nullptr || buf == nullptr || len == 0U) {
        return;
    }
    _uart->write(buf, len);
}

void AP_Proximity_RPLidarA2::transport_discard_rx()
{
    if (_uart != nullptr) {
        _uart->discard_input();
    }
}

void AP_Proximity_RPLidarA2::note_transport_connected()
{
    // UDP has no boot banner to wait out. Backdate the reset timestamp so
    // this same update sends GET_DEVICE_INFO. UART never calls this.
    const uint32_t now = AP_HAL::millis();
    _last_reset_ms = now - 3001U;
}

void AP_Proximity_RPLidarA2::s3_note_step()
{
    _s3_step_ms = AP_HAL::millis();
}

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
    if (!transport_ready()) {
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

    // Ethernet S3E may not emit the UART boot banner. If GET_INFO never
    // arrives, still bring the known S3 express path up.
    if (prefer_s3_express() && model == Model::UNKNOWN && !_s3_forced_express &&
        (now_ms - _last_reset_ms) > 4000U &&
        (_state == State::RESET || _state == State::AWAITING_RESPONSE)) {
        _s3_forced_express = true;
        model = Model::S3;
        _s3_bootstrap = S3Bootstrap::SentHealth;
        _byte_count = 0;
        send_request_for_health();
        _state = State::AWAITING_RESPONSE;
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
    if (now_ms - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::NoData);
        Debug(1, "LIDAR NO DATA");
        if (model == Model::S3) {
            // Do not RESET a live dense stream on the legacy 10s timer.
            // s3_manage_link retries the current step, then soft-restarts
            // EXPRESS, and only power-cycles the protocol after a long stall.
            s3_manage_link(now_ms);
        } else if (now_ms - _last_reset_ms > 10000) {
            reset_rplidar();
        }
    } else {
        set_status(AP_Proximity::Status::Good);
        _s3_step_retries = 0;
    }
}

// get maximum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_max() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
        return 8.0f;
    case Model::A2:
        return 16.0f;
    case Model::C1:
        return 12.0f;
    case Model::S1:
        return 40.0f;
    case Model::S3:
        return 40.0f;
    }
    return 0.0f;
}

// get minimum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_min() const
{
    switch (model) {
    case Model::UNKNOWN:
        return 0.0f;
    case Model::A1:
    case Model::A2:
    case Model::C1:
    case Model::S1:
    case Model::S3:
        return 0.2f;
    }
    return 0.0f;
}

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    if (model == Model::S3) {
        send_stop();
        transport_discard_rx();
    }
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    transport_write(tx_buffer, 2);
    Debug(1, "LIDAR reset");
    // To-Do: ensure delay of 8m after sending reset request
    _last_reset_ms =  AP_HAL::millis();
    reset();
    s3_note_step();
}

// set Lidar into legacy SCAN mode (non-S3, or S3 fallback path if device_info used legacy scan)
void AP_Proximity_RPLidarA2::send_scan_mode_request()
{
    const uint8_t cmd = RPLIDAR_CMD_SCAN;
    const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, cmd};
    transport_write(tx_buffer, 2);
    Debug(1, "Sent SCAN mode request");
    s3_note_step();
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
    _s3_scan_requested = true;
    _s3_bootstrap = S3Bootstrap::None;
    s3_note_step();
}

void AP_Proximity_RPLidarA2::send_stop()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_STOP};
    transport_write(tx_buffer, 2);
    Debug(1, "Sent STOP");
}

void AP_Proximity_RPLidarA2::send_command_with_payload_xor(const uint8_t cmd, const void *payload, const uint8_t payload_len)
{
    uint8_t buf[3U + 32U + 1U];
    const uint8_t n = AP_RPLidar::build_payload_cmd(buf, sizeof(buf), cmd,
                      static_cast<const uint8_t *>(payload), payload_len);
    if (n == 0U) {
        return;
    }
    transport_write(buf, n);
}

void AP_Proximity_RPLidarA2::send_s3_motor_rpm(const uint16_t rpm)
{
    send_command_with_payload_xor(RPLIDAR_CMD_HQ_MOTOR_SPEED, &rpm, sizeof(rpm));
    Debug(1, "Sent S3 motor RPM %u", unsigned(rpm));
}

// send request for sensor health
void AP_Proximity_RPLidarA2::send_request_for_health()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    transport_write(tx_buffer, 2);
    Debug(1, "Sent health request");
    s3_note_step();
}

void AP_Proximity_RPLidarA2::send_request_for_sample_rate()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_SAMPLERATE};
    transport_write(tx_buffer, 2);
    Debug(1, "Sent sample rate request");
    s3_note_step();
}

// send request for device information
void AP_Proximity_RPLidarA2::send_request_for_device_info()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_INFO};
    transport_write(tx_buffer, 2);
    Debug(1, "Sent device information request");
    s3_note_step();
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
    _s3_bootstrap = S3Bootstrap::None;
    _s3_step_retries = 0;
    _s3_scan_requested = false;
    _s3_forced_express = false;
    AP_RPLidar::dense_reset(_dense);
    _s3_pending_gcs_connected_msg = false;
    _s3_pending_gcs_express_msg = false;
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

    // After STOP+RESET, scan bytes still in the FIFO can look like the
    // "RP..." banner. Drop them briefly; GET_INFO at 3s covers a missed banner.
    if (model == Model::S3 && _state == State::RESET &&
        (AP_HAL::millis() - _last_reset_ms) < 100U) {
        transport_discard_rx();
        _byte_count = 0;
        return;
    }

    if (_state == State::S3_WAIT_MOTOR_SETTLE) {
        uint8_t dump[128];
        while (transport_available() > 0U) {
            const uint32_t n = MIN(transport_available(), uint32_t(sizeof(dump)));
            if (transport_read(dump, n) == 0U) {
                break;
            }
        }
        return;
    }

    const uint32_t nbytes = transport_available();
    if (nbytes == 0) {
        return;
    }
    const uint32_t bytes_to_read = MIN(nbytes, uint32_t(sizeof(_payload) - _byte_count));
    if (bytes_to_read == 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        reset();
        return;
    }
    const uint32_t bytes_read = transport_read(&_payload[_byte_count], bytes_to_read);
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

        switch (_state) {
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
            char xbuffer[64] {};
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
                // Noise before a descriptor. Slide to the next 0xA5
                // instead of abandoning the session (that used to force a
                // hunt for the boot banner and, on S3, a 10s RESET).
                consume_bytes(1);
                break;
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
            static const _descriptor SAMPLERATE_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x15 }
            };
            static const _descriptor DENSE_CAPSULE_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x54, 0x00, 0x00, 0x40, RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED }
            };
            Debug(2,"LIDAR descriptor found");
            if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
                consume_bytes(sizeof(_descriptor));
            } else if (memcmp((void*)&_payload[0], DENSE_CAPSULE_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DENSE_CAPSULE;
                _s3_valid_capsule_ms = AP_HAL::millis();
                _last_distance_received_ms = _s3_valid_capsule_ms;
                consume_bytes(sizeof(_descriptor));
            } else if (memcmp((void*)&_payload[0], DEVICE_INFO_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DEVICE_INFO;
                consume_bytes(sizeof(_descriptor));
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
                consume_bytes(sizeof(_descriptor));
            } else if (memcmp((void*)&_payload[0], SAMPLERATE_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SAMPLE_RATE;
                consume_bytes(sizeof(_descriptor));
            } else {
                // 0xA5 was not the start of a known descriptor. Slide one byte.
                consume_bytes(1);
            }
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

        case State::AWAITING_DENSE_CAPSULE: {
            // Resync on the 0xA/0x5 nibbles. A checksum miss is a false
            // header: slide one byte (checksum fallback) and keep the
            // previous good capsule. Do not RESET the lidar here.
            const uint32_t start_us = AP_HAL::micros();
            uint8_t parsed = 0;
            while (_byte_count >= 2U) {
                if (parsed >= 24U || (AP_HAL::micros() - start_us) > 2000U) {
                    return;
                }
                if (!AP_RPLidar::dense_sync_ok(&_payload[0])) {
                    consume_bytes(1);
                    continue;
                }
                if (_byte_count < RPLIDAR_DENSE_CAPSULE_BYTES) {
                    return;
                }
                if (!AP_RPLidar::dense_checksum_ok(&_payload[0])) {
                    consume_bytes(1);
                    continue;
                }
                _s3_valid_capsule_ms = AP_HAL::millis();
                _last_distance_received_ms = _s3_valid_capsule_ms;
                parse_response_dense_capsule();
                consume_bytes(RPLIDAR_DENSE_CAPSULE_BYTES);
                parsed++;
            }
            return;
        }

        case State::AWAITING_HEALTH:
            if (_byte_count < sizeof(_payload.sensor_health)) {
                return;
            }
            parse_response_health();
            consume_bytes(sizeof(_payload.sensor_health));
            break;

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
    case 0x41:
        model=Model::C1;
        device_type="C1";
        break;
    case 0x61:
        model = Model::S1;
        device_type = "S1";
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
    if (model != Model::S3 && prefer_s3_express()) {
        // S3M1-R2E is this backend's only device. An unexpected model byte
        // still speaks Dense Express; record it as S3 so recovery stays on
        // the express path.
        model = Model::S3;
        device_type = "S3E";
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar %s hw=%u fw=%u.%u", device_type, _payload.device_info.hardware, _payload.device_info.firmware_minor, _payload.device_info.firmware_major);
    // S2/S3 UART default is 1 Mbps in Slamtec SDK; model 0x82 uses 460800.
    if (model == Model::S3) {
        _s3_pending_gcs_connected_msg = true;
        if (_uart != nullptr) {
            if (model_id == 0x82) {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: set this serial port to 460800 baud");
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RPLidar S3: set this serial port to 1000000 baud");
            }
        }
        _s3_bootstrap = S3Bootstrap::SentHealth;
        send_request_for_health();
        _state = State::AWAITING_RESPONSE;
        return;
    }

    send_scan_mode_request();
    _state = State::AWAITING_RESPONSE;
}

void AP_Proximity_RPLidarA2::handle_proximity_sample(const float angle_deg, const float distance_m)
{
    const float dm = MIN(distance_m, distance_max());
    _last_distance_received_ms = AP_HAL::millis();
    if (note_face_sample(angle_deg, dm)) {
        database_push(_last_angle_deg, _last_distance_m);
    }
}

bool AP_Proximity_RPLidarA2::note_face_sample(const float angle_deg, const float distance_m)
{
    if (ignore_reading(angle_deg, distance_m)) {
        return false;
    }
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
    if (!(distance_m > distance_min())) {
        return false;
    }
    if (!_last_distance_valid || (distance_m < _last_distance_m)) {
        _last_distance_m = distance_m;
        _last_distance_valid = true;
        _last_angle_deg = angle_deg;
    }
    return true;
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
    // Caller has already accepted the XOR checksum. Activity time is
    // refreshed there so an all-zero scan still counts as a live link.
    AP_RPLidar::Point pts[AP_RPLidar::DENSE_POINTS];
    const uint8_t n = AP_RPLidar::dense_emit(_dense, &_payload[0], pts, AP_RPLidar::DENSE_POINTS);
    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;

    float closest_m = 0.0f;
    float closest_ang = 0.0f;
    bool have_closest = false;
    for (uint8_t i = 0; i < n; i++) {
        const float angle_deg = wrap_360(pts[i].angle_deg * angle_sign + params.yaw_correction);
        const float dm = MIN(pts[i].distance_m, distance_max());
        if (!note_face_sample(angle_deg, dm)) {
            continue;
        }
        if (!have_closest || dm < closest_m) {
            have_closest = true;
            closest_m = dm;
            closest_ang = angle_deg;
        }
    }
    // One OA push per capsule. Pushing all ~32k ranges/s stalls the
    // flight loop, the UART overruns, and the 10s reset then kills scan.
    if (have_closest) {
        Vector3f current_pos;
        Matrix3f body_to_ned;
        if (database_prepare_for_push(current_pos, body_to_ned)) {
            database_push(closest_ang, 0.0f, closest_m, AP_HAL::millis(), current_pos, body_to_ned);
        }
    }
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
    _dense.express_sample_us = _payload.sample_rate.express_sample_duration_us;

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

bool AP_Proximity_RPLidarA2::s3_soft_restart()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _s3_soft_restart_ms < 1000U) {
        return false;
    }
    _s3_soft_restart_ms = now;
    send_stop();
    transport_discard_rx();
    _byte_count = 0;
    _dense.have_prev = false;
    _dense.last_sync = 0;
    _state = State::S3_WAIT_MOTOR_SETTLE;
    _s3_deadline_ms = now + RPLIDAR_S3_MOTOR_SETTLE_MS;
    _s3_bootstrap = S3Bootstrap::None;
    s3_note_step();
    return true;
}

void AP_Proximity_RPLidarA2::s3_manage_link(const uint32_t now_ms)
{
    const uint32_t last_ok = MAX(_last_distance_received_ms, _s3_valid_capsule_ms);
    if ((now_ms - last_ok) > 8000U && (now_ms - _last_reset_ms) > 8000U) {
        reset_rplidar();
        return;
    }

    const bool waiting = _state == State::AWAITING_RESPONSE ||
                         _state == State::AWAITING_HEALTH ||
                         _state == State::AWAITING_DEVICE_INFO ||
                         _state == State::AWAITING_SAMPLE_RATE;
    if (waiting && (now_ms - _s3_step_ms) > 500U) {
        if (_s3_step_retries >= 3U) {
            if (now_ms - _last_reset_ms > 2000U) {
                reset_rplidar();
            }
            return;
        }
        _byte_count = 0;
        bool sent = false;
        switch (_s3_bootstrap) {
        case S3Bootstrap::SentHealth:
            send_request_for_health();
            _state = State::AWAITING_RESPONSE;
            sent = true;
            break;
        case S3Bootstrap::SentSampleRate:
            send_request_for_sample_rate();
            _state = State::AWAITING_RESPONSE;
            sent = true;
            break;
        case S3Bootstrap::SentMotor:
            sent = s3_soft_restart();
            break;
        case S3Bootstrap::None:
        default:
            if (_s3_scan_requested) {
                sent = s3_soft_restart();
            } else {
                send_request_for_device_info();
                _state = State::AWAITING_RESPONSE;
                sent = true;
            }
            break;
        }
        if (sent) {
            _s3_step_retries++;
        }
        return;
    }

    if ((_state == State::AWAITING_DENSE_CAPSULE || _state == State::AWAITING_SCAN_DATA) &&
        (now_ms - last_ok) > 1000U) {
        s3_soft_restart();
    }
}

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
