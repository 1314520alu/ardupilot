


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
 * ArduPilot device driver for SLAMTEC RPLIDAR A2 (16m range version)
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_RPLidarA2.h"

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_RPLidarA2.h"
#include <AP_InternalError/AP_InternalError.h>

#include <ctype.h>
#include <stdio.h>

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
#define RPLIDAR_CMD_GET_SAMPLERATE     0x59

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82
#define RPLIDAR_CMD_GET_LIDAR_CONF     0x84
#define RPLIDAR_CMD_MOTOR_SPEED_CTRL   0xA8

extern const AP_HAL::HAL& hal;

void AP_Proximity_RPLidarA2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // request device info 3sec after reset
    // required for S1 support that sends only 9 bytes after a reset (A1,A2 send 63)
    uint32_t now_ms = AP_HAL::millis();
    if ((_state == State::RESET) && (now_ms - _last_reset_ms > 3000)) {
        send_request_for_device_info();
        _state = State::AWAITING_RESPONSE;
        _byte_count = 0;
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
    case Model::S2:
        return 30.0f;
    case Model::S3:
        return 50.0f;
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
    case Model::S2:
    case Model::S3:
        return 0.2f;
    }
    return 0.0f;
}

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    Debug(1, "LIDAR reset");
    // To-Do: ensure delay of 8m after sending reset request
    _last_reset_ms =  AP_HAL::millis();
    reset();
}

// set Lidar into SCAN mode
void AP_Proximity_RPLidarA2::send_scan_mode_request()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent scan mode request");
}

// send request for sensor health
void AP_Proximity_RPLidarA2::send_request_for_health()                                    //not called yet
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent health request");
}

// send request for device information
void AP_Proximity_RPLidarA2::send_request_for_device_info()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_INFO};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent device information request");
}

// send STOP command to halt scanning
void AP_Proximity_RPLidarA2::send_stop_scan()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_STOP};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent STOP command");
}

// send MOTOR_SPEED_CTRL command to set motor RPM
void AP_Proximity_RPLidarA2::send_motor_speed_ctrl(uint16_t rpm)
{
    // MOTOR_SPEED_CTRL request format:
    // A5 A8 [payload_len=2] [rpm_low] [rpm_high] [checksum]
    // Checksum = 0xA5 ^ 0xA8 ^ 0x02 ^ rpm_low ^ rpm_high
    
    uint8_t rpm_low = rpm & 0xFF;
    uint8_t rpm_high = (rpm >> 8) & 0xFF;
    
    uint8_t checksum = 0xA5 ^ 0xA8 ^ 0x02 ^ rpm_low ^ rpm_high;
    
    uint8_t tx_buffer[6] {
        RPLIDAR_PREAMBLE,
        RPLIDAR_CMD_MOTOR_SPEED_CTRL,
        0x02,           // payload length
        rpm_low,        // RPM low byte
        rpm_high,       // RPM high byte
        checksum        // checksum
    };
    
    _uart->write(tx_buffer, 6);
    
    float hz = rpm / 60.0f;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Set motor speed: %u RPM (%.1f Hz)", rpm, hz);
    Debug(1, "Sent MOTOR_SPEED_CTRL: %u RPM (%.1f Hz)", rpm, hz);
}

// send request for EXPRESS_SCAN (high-speed scanning mode)
void AP_Proximity_RPLidarA2::send_express_scan_request()
{
    // EXPRESS_SCAN requires 5 bytes of payload: working_mode + 4 reserved bytes
    uint8_t tx_buffer[7] {
        RPLIDAR_PREAMBLE,
        RPLIDAR_CMD_EXPRESS_SCAN,
        5,          // payload length
        _working_mode,  // working mode (from GET_LIDAR_CONF or default 0)
        0, 0, 0     // reserved bytes
    };
    
    // Calculate checksum according to protocol
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 6; i++) {
        checksum ^= tx_buffer[i];
    }
    tx_buffer[6] = checksum;
    
    _uart->write(tx_buffer, 7);
    
    if (_working_mode == 0) {
        Debug(1, "Sent EXPRESS_SCAN request (mode=%u, legacy)", _working_mode);
    } else {
        Debug(1, "Sent EXPRESS_SCAN request (mode=%u, optimal)", _working_mode);
    }
}

// send request for LIDAR configuration
void AP_Proximity_RPLidarA2::send_request_for_lidar_conf()
{
    // GET_LIDAR_CONF with type 0x7C to query recommended scan mode
    // According to protocol: type 0x7C returns recommended mode ID (uint16)
    static const uint8_t tx_buffer[7] {
        RPLIDAR_PREAMBLE,
        RPLIDAR_CMD_GET_LIDAR_CONF,
        4,          // payload length (type field)
        0x7C, 0, 0, 0  // type 0x7C = recommended scan mode
    };
    _uart->write(tx_buffer, 7);
    Debug(1, "Sent GET_LIDAR_CONF request (type=0x7C for recommended mode)");
}

// send request for sample rate information
void AP_Proximity_RPLidarA2::send_request_for_samplerate()
{
    static const uint8_t tx_buffer[2] {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_SAMPLERATE};
    _uart->write(tx_buffer, 2);
    Debug(1, "Sent GET_SAMPLERATE request");
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
    // Send STOP command before resetting to gracefully halt scanning
    if (_state == State::AWAITING_SCAN_DATA || 
        _state == State::AWAITING_EXPRESS_SCAN_DATA) {
        send_stop_scan();
        hal.scheduler->delay(10);  // Wait at least 10ms as per protocol
    }
    
    _state = State::RESET;
    _byte_count = 0;
    _is_s3_mode = false;
    _working_mode = 0;
    _best_scan_mode_id = 0;
    _last_scan_start_ms = 0;
    _scan_frequency_hz = 0.0f;
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
            
            // 按照官方推荐流程：先检查健康状态
            send_request_for_health();
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
            static const _descriptor EXPRESS_SCAN_DESCRIPTOR[] {
                { RPLIDAR_PREAMBLE, 0x5A, 0x54, 0x00, 0x00, 0x40, 0x85 }
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
            Debug(2,"LIDAR descriptor found");
            if (memcmp((void*)&_payload[0], EXPRESS_SCAN_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_EXPRESS_SCAN_DATA;
                _is_s3_mode = true;
            } else if (memcmp((void*)&_payload[0], SCAN_DATA_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SCAN_DATA;
            } else if (memcmp((void*)&_payload[0], DEVICE_INFO_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_DEVICE_INFO;
            } else if (memcmp((void*)&_payload[0], HEALTH_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_HEALTH;
            } else if (memcmp((void*)&_payload[0], SAMPLERATE_DESCRIPTOR, sizeof(_descriptor)) == 0) {
                _state = State::AWAITING_SAMPLERATE;
            } else {
                // unknown descriptor. Check if it's LIDAR_CONF (variable length)
                if (_payload[0] == RPLIDAR_PREAMBLE && _payload[1] == 0x5A && _payload[6] == 0x20) {
                    _state = State::AWAITING_LIDAR_CONF;
                }
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

        case State::AWAITING_HEALTH:
            if (_byte_count < sizeof(_payload.sensor_health)) {
                return;
            }
            parse_response_health();
            consume_bytes(sizeof(_payload.sensor_health));
            
            // 根据健康状态决定下一步
            // status: 0=正常, 1=警告, 2=错误, 3=硬件故障
            if (_payload.sensor_health.status >= 2) {
                // 如果有错误或故障，执行RESET
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPLidar health issue (status=%u), resetting...", 
                              _payload.sensor_health.status);
                reset_rplidar();
                _last_reset_ms = AP_HAL::millis();
                _state = State::RESET;
            } else {
                // 健康状态正常，继续获取设备信息
                send_request_for_device_info();
                _state = State::AWAITING_RESPONSE;
            }
            break;

        case State::AWAITING_EXPRESS_SCAN_DATA:
            if (_byte_count < sizeof(_payload.express_scan)) {
                return;
            }
            parse_response_express_scan();
            consume_bytes(sizeof(_payload.express_scan));
            break;

        case State::AWAITING_SAMPLERATE:
            if (_byte_count < sizeof(_payload.samplerate)) {
                return;
            }
            parse_response_samplerate();
            consume_bytes(sizeof(_payload.samplerate));
            // After getting sample rate, try EXPRESS_SCAN
            send_express_scan_request();
            _state = State::AWAITING_RESPONSE;
            break;

        case State::AWAITING_LIDAR_CONF:
            // GET_LIDAR_CONF response has variable length; minimum 4 bytes for type field
            if (_byte_count < 4) {
                return;
            }
            parse_response_lidar_conf();
            // For now, skip variable-length parsing and move to EXPRESS_SCAN
            // In a more complete implementation, would parse the full response
            consume_bytes(MIN(_byte_count, (uint16_t)sizeof(_payload.lidar_conf)));
            break;
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_device_info()
{
    Debug(1, "Received DEVICE_INFO");
    const char *device_type = "UNKNOWN";
    const char *mode_name = "SCAN";  // 默认模式
    
    switch (_payload.device_info.model) {
    case 0x18:
        model = Model::A1;
        device_type = "A1";
        break;
    case 0x28:
        model = Model::A2;
        device_type = "A2";
        break;
    case 0x41:
        model = Model::C1;
        device_type = "C1";
        break;
    case 0x61:
        model = Model::S1;
        device_type = "S1";
        break;
    case 0x71:
        model = Model::S2;
        device_type = "S2";
        _is_s3_mode = true;  // S-series use EXPRESS_SCAN
        mode_name = "EXPRESS_SCAN";
        break;
    case 0x81:
        model = Model::S3;
        device_type = "S3";
        _is_s3_mode = true;  // S3 supports EXPRESS_SCAN
        mode_name = "EXPRESS_SCAN";
        break;
    default:
        Debug(1, "Unknown device (%u)", _payload.device_info.model);
    }
    
    // 在Mission Planner消息窗口显示雷达型号和固件信息
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RPLidar %s detected | HW=%u FW=%u.%u | %s",
                  device_type,
                  _payload.device_info.hardware,
                  _payload.device_info.firmware_major,
                  _payload.device_info.firmware_minor,
                  mode_name);
    
    // If S-series, follow official workflow: GET_LIDAR_CONF → EXPRESS_SCAN with best mode
    if (_is_s3_mode) {
        // Query LIDAR configuration to get best scan mode
        send_request_for_lidar_conf();
        _state = State::AWAITING_RESPONSE;
    } else {
        // For non-S-series, use traditional SCAN mode
        send_scan_mode_request();
        _state = State::AWAITING_RESPONSE;
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
    
    // 检测新的一圈扫描开始（startbit=1），计算实际扫描频率
    if (_payload.sensor_scan.startbit == 1) {
        uint32_t current_ms = AP_HAL::millis();
        if (_last_scan_start_ms > 0) {
            uint32_t delta_ms = current_ms - _last_scan_start_ms;
            if (delta_ms > 0) {
                // 计算实际扫描频率
                _scan_frequency_hz = 1000.0f / delta_ms;
                
                // 每10圈输出一次频率信息到Mission Planner
                static uint8_t freq_report_counter = 0;
                freq_report_counter++;
                if (freq_report_counter >= 10) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scan Freq: %.2f Hz (%.1f RPM)", 
                                  _scan_frequency_hz, 
                                  _scan_frequency_hz * 60.0f);
                    freq_report_counter = 0;
                }
            }
        }
        _last_scan_start_ms = current_ms;
    }
    
#if RP_DEBUG_LEVEL >= 2
    const float quality = _payload.sensor_scan.quality;
    Debug(2, "   D%02.2f A%03.1f Q%0.2f", distance_m, angle_deg, quality);
#endif
    _last_distance_received_ms = AP_HAL::millis();
    if (!ignore_reading(angle_deg, distance_m)) {
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

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
        if (distance_m > distance_min()) {
            // update shortest distance
            if (!_last_distance_valid || (distance_m < _last_distance_m)) {
                _last_distance_m = distance_m;
                _last_distance_valid = true;
                _last_angle_deg = angle_deg;
            }
            // update OA database
            database_push(_last_angle_deg, _last_distance_m);
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_health()
{
    // health issue if status is "3" ->HW error
    if (_payload.sensor_health.status == 3) {
        Debug(1, "LIDAR Error");
    }
    Debug(1, "LIDAR Healthy");
}

void AP_Proximity_RPLidarA2::parse_response_express_scan()
{
    // EXPRESS_SCAN 密实格式：
    // - 84 字节总长
    // - sync1 (0xA) + ChkSum[3:0] in byte 0
    // - sync2 (0x5) + ChkSum[7:4] in byte 1
    // - start_angle_q6 (14-bit) + S flag (1-bit) in bytes 2-3
    // - 40 x cabin (distance) in bytes 4-83
    
    const uint8_t sync1 = _payload.express_scan.sync1_chk_low & 0x0F;
    const uint8_t sync2 = _payload.express_scan.sync2_chk_high & 0x0F;
    
    if (sync1 != 0xA || sync2 != 0x5) {
        Debug(1, "EXPRESS_SCAN: Invalid sync (0x%X, 0x%X)", sync1, sync2);
        return;
    }
    
    // Extract angle and S flag
    const uint16_t angle_raw = _payload.express_scan.start_angle_and_s;
    const uint16_t angle_q6 = (angle_raw & 0x3FFF);  // 14 bits
    const uint8_t s_flag = (angle_raw >> 14) & 0x01;   // bit 14
    
    const float base_angle = angle_q6 / 64.0f;
    
    _last_distance_received_ms = AP_HAL::millis();
    
    // 检测新的一圈扫描开始（S=1），计算实际扫描频率
    if (s_flag == 1) {
        uint32_t current_ms = AP_HAL::millis();
        if (_last_scan_start_ms > 0) {
            uint32_t delta_ms = current_ms - _last_scan_start_ms;
            if (delta_ms > 0) {
                // 计算实际扫描频率：60秒 / 每圈耗时(秒) = RPM，再除以60得到Hz
                _scan_frequency_hz = 1000.0f / delta_ms;
                
                // 每10圈输出一次频率信息到Mission Planner
                static uint8_t freq_report_counter = 0;
                freq_report_counter++;
                if (freq_report_counter >= 10) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scan Freq: %.2f Hz (%.1f RPM)", 
                                  _scan_frequency_hz, 
                                  _scan_frequency_hz * 60.0f);
                    freq_report_counter = 0;
                }
            }
        }
        _last_scan_start_ms = current_ms;
    }
    
    // Process 40 cabin samples
    for (uint8_t i = 0; i < 40; i++) {
        const uint16_t distance_mm = _payload.express_scan.cabin[i].distance;
        
        if (distance_mm == 0) {
            // Invalid point
            continue;
        }
        
        // Calculate angle for this point: increment by approximately 9 degrees per sample (360/40)
        const float angle_deg = wrap_360(base_angle + (i * 9.0f));
        const float distance_m = distance_mm / 1000.0f;
        
        // Apply orientation and yaw correction
        const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;
        const float corrected_angle = wrap_360(angle_deg * angle_sign + params.yaw_correction);
        
        if (!ignore_reading(corrected_angle, distance_m)) {
            const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(corrected_angle);
            
            // Update boundary
            frontend.boundary.set_face_attributes(face, corrected_angle, distance_m, state.instance);
            
            // Update OA database
            database_push(corrected_angle, distance_m);
        }
    }
    
    Debug(2, "EXPRESS_SCAN: base_angle=%.1f S=%u", base_angle, s_flag);
}

void AP_Proximity_RPLidarA2::parse_response_samplerate()
{
    const uint16_t tstandard = _payload.samplerate.tstandard;
    const uint16_t texpress = _payload.samplerate.texpress;
    
    Debug(1, "Sample Rate - Standard: %u µs, Express: %u µs", tstandard, texpress);
    
    // 计算扫描频率（Hz）
    // 标准模式：每点耗时tstandard微秒，一圈约400点
    // 高速模式：每点耗时texpress微秒，一圈约400点
    float freq_standard = (tstandard > 0) ? (1000000.0f / tstandard / 400.0f) : 0.0f;
    float freq_express = (texpress > 0) ? (1000000.0f / texpress / 400.0f) : 0.0f;
    
    // 在Mission Planner消息窗口显示工作模式和扫描频率
    if (_is_s3_mode) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode: EXPRESS_SCAN | Freq: %.1f Hz (%.0f pts/s)", 
                      freq_express, 
                      (texpress > 0) ? (1000000.0f / texpress) : 0.0f);
        _scan_frequency_hz = freq_express;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode: SCAN | Freq: %.1f Hz (%.0f pts/s)", 
                      freq_standard,
                      (tstandard > 0) ? (1000000.0f / tstandard) : 0.0f);
        _scan_frequency_hz = freq_standard;
    }
}

void AP_Proximity_RPLidarA2::parse_response_lidar_conf()
{
    // GET_LIDAR_CONF response contains configuration information
    const uint32_t conf_type = _payload.lidar_conf.type;
    Debug(1, "LIDAR_CONF type: 0x%X", (unsigned)conf_type);
    
    // Type 0x7C: Recommended scan mode ID (uint16)
    if (conf_type == 0x7C && _byte_count >= 6) {
        // Response format: type(4 bytes) + mode_id(2 bytes)
        _best_scan_mode_id = _payload.lidar_conf.data[0] | (_payload.lidar_conf.data[1] << 8);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Best scan mode ID: %u", _best_scan_mode_id);
        
        // Update working mode to use the best mode
        _working_mode = (uint8_t)_best_scan_mode_id;
        
        // Set motor speed to 600 RPM (10 Hz) for S-series
        if (_is_s3_mode) {
            send_motor_speed_ctrl(600);  // 600 RPM = 10 Hz
            hal.scheduler->delay(50);    // Wait 50ms after setting speed
        }
        
        // Now get sample rate info
        send_request_for_samplerate();
        _state = State::AWAITING_RESPONSE;
        return;
    }
    
    // Type 0x70: Number of supported scan modes (uint16)
    if (conf_type == 0x70 && _byte_count >= 6) {
        const uint16_t num_modes = _payload.lidar_conf.data[0] | (_payload.lidar_conf.data[1] << 8);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Supported scan modes: %u", num_modes);
    }
    
    // For other types or if parsing fails, proceed with default mode
    // Get sample rate and then start EXPRESS_SCAN
    send_request_for_samplerate();
    _state = State::AWAITING_RESPONSE;
}

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
