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
 * models (A1, A2, C1, S1, S3). A-series use legacy SCAN; S3 uses EXPRESS_SCAN dense (0x85).
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 *
 * RPLIDAR S3: see AP_Proximity_RPLidarA2.cpp file header (EXPRESS_SCAN dense / 0x85,
 * GET_HEALTH, GET_SAMPLERATE, HQ motor, STOP before RESET). On CUAV-X7 use a UART with
 * SERIALn_PROTOCOL=Lidar360(11), baud 1000000 (460800 if model byte 0x82), PRXn_TYPE=RPLidarA2(5),
 * and set PRXn_MAX=40 for 40m OA range.
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

/*

# to connect device to SITL:
./Tools/autotest/sim_vehicle.py -v Rover --gdb --debug -A --serial5=uart:/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:115200
param set SERIAL5_PROTOCOL 11
param set SERIAL5_BAUD 115200
param set PRX1_TYPE 5
reboot

# short outer-two wires on JST plug to get it to spin

*/


#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARA2_ENABLED

#include "AP_Proximity_Backend_Serial.h"

class AP_Proximity_RPLidarA2 : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:

    enum class State {
        RESET = 56,
        AWAITING_RESPONSE,
        AWAITING_SCAN_DATA,
        AWAITING_DENSE_CAPSULE,
        AWAITING_HEALTH,
        AWAITING_DEVICE_INFO,
        AWAITING_SAMPLE_RATE,
        S3_WAIT_MOTOR_SETTLE,
    } _state = State::RESET;

    // S3-only multi-step bring-up (Slamtec recommended order, adapted for legacy SCAN).
    enum class S3Bootstrap : uint8_t {
        None = 0,
        SentHealth,
        SentSampleRate,
        SentMotor,
    };

    // send request for something from sensor
    void send_stop();
    void send_request_for_health();
    void send_request_for_sample_rate();
    void send_scan_mode_request();
    void send_s3_express_scan_dense();
    void send_request_for_device_info();
    void send_s3_motor_rpm(uint16_t rpm);
    void send_command_with_payload_xor(uint8_t cmd, const void *payload, uint8_t payload_len);

    void parse_response_data();
    void parse_response_dense_capsule();
    void parse_response_health();
    void parse_response_device_info();
    void parse_response_sample_rate();

    void handle_proximity_sample(float angle_deg, float distance_m);

    void try_send_s3_gcs_pending();

    void get_readings();
    void reset_rplidar();
    void reset();

    // remove bytes from read buffer:
    void consume_bytes(uint16_t count);

    uint8_t _sync_error;
    uint16_t _byte_count;

    // request related variables
    uint32_t  _last_distance_received_ms;     ///< system time of last distance measurement received from sensor
    uint32_t  _last_reset_ms;
    S3Bootstrap _s3_bootstrap{S3Bootstrap::None};
    uint32_t _s3_deadline_ms{0};
    uint16_t _s3_express_sample_us{0};
    bool _s3_dense_have_prev{false};
    uint8_t _s3_dense_prev_payload[84];
    int8_t _s3_dense_last_sync{0};

    // STATUSTEXT for Mission Planner: deferred until MAVLink active; keep text short ASCII (MP UTF-8 chunking).
    bool _s3_pending_gcs_connected_msg{false};
    bool _s3_pending_gcs_express_msg{false};

    // face related variables
    AP_Proximity_Boundary_3D::Face _last_face;///< last face requested
    float _last_angle_deg;                    ///< yaw angle (in degrees) of _last_distance_m
    float _last_distance_m;                   ///< shortest distance for _last_face
    bool _last_distance_valid;                ///< true if _last_distance_m is valid

    struct PACKED _device_info {
        uint8_t model;
        uint8_t firmware_minor;
        uint8_t firmware_major;
        uint8_t hardware;
        uint8_t serial[16];
   };

    struct PACKED _sensor_scan {
        uint8_t startbit      : 1;            ///< on the first revolution 1 else 0
        uint8_t not_startbit  : 1;            ///< complementary to startbit
        uint8_t quality       : 6;            ///< Related the reflected laser pulse strength
        uint8_t checkbit      : 1;            ///< always set to 1
        uint16_t angle_q6     : 15;           ///< Actual heading = angle_q6/64.0 Degree
        uint16_t distance_q2  : 16;           ///< Actual Distance = distance_q2/4.0 mm
    };

    struct PACKED _sensor_health {
        uint8_t status;                       ///< status definition: 0 good, 1 warning, 2 error
        uint16_t error_code;                  ///< the related error code
    };

    struct PACKED _sample_rate {
        uint16_t std_sample_duration_us;
        uint16_t express_sample_duration_us;
    };

    // Slamtec dense express capsule (SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED = 0x85)
    struct PACKED _dense_cabin {
        uint16_t distance_mm;
    };
    struct PACKED _dense_capsule_meas {
        uint8_t s_checksum_1;
        uint8_t s_checksum_2;
        uint16_t start_angle_sync_q6;
        _dense_cabin cabins[40];
    };
    static_assert(sizeof(_dense_capsule_meas) == 84U, "Slamtec dense express capsule");

    struct PACKED _descriptor {
        uint8_t bytes[7];
    };

    // we don't actually *need* to store this.  If we don't, _payload
    // can be just 7 bytes, but that doesn't make for efficient
    // reading.  It also simplifies the state machine to have the read
    // buffer at least this big.  Note that we force the buffer to a
    // larger size below anyway.
    struct PACKED _rpi_information {
        uint8_t bytes[63];
    };

    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        _sensor_scan sensor_scan;
        _sensor_health sensor_health;
        _sample_rate sample_rate;
        _descriptor descriptor;
        _rpi_information information;
        _device_info device_info;
        uint8_t forced_buffer_size[2048]; // just so we read(...) efficiently
    } _payload;
    static_assert(sizeof(_payload) >= 63, "Needed for parsing out reboot data");

    enum class Model {
        UNKNOWN,
        A1,
        A2,
        C1,
        S1,
        S3,
    } model = Model::UNKNOWN;

    bool make_first_byte_in_payload(uint8_t desired_byte);
};

#endif // AP_PROXIMITY_RPLIDARA2_ENABLED
