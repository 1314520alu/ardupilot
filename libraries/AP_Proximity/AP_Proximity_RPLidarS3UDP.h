#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARS3UDP_ENABLED

#include "AP_Proximity_RPLidarA2.h"
#include <AP_Networking/AP_Networking_address.h>
#include <AP_HAL/utility/RingBuffer.h>

class SocketAPM;

/*
 * Slamtec RPLIDAR S3M1-R2E (Ethernet) proximity backend.
 *
 * The lidar speaks the same RPLIDAR request/response bytes as the UART S3,
 * one command or capsule stream carried in UDP datagrams. This class is the
 * UDP transport for AP_Proximity_RPLidarA2; Dense Express parsing is shared.
 *
 * Bench (FC already on 192.168.144.0/24, lidar at 192.168.144.2:8089):
 *   PRX1_TYPE = 19
 *   PRX1_PORT = 8089
 *   PRX1_IP0..3 = 192 168 144 2
 *   PRX1_MAX = 40
 *   NET_ENABLE = 1
 *   NET_DHCP = 0
 *   NET_IPADDR0..3 = 192 168 144 15   (board default on uav-dev-fc-um982)
 * Requires an Ethernet-capable flight controller with AP_NETWORKING_ENABLED.
 * Factory Slamtec address before setLidarIpConf is 192.168.11.2:8089.
 */
class AP_Proximity_RPLidarS3UDP : public AP_Proximity_RPLidarA2
{
public:
    AP_Proximity_RPLidarS3UDP(AP_Proximity &_frontend,
                              AP_Proximity::Proximity_State &prox_state,
                              AP_Proximity_Params &_params);

    ~AP_Proximity_RPLidarS3UDP() override;

    void update(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool transport_ready() const override
    {
        return _connected;
    }
    uint32_t transport_available() const override;
    uint32_t transport_read(uint8_t *buf, uint32_t maxlen) override;
    void transport_write(const uint8_t *buf, uint32_t len) override;
    void transport_discard_rx() override;
    bool prefer_s3_express() const override
    {
        return true;
    }

private:
    void service_socket();
    void close_socket();

    AP_Int32 _port;
    AP_Networking_IPV4 _ip{"192.168.144.2"};

    SocketAPM *_sock;
    bool _connected;
    uint32_t _connected_ip;
    uint16_t _connected_port;
    uint32_t _last_connect_attempt_ms;
    uint32_t _last_connect_fail_msg_ms;

    uint8_t _rx_storage[2048];
    ByteBuffer _rx;
    // Datagram scratch. Kept off the scheduler stack (ChibiOS frame limit is 1300).
    uint8_t _pkt[1500];
};

#endif // AP_PROXIMITY_RPLIDARS3UDP_ENABLED
