#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RPLIDARS3UDP_ENABLED

#include "AP_Proximity_RPLidarS3UDP.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_Proximity_RPLidarS3UDP::var_info[] = {

    // @Param: PORT
    // @DisplayName: S3E UDP port
    // @Description: UDP port of the Slamtec RPLIDAR S3 Ethernet lidar (S3M1-R2E). Slamtec factory default is 8089. This fork's bench unit is 192.168.144.2:8089.
    // @Range: 1 65535
    // @User: Standard
    AP_GROUPINFO("PORT", 1, AP_Proximity_RPLidarS3UDP, _port, 8089),

    // @Group: IP
    // @Path: ../AP_Networking/AP_Networking_address.cpp
    AP_SUBGROUPINFO(_ip, "IP", 2, AP_Proximity_RPLidarS3UDP, AP_Networking_IPV4),

    AP_GROUPEND
};

AP_Proximity_RPLidarS3UDP::AP_Proximity_RPLidarS3UDP(AP_Proximity &_frontend,
        AP_Proximity::Proximity_State &prox_state,
        AP_Proximity_Params &_params) :
    AP_Proximity_RPLidarA2(_frontend, prox_state, _params),
    _rx{_rx_storage, sizeof(_rx_storage)}
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

AP_Proximity_RPLidarS3UDP::~AP_Proximity_RPLidarS3UDP()
{
    close_socket();
}

void AP_Proximity_RPLidarS3UDP::close_socket()
{
    if (_sock != nullptr) {
        _sock->close();
        delete _sock;
        _sock = nullptr;
    }
    _connected = false;
    _rx.clear();
}

void AP_Proximity_RPLidarS3UDP::service_socket()
{
    const uint32_t now = AP_HAL::millis();
    const int32_t port_param = _port.get();
    if (port_param <= 0 || port_param > 65535) {
        close_socket();
        return;
    }
    const uint16_t port = uint16_t(port_param);
    const uint32_t ip = _ip.get_uint32();
    if (ip == 0U) {
        close_socket();
        return;
    }

    if (_sock != nullptr && (ip != _connected_ip || port != _connected_port)) {
        close_socket();
    }

    if (_sock == nullptr) {
        if (now - _last_connect_attempt_ms < 1000U) {
            return;
        }
        _last_connect_attempt_ms = now;
        _sock = NEW_NOTHROW SocketAPM(true);
        if (_sock == nullptr) {
            return;
        }
        _sock->set_blocking(false);
        char ipstr[16];
        hal.util->snprintf(ipstr, sizeof(ipstr), "%u.%u.%u.%u",
                           unsigned(_ip.addr[0].get()),
                           unsigned(_ip.addr[1].get()),
                           unsigned(_ip.addr[2].get()),
                           unsigned(_ip.addr[3].get()));
        if (!_sock->connect(ipstr, port)) {
            close_socket();
            if (now - _last_connect_fail_msg_ms > 5000U) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PRX%u: S3E UDP connect %s:%u failed",
                              unsigned(state.instance) + 1U, ipstr, unsigned(port));
                _last_connect_fail_msg_ms = now;
            }
            return;
        }
        _connected = true;
        _connected_ip = ip;
        _connected_port = port;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PRX%u: S3E UDP %s:%u",
                      unsigned(state.instance) + 1U, ipstr, unsigned(port));
        note_transport_connected();
    }

    for (uint8_t i = 0; i < 30U && _sock != nullptr; i++) {
        const ssize_t n = _sock->recv(_pkt, sizeof(_pkt), 0);
        if (n <= 0) {
            break;
        }
        if (_rx.space() < uint32_t(n)) {
            // Parser is behind. Drop this datagram; dense resync tolerates gaps.
            break;
        }
        _rx.write(_pkt, uint32_t(n));
    }
}

void AP_Proximity_RPLidarS3UDP::update(void)
{
    service_socket();
    if (!_connected) {
        set_status(AP_Proximity::Status::NotConnected);
        return;
    }
    AP_Proximity_RPLidarA2::update();
}

uint32_t AP_Proximity_RPLidarS3UDP::transport_available() const
{
    return _rx.available();
}

uint32_t AP_Proximity_RPLidarS3UDP::transport_read(uint8_t *buf, uint32_t maxlen)
{
    if (buf == nullptr || maxlen == 0U) {
        return 0;
    }
    return _rx.read(buf, maxlen);
}

void AP_Proximity_RPLidarS3UDP::transport_write(const uint8_t *buf, uint32_t len)
{
    // One RPLIDAR command per UDP datagram. The lidar parses the same
    // bytes it would see on UART.
    if (_sock == nullptr || !_connected || buf == nullptr || len == 0U) {
        return;
    }
    _sock->send(buf, len);
}

void AP_Proximity_RPLidarS3UDP::transport_discard_rx()
{
    _rx.clear();
}

#endif // AP_PROXIMITY_RPLIDARS3UDP_ENABLED
