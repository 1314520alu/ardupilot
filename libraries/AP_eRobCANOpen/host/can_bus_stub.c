/*
 * Host stand-in for CAN2. Records TX and answers the §5.5.1 SDO/PDO
 * sequence so the master can be checked without a servo or a flight stack.
 */

#include "can_bus_stub.h"
#include "erob_servo.h"

#include <string.h>

#define TX_LOG_MAX 512
#define RX_Q_MAX 16

struct can_bus {
    uint32_t fake_ms;
    uint8_t node_id;
    int32_t sim_pos;
    uint16_t sim_sw;
    can_frame_t tx_log[TX_LOG_MAX];
    uint16_t tx_count;
    int overflow;
    can_frame_t rx_q[RX_Q_MAX];
    uint8_t rx_head;
    uint8_t rx_count;
};

static struct can_bus g_bus;

static void rx_push(struct can_bus *bus, const can_frame_t *fr)
{
    uint8_t slot;
    if (bus->rx_count >= RX_Q_MAX) {
        bus->overflow = 1;
        return;
    }
    slot = (uint8_t)((bus->rx_head + bus->rx_count) % RX_Q_MAX);
    bus->rx_q[slot] = *fr;
    bus->rx_count++;
}

static void reply_sdo(struct can_bus *bus, const can_frame_t *req)
{
    can_frame_t rx;
    uint8_t cs = req->data[0];
    uint16_t index = (uint16_t)(req->data[1] | (req->data[2] << 8));
    memset(&rx, 0, sizeof(rx));
    rx.id = 0x580u + bus->node_id;
    rx.dlc = 8;
    rx.data[1] = req->data[1];
    rx.data[2] = req->data[2];
    rx.data[3] = req->data[3];
    if (cs == 0x40 && index == 0x6064) {
        int32_t pos = bus->sim_pos;
        rx.data[0] = 0x43;
        rx.data[4] = (uint8_t)((uint32_t)pos);
        rx.data[5] = (uint8_t)((uint32_t)pos >> 8);
        rx.data[6] = (uint8_t)((uint32_t)pos >> 16);
        rx.data[7] = (uint8_t)((uint32_t)pos >> 24);
    } else if (cs == 0x40 && index == 0x6061) {
        rx.data[0] = 0x4f;
        rx.data[4] = EROB_MODE_CSP;
    } else if (cs == 0x40 && index == 0x603F) {
        rx.data[0] = 0x4b;
        rx.data[4] = 0x34;
        rx.data[5] = 0x12;
    } else if (cs == 0x40 && index == 0x3B68) {
        rx.data[0] = 0x43;
        rx.data[4] = 0x01;
        rx.data[5] = 0xff;
    } else if (cs == 0x40 && index == 0x22A2) {
        /* -5 C, stored as int16. The dictionary type is UINT. */
        rx.data[0] = 0x4b;
        rx.data[4] = 0xfb;
        rx.data[5] = 0xff;
    } else if (cs == 0x40 && index == 0x2241) {
        rx.data[0] = 0x43;
        rx.data[4] = 42;
    } else if (cs == 0x2f || cs == 0x2b || cs == 0x23) {
        rx.data[0] = 0x60;
    } else {
        rx.data[0] = 0x80;
    }
    rx_push(bus, &rx);
}

static void put_i32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
    p[3] = (uint8_t)(u >> 24);
}

static void push_tpdo(struct can_bus *bus, uint16_t cob, uint8_t dlc, const uint8_t *data)
{
    can_frame_t rx;
    memset(&rx, 0, sizeof(rx));
    rx.id = cob;
    rx.dlc = dlc;
    memcpy(rx.data, data, dlc);
    rx_push(bus, &rx);
}

static void reply_rpdo(struct can_bus *bus, const can_frame_t *req)
{
    uint8_t b[8];
    uint16_t cw = (uint16_t)(req->data[0] | (req->data[1] << 8));
    uint16_t sw = EROB_SW_SWITCH_ON_DIS;
    uint8_t n = bus->node_id;
    if (cw == EROB_CW_SHUTDOWN) {
        sw = EROB_SW_READY_TO_SW;
    } else if (cw == EROB_CW_SWITCH_ON) {
        sw = EROB_SW_SWITCHED_ON;
    } else if (cw == EROB_CW_ENABLE_OP) {
        sw = EROB_SW_OP_ENABLED;
    }
    bus->sim_sw = sw;

    memset(b, 0, sizeof(b));
    b[0] = (uint8_t)(sw & 0xff);
    b[1] = (uint8_t)(sw >> 8);
    put_i32(&b[2], bus->sim_pos);
    push_tpdo(bus, (uint16_t)(0x180u + n), 6, b);

    memset(b, 0, sizeof(b));
    put_i32(&b[0], 111);
    put_i32(&b[4], 222);
    push_tpdo(bus, (uint16_t)(0x280u + n), 8, b);

    memset(b, 0, sizeof(b));
    put_i32(&b[0], 333);
    b[4] = (uint8_t)(int16_t)-7;
    b[5] = (uint8_t)((uint16_t)(int16_t)-7 >> 8);
    b[6] = 9;
    push_tpdo(bus, (uint16_t)(0x380u + n), 8, b);

    memset(b, 0, sizeof(b));
    put_i32(&b[0], 48000);
    put_i32(&b[4], 15);
    push_tpdo(bus, (uint16_t)(0x480u + n), 8, b);
}

can_bus_t *can_bus_stub_get(void)
{
    memset(&g_bus, 0, sizeof(g_bus));
    return &g_bus;
}

void can_bus_stub_advance(uint32_t ms)
{
    g_bus.fake_ms += ms;
}

uint16_t can_bus_stub_tx_count(void)
{
    return g_bus.tx_count;
}

const can_frame_t *can_bus_stub_tx(uint16_t index)
{
    if (index >= g_bus.tx_count) {
        return NULL;
    }
    return &g_bus.tx_log[index];
}

int can_bus_stub_overflow(void)
{
    return g_bus.overflow;
}

int can_bus_send(can_bus_t *bus, const can_frame_t *fr)
{
    uint32_t id;
    if (bus->tx_count < TX_LOG_MAX) {
        bus->tx_log[bus->tx_count++] = *fr;
    } else {
        bus->overflow = 1;
    }
    if (fr->is_ext) {
        return 0;
    }
    id = fr->id & 0x7ffu;
    if (id == 0x000 && fr->dlc >= 2) {
        bus->node_id = fr->data[1];
        if (fr->data[0] == CO_NMT_RESET_COMM && bus->node_id != 0) {
            can_frame_t boot;
            memset(&boot, 0, sizeof(boot));
            boot.id = 0x700u + bus->node_id;
            boot.dlc = 1;
            boot.data[0] = 0x00;
            rx_push(bus, &boot);
        }
        return 0;
    }
    if (bus->node_id != 0 && id == (0x600u + bus->node_id) && fr->dlc >= 4) {
        reply_sdo(bus, fr);
        return 0;
    }
    if (bus->node_id != 0 && id == (0x200u + bus->node_id) && fr->dlc >= 6) {
        reply_rpdo(bus, fr);
    }
    return 0;
}

int can_bus_recv(can_bus_t *bus, can_frame_t *fr, uint32_t timeout_ms)
{
    if (bus->rx_count == 0) {
        bus->fake_ms += timeout_ms;
        return -1;
    }
    *fr = bus->rx_q[bus->rx_head];
    bus->rx_head = (uint8_t)((bus->rx_head + 1) % RX_Q_MAX);
    bus->rx_count--;
    return 0;
}

uint32_t can_bus_millis(can_bus_t *bus)
{
    return bus->fake_ms;
}
