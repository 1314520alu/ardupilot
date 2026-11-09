/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Minimal CANopen master. Kept under src/ so ArduPilot's waf glob
 * (top-level *.c plus utility/) does not link it into vehicle firmware.
 */

#include "../canopen_master.h"
#include "../erob_config.h"

#include <string.h>

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "AP_eRobCANOpen SDO payloads are native little-endian"
#endif

void co_master_handle_frame(canopen_master_t *m, const can_frame_t *rx)
{
    uint32_t id;

    if (rx->is_ext) {
        return;
    }
    id = rx->id & 0x7ffu;

    if (id == (uint32_t)(CO_COB_HB_BASE + m->node_id) && rx->dlc >= 1) {
        m->last_pdo_hb_ms = can_bus_millis(m->bus);
        if (rx->data[0] == 0x00) {
            m->boot_seen = true;
            if (m->on_boot != NULL) {
                m->on_boot(m->hook_user, m->node_id);
            }
        }
        return;
    }
    if (id == (uint32_t)(CO_COB_EMCY_BASE + m->node_id) && rx->dlc >= 3) {
        uint16_t ec = (uint16_t)(rx->data[0] | (rx->data[1] << 8));
        if (m->on_emcy != NULL) {
            m->on_emcy(m->hook_user, ec, rx->data[2]);
        }
        return;
    }
    if (id == (uint32_t)(CO_COB_SDO_TX_BASE + m->node_id) && rx->dlc >= 8) {
        m->sdo_reply_index = (uint16_t)(rx->data[1] | (rx->data[2] << 8));
        m->sdo_reply_sub = rx->data[3];
        m->sdo_reply_cs = rx->data[0];
        memcpy(m->sdo_reply_data, &rx->data[4], 4);
        m->sdo_reply_ready = true;
        return;
    }
    if (m->on_tpdo != NULL) {
        uint8_t i;
        for (i = 0; i < 4; i++) {
            if (id == m->tpdo_cob[i]) {
                m->last_pdo_hb_ms = can_bus_millis(m->bus);
                m->on_tpdo(m->hook_user, i, rx->data, rx->dlc);
                return;
            }
        }
    }
}

void co_master_init(canopen_master_t *m, can_bus_t *bus, uint8_t node_id)
{
    memset(m, 0, sizeof(*m));
    m->bus = bus;
    m->node_id = node_id;
    m->sync_period_us = EROB_SYNC_PERIOD_US;
    m->sdo_timeout_ms = 100;
    co_master_use_default_cob(m);
}

void co_master_set_hooks(canopen_master_t *m, co_tpdo_hook_t on_tpdo,
                         co_emcy_hook_t on_emcy, co_boot_hook_t on_boot, void *user)
{
    m->on_tpdo = on_tpdo;
    m->on_emcy = on_emcy;
    m->on_boot = on_boot;
    m->hook_user = user;
}

void co_master_use_default_cob(canopen_master_t *m)
{
    uint8_t n = m->node_id;
    m->rpdo_cob[0] = (uint16_t)(CO_COB_RPDO1_BASE + n);
    m->tpdo_cob[0] = (uint16_t)(CO_COB_TPDO1_BASE + n);
    m->rpdo_cob[1] = (uint16_t)(CO_COB_RPDO2_BASE + n);
    m->tpdo_cob[1] = (uint16_t)(CO_COB_TPDO2_BASE + n);
    m->rpdo_cob[2] = (uint16_t)(CO_COB_RPDO3_BASE + n);
    m->tpdo_cob[2] = (uint16_t)(CO_COB_TPDO3_BASE + n);
    m->rpdo_cob[3] = (uint16_t)(CO_COB_RPDO4_BASE + n);
    m->tpdo_cob[3] = (uint16_t)(CO_COB_TPDO4_BASE + n);
}

void co_master_set_sync(canopen_master_t *m, bool enable, uint32_t period_us)
{
    m->sync_enabled = enable;
    if (period_us != 0) {
        m->sync_period_us = period_us;
    }
}

int co_nmt(canopen_master_t *m, uint8_t cmd, uint8_t node_id)
{
    can_frame_t fr;
    memset(&fr, 0, sizeof(fr));
    fr.id = CO_COB_NMT;
    fr.dlc = 2;
    fr.data[0] = cmd;
    fr.data[1] = node_id;
    return can_bus_send(m->bus, &fr);
}

int co_sync(canopen_master_t *m)
{
    can_frame_t fr;
    memset(&fr, 0, sizeof(fr));
    fr.id = CO_COB_SYNC;
    fr.dlc = 0;
    m->last_sync_ms = can_bus_millis(m->bus);
    return can_bus_send(m->bus, &fr);
}

static int sdo_xfer(canopen_master_t *m, uint8_t cs, uint16_t index, uint8_t sub,
                    const uint8_t *wdata, uint8_t wlen, uint8_t *rdata, uint8_t rlen)
{
    can_frame_t tx;
    uint32_t t0;

    memset(&tx, 0, sizeof(tx));
    tx.id = (uint32_t)(CO_COB_SDO_RX_BASE + m->node_id);
    tx.dlc = 8;
    tx.data[0] = cs;
    tx.data[1] = (uint8_t)(index & 0xff);
    tx.data[2] = (uint8_t)(index >> 8);
    tx.data[3] = sub;
    if (wdata != NULL && wlen > 0) {
        memcpy(&tx.data[4], wdata, wlen > 4 ? 4 : wlen);
    }
    if (can_bus_send(m->bus, &tx) != 0) {
        return -1;
    }

    t0 = can_bus_millis(m->bus);
    for (;;) {
        can_frame_t rx;
        int n;
        uint16_t rx_index;

        if ((can_bus_millis(m->bus) - t0) > m->sdo_timeout_ms) {
            return -3;
        }
        n = can_bus_recv(m->bus, &rx, 10);
        if (n != 0) {
            continue;
        }
        if (rx.is_ext || rx.id != (uint32_t)(CO_COB_SDO_TX_BASE + m->node_id)) {
            co_master_handle_frame(m, &rx);
            continue;
        }
        if (rx.dlc < 4) {
            continue;
        }
        if (rx.data[0] == 0x80) {
            return -2;
        }
        rx_index = (uint16_t)(rx.data[1] | (rx.data[2] << 8));
        if (rx_index != index || rx.data[3] != sub) {
            continue;
        }
        if ((rx.data[0] & 0xe0) == 0x60 || (rx.data[0] & 0xe0) == 0x40) {
            if (rdata != NULL && rlen > 0) {
                memcpy(rdata, &rx.data[4], rlen > 4 ? 4 : rlen);
            }
            return 0;
        }
    }
}

int co_sdo_write(canopen_master_t *m, uint16_t index, uint8_t sub,
                 const void *val, uint8_t size)
{
    uint8_t cs;
    if (size == 1) {
        cs = 0x2f;
    } else if (size == 2) {
        cs = 0x2b;
    } else if (size == 4) {
        cs = 0x23;
    } else {
        return -1;
    }
    return sdo_xfer(m, cs, index, sub, (const uint8_t *)val, size, NULL, 0);
}

int co_sdo_read(canopen_master_t *m, uint16_t index, uint8_t sub,
                void *val, uint8_t size)
{
    if (size != 1 && size != 2 && size != 4) {
        return -1;
    }
    return sdo_xfer(m, 0x40, index, sub, NULL, 0, (uint8_t *)val, size);
}

int co_sdo_read_send(canopen_master_t *m, uint16_t index, uint8_t sub)
{
    can_frame_t tx;

    memset(&tx, 0, sizeof(tx));
    tx.id = (uint32_t)(CO_COB_SDO_RX_BASE + m->node_id);
    tx.dlc = 8;
    tx.data[0] = 0x40;
    tx.data[1] = (uint8_t)(index & 0xff);
    tx.data[2] = (uint8_t)(index >> 8);
    tx.data[3] = sub;
    m->sdo_reply_ready = false;
    return can_bus_send(m->bus, &tx);
}

int co_rpdo_send(canopen_master_t *m, uint8_t pdo_num, const uint8_t *data, uint8_t dlc)
{
    can_frame_t fr;
    if (pdo_num > 3 || dlc > 8 || data == NULL) {
        return -1;
    }
    memset(&fr, 0, sizeof(fr));
    fr.id = m->rpdo_cob[pdo_num];
    fr.dlc = dlc;
    memcpy(fr.data, data, dlc);
    return can_bus_send(m->bus, &fr);
}

int co_master_wait_boot(canopen_master_t *m, uint32_t timeout_ms)
{
    uint32_t t0 = can_bus_millis(m->bus);
    m->boot_seen = false;
    while (!m->boot_seen) {
        can_frame_t rx;
        if (can_bus_recv(m->bus, &rx, 10) == 0) {
            co_master_handle_frame(m, &rx);
        }
        if (m->boot_seen) {
            return 0;
        }
        if ((can_bus_millis(m->bus) - t0) > timeout_ms) {
            return -1;
        }
    }
    return 0;
}

void co_master_poll_rx(canopen_master_t *m)
{
    can_frame_t rx;
    while (can_bus_recv(m->bus, &rx, 0) == 0) {
        co_master_handle_frame(m, &rx);
    }
}

void co_master_poll_sync(canopen_master_t *m)
{
    uint32_t now;
    uint32_t period_ms;

    if (!m->sync_enabled || m->sync_period_us == 0) {
        return;
    }
    /* Millisecond tick. 4 ms is exact; sub-ms periods need a hardware timer. */
    period_ms = m->sync_period_us / 1000u;
    if (period_ms == 0) {
        period_ms = 1;
    }
    now = can_bus_millis(m->bus);
    if ((now - m->last_sync_ms) >= period_ms) {
        (void)co_sync(m);
    }
}

void co_master_poll(canopen_master_t *m)
{
    co_master_poll_rx(m);
    co_master_poll_sync(m);
}

bool erob_id_is_reserved(uint16_t id, uint8_t node_id)
{
    static const uint16_t bases[] = {
        CO_COB_EMCY_BASE,
        CO_COB_TPDO1_BASE,
        CO_COB_RPDO1_BASE,
        CO_COB_TPDO2_BASE,
        CO_COB_RPDO2_BASE,
        CO_COB_TPDO3_BASE,
        CO_COB_RPDO3_BASE,
        CO_COB_TPDO4_BASE,
        CO_COB_RPDO4_BASE,
        CO_COB_SDO_TX_BASE,
        CO_COB_SDO_RX_BASE,
        CO_COB_HB_BASE
    };
    uint16_t std_id = (uint16_t)(id & 0x7ffu);
    uint8_t i;

    if (std_id == CO_COB_NMT || std_id == CO_COB_SYNC) {
        return true;
    }
    if (node_id < 1 || node_id > 127) {
        return false;
    }
    for (i = 0; i < (uint8_t)(sizeof(bases) / sizeof(bases[0])); i++) {
        if (std_id == (uint16_t)(bases[i] + node_id)) {
            return true;
        }
    }
    return false;
}
