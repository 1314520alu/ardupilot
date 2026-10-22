#pragma once

/*
 * Minimal CANopen master for one eRob node.
 * Subset: NMT, expedited SDO, SYNC 0x080, fixed PDO COB-IDs.
 * Not a general stack (no segmented SDO, LSS, or PDO mapping engine).
 */

#include "can_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CO_NMT_START       = 0x01,
    CO_NMT_STOP        = 0x02,
    CO_NMT_ENTER_PREOP = 0x80,
    CO_NMT_RESET_NODE  = 0x81,
    CO_NMT_RESET_COMM  = 0x82
};

enum {
    CO_COB_NMT         = 0x000,
    CO_COB_SYNC        = 0x080,
    CO_COB_EMCY_BASE   = 0x080, /* + node id */
    CO_COB_TPDO1_BASE  = 0x180,
    CO_COB_RPDO1_BASE  = 0x200,
    CO_COB_TPDO2_BASE  = 0x280,
    CO_COB_RPDO2_BASE  = 0x300,
    CO_COB_TPDO3_BASE  = 0x380,
    CO_COB_RPDO3_BASE  = 0x400,
    CO_COB_TPDO4_BASE  = 0x480,
    CO_COB_RPDO4_BASE  = 0x500,
    CO_COB_SDO_TX_BASE = 0x580, /* slave -> master */
    CO_COB_SDO_RX_BASE = 0x600, /* master -> slave */
    CO_COB_HB_BASE     = 0x700
};

typedef void (*co_tpdo_hook_t)(void *user, uint8_t pdo_num,
                               const uint8_t *data, uint8_t dlc);
typedef void (*co_emcy_hook_t)(void *user, uint16_t err_code, uint8_t err_reg);
typedef void (*co_boot_hook_t)(void *user, uint8_t node_id);

typedef struct {
    can_bus_t *bus;
    uint8_t node_id;
    uint32_t sync_period_us;
    uint32_t last_sync_ms;
    bool sync_enabled;
    bool boot_seen;

    uint16_t rpdo_cob[4];
    uint16_t tpdo_cob[4];

    co_tpdo_hook_t on_tpdo;
    co_emcy_hook_t on_emcy;
    co_boot_hook_t on_boot;
    void *hook_user;

    uint32_t sdo_timeout_ms;
} canopen_master_t;

void co_master_init(canopen_master_t *m, can_bus_t *bus, uint8_t node_id);

void co_master_set_hooks(canopen_master_t *m,
                         co_tpdo_hook_t on_tpdo,
                         co_emcy_hook_t on_emcy,
                         co_boot_hook_t on_boot,
                         void *user);

void co_master_use_default_cob(canopen_master_t *m);

int co_nmt(canopen_master_t *m, uint8_t cmd, uint8_t node_id);

int co_sync(canopen_master_t *m);

/* Expedited SDO. size is 1, 2, or 4. Payload is little-endian. */
int co_sdo_write(canopen_master_t *m, uint16_t index, uint8_t sub,
                 const void *val, uint8_t size);
int co_sdo_read(canopen_master_t *m, uint16_t index, uint8_t sub,
                void *val, uint8_t size);

int co_rpdo_send(canopen_master_t *m, uint8_t pdo_num,
                 const uint8_t *data, uint8_t dlc);

/* Block until boot-up 0x700+id / 0x00, or timeout. Other frames are dispatched. */
int co_master_wait_boot(canopen_master_t *m, uint32_t timeout_ms);

void co_master_poll_rx(canopen_master_t *m);
void co_master_poll_sync(canopen_master_t *m);

/* RX then SYNC. eRob cyclic path sends RPDO between the two. */
void co_master_poll(canopen_master_t *m);

void co_master_set_sync(canopen_master_t *m, bool enable, uint32_t period_us);

#ifdef __cplusplus
}
#endif
