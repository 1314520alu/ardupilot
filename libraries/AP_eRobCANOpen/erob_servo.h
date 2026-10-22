#pragma once

#include "canopen_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 0x6060, manual table 5-1. Homing and interpolated position are unsupported. */
enum {
    EROB_MODE_PP  = 0x01,
    EROB_MODE_PV  = 0x03,
    EROB_MODE_PT  = 0x04,
    EROB_MODE_CSP = 0x08,
    EROB_MODE_CSV = 0x09,
    EROB_MODE_CST = 0x0A
};

/* Manual §5.5.1 / §5.4. Control word 0x001F appears only in the sample move. */
enum {
    EROB_CW_SHUTDOWN    = 0x0006,
    EROB_CW_SWITCH_ON   = 0x0007,
    EROB_CW_ENABLE_OP   = 0x000F,
    EROB_CW_FAULT_RESET = 0x0080
};

enum {
    EROB_SW_MASK          = 0x006F,
    EROB_SW_SWITCH_ON_DIS = 0x0040,
    EROB_SW_READY_TO_SW   = 0x0021,
    EROB_SW_SWITCHED_ON   = 0x0023,
    EROB_SW_OP_ENABLED    = 0x0027,
    EROB_SW_FAULT_BIT     = 0x0008
};

typedef enum {
    EROB_ST_IDLE = 0,
    EROB_ST_CONFIG,
    EROB_ST_ENABLE_SEQ,
    EROB_ST_ENABLED,
    EROB_ST_FAULT,
    EROB_ST_ERROR
} erob_state_t;

typedef struct {
    canopen_master_t *co;
    uint8_t mode;
    erob_state_t state;
    uint8_t enable_step;
    uint32_t enable_ts_ms;
    uint32_t enable_done_ms;
    uint16_t statusword;
    int32_t pos_actual;
    int32_t target_pos;
    int32_t hold_pos;
    int32_t target_vel; /* stored only; PDO map is CSP position, see README */
    bool feedback_valid;
} erob_servo_t;

void erob_init(erob_servo_t *s, canopen_master_t *co);

/*
 * Manual §5.5.1 fixed map for this node:
 *   RxPDO1 = 0x6040:00 u16 + 0x607A:00 i32
 *   TxPDO1 = 0x6041:00 u16 + 0x6064:00 i32
 *   0x1006 = sync period in microseconds
 * Slave SYNC producer is left disabled (0x1005 = 0x00000080).
 */
int erob_configure_pdo_csp(erob_servo_t *s);

int erob_set_mode(erob_servo_t *s, uint8_t mode);
int erob_enable(erob_servo_t *s);
int erob_disable(erob_servo_t *s);

void erob_set_target_pos(erob_servo_t *s, int32_t plus);
void erob_set_target_vel(erob_servo_t *s, int32_t plus_per_s);

/* CSP state machine and RPDO. Does not read the bus or emit SYNC. */
void erob_poll_motion(erob_servo_t *s);

/* Call on the CAN2 cycle: RX, RPDO, then SYNC. */
void erob_poll(erob_servo_t *s);

/*
 * Map one servo PWM onto the CSP target, in plus.
 * pwm_max <= pwm_min returns plus_min. pwm outside the PWM span is clamped.
 */
int32_t erob_pwm_to_plus(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max,
                         int32_t plus_min, int32_t plus_max);

erob_state_t erob_get_state(const erob_servo_t *s);
uint16_t erob_get_statusword(const erob_servo_t *s);
int32_t erob_get_position(const erob_servo_t *s);

#ifdef __cplusplus
}
#endif
