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

/* Which feedback fields have been seen at least once. */
enum {
    EROB_FB_TPDO1 = 1u << 0, /* 0x6041 + 0x6064 */
    EROB_FB_TPDO2 = 1u << 1, /* 0x2240 + 0x20A0 */
    EROB_FB_TPDO3 = 1u << 2, /* 0x606C + 0x6077 + 0x6078 */
    EROB_FB_TPDO4 = 1u << 3, /* 0x6079 + 0x60F4 */
    EROB_FB_ERR   = 1u << 4, /* 0x603F */
    EROB_FB_WARN  = 1u << 5, /* 0x3B68 */
    EROB_FB_TEMP  = 1u << 6, /* 0x22A2 */
    EROB_FB_MODE  = 1u << 7, /* 0x6061 */
    EROB_FB_DIFF  = 1u << 8, /* 0x2241 */
    EROB_FB_IRATED = 1u << 9, /* 0x6075 non-zero */
    EROB_FB_TRATED = 1u << 10 /* 0x6076 non-zero */
};

/*
 * Latest sample for one axis. Units are the raw objects:
 * plus, plus/s, permille of rated, millivolts, degrees C.
 * Motor RPM and winding temperature are not separate objects.
 */
typedef struct {
    uint16_t statusword;
    int32_t pos_actual;
    int32_t motor_enc;
    int32_t load_enc;
    int32_t vel_plus_s;
    int16_t torque_permille;
    int16_t current_permille;
    uint32_t dc_mv;
    int32_t follow_err;
    uint16_t error_code;
    uint32_t warning_code;
    int16_t drive_temp_c;
    int8_t mode_disp;
    int32_t enc_diff;
    uint32_t rated_current_mA; /* 0x6075, 0 if unread */
    uint32_t rated_torque_mNm; /* 0x6076, 0 if unread */
    uint16_t valid;
    bool op_enabled;
} erob_feedback_t;

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
    erob_feedback_t fb;
    uint8_t diag_slot;
    uint8_t diag_size;
    uint16_t diag_index;
    uint32_t diag_tx_ms;
    bool diag_wait;
} erob_servo_t;

void erob_init(erob_servo_t *s, canopen_master_t *co);

/*
 * Pre-op map for this node. RxPDO1 stays the CSP command.
 * TxPDO1..4 are fixed 8-byte maps (see README). 0x1006 is microseconds.
 * Slave SYNC producer is left disabled (0x1005 = 0x00000080).
 * 0x603F, 0x3B68, 0x22A2, 0x6061 and 0x2241 are polled by SDO, not PDO.
 */
int erob_configure_pdo_csp(erob_servo_t *s);

int erob_set_mode(erob_servo_t *s, uint8_t mode);
int erob_enable(erob_servo_t *s);
int erob_disable(erob_servo_t *s);

void erob_set_target_pos(erob_servo_t *s, int32_t plus);
void erob_set_target_vel(erob_servo_t *s, int32_t plus_per_s);

/* CSP state machine and RPDO. Does not read the bus or emit SYNC. */
void erob_poll_motion(erob_servo_t *s);

/* One non-blocking diagnostic SDO. Call after RX has been dispatched. */
void erob_poll_diag(erob_servo_t *s);

/* Call on the CAN2 cycle: RX, RPDO, diag SDO, then SYNC. */
void erob_poll(erob_servo_t *s);

/*
 * Map one servo PWM onto the CSP target, in plus.
 * pwm_max <= pwm_min returns plus_min. pwm outside the PWM span is clamped.
 */
int32_t erob_pwm_to_plus(uint16_t pwm, uint16_t pwm_min, uint16_t pwm_max,
                         int32_t plus_min, int32_t plus_max);

/* Absolute 0x607A count: center + mapped offset. Saturates to int32. */
int32_t erob_target_plus(int32_t mapped, int32_t center);

/* 0x6064 relative to center, for degrees. Saturates to int32. */
int32_t erob_plus_rel_center(int32_t absolute, int32_t center);

erob_state_t erob_get_state(const erob_servo_t *s);
uint16_t erob_get_statusword(const erob_servo_t *s);
int32_t erob_get_position(const erob_servo_t *s);
const erob_feedback_t *erob_feedback(const erob_servo_t *s);

/* Ignore a zero rating. A failed or zero read leaves the valid bit clear. */
void erob_cache_rating(erob_feedback_t *fb, uint16_t index, uint32_t value);

/*
 * Output angle and output rpm. counts_per_rev is encoder counts in one
 * output revolution. A non-positive value uses EROB_PLUS_PER_REV (524288).
 */
float erob_position_deg(int32_t plus, int32_t counts_per_rev);
float erob_output_rpm(int32_t plus_per_s, int32_t counts_per_rev);

/* permille of rated. rated_mA is 0x6075, rated_mNm is 0x6076. */
float erob_current_a(int16_t permille, uint32_t rated_mA);
float erob_torque_nm(int16_t permille, uint32_t rated_mNm);

#ifdef __cplusplus
}
#endif
