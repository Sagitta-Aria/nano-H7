#ifndef MOTION_H
#define MOTION_H
#include <stdint.h>
#include "app_config.h"
typedef struct {
    int32_t position;
    int32_t sps;
    uint32_t remaining;
    uint32_t command_id;
    unsigned active;
} axis_status_t;
void motion_init(void);
void motion_tick(void);
int motion_arm(unsigned enable);
void motion_stop(void);
void motion_emergency(void);
int motion_move(unsigned axis, int32_t steps, unsigned sps, unsigned accel, uint32_t id);
int motion_wheels(const int32_t sps[4], unsigned accel, uint32_t id);
/* Atomic M1..M4 start; signs are +/-1, steps=0 selects continuous motion.
 * Shared first compare and equal speed profiles; no effect on M5..M7. */
int motion_chassis_start(const int32_t signs[4], uint32_t steps, unsigned sps, unsigned accel, uint32_t id);
/* Finite heading-assisted translation. All wheels keep their translation DIR;
 * yaw adds/subtracts speed, never changes DIR. Distance is sum of wheel pulses
 * /4, not four equal endpoints. Correction is limited to one third of speed;
 * finish every STEP high before releasing ownership. M5..M7 are unaffected. */
int motion_assist_start(const int32_t signs[4], const int32_t yaw[4], uint32_t steps,
                        unsigned sps, unsigned accel, uint32_t id);
void motion_assist_correct(int32_t correction_sps);
/* Synchronized DDA wheel pulses for a straight translation. Zero wheels hold
 * low; sps/accel apply to the largest wheel count. Not for rotation or jogging. */
int motion_line_start(const int32_t steps[4], unsigned sps, unsigned accel, uint32_t id);
void motion_chassis_stop(void);
unsigned motion_chassis_active(void);
void motion_status(unsigned axis, axis_status_t *out);
unsigned motion_armed(void);
uint32_t motion_faults(void);
typedef struct {
    int32_t position, down_sign;
    unsigned initialized, active;
    uint32_t up_room, down_room;
    int32_t minimum, maximum;
} lift_status_t;
/* First initialization assumes physical top; subsequent calls preserve it.
 * These are pulse-count soft limits, not physical homing or position feedback. */
int motion_lift_init(void);
int motion_lift_direction(int32_t down_sign);
void motion_lift_status(lift_status_t *out);
const char *motion_lift_error(int32_t raw_steps);
typedef struct {
    int32_t position, minimum, maximum;
    unsigned referenced, configured, active;
} aux_status_t;
/* M5 uses downward depth; M6 uses raw signed pulses. No encoder/homing here.
 * Bounds and reference changes require disarmed, stationary axes. Raw MOVE
 * obeys M5 bounds and, after CONFIG, M6 bounds as well. */
int motion_aux_config(unsigned axis, int32_t minimum, int32_t maximum);
int motion_aux_zero(unsigned axis);
void motion_aux_status(unsigned axis, aux_status_t *out);
int motion_aux_move(unsigned axis, int32_t value, unsigned absolute,
                    unsigned sps, unsigned accel, uint32_t id);
void motion_aux_stop(unsigned axis);
#endif
