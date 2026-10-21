#ifndef CHASSIS_H
#define CHASSIS_H
#include <stdint.h>
typedef enum { CH_FWD, CH_BACK, CH_LEFT, CH_RIGHT, CH_CCW, CH_CW, CH_DIRECTION_COUNT } chassis_direction_t;
/* Relative wheel-step commands only: no world coordinates or odometry. */
int chassis_parse_direction(const char *text);
int chassis_move(unsigned direction, uint32_t steps, unsigned sps, unsigned accel, uint32_t id);
/* Finite translation only (FWD/BACK/LEFT/RIGHT), captures fresh yaw at start. */
int chassis_straight(unsigned direction, uint32_t steps, unsigned sps, unsigned accel,
                     uint32_t id, uint32_t now_ms);
/* Called before replacing IMU feedback; failures latch and cannot auto-resume. */
void chassis_imu_guard(unsigned valid, uint32_t epoch, unsigned previously_fresh);
unsigned chassis_imu_required(void);
int chassis_jog(unsigned direction, unsigned sps, unsigned accel, uint32_t id, uint32_t now_ms);
void chassis_halt(void);
/* Called by MOTION CLEAR after its low-level idle/disarmed check. */
void chassis_clear_fault(void);
void chassis_tick(uint32_t now_ms);
int chassis_config(unsigned invert_mask);
unsigned chassis_busy(void);
unsigned chassis_invert_mask(void);
void chassis_report(char *out, unsigned capacity);
#endif
