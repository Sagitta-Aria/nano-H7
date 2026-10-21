#ifndef TURN_H
#define TURN_H
#include <stdint.h>
/* Standalone yaw control, clockwise positive, centidegrees. Task-only APIs;
 * shares wheel ownership with chassis/navigation, never commands M5..M7. */
void turn_feedback(int32_t angle_cd, unsigned valid, uint32_t epoch, uint32_t now_ms);
int turn_zero(uint32_t now_ms);
int turn_start(unsigned absolute, int32_t angle_cd, unsigned sps, unsigned accel,
               uint32_t id, uint32_t now_ms);
/* Optional completion band belongs to one visual correction, not calibration. */
int turn_start_band(unsigned absolute, int32_t angle_cd, unsigned sps, unsigned accel,
                    uint32_t id, uint32_t now_ms, unsigned band_cd);
/* Shared IMU lease. Same-session recovery requires three fresh samples/100 ms.
 * Elapsed pause time freezes controller deadlines; source resets remain faults. */
unsigned turn_imu_waiting(uint32_t now_ms);
unsigned turn_imu_paused(void);
uint32_t turn_imu_paused_ms(uint32_t now_ms);
/* Shared turn/cruise gain and completion band; idle config, RAM only.
 * Gain units 0.01 step/s/degree, 0..INT32_MAX (wire storage only). P=0 disables
 * yaw actuation; speed calculation uses 64-bit saturation before narrowing.
 * Tolerance 1..1000 centidegrees. The legacy
 * one-argument API preserves tolerance; the joint API applies both atomically.
 * In-place turns use the adjustable floor (boot TURN_MIN_SPS) outside tolerance, capped by the task speed;
 * straight-line yaw correction retains fine P without that speed floor. */
int turn_config(unsigned kp_centi);
int turn_configure(unsigned kp_centi, unsigned tolerance_cd);
/* Shared rotation floor in step/s, RAM only; translating yaw stays fine P.
 * Atomically reject all three settings while any chassis owner is active. */
int turn_tune(unsigned kp_centi, unsigned tolerance_cd, unsigned min_sps);
unsigned turn_tolerance_cd(void);
unsigned turn_control_speed(unsigned error_cd, unsigned max_sps);
/* Snapshot fresh absolute yaw and IMU session for straight controllers. */
int turn_heading(uint32_t now_ms, int32_t *angle_cd, uint32_t *epoch);
/* Signed yaw correction, sharing the displayed P and tolerance deadband. */
int32_t turn_heading_correction(int32_t error_cd, unsigned max_sps);
void turn_tick(uint32_t now_ms);
void turn_stop(void);
/* Idle/disarmed fault acknowledgement, not a new yaw reference or motion. */
void turn_clear_fault(void);
unsigned turn_busy(void);
void turn_report(char *out, unsigned capacity, uint32_t now_ms);
#endif
