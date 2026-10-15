#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <stdint.h>
#define NAV_COORD_MAX 1000000L
/* First departure after RESET defines +Y=0, clockwise +X=90. Pulse coordinates only.
 * RESET requires physically placing the car at O while disarmed. */
int navigation_origin(int32_t x, int32_t y);
int navigation_point(unsigned index, int32_t x, int32_t y);
int navigation_attitude(unsigned index, int32_t angle_cd);
/* Three strictly increasing lane centre coordinates per axis (0=X, 1=Y).
 * Task-only, idle configuration; all mission points must lie on this network.
 * This describes centreline routes, not a footprint/obstacle collision map. */
int navigation_grid(unsigned axis, int32_t low, int32_t middle, int32_t high);
/* Fresh verified H723 yaw, clockwise centidegrees, USB session epoch. */
void navigation_imu(int32_t angle_cd, unsigned valid, uint32_t epoch, uint32_t now_ms);
#define NAV_IMU_TIMEOUT_MS 300U
#define NAV_SAMPLE_GAP_MS 200U
#define NAV_SETTLE_MS 300U
/* Boot default only; runtime cruise/turn band comes from turn_tolerance_cd(). */
#define NAV_HEADING_TOLERANCE_CD 20L
#define NAV_TRAVEL_TOLERANCE_CD 500L
#define NAV_TURN_TIMEOUT_MS 60000U
int navigation_parameters(unsigned sps, unsigned accel, uint32_t dwell_ms);
#define NAV_PATH_MAX 64U
#define NAV_UPLOAD_TIMEOUT_MS 5000U
/* Idle-only transaction. STOP/config/reset invalidate staged and unused paths.
 * COMMIT checks CRC, lane edges and all five task arrivals; never starts motion. */
int navigation_begin(uint32_t path_id, unsigned count, int32_t x, int32_t y, uint32_t now_ms);
int navigation_waypoint(uint32_t path_id, unsigned index, int32_t x, int32_t y, unsigned leg, uint32_t now_ms);
int navigation_commit(uint32_t path_id, unsigned crc, uint32_t now_ms);
int navigation_start(uint32_t id, uint32_t path_id, uint32_t now_ms);
void navigation_tick(uint32_t now_ms);
void navigation_stop(void);
void navigation_invalidate(void);
unsigned navigation_busy(void);
void navigation_report(char *out, unsigned capacity, uint32_t now_ms);
/* Future ultrasonic hook: blocked stops and preserves the goal. Clearing it
 * never resumes motion. Replanning requires the future obstacle map/planner. */
void navigation_obstacle(unsigned blocked);
int navigation_replan(void);
#endif
