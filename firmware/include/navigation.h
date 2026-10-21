#ifndef NAVIGATION_H
#define NAVIGATION_H
#include <stdint.h>
#define NAV_COORD_MAX 1000000L
/* First departure after RESET defines +Y=0, clockwise +X=90. Pulse coordinates only.
 * RESET requires physically placing the car at O while disarmed. */
int navigation_origin(int32_t x, int32_t y);
/* Operator-confirmed O/X placement only, disarmed and idle. Keep station zero
 * at (ox,oy); fresh yaw at the first START represents map angle heading_cd.
 * No movement or sensor offset is issued, and STOP/START preserves the frame. */
int navigation_reference(int32_t ox, int32_t oy, int32_t x, int32_t y, int32_t heading_cd);
/* Legacy command retained as a disabled compatibility query. Inner moves and
 * all route policy are owned by the host; nonzero values are rejected. */
int navigation_inset(unsigned steps);
void navigation_inset_report(char *out, unsigned capacity);
/* Station table and route are sized by the host before any row is sent, so no
 * mission shape is assumed here. A station may appear in the route repeatedly;
 * its coordinate and arrival attitude are shared by every occurrence, while
 * the hold flag belongs to the individual leg. */
#define NAV_STATION_MAX 32U
#define NAV_LEG_MAX 32U
int navigation_stations(unsigned count);
int navigation_legs(unsigned count);
int navigation_leg(unsigned index, unsigned station, unsigned hold);
int navigation_point(unsigned index, int32_t x, int32_t y);
int navigation_attitude(unsigned index, int32_t angle_cd);
/* Release a leg that arrived with its hold flag set. Naming the leg keeps a
 * late release for the previous station from starting the next one. */
int navigation_go(uint32_t path_id, unsigned leg, uint32_t now_ms);
/* Held-leg work only: bounded body-axis corrections. FIX is the host's explicit
 * assertion that calibrated external vision verified this station's vehicle
 * reference pose; wheel counts alone never authorise that assertion. */
#define NAV_MICRO_MAX 16U
#define NAV_MICRO_STEPS_MAX 1000U
int navigation_nudge(uint32_t path_id, unsigned leg, unsigned direction,
    unsigned steps, unsigned sps, unsigned accel, uint32_t id, uint32_t now_ms);
int navigation_fix(uint32_t path_id, unsigned leg, uint32_t now_ms);
/* Generic external-controller handoff. Pose updates preserve the departure
 * yaw frame; release is explicit and requires an idle legal-lane coordinate. */
unsigned navigation_suspended(void);
int navigation_suspend(uint32_t path_id, unsigned leg, uint32_t now_ms);
int navigation_locate(uint32_t path_id, unsigned leg, int32_t x, int32_t y,
    unsigned release, uint32_t now_ms);
void navigation_align_report(char *out, unsigned capacity);
/* Three strictly increasing lane centre coordinates per axis (0=X, 1=Y).
 * Task-only, idle configuration; all mission points must lie on this network.
 * This describes centreline routes, not a footprint/obstacle collision map. */
int navigation_grid(unsigned axis, int32_t low, int32_t middle, int32_t high);
/* Fresh verified H723 yaw, clockwise centidegrees, USB session epoch. */
void navigation_imu(int32_t angle_cd, unsigned valid, uint32_t epoch, uint32_t now_ms);
#define NAV_IMU_TIMEOUT_MS 300U
#define NAV_SAMPLE_GAP_MS 200U
/* Finish on fresh stationary feedback, without an additional wall-clock dwell. */
#define NAV_SETTLE_SAMPLES 3U
/* Boot default only; runtime cruise/turn band comes from turn_tolerance_cd(). */
#define NAV_HEADING_TOLERANCE_CD 20L
#define NAV_TRAVEL_TOLERANCE_CD 500L
#define NAV_TURN_TIMEOUT_MS 60000U
int navigation_parameters(unsigned sps, unsigned accel, uint32_t dwell_ms);
/* Mecanum strafe loses travel per pulse to roller slip, so lateral distance is
 * not equal to forward distance for the same pulse count. 1.0f is the
 * undistorted geometric value; the real figure must be measured on the field
 * and is deliberately not guessed here. Keep in step with the host's
 * LATERAL_SCALE in navigation_sim.py. */
#define NAV_LATERAL_SCALE 1.0f
#define NAV_PATH_MAX 64U
#define NAV_UPLOAD_TIMEOUT_MS 5000U
/* Idle-only transaction. STOP/config/reset invalidate staged and unused paths.
 * COMMIT checks CRC, lane edges and every configured arrival; never starts motion. */
int navigation_begin(uint32_t path_id, unsigned count, int32_t x, int32_t y, uint32_t now_ms);
int navigation_waypoint(uint32_t path_id, unsigned index, int32_t x, int32_t y, unsigned leg, uint32_t now_ms);
int navigation_commit(uint32_t path_id, unsigned crc, uint32_t now_ms);
int navigation_start(uint32_t id, uint32_t path_id, uint32_t now_ms);
/* Generic NX executor: START HOST stops at a checkpoint. Kinds are TURN=0,
 * MOVE=1, ARRIVE=2, LEAVE=3, DONE=4. op must increase within this path.
 * MOVE holds supplied map yaw; ARRIVE declares a verified station arrival.
 * No automatic inner movement, arrival attitude choice or route advancement. */
int navigation_host_start(uint32_t id, uint32_t path, uint32_t now_ms);
int navigation_execute(uint32_t path, uint32_t op, unsigned kind, int32_t x,
    int32_t y, int32_t heading_cd, unsigned sps, unsigned accel, uint32_t now_ms);
void navigation_exec_report(char *out, unsigned capacity, uint32_t now_ms);
void navigation_tick(uint32_t now_ms);
void navigation_stop(void);
void navigation_invalidate(void);
/* MOTION CLEAR only, after all axes are idle/disarmed; discards the old route. */
void navigation_clear_fault(void);
unsigned navigation_busy(void);
void navigation_report(char *out, unsigned capacity, uint32_t now_ms);
/* Future ultrasonic hook: blocked stops and preserves the goal. Clearing it
 * never resumes motion. Replanning requires the future obstacle map/planner. */
void navigation_obstacle(unsigned blocked);
int navigation_replan(void);
#endif
