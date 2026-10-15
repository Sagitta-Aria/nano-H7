/* Independent H723 yaw turn. REL captures heading at acceptance; ABS uses an
 * operator-set RAM zero. No pulse-to-angle guess, route upload or XY reset.
 * Timer edges remain owned by motion.c; all shared state uses PRIMASK locks. */
#include "turn.h"
#include "navigation.h"
#include "chassis.h"
#include "motion.h"
#include "board.h"
#include <stdio.h>

enum { TURN_IDLE, TURNING, TURN_DONE, TURN_STOPPED, TURN_FAULT };
static const char *const names[] = {"idle", "turning", "done", "stopped", "fault"};
static unsigned state, valid, zero_ready, absolute_mode, mask, max_sps, acceleration;
static int32_t heading, zero_heading, origin, target, direction;
static uint32_t sample_time, epoch, revision, move_id, started;
static uint32_t stable_revision, stable_started, stable_last;
static unsigned stable_count;
static unsigned kp_centi = 2000U;
static unsigned tolerance_cd = NAV_HEADING_TOLERANCE_CD;
static const char *fault = "none";

/* Shortest signed yaw difference; +180 deterministically turns clockwise. */
static int32_t wrap(int32_t angle)
{ while (angle > 18000) angle -= 36000; while (angle <= -18000) angle += 36000; return angle; }
static unsigned fresh(uint32_t now)
{ return valid && (uint32_t)(now - sample_time) < NAV_IMU_TIMEOUT_MS; }
unsigned turn_busy(void) { return state == TURNING; }

/* Caller holds the IRQ lock. Finish STEP highs and retain the stopped result. */
static void fail(const char *reason)
{ motion_chassis_stop(); state = TURN_FAULT; fault = reason; direction = 0; }

/* Called by the NAV IMU bridge, including while no cruise or turn is active.
 * A changed H723 session invalidates zero; delayed data never resumes a turn. */
void turn_feedback(int32_t angle_cd, unsigned sample_valid, uint32_t sample_epoch, uint32_t now)
{
    uint32_t key = board_lock();
    chassis_imu_guard(sample_valid, sample_epoch, fresh(now));
    if (sample_epoch != epoch) {
        zero_ready = 0;
        if (turn_busy()) fail("imu_changed");
    } else if (turn_busy() && !fresh(now)) fail("imu_stale");
    heading = wrap(angle_cd); valid = sample_valid != 0;
    epoch = sample_epoch; sample_time = now; ++revision;
    if (!valid && turn_busy()) fail("imu_invalid");
    board_unlock(key);
}

/* Set only the turn reference while wheels are idle and yaw is fresh.
 * Does not arm outputs, move wheels, or alter navigation coordinates. */
int turn_zero(uint32_t now)
{
    uint32_t key = board_lock();
    if (!fresh(now) || chassis_busy() || motion_chassis_active()) { board_unlock(key); return 0; }
    zero_heading = origin = heading; zero_ready = 1; target = 0;
    state = TURN_IDLE; fault = "none";
    board_unlock(key); return 1;
}

/* Accept one exclusive task. ABS requires an explicit zero in this IMU session;
 * REL works without a zero. Accepted is not completion; poll TURN STAT. */
int turn_start(unsigned absolute, int32_t angle_cd, unsigned sps, unsigned accel,
               uint32_t id, uint32_t now)
{
    uint32_t key;
    if (absolute > 1 || angle_cd < -18000 || angle_cd > 18000 ||
        !sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000) return 0;
    key = board_lock();
    if (!motion_armed() || !fresh(now) || (absolute && !zero_ready) ||
        chassis_busy() || motion_chassis_active() || motion_faults()) { board_unlock(key); return 0; }
    absolute_mode = absolute; origin = absolute ? zero_heading : heading;
    target = wrap(angle_cd); max_sps = sps; acceleration = accel;
    mask = chassis_invert_mask(); move_id = id; started = now;
    stable_count = 0; stable_revision = revision; direction = 0;
    state = TURNING; fault = "none";
    board_unlock(key); return 1;
}

/* Stop this task only. Shared HALT/STOP calls this before releasing ownership;
 * a still-high STEP keeps new commands blocked until its falling edge. */
void turn_stop(void)
{
    uint32_t key = board_lock();
    if (turn_busy()) { motion_chassis_stop(); state = TURN_STOPPED; direction = 0; }
    board_unlock(key);
}

/* Configure shared P only while the wheels and both controllers are idle. */
int turn_config(unsigned kp)
{ return turn_configure(kp, turn_tolerance_cd()); }

/* Apply gain and completion band atomically; active wheel tasks reject edits. */
int turn_configure(unsigned kp, unsigned tolerance)
{
    uint32_t key;
    if (kp > INT32_MAX || tolerance < 1U || tolerance > 1000U) return 0;
    key = board_lock();
    if (chassis_busy() || motion_chassis_active()) { board_unlock(key); return 0; }
    kp_centi = kp; tolerance_cd = tolerance; board_unlock(key); return 1;
}
/* Shared cruise/standalone completion tolerance in centidegrees, RAM only. */
unsigned turn_tolerance_cd(void)
{ uint32_t key = board_lock(); unsigned value = tolerance_cd; board_unlock(key); return value; }
/* P in step/s/degree. Near target, permit 1 step/s rather than imposing the
 * former 30 step/s minimum. Caller max_sps replaces the hidden 600 cap. */
unsigned turn_control_speed(unsigned magnitude, unsigned limit)
{
    unsigned sps;
    uint64_t scaled;
    if (!kp_centi) return 0; /* P=0 disables yaw actuation, including the floor. */
    /* Large user gains must saturate before narrowing; 32-bit multiplication
     * can wrap and silently turn a large gain into a slow correction. */
    scaled = (uint64_t)magnitude * kp_centi + 5000U;
    if (scaled >= (uint64_t)limit * 10000U) return limit;
    sps = (unsigned)(scaled / 10000U);
    if (!sps) sps = 1U;
    if (sps > limit) sps = limit;
    return sps;
}
int turn_heading(uint32_t now, int32_t *angle_cd, uint32_t *session)
{
    uint32_t key = board_lock();
    unsigned available = fresh(now);
    *angle_cd = heading; *session = epoch;
    board_unlock(key); return available != 0;
}
int32_t turn_heading_correction(int32_t error_cd, unsigned max_sps)
{
    unsigned magnitude = (unsigned)(error_cd < 0 ? -error_cd : error_cd);
    int32_t correction;
    if (magnitude <= turn_tolerance_cd()) return 0;
    correction = (int32_t)turn_control_speed(magnitude, max_sps);
    return error_cd < 0 ? -correction : correction;
}
/* 1 ms controller; completion requires >=3 samples spanning 300 ms within
 * the configured tolerance. Stop before reversing; preserve freshness guards. */
void turn_tick(uint32_t now)
{
    int32_t error, wanted, signs[4], velocities[4];
    unsigned magnitude, sps, i;
    uint32_t key = board_lock();
    if (!turn_busy()) { board_unlock(key); return; }
    if (!motion_armed()) { turn_stop(); board_unlock(key); return; }
    if (!fresh(now)) { fail("imu_stale"); board_unlock(key); return; }
    if ((uint32_t)(now - started) >= NAV_TURN_TIMEOUT_MS) { fail("turn_timeout"); board_unlock(key); return; }
    error = wrap(target - wrap(heading - origin));
    magnitude = (unsigned)(error < 0 ? -error : error);
    if (magnitude <= tolerance_cd) {
        motion_chassis_stop(); direction = 0;
        if (motion_chassis_active()) { stable_count = 0; board_unlock(key); return; }
        if (revision != stable_revision) {
            stable_revision = revision;
            if (!stable_count || (uint32_t)(sample_time - stable_last) > NAV_SAMPLE_GAP_MS) {
                stable_started = sample_time; stable_count = 0;
            }
            stable_last = sample_time; ++stable_count;
            if (stable_count >= 3 && (uint32_t)(stable_last - stable_started) >= NAV_SETTLE_MS)
                state = TURN_DONE;
        }
        board_unlock(key); return;
    }
    stable_count = 0; stable_revision = revision;
    wanted = error > 0 ? 1 : -1;
    if (direction && direction != wanted) {
        motion_chassis_stop(); direction = 0; board_unlock(key); return;
    }
    sps = turn_control_speed(magnitude, max_sps);
    if (!sps) { motion_chassis_stop(); direction = 0; board_unlock(key); return; }
    for (i = 0; i < 4; ++i) {
        signs[i] = ((i & 1U) ? -wanted : wanted) * ((mask & (1U << i)) ? -1 : 1);
        velocities[i] = signs[i] * (int32_t)sps;
    }
    if (!motion_chassis_active()) {
        if (!motion_chassis_start(signs, 0, sps, acceleration, move_id)) fail("turn_start");
        else direction = wanted;
    } else if (direction == wanted && !motion_wheels(velocities, acceleration, move_id)) fail("turn_update");
    board_unlock(key);
}

/* Coherent command/status snapshot; current/target/error use the task's origin.
 * Feedback freshness is explicit: stale cached angles are never confirmation. */
void turn_report(char *out, unsigned capacity, uint32_t now)
{
    uint32_t key = board_lock();
    int32_t current = wrap(heading - origin);
    int32_t saved_target = target;
    unsigned saved_state = state, saved_mode = absolute_mode, saved_zero = zero_ready;
    unsigned saved_kp = kp_centi;
    unsigned saved_tolerance = tolerance_cd;
    unsigned saved_fresh = fresh(now), active = turn_busy() ? motion_chassis_active() : 0U;
    uint32_t id = move_id;
    const char *saved_fault = fault;
    board_unlock(key);
    snprintf(out, capacity, "OK cmd=TURN_STAT state=%s mode=%s zero=%u current_cd=%ld target_cd=%ld error_cd=%ld imu=%u move_id=%lu active=%u fault=%s kp_centi=%u tolerance_cd=%u",
        names[saved_state], saved_mode ? "ABS" : "REL", saved_zero, (long)current,
        (long)saved_target, (long)wrap(saved_target - current), saved_fresh, (unsigned long)id,
        active, saved_fault, saved_kp, saved_tolerance);
}
