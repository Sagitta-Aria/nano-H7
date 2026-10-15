/* Body primitives in raw M1..M4 order: rear-left, rear-right, front-left,
 * front-right. Reverse only FWD/BACK from v0.2.5 per observed direction;
 * lateral and clockwise/counter-clockwise commands retain their polarity.
 * Hardware DIR polarity is separate from this wheel-direction matrix.
 * Called from tasks only; short PRIMASK sections serialize comm and planner. */
#include "chassis.h"
#include "motion.h"
#include "board.h"
#include "navigation.h"
#include "turn.h"
#include <stdio.h>
#include <string.h>
static const char *const names[] = {"FWD", "BACK", "LEFT", "RIGHT", "CCW", "CW"};
static const int8_t signs[6][4] = {
    { 1, 1, 1, 1}, {-1,-1,-1,-1}, { 1,-1,-1, 1},
    {-1, 1, 1,-1}, {-1, 1,-1, 1}, { 1,-1, 1,-1}
};
enum { IDLE, MOVING, DONE, STOPPED, JOGGING, SWITCHING, FAULT };
static const char *const states[] = {"idle", "moving", "done", "stopped", "jogging", "switching", "fault"};
static unsigned mode, direction, pending_direction, speed, acceleration;
static unsigned invert_mask = CHASSIS_INVERT_MASK;
static uint32_t move_id, total_steps, last_jog;
static unsigned assisted;
static int32_t straight_target, straight_error;
static uint32_t straight_epoch;
static const char *straight_fault = "none";
static int32_t wrap(int32_t angle)
{ while (angle > 18000) angle -= 36000; while (angle <= -18000) angle += 36000; return angle; }
static void straight_fail(const char *reason)
{ motion_chassis_stop(); mode = FAULT; straight_fault = reason; }

void chassis_imu_guard(unsigned valid, uint32_t epoch, unsigned previously_fresh)
{
    if (mode != MOVING || !assisted) return;
    if (epoch != straight_epoch) straight_fail("imu_changed");
    else if (!previously_fresh) straight_fail("imu_stale");
    else if (!valid) straight_fail("imu_invalid");
}

int chassis_parse_direction(const char *text)
{
    unsigned i;
    for (i = 0; i < CH_DIRECTION_COUNT; ++i) if (strcmp(text, names[i]) == 0) return (int)i;
    return -1;
}

static int launch(unsigned dir, uint32_t steps, unsigned sps, unsigned accel, uint32_t id)
{
    unsigned i;
    int32_t physical[4];
    for (i = 0; i < 4; ++i) physical[i] = signs[dir][i] * ((invert_mask & (1U << i)) ? -1 : 1);
    return motion_chassis_start(physical, steps, sps, accel, id);
}

/* Owning a jog also blocks raw wheel commands during a pending direction change. */
unsigned chassis_busy(void) { return mode == MOVING || mode == JOGGING || mode == SWITCHING || navigation_busy() || turn_busy(); }
unsigned chassis_invert_mask(void) { return invert_mask; }

int chassis_move(unsigned dir, uint32_t steps, unsigned sps, unsigned accel, uint32_t id)
{
    uint32_t key;
    if (dir >= CH_DIRECTION_COUNT || !steps || steps > MAX_MOVE_STEPS) return 0;
    key = board_lock();
    if (chassis_busy() || !launch(dir, steps, sps, accel, id)) { board_unlock(key); return 0; }
    direction = dir; total_steps = steps; move_id = id; mode = MOVING;
    assisted = 0; straight_fault = "none";
    navigation_invalidate();
    board_unlock(key);
    return 1;
}

int chassis_straight(unsigned dir, uint32_t steps, unsigned sps, unsigned accel,
                     uint32_t id, uint32_t now)
{
    unsigned i;
    int32_t physical[4], yaw[4], angle;
    uint32_t epoch, key;
    if (dir > CH_RIGHT || !steps || steps > MAX_MOVE_STEPS) return 0;
    key = board_lock();
    if (chassis_busy() || !turn_heading(now, &angle, &epoch)) { board_unlock(key); return 0; }
    for (i = 0; i < 4; ++i) {
        int32_t polarity = (invert_mask & (1U << i)) ? -1 : 1;
        physical[i] = signs[dir][i] * polarity;
        yaw[i] = signs[CH_CW][i] * polarity;
    }
    if (!motion_assist_start(physical, yaw, steps, sps, accel, id)) { board_unlock(key); return 0; }
    straight_target = angle; straight_epoch = epoch; straight_error = 0;
    straight_fault = "none"; assisted = 1; speed = sps;
    direction = dir; total_steps = steps; move_id = id; mode = MOVING;
    navigation_invalidate();
    board_unlock(key); return 1;
}

/* GUI must renew this lease, independently of transport PING. A blocked GUI
 * cannot leave motion latched while its serial worker continues heartbeat. */
int chassis_jog(unsigned dir, unsigned sps, unsigned accel, uint32_t id, uint32_t now_ms)
{
    uint32_t key;
    if (dir >= CH_DIRECTION_COUNT || !sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000) return 0;
    key = board_lock();
    if (!motion_armed() || mode == MOVING || navigation_busy() || turn_busy()) { board_unlock(key); return 0; }
    if (mode != JOGGING && mode != SWITCHING) {
        if (!launch(dir, 0, sps, accel, id)) { board_unlock(key); return 0; }
        direction = dir; mode = JOGGING; move_id = id;
        assisted = 0; straight_fault = "none";
    } else if (dir != direction || sps != speed || accel != acceleration || mode == SWITCHING) {
        /* Complete the current high pulse before changing DIR. No new rising
         * edge is allowed during SWITCHING; the next launch has a DIR guard. */
        motion_chassis_stop(); mode = SWITCHING; move_id = id;
    }
    pending_direction = dir; speed = sps; acceleration = accel;
    total_steps = 0; last_jog = now_ms;
    navigation_invalidate();
    board_unlock(key);
    return 1;
}

/* Stop only M1..M4, cancel pending changes, and preserve M5..M7 operation. */
void chassis_halt(void)
{
    uint32_t key = board_lock();
    motion_chassis_stop(); mode = STOPPED;
    navigation_stop();
    turn_stop();
    board_unlock(key);
}

void chassis_tick(uint32_t now_ms)
{
    axis_status_t s;
    unsigned i, complete = 1;
    uint32_t key = board_lock();
    if (chassis_busy() && !motion_armed()) { chassis_halt(); board_unlock(key); return; }
    if ((mode == JOGGING || mode == SWITCHING) && (uint32_t)(now_ms - last_jog) >= CHASSIS_JOG_TIMEOUT_MS) {
        chassis_halt(); board_unlock(key); return;
    }
    if (mode == SWITCHING && !motion_chassis_active()) {
        if (launch(pending_direction, 0, speed, acceleration, move_id)) {
            direction = pending_direction; mode = JOGGING;
        } else mode = STOPPED;
    }
    if (mode == MOVING && assisted) {
        int32_t angle; uint32_t epoch;
        if (!turn_heading(now_ms, &angle, &epoch)) straight_fail("imu_stale");
        else if (epoch != straight_epoch) straight_fail("imu_changed");
        else {
            straight_error = wrap(straight_target - angle);
            if (straight_error > NAV_TRAVEL_TOLERANCE_CD || straight_error < -NAV_TRAVEL_TOLERANCE_CD)
                straight_fail("heading_deviation");
            else motion_assist_correct(turn_heading_correction(straight_error, speed));
        }
    }
    if (mode == MOVING && !motion_chassis_active()) {
        for (i = 0; i < 4; ++i) {
            motion_status(i, &s);
            if (s.remaining || s.command_id != move_id) complete = 0;
        }
        mode = complete ? DONE : STOPPED;
    }
    board_unlock(key);
}

/* RAM-only calibration while all wheels are stationary. Raw axis commands
 * bypass this inversion; use them to determine which DIR means wheel-forward. */
int chassis_config(unsigned mask)
{
    uint32_t key = board_lock();
    if (mask > 15 || chassis_busy() || motion_chassis_active()) { board_unlock(key); return 0; }
    invert_mask = mask;
    navigation_invalidate();
    board_unlock(key);
    return 1;
}

void chassis_report(char *out, unsigned capacity)
{
    axis_status_t s;
    unsigned i, saved_mode, saved_dir, saved_mask, active, saved_assisted;
    int32_t saved_target, saved_error;
    const char *saved_fault;
    uint32_t id, total, remaining = 0, key = board_lock();
    saved_mode = mode; saved_dir = direction; saved_mask = invert_mask;
    active = motion_chassis_active(); id = move_id; total = total_steps;
    saved_assisted = assisted; saved_target = straight_target;
    saved_error = straight_error; saved_fault = straight_fault;
    for (i = 0; i < 4; ++i) {
        motion_status(i, &s);
        if (s.command_id == id && s.remaining > remaining) remaining = s.remaining;
    }
    board_unlock(key); /* Formatting must not delay STEP timer interrupts. */
    snprintf(out, capacity, "OK cmd=CHASSIS state=%s direction=%s move_id=%lu total=%lu remaining=%lu active=%u invert=%u assist=%u target_cd=%ld error_cd=%ld fault=%s",
             states[saved_mode], names[saved_dir], (unsigned long)id, (unsigned long)total,
             (unsigned long)remaining, active, saved_mask, saved_assisted, (long)saved_target,
             (long)saved_error, saved_fault);
}
