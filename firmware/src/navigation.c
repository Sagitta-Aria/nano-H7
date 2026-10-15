/* Host-planned lane-network executor. H723 yaw arrives via Nano; XY remains
 * pulse odometry. Upload validation never searches or invents a route. */
#include "navigation.h"
#include "motion.h"
#include "chassis.h"
#include "board.h"
#include "turn.h"
#include "protocol.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
enum { NAV_IDLE, NAV_TURNING, NAV_MOVING, NAV_ORIENTING, NAV_WAITING, NAV_DONE, NAV_STOPPED, NAV_BLOCKED, NAV_FAULT };
static const char *const states[] = {"idle", "turning", "moving", "orienting", "waiting", "done", "stopped", "blocked", "fault"};
static const unsigned route[] = {1, 2, 3, 4, 1};
static const char *const labels[] = {"O", "A", "B", "C", "D"};
static int32_t grid[2][3] = {{0, 14000, 28000}, {0, 14000, 28000}};
/* Full mission, with separate staging: incomplete uploads cannot execute. */
typedef struct { int32_t x, y; unsigned leg; } waypoint_t;
static waypoint_t staged[NAV_PATH_MAX], uploaded[NAV_PATH_MAX];
static int32_t path[NAV_PATH_MAX][2];
static unsigned path_count, path_index, uploaded_count, upload_count, received;
static unsigned uploading, committed;
static uint32_t path_id, upload_id, upload_epoch, upload_time;
static int32_t start_x, start_y;
static uint16_t upload_crc;
static int32_t points[5][2], attitudes[5], previous[4];
static float x, y;
static float heading_sin, heading_cos = 1.0f;
static unsigned state, ready, configured, leg, mask, speed = 500, acceleration = 2000;
static uint32_t command_id, dwell = 3000, wait_started, turn_started;
static unsigned obstacle, translating, frame_set;
static int32_t heading, initial_heading, target_heading, turn_direction;
static uint32_t imu_time, imu_epoch, imu_revision, stable_revision, stable_started, stable_last;
static unsigned imu_valid, stable_count;
static const char *fault = "none";
#define RAD_PER_CD (3.14159265358979323846f / 18000.0f)
/* Clockwise +Y=0; +180 and -180 are equivalent. */
static int32_t wrap(int32_t angle)
{ while (angle > 18000) angle -= 36000; while (angle <= -18000) angle += 36000; return angle; }
static int coordinate_ok(int32_t px, int32_t py)
{ return px >= -NAV_COORD_MAX && px <= NAV_COORD_MAX && py >= -NAV_COORD_MAX && py <= NAV_COORD_MAX; }
/* A valid centre lies on one bounded horizontal or vertical lane. */
static int on_grid(int32_t px, int32_t py)
{
    unsigned i;
    if (px < grid[0][0] || px > grid[0][2] || py < grid[1][0] || py > grid[1][2]) return 0;
    for (i = 0; i < 3; ++i) if (px == grid[0][i] || py == grid[1][i]) return 1;
    return 0;
}
/* Only collinear points on an actual lane can connect. An arbitrary L-shaped
 * route through the four cell interiors is deliberately not allowed. */
static int connected(const int32_t a[2], const int32_t b[2])
{
    unsigned i;
    for (i = 0; i < 3; ++i)
        if ((a[0] == b[0] && a[0] == grid[0][i]) ||
            (a[1] == b[1] && a[1] == grid[1][i])) return 1;
    return 0;
}
static void update_pose(void);
/* Cancel both staged and unconsumed routes after STOP or configuration edits. */
static void clear_upload(void)
{ uploading = committed = 0; received = 0; }
/* All upload entry points run under board_lock, only while wheels are idle. */
static int upload_idle(void)
{ return !navigation_busy() && !chassis_busy() && !motion_chassis_active(); }
int navigation_begin(uint32_t id, unsigned count, int32_t px, int32_t py, uint32_t now)
{
    uint32_t key = board_lock();
    if (!id || count < 5 || count > NAV_PATH_MAX || !ready || configured != 31U ||
        !upload_idle() || !coordinate_ok(px, py)) { board_unlock(key); return 0; }
    update_pose();
    if (px != (int32_t)roundf(x) || py != (int32_t)roundf(y) || !on_grid(px, py)) {
        clear_upload(); board_unlock(key); return 0;
    }
    clear_upload(); uploading = 1; upload_id = id; upload_count = count;
    start_x = px; start_y = py; upload_epoch = imu_epoch; upload_time = now;
    upload_crc = 0xffffU;
    board_unlock(key); return 1;
}
/* CRC16-CCITT-FALSE over canonical ASCII "index x y leg\n", in wire order. */
static uint16_t append_crc(uint16_t crc, const char *text)
{
    unsigned bit;
    while (*text) {
        crc ^= (uint16_t)(uint8_t)*text++ << 8;
        for (bit = 0; bit < 8; ++bit) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0));
    }
    return crc;
}
int navigation_waypoint(uint32_t id, unsigned index, int32_t px, int32_t py, unsigned task_leg, uint32_t now)
{
    char row[64]; uint32_t key = board_lock();
    if (!uploading || !upload_idle() || id != upload_id || index != received ||
        index >= upload_count || task_leg > 4 || !coordinate_ok(px, py) ||
        (uint32_t)(now - upload_time) >= NAV_UPLOAD_TIMEOUT_MS) { board_unlock(key); return 0; }
    staged[index].x = px; staged[index].y = py; staged[index].leg = task_leg;
    snprintf(row, sizeof(row), "%u %ld %ld %u\n", index, (long)px, (long)py, task_leg);
    upload_crc = append_crc(upload_crc, row); ++received; upload_time = now;
    board_unlock(key); return 1;
}
/* Validate the entire mission before publishing. Repeated-position entries
 * are allowed only at task arrivals, preserving zero-distance orientation. */
int navigation_commit(uint32_t id, unsigned crc, uint32_t now)
{
    unsigned i, expected = 0, final; int32_t last[2], next[2]; uint32_t key = board_lock();
    if (!uploading || !upload_idle() || id != upload_id || received != upload_count ||
        crc != upload_crc || upload_epoch != imu_epoch || !ready ||
        (uint32_t)(now - upload_time) >= NAV_UPLOAD_TIMEOUT_MS) { board_unlock(key); return 0; }
    update_pose(); last[0] = start_x; last[1] = start_y;
    if (last[0] != (int32_t)roundf(x) || last[1] != (int32_t)roundf(y)) { board_unlock(key); return 0; }
    for (i = 0; i < 5; ++i) if (!on_grid(points[i][0], points[i][1])) { board_unlock(key); return 0; }
    for (i = 0; i < received; ++i) {
        next[0] = staged[i].x; next[1] = staged[i].y;
        final = i + 1 == received || staged[i + 1].leg != expected;
        if (expected > 4 || staged[i].leg != expected || !on_grid(next[0], next[1]) ||
            !connected(last, next) || (!final && next[0] == last[0] && next[1] == last[1]) ||
            (final && (next[0] != points[route[expected]][0] || next[1] != points[route[expected]][1]))) {
            board_unlock(key); return 0;
        }
        last[0] = next[0]; last[1] = next[1];
        if (final) ++expected;
    }
    if (expected != 5) { board_unlock(key); return 0; }
    for (i = 0; i < received; ++i) uploaded[i] = staged[i];
    uploaded_count = received; path_id = id; committed = 1; uploading = 0;
    board_unlock(key); return 1;
}
/* Copy the selected task leg; Nano already decided every waypoint. */
static int load_leg(void)
{
    unsigned i; int32_t px = (int32_t)roundf(x), py = (int32_t)roundf(y);
    path_count = path_index = 0;
    for (i = 0; i < uploaded_count; ++i) if (uploaded[i].leg == leg) {
        if (uploaded[i].x == px && uploaded[i].y == py) continue;
        px = path[path_count][0] = uploaded[i].x;
        py = path[path_count][1] = uploaded[i].y; ++path_count;
    }
    return 1;
}
static int feedback_fresh(uint32_t now)
{ return imu_valid && (uint32_t)(now - imu_time) < NAV_IMU_TIMEOUT_MS; }
/* Consume every wheel counter, but only forward travel contributes to XY.
 * Keep consuming the final high pulse after STOP; rotation never translates. */
static void update_pose(void)
{
    unsigned i;
    float distance = 0;
    axis_status_t axis;
    for (i = 0; i < 4; ++i) {
        motion_status(i, &axis);
        distance += ((float)axis.position - previous[i]) * ((mask & (1U << i)) ? -1 : 1);
        previous[i] = axis.position;
    }
    if (translating) {
        x += distance * .25f * heading_sin; y += distance * .25f * heading_cos;
        if (!motion_chassis_active()) translating = 0;
    }
}
unsigned navigation_busy(void)
{ return state == NAV_TURNING || state == NAV_MOVING || state == NAV_ORIENTING || state == NAV_WAITING || state == NAV_BLOCKED; }
static void fail(const char *reason)
{ motion_chassis_stop(); state = NAV_FAULT; fault = reason; stable_count = 0; }
/* Task-only feedback. USB reconnect/reset terminates the mission instead of
 * silently accepting a different yaw origin. Heading units are centidegrees. */
void navigation_imu(int32_t angle_cd, unsigned valid, uint32_t epoch, uint32_t now)
{
    uint32_t key = board_lock();
    turn_feedback(angle_cd, valid, epoch, now);
    if (ready) update_pose();
    if (navigation_busy()) {
        if (imu_epoch != epoch) fail("imu_changed");
        else if (!feedback_fresh(now)) fail("imu_stale");
    }
    if (frame_set && imu_epoch != epoch) {
        /* Old XY cannot be rotated into a sensor origin that has been reset. */
        ready = 0; frame_set = translating = 0; clear_upload();
    }
    if (imu_epoch != epoch) clear_upload();
    heading = wrap(angle_cd); imu_valid = valid != 0;
    imu_time = now; imu_epoch = epoch; ++imu_revision;
    if (frame_set) {
        float angle = wrap(heading - initial_heading) * RAD_PER_CD;
        heading_sin = sinf(angle); heading_cos = cosf(angle);
    }
    if (!imu_valid && navigation_busy()) fail("imu_invalid");
    board_unlock(key);
}
int navigation_origin(int32_t px, int32_t py)
{
    unsigned i; axis_status_t axis; uint32_t key;
    if (!coordinate_ok(px, py)) return 0;
    key = board_lock();
    if (motion_armed() || navigation_busy() || chassis_busy() || motion_chassis_active()) { board_unlock(key); return 0; }
    for (i = 0; i < 4; ++i) { motion_status(i, &axis); previous[i] = axis.position; }
    mask = chassis_invert_mask(); x = px; y = py; points[0][0] = px; points[0][1] = py;
    ready = 1; frame_set = translating = 0; state = NAV_IDLE; leg = 0; obstacle = 0; fault = "none";
    path_count = path_index = 0; clear_upload();
    board_unlock(key); return 1;
}
int navigation_point(unsigned index, int32_t px, int32_t py)
{
    uint32_t key;
    if (index >= 5 || !coordinate_ok(px, py)) return 0;
    key = board_lock();
    if (navigation_busy()) { board_unlock(key); return 0; }
    if (index == 0 && (points[0][0] != px || points[0][1] != py)) ready = 0;
    clear_upload(); points[index][0] = px; points[index][1] = py; configured |= 1U << index;
    board_unlock(key); return 1;
}
/* Arrival attitude is independent of the incoming segment bearing. */
int navigation_attitude(unsigned index, int32_t angle_cd)
{
    uint32_t key;
    if (index >= 5 || angle_cd < -18000 || angle_cd > 18000) return 0;
    key = board_lock();
    if (navigation_busy()) { board_unlock(key); return 0; }
    clear_upload(); attitudes[index] = wrap(angle_cd); board_unlock(key); return 1;
}
int navigation_grid(unsigned axis, int32_t low, int32_t middle, int32_t high)
{
    uint32_t key;
    if (axis > 1 || !coordinate_ok(low, high) || !coordinate_ok(middle, 0) ||
        low >= middle || middle >= high) return 0;
    key = board_lock();
    if (navigation_busy()) { board_unlock(key); return 0; }
    grid[axis][0] = low; grid[axis][1] = middle; grid[axis][2] = high;
    path_count = path_index = 0; clear_upload();
    board_unlock(key); return 1;
}
int navigation_parameters(unsigned sps, unsigned accel, uint32_t dwell_ms)
{
    uint32_t key;
    if (!sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000 || dwell_ms > 60000) return 0;
    key = board_lock();
    if (navigation_busy()) { board_unlock(key); return 0; }
    clear_upload(); speed = sps; acceleration = accel; dwell = dwell_ms; board_unlock(key); return 1;
}
static void begin_turn(unsigned next_state, int32_t angle_cd, uint32_t now)
{
    state = next_state; target_heading = wrap(angle_cd); turn_started = now;
    stable_count = 0; stable_revision = imu_revision; turn_direction = 0;
}
/* Face only a cardinal direction. Intersections never trigger arrival dwell. */
static void launch_segment(uint32_t now)
{
    unsigned target = route[leg];
    int32_t dx, dy;
    if (path_index == path_count) begin_turn(NAV_ORIENTING, attitudes[target], now);
    else {
        dx = path[path_index][0] - (int32_t)roundf(x);
        dy = path[path_index][1] - (int32_t)roundf(y);
        if ((dx && dy) || (!dx && !dy)) { fail("grid_segment"); return; }
        begin_turn(NAV_TURNING, dx ? (dx > 0 ? 9000 : -9000) : (dy > 0 ? 0 : 18000), now);
    }
}
static void launch_leg(uint32_t now)
{
    if (!load_leg()) { fail("off_grid"); return; }
    launch_segment(now);
}
/* Heading-assisted translation along one grid axis, using the field bearing.
 * Capturing measured yaw here would accept alignment error as a new target. */
static int launch_forward(void)
{
    unsigned i;
    int32_t dx = path[path_index][0] - (int32_t)roundf(x);
    int32_t dy = path[path_index][1] - (int32_t)roundf(y);
    uint32_t steps = (uint32_t)(abs(dx) + abs(dy));
    int32_t signs[4], yaw[4];
    if (!steps || (dx && dy)) return 0;
    for (i = 0; i < 4; ++i) {
        signs[i] = (mask & (1U << i)) ? -1 : 1;
        yaw[i] = ((i & 1U) ? -1 : 1) * signs[i];
    }
    if (!motion_assist_start(signs, yaw, steps, speed, acceleration, command_id)) return 0;
    motion_assist_correct(turn_heading_correction(wrap(target_heading - wrap(heading - initial_heading)), speed));
    translating = 1; state = NAV_MOVING; return 1;
}
/* Feedback P control needs no guessed pulses-per-degree conversion. Slow down
 * near the goal, stop before reversal, confirm several stationary samples. */
static void cruise_turn_tick(uint32_t now)
{
    int32_t error = wrap(target_heading - wrap(heading - initial_heading));
    unsigned magnitude = (unsigned)(error < 0 ? -error : error), i, sps;
    int32_t direction = error > 0 ? 1 : -1, signs[4], velocities[4];
    if ((uint32_t)(now - turn_started) >= NAV_TURN_TIMEOUT_MS) { fail("turn_timeout"); return; }
    if (magnitude <= turn_tolerance_cd()) {
        motion_chassis_stop(); turn_direction = 0;
        if (motion_chassis_active()) { stable_count = 0; return; }
        if (imu_revision != stable_revision) {
            stable_revision = imu_revision;
            if (!stable_count || (uint32_t)(imu_time - stable_last) > NAV_SAMPLE_GAP_MS) {
                stable_started = imu_time; stable_count = 0;
            }
            stable_last = imu_time; ++stable_count;
            if (stable_count >= 3 && (uint32_t)(stable_last - stable_started) >= NAV_SETTLE_MS) {
                if (state == NAV_TURNING) { if (!launch_forward()) fail("move_start"); }
                else { state = NAV_WAITING; wait_started = now; }
            }
        }
        return;
    }
    stable_count = 0; stable_revision = imu_revision;
    if (turn_direction && turn_direction != direction) { motion_chassis_stop(); turn_direction = 0; return; }
    sps = turn_control_speed(magnitude, speed);
    if (!sps) { motion_chassis_stop(); turn_direction = 0; return; }
    for (i = 0; i < 4; ++i) {
        signs[i] = ((i & 1U) ? -direction : direction) * ((mask & (1U << i)) ? -1 : 1);
        velocities[i] = signs[i] * (int32_t)sps;
    }
    if (!motion_chassis_active()) {
        if (!motion_chassis_start(signs, 0, sps, acceleration, command_id)) { fail("turn_start"); return; }
        turn_direction = direction;
    } else if (turn_direction == direction) {
        if (!motion_wheels(velocities, acceleration, command_id)) fail("turn_update");
    }
}
int navigation_start(uint32_t id, uint32_t requested_path, uint32_t now)
{
    unsigned i; axis_status_t axis; uint32_t key = board_lock();
    if (!committed || (requested_path && requested_path != path_id) || !ready || configured != 31U || obstacle || navigation_busy() || chassis_busy() || motion_chassis_active() || !motion_armed() || !feedback_fresh(now)) { board_unlock(key); return 0; }
    if (!on_grid((int32_t)roundf(x), (int32_t)roundf(y))) { fail("off_grid"); board_unlock(key); return 0; }
    for (i = 0; i < 5; ++i)
        if (!on_grid(points[i][0], points[i][1])) { fail("off_grid"); board_unlock(key); return 0; }
    update_pose();
    if (upload_epoch != imu_epoch || start_x != (int32_t)roundf(x) || start_y != (int32_t)roundf(y)) {
        clear_upload(); board_unlock(key); return 0;
    }
    committed = 0;
    mask = chassis_invert_mask();
    for (i = 0; i < 4; ++i) { motion_status(i, &axis); previous[i] = axis.position; }
    if (!frame_set) {
        /* Keep the field/grid frame across STOP/START. RESET establishes a new
         * origin; recapturing yaw here would rotate the physical lane map. */
        initial_heading = heading; frame_set = 1;
        heading_sin = 0; heading_cos = 1.0f;
    }
    translating = 0;
    leg = 0; command_id = id; fault = "none"; launch_leg(now); board_unlock(key); return 1;
}
void navigation_stop(void)
{
    uint32_t key = board_lock();
    turn_stop(); clear_upload();
    if (ready) update_pose();
    if (navigation_busy()) state = NAV_STOPPED;
    motion_chassis_stop(); stable_count = 0; board_unlock(key);
}
/* Manual wheel motion/calibration invalidates pulse coordinates. */
void navigation_invalidate(void)
{ uint32_t key = board_lock(); ready = 0; translating = 0; clear_upload(); board_unlock(key); }
void navigation_tick(uint32_t now)
{
    unsigned i, complete = 1; axis_status_t axis; uint32_t key = board_lock();
    if (ready) update_pose();
    if (uploading && (uint32_t)(now - upload_time) >= NAV_UPLOAD_TIMEOUT_MS) clear_upload();
    if (navigation_busy() && !motion_armed()) { navigation_stop(); board_unlock(key); return; }
    if (navigation_busy() && !feedback_fresh(now)) { fail("imu_stale"); board_unlock(key); return; }
    if (state == NAV_TURNING || state == NAV_ORIENTING) cruise_turn_tick(now);
    else if (state == NAV_MOVING) {
        int32_t error = wrap(target_heading - wrap(heading - initial_heading));
        if (error > NAV_TRAVEL_TOLERANCE_CD || error < -NAV_TRAVEL_TOLERANCE_CD) fail("heading_deviation");
        else if (motion_chassis_active()) motion_assist_correct(turn_heading_correction(error, speed));
        else {
            for (i = 0; i < 4; ++i) { motion_status(i, &axis); if (axis.remaining || axis.command_id != command_id) complete = 0; }
            if (!complete) fail("move_interrupted");
            else {
                /* Commanded endpoint; wheel slip is not observable. */
                x = path[path_index][0]; y = path[path_index][1];
                ++path_index;
                launch_segment(now);
            }
        }
    } else if (state == NAV_WAITING) {
        int32_t error = wrap(attitudes[route[leg]] - wrap(heading - initial_heading));
        if (error > (int32_t)turn_tolerance_cd() || error < -(int32_t)turn_tolerance_cd()) begin_turn(NAV_ORIENTING, attitudes[route[leg]], now);
        else if ((uint32_t)(now - wait_started) >= dwell) {
            if (++leg == 5) { leg = 4; state = NAV_DONE; }
            else launch_leg(now);
        }
    }
    board_unlock(key);
}
void navigation_obstacle(unsigned blocked)
{
    uint32_t key = board_lock(); obstacle = blocked != 0;
    if (obstacle && navigation_busy()) { navigation_stop(); state = NAV_BLOCKED; }
    board_unlock(key);
}
int navigation_replan(void) { return 0; }
void navigation_report(char *out, unsigned capacity, uint32_t now)
{
    uint32_t remaining = 0, elapsed, key = board_lock();
    if (ready) update_pose(); elapsed = now - wait_started;
    if (state == NAV_WAITING && elapsed < dwell) remaining = dwell - elapsed;
    snprintf(out, capacity, "OK cmd=NAV_STAT state=%s ready=%u x=%ld y=%ld target=%s tx=%ld ty=%ld leg=%u wait_ms=%lu move_id=%lu obstacle=%u planner=host_grid heading_cd=%ld target_cd=%ld arrival_cd=%ld imu=%u fault=%s wx=%ld wy=%ld segment=%u segments=%u path_id=%lu route_ready=%u",
        states[state], ready, (long)roundf(x), (long)roundf(y), labels[route[leg]], (long)points[route[leg]][0], (long)points[route[leg]][1], leg + 1U,
        (unsigned long)remaining, (unsigned long)command_id, obstacle, (long)(frame_set ? wrap(heading - initial_heading) : 0),
        (long)target_heading, (long)attitudes[route[leg]], feedback_fresh(now), fault,
        (long)(path_index < path_count ? path[path_index][0] : points[route[leg]][0]),
        (long)(path_index < path_count ? path[path_index][1] : points[route[leg]][1]),
        path_index < path_count ? path_index + 1U : path_count, path_count, (unsigned long)path_id, committed);
    board_unlock(key);
}
