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
enum { NAV_IDLE, NAV_TURNING, NAV_MOVING, NAV_ORIENTING, NAV_WAITING, NAV_DONE, NAV_STOPPED, NAV_BLOCKED, NAV_FAULT, NAV_NUDGING, NAV_SUSPENDED, NAV_CHECKPOINT };
static const char *const states[] = {"idle", "turning", "moving", "orienting", "waiting", "done", "stopped", "blocked", "fault", "nudging", "suspended", "checkpoint"};
static unsigned external_tracking;
/* Route and station table are uploaded, never hardcoded. Station names stay on
 * the host: NAV_STAT reports the index, so the board stores no name strings. */
static unsigned route[NAV_LEG_MAX], holds[NAV_LEG_MAX];
static unsigned station_count, leg_count;
static unsigned released;
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
static int32_t points[NAV_STATION_MAX][2], attitudes[NAV_STATION_MAX], previous[4];
static float x, y;
static float heading_sin, heading_cos = 1.0f;
static uint32_t point_mask, attitude_mask, leg_mask;
static unsigned state, ready, leg, mask, speed = 500, acceleration = 2000;
static uint32_t command_id, dwell = 3000, wait_started, turn_started;
static unsigned obstacle, translating, frame_set;
static int32_t heading, initial_heading, target_heading, turn_direction;
static int32_t departure_heading;
static uint32_t imu_time, imu_epoch, imu_revision, stable_revision, stable_last;
static unsigned imu_valid, stable_count;
static const char *fault = "none";
static unsigned micro_count, micro_steps, micro_speed;
static uint32_t micro_id, micro_started;
static uint32_t pause_seen;
/* Host-owned execution accepts exactly one bounded generic action at a time. */
static unsigned host_mode, host_next_leg;
static uint32_t host_op;
static int32_t host_x, host_y, host_hold_heading;
/* A heading recovery belongs to the current operation, never a new route. */
static unsigned recovering, recovery_braking, vector_travel;
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
/* Every row of both tables present. Built by shifting one bit at a time so a
 * count of 32 cannot overflow the shift the way (1U << count) - 1 would. */
static uint32_t full_mask(unsigned count)
{
    uint32_t bits = 0; unsigned i;
    for (i = 0; i < count && i < 32U; ++i) bits |= 1UL << i;
    return bits;
}
static int configured(void)
{
    if (!station_count || !leg_count) return 0;
    if (point_mask != full_mask(station_count)) return 0;
    if (attitude_mask != full_mask(station_count)) return 0;
    return leg_mask == full_mask(leg_count);
}
/* Station index of the current leg; 0 before a route exists. */
static unsigned station_of(unsigned index)
{ return index < leg_count ? route[index] : 0U; }
/* All upload entry points run under board_lock, only while wheels are idle. */
static int upload_idle(void)
{ return !navigation_busy() && !navigation_suspended() && !chassis_busy() && !motion_chassis_active(); }
int navigation_begin(uint32_t id, unsigned count, int32_t px, int32_t py, uint32_t now)
{
    uint32_t key = board_lock();
    if (!id || count < leg_count || count > NAV_PATH_MAX || !ready || !configured() ||
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
        index >= upload_count || task_leg >= leg_count || !coordinate_ok(px, py) ||
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
    if (!configured()) { board_unlock(key); return 0; }
    for (i = 0; i < station_count; ++i)
        if (!on_grid(points[i][0], points[i][1])) { board_unlock(key); return 0; }
    for (i = 0; i < received; ++i) {
        next[0] = staged[i].x; next[1] = staged[i].y;
        final = i + 1 == received || staged[i + 1].leg != expected;
        if (expected >= leg_count || staged[i].leg != expected || !on_grid(next[0], next[1]) ||
            !connected(last, next) || (!final && next[0] == last[0] && next[1] == last[1]) ||
            (final && (next[0] != points[route[expected]][0] || next[1] != points[route[expected]][1]))) {
            board_unlock(key); return 0;
        }
        last[0] = next[0]; last[1] = next[1];
        if (final) ++expected;
    }
    if (expected != leg_count) { board_unlock(key); return 0; }
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
/* Consume every wheel counter; forward and lateral travel contribute to XY.
 * Keep consuming the final high pulse after STOP; rotation never translates. */
static void update_pose(void)
{
    unsigned i;
    float delta[4], forward, lateral;
    axis_status_t axis;
    for (i = 0; i < 4; ++i) {
        motion_status(i, &axis);
        delta[i] = ((float)axis.position - previous[i]) * ((mask & (1U << i)) ? -1 : 1);
        previous[i] = axis.position;
    }
    if (translating || (state == NAV_SUSPENDED && external_tracking)) {
        /* The chassis sign rows for forward {1,1,1,1} and strafe-left
         * {1,-1,-1,1} are orthogonal, so a dot product with each recovers that
         * component; no matrix inverse and no wheel geometry are needed. Wheel
         * order is M1 rear-left, M2 rear-right, M3 front-left, M4 front-right.
         * The rotation row is deliberately not decoded: it would need wheelbase
         * and track, and turning in place never translates. */
        forward = (delta[0] + delta[1] + delta[2] + delta[3]) * .25f;
        lateral = (delta[0] - delta[1] - delta[2] + delta[3]) * .25f * NAV_LATERAL_SCALE;
        x += forward * heading_sin - lateral * heading_cos;
        y += forward * heading_cos + lateral * heading_sin;
        if (!motion_chassis_active()) translating = 0;
    }
}
unsigned navigation_busy(void)
{ return state == NAV_TURNING || state == NAV_MOVING || state == NAV_ORIENTING || state == NAV_WAITING || state == NAV_BLOCKED || state == NAV_NUDGING || state == NAV_CHECKPOINT; }
unsigned navigation_suspended(void) { return state == NAV_SUSPENDED; }
static void fail(const char *reason)
{ if (state == NAV_SUSPENDED) ready = 0; recovering = recovery_braking = 0; motion_chassis_stop(); state = NAV_FAULT; fault = reason; stable_count = 0; }
/* Task-only feedback. USB reconnect/reset terminates the mission instead of
 * silently accepting a different yaw origin. Heading units are centidegrees. */
void navigation_imu(int32_t angle_cd, unsigned valid, uint32_t epoch, uint32_t now)
{
    uint32_t key = board_lock();
    turn_feedback(angle_cd, valid, epoch, now);
    if (ready) update_pose();
    if (navigation_busy() || state == NAV_SUSPENDED) {
        if (imu_epoch != epoch) fail("imu_changed");
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
    point_mask |= 1UL;
    host_mode = 0;
    ready = 1; frame_set = translating = 0; state = NAV_IDLE; leg = 0; released = 0; obstacle = 0; fault = "none";
    departure_heading = 0;
    micro_count = micro_steps = 0;
    path_count = path_index = 0; clear_upload();
    board_unlock(key); return 1;
}
/* Extend RESET without changing its legacy O semantics. Input validation must
 * precede origin mutation; the initial yaw is captured only by fresh START. */
int navigation_reference(int32_t ox, int32_t oy, int32_t px, int32_t py, int32_t angle)
{
    uint32_t key;
    if (!coordinate_ok(px, py) || angle < -18000 || angle > 18000) return 0;
    key = board_lock();
    if (!navigation_origin(ox, oy)) { board_unlock(key); return 0; }
    x = px; y = py; departure_heading = wrap(angle);
    board_unlock(key); return 1;
}
/* Size the station table. Resizing drops stale rows from a differently shaped
 * mission but keeps the origin, because `ready` means pulse coordinates exist
 * and only a physical RESET or a moved origin may clear that. */
int navigation_stations(unsigned count)
{
    uint32_t key;
    if (count < 2U || count > NAV_STATION_MAX) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED) { board_unlock(key); return 0; }
    clear_upload();
    station_count = count;
    point_mask = ready ? (point_mask & 1UL) : 0;
    attitude_mask = 0;
    leg_count = leg_mask = 0;
    board_unlock(key); return 1;
}
int navigation_legs(unsigned count)
{
    uint32_t key;
    if (!count || count > NAV_LEG_MAX) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED || !station_count) { board_unlock(key); return 0; }
    clear_upload();
    leg_count = count; leg_mask = 0;
    board_unlock(key); return 1;
}
int navigation_leg(unsigned index, unsigned station, unsigned hold)
{
    uint32_t key;
    if (hold > 1U) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED || !leg_count || index >= leg_count || station >= station_count) {
        board_unlock(key); return 0;
    }
    clear_upload();
    route[index] = station; holds[index] = hold;
    leg_mask |= 1UL << index;
    board_unlock(key); return 1;
}
int navigation_point(unsigned index, int32_t px, int32_t py)
{
    uint32_t key;
    if (!coordinate_ok(px, py)) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED || !station_count || index >= station_count) { board_unlock(key); return 0; }
    if (index == 0 && (points[0][0] != px || points[0][1] != py)) ready = 0;
    clear_upload(); points[index][0] = px; points[index][1] = py;
    point_mask |= 1UL << index;
    board_unlock(key); return 1;
}
/* Arrival attitude is independent of the incoming segment bearing. */
int navigation_attitude(unsigned index, int32_t angle_cd)
{
    uint32_t key;
    if (angle_cd < -18000 || angle_cd > 18000) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED || !station_count || index >= station_count) { board_unlock(key); return 0; }
    clear_upload(); attitudes[index] = wrap(angle_cd);
    attitude_mask |= 1UL << index;
    board_unlock(key); return 1;
}
/* Release a held leg. The dwell timer restarts from the release so the
 * configured settle still applies after vision finishes. */
int navigation_go(uint32_t id, unsigned index, uint32_t now)
{
    uint32_t key = board_lock();
    if (!id || id != path_id || state != NAV_WAITING || index != leg ||
        !leg_count || !holds[leg] || released || micro_count || motion_chassis_active() ||
        fabsf(x - points[station_of(leg)][0]) > .51f || fabsf(y - points[station_of(leg)][1]) > .51f ||
        !motion_armed() || !feedback_fresh(now)) { board_unlock(key); return 0; }
    released = 1; wait_started = now;
    board_unlock(key); return 1;
}
/* Micro motions use the same heading assistance and pose integrator as cruise.
 * The total outward pulse budget bounds the entire local excursion, even when
 * alternate left/right corrections cancel in net displacement. */
int navigation_nudge(uint32_t id, unsigned index, unsigned dir, unsigned steps,
                     unsigned sps, unsigned accel, uint32_t operation, uint32_t now)
{
    static const int8_t rows[4][4] = {{1,1,1,1},{-1,-1,-1,-1},{1,-1,-1,1},{-1,1,1,-1}};
    unsigned i; int32_t signs[4], yaw[4]; uint32_t key = board_lock();
    if (!id || id != path_id || index != leg || state != NAV_WAITING || !leg_count ||
        !holds[leg] || released || dir > 3 || !steps || steps > NAV_MICRO_STEPS_MAX ||
        micro_steps + steps > NAV_MICRO_STEPS_MAX || micro_count >= NAV_MICRO_MAX ||
        !sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000 ||
        abs(wrap((host_mode ? host_hold_heading : attitudes[station_of(leg)]) - wrap(heading - initial_heading))) > (int32_t)turn_tolerance_cd() ||
        !motion_armed() || !feedback_fresh(now) || motion_chassis_active()) { board_unlock(key); return 0; }
    update_pose();
    for (i = 0; i < 4; ++i) {
        signs[i] = rows[dir][i] * ((mask & (1U << i)) ? -1 : 1);
        yaw[i] = ((i & 1U) ? -1 : 1) * ((mask & (1U << i)) ? -1 : 1);
    }
    if (!motion_assist_start(signs, yaw, steps, sps, accel, operation)) { board_unlock(key); return 0; }
    micro_steps += steps; micro_speed = sps; micro_id = operation;
    target_heading = (host_mode ? host_hold_heading : attitudes[station_of(leg)]); micro_started = now;
    translating = 1; state = NAV_NUDGING;
    board_unlock(key); return 1;
}
/* External-reference update, never an automatic snap after a wheel move.
 * The configured station is the vehicle reference pose after camera/tool
 * offset calibration. The host must verify the matching target before FIX. */
int navigation_fix(uint32_t id, unsigned index, uint32_t now)
{
    unsigned i, target; axis_status_t axis; int32_t error;
    uint32_t key = board_lock();
    if (!id || id != path_id || index != leg || state != NAV_WAITING || !leg_count ||
        !holds[leg] || released || !motion_armed() || !feedback_fresh(now) || motion_chassis_active()) { board_unlock(key); return 0; }
    target = station_of(leg); error = wrap((host_mode ? host_hold_heading : attitudes[target]) - wrap(heading - initial_heading));
    if (abs(error) > (int32_t)turn_tolerance_cd() || !on_grid(points[target][0], points[target][1])) { board_unlock(key); return 0; }
    update_pose();
    for (i = 0; i < 4; ++i) { motion_status(i, &axis); previous[i] = axis.position; }
    x = points[target][0]; y = points[target][1]; translating = 0;
    micro_count = micro_steps = 0;
    board_unlock(key); return 1;
}
/* Generic ownership transfer: no target, task order or movement distance is
 * stored here. The host owns localization while the executor is suspended. */
int navigation_suspend(uint32_t id, unsigned index, uint32_t now)
{
    uint32_t key = board_lock();
    if (!id || id != path_id || index != leg || !ready || !frame_set ||
        state != NAV_WAITING || !leg_count || !holds[leg] || released ||
        !motion_armed() || !feedback_fresh(now) || motion_chassis_active() || turn_busy()) {
        board_unlock(key); return 0;
    }
    update_pose(); state = NAV_SUSPENDED; translating = external_tracking = 0;
    board_unlock(key); return 1;
}
/* Explicit host localization in the existing departure frame. POSE starts
 * generic wheel odometry; RESTORE ends the handoff on a legal lane, discards
 * the old route and permits a new upload. Neither command changes yaw origin,
 * moves a wheel, arms outputs, or infers a station from a visual target. */
int navigation_locate(uint32_t id, unsigned index, int32_t px, int32_t py,
                      unsigned release, uint32_t now)
{
    uint32_t key = board_lock();
    if (!id || id != path_id || index != leg || !ready || !frame_set ||
        state != NAV_SUSPENDED || !coordinate_ok(px, py) || (release && !on_grid(px, py)) ||
        !motion_armed() || !feedback_fresh(now) || motion_chassis_active() || chassis_busy()) {
        board_unlock(key); return 0;
    }
    update_pose(); x = px; y = py; translating = 0; external_tracking = !release;
    if (release) { state = NAV_STOPPED; clear_upload(); }
    board_unlock(key); return 1;
}
/* Task-only report: snapshot under the IRQ lock, format with STEP IRQs enabled. */
void navigation_align_report(char *out, unsigned capacity)
{
    unsigned saved_state, saved_leg, pending, used;
    uint32_t saved_path, operation;
    uint32_t key = board_lock();
    saved_state = state; saved_leg = leg + 1U; saved_path = path_id;
    pending = micro_count; used = micro_steps; operation = micro_id;
    board_unlock(key);
    snprintf(out, capacity, "OK cmd=NAV_ALIGN v=2 suspend=1 state=%s path_id=%lu leg=%u pending=%u used=%u max=%u op_id=%lu",
        states[saved_state], (unsigned long)saved_path, saved_leg, pending, used,
        NAV_MICRO_STEPS_MAX, (unsigned long)operation);
}
int navigation_grid(unsigned axis, int32_t low, int32_t middle, int32_t high)
{
    uint32_t key;
    if (axis > 1 || !coordinate_ok(low, high) || !coordinate_ok(middle, 0) ||
        low >= middle || middle >= high) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED) { board_unlock(key); return 0; }
    grid[axis][0] = low; grid[axis][1] = middle; grid[axis][2] = high;
    path_count = path_index = 0; clear_upload();
    board_unlock(key); return 1;
}
int navigation_parameters(unsigned sps, unsigned accel, uint32_t dwell_ms)
{
    uint32_t key;
    if (!sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000 || dwell_ms > 60000) return 0;
    key = board_lock();
    if (navigation_busy() || state == NAV_SUSPENDED) { board_unlock(key); return 0; }
    clear_upload(); speed = sps; acceleration = accel; dwell = dwell_ms;  board_unlock(key); return 1;
}
/* Compatibility query only. The old firmware-owned excursion is retired. */
int navigation_inset(unsigned steps) { return steps == 0; }
void navigation_inset_report(char *out, unsigned capacity)
{ snprintf(out, capacity, "OK cmd=NAV_INSET v=1 steps=0 phase=0 retired=1"); }
static void begin_turn(unsigned next_state, int32_t angle_cd, uint32_t now, unsigned unused)
{
    (void)unused; state = next_state; target_heading = wrap(angle_cd); turn_started = now;
    stable_count = 0; stable_revision = imu_revision; turn_direction = 0;
}
/* Brake the translation, close yaw error, then recompute the remaining vector
 * from integrated XY. No STOP, fault latch, path reset or host retry is issued. */
static void recover_heading(uint32_t now)
{
    if (!host_mode && !vector_travel) {
        host_x = path[path_index][0]; host_y = path[path_index][1];
    }
    motion_chassis_stop();
    recovering = recovery_braking = 1;
    begin_turn(NAV_TURNING, target_heading, now, 0);
}
/* Map displacement to a heading-assisted pure translation. Destination and
 * yaw are supplied by the host; no field-centre or station rules live here. */
static int host_move_start(void)
{
    float angle = wrap(heading - initial_heading) * RAD_PER_CD;
    float dx = host_x - x, dy = host_y - y;
    float forward = dx * sinf(angle) + dy * cosf(angle);
    float lateral = (-dx * cosf(angle) + dy * sinf(angle)) / NAV_LATERAL_SCALE;
    int32_t steps[4], yaw[4]; unsigned i;
    for (i = 0; i < 4; ++i) {
        int32_t polarity = (mask & (1U << i)) ? -1 : 1;
        steps[i] = (int32_t)roundf(forward + ((i == 0U || i == 3U) ? lateral : -lateral)) * polarity;
        yaw[i] = ((i & 1U) ? -1 : 1) * polarity;
    }
    if (!motion_vector_start(steps, yaw, speed, acceleration, command_id)) return 0;
    translating = vector_travel = 1; state = NAV_MOVING; return 1;
}
/* Monotonic operation IDs supplement wire deduplication: an old action cannot
 * run again after its protocol cache entry expires. Every new action starts
 * from a stationary checkpoint and retains the original map/IMU session. */
int navigation_execute(uint32_t path, uint32_t op, unsigned kind, int32_t px,
                       int32_t py, int32_t angle, unsigned sps, unsigned accel, uint32_t now)
{
    uint32_t key = board_lock();
    if (!host_mode || path != path_id || !op || op <= host_op || !ready || obstacle ||
        !motion_armed() || !feedback_fresh(now) || motion_chassis_active() ||
        (state != NAV_CHECKPOINT && !(state == NAV_WAITING && kind >= 3U)) ||
        !sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000 ||
        angle < -18000 || angle > 18000 || kind > 4U ||
        (kind == 1U && !coordinate_ok(px,py))) { board_unlock(key); return 0; }
    update_pose();
    if (kind == 2U && (px < 0 || (unsigned)px != host_next_leg || (unsigned)px >= leg_count ||
        fabsf(x - points[route[px]][0]) > .51f || fabsf(y - points[route[px]][1]) > .51f)) {
        board_unlock(key); return 0;
    }
    if ((kind == 3U || kind == 4U) && (state != NAV_WAITING || (holds[leg] && !released) ||
        (kind == 4U && leg + 1U != leg_count))) { board_unlock(key); return 0; }
    host_op = op; command_id = op;
    speed = sps; acceleration = accel; turn_started = now;
    target_heading = wrap(angle); stable_count = 0; stable_revision = imu_revision; turn_direction = 0;
    if (kind == 0U) state = NAV_TURNING;
    else if (kind == 1U) {
        host_x = px; host_y = py;
        if (abs(wrap(target_heading - wrap(heading - initial_heading))) > NAV_TRAVEL_TOLERANCE_CD) recover_heading(now);
        else if (fabsf(x-px) <= .51f && fabsf(y-py) <= .51f) { x=px; y=py; state=NAV_CHECKPOINT; }
        else if (!host_move_start()) fail("move_start");
    } else if (kind == 2U) {
        leg = (unsigned)px; ++host_next_leg; host_hold_heading = target_heading;
        released = 0; wait_started = now; state = NAV_WAITING; micro_count = micro_steps = 0;
    } else state = kind == 4U ? NAV_DONE : NAV_CHECKPOINT;
    board_unlock(key); return 1;
}
void navigation_exec_report(char *out, unsigned capacity, uint32_t now)
{
    unsigned host, current_state, is_ready, valid, active, tolerance;
    uint32_t path, op, key = board_lock(); float px,py;
    int32_t angle,target; const char *reason;
    host=host_mode; path=path_id; op=host_op; current_state=state;
    px=x; py=y; angle=wrap(heading-initial_heading); target=target_heading;
    is_ready=ready; valid=feedback_fresh(now); active=motion_chassis_active();
    tolerance=turn_tolerance_cd(); reason=fault; board_unlock(key);
    snprintf(out, capacity, "OK cmd=NAV_EXEC v=1 host=%u path_id=%lu op_id=%lu state=%s x=%ld y=%ld heading_cd=%ld target_cd=%ld ready=%u imu=%u active=%u tolerance_cd=%u fault=%s paused=%u",
        host,(unsigned long)path,(unsigned long)op,states[current_state],
        (long)roundf(px),(long)roundf(py),(long)angle,(long)target,
        is_ready,valid,active,tolerance,reason,turn_imu_waiting(now));
}
/* Face only a cardinal direction. Intersections never trigger arrival dwell. */
static void launch_segment(uint32_t now)
{
    unsigned target = station_of(leg);
    int32_t dx, dy;
    if (path_index == path_count) begin_turn(NAV_ORIENTING, attitudes[target], now, 1);
    else {
        dx = path[path_index][0] - (int32_t)roundf(x);
        dy = path[path_index][1] - (int32_t)roundf(y);
        if ((dx && dy) || (!dx && !dy)) { fail("grid_segment"); return; }
        begin_turn(NAV_TURNING, dx ? (dx > 0 ? 9000 : -9000) : (dy > 0 ? 0 : 18000), now, 1);
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
    translating = 1; vector_travel = 0; state = NAV_MOVING; return 1;
}
/* A host turn ends at a checkpoint. Compatibility routes advance locally;
 * waiting-heading recovery returns to waiting without another host action. */
static void finish_turn(uint32_t now)
{
    if (recovering) {
        recovering = recovery_braking = 0; turn_started = now;
        if (fabsf(x-host_x) <= .51f && fabsf(y-host_y) <= .51f) {
            x = host_x; y = host_y;
            if (host_mode) state = NAV_CHECKPOINT;
            else { ++path_index; launch_segment(now); }
        } else if (!host_move_start()) fail("move_start");
        return;
    }
    if (host_mode && state == NAV_TURNING) { state = NAV_CHECKPOINT; return; }
    if (state == NAV_TURNING) { if (!launch_forward()) fail("move_start"); }
    else { state = NAV_WAITING; wait_started = now; }
}
/* Feedback P control needs no guessed pulses-per-degree conversion. Slow down
 * near the goal, stop before reversal, confirm several stationary samples. */
static void cruise_turn_tick(uint32_t now)
{
    int32_t error = wrap(target_heading - wrap(heading - initial_heading));
    unsigned magnitude = (unsigned)(error < 0 ? -error : error), i, sps;
    int32_t direction = error > 0 ? 1 : -1, signs[4], velocities[4];
    if (recovery_braking) {
        if (motion_chassis_active()) return;
        translating = 0; recovery_braking = 0;
    }
    if ((uint32_t)(now - turn_started) >= NAV_TURN_TIMEOUT_MS) { fail("turn_timeout"); return; }
    if (magnitude <= turn_tolerance_cd()) {
        motion_chassis_stop(); turn_direction = 0;
        if (motion_chassis_active()) { stable_count = 0; return; }
        if (imu_revision != stable_revision) {
            stable_revision = imu_revision;
            if (!stable_count || (uint32_t)(imu_time - stable_last) > NAV_SAMPLE_GAP_MS) {
                stable_count = 0;
            }
            stable_last = imu_time; ++stable_count;
            if (stable_count >= NAV_SETTLE_SAMPLES) {
                finish_turn(now);
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
static int start_navigation(uint32_t id, uint32_t requested_path, uint32_t now, unsigned host)
{
    unsigned i; axis_status_t axis; uint32_t key = board_lock();
    if (!committed || (requested_path && requested_path != path_id) || !ready || !configured() || obstacle || navigation_busy() || chassis_busy() || motion_chassis_active() || !motion_armed() || !feedback_fresh(now)) { board_unlock(key); return 0; }
    if (!on_grid((int32_t)roundf(x), (int32_t)roundf(y))) { fail("off_grid"); board_unlock(key); return 0; }
    for (i = 0; i < station_count; ++i)
        if (!on_grid(points[i][0], points[i][1])) { fail("off_grid"); board_unlock(key); return 0; }
    update_pose();
    if (upload_epoch != imu_epoch || start_x != (int32_t)roundf(x) || start_y != (int32_t)roundf(y)) {
        clear_upload(); board_unlock(key); return 0;
    }
    committed = 0;
    mask = chassis_invert_mask();
    for (i = 0; i < 4; ++i) { motion_status(i, &axis); previous[i] = axis.position; }
    host_mode = host; host_op = 0; host_next_leg = 0;
    if (!frame_set) {
        target_heading = departure_heading;
        /* Keep the field/grid frame across STOP/START. RESET establishes a new
         * origin; recapturing yaw here would rotate the physical lane map. */
        initial_heading = wrap(heading - departure_heading); frame_set = 1;
        heading_sin = sinf(departure_heading * RAD_PER_CD);
        heading_cos = cosf(departure_heading * RAD_PER_CD);
    }
    translating = recovering = recovery_braking = vector_travel = 0;
    leg = 0; released = 0; micro_count = micro_steps = 0; command_id = id; fault = "none"; if (host_mode) state = NAV_CHECKPOINT; else launch_leg(now); board_unlock(key); return 1;
}
int navigation_start(uint32_t id, uint32_t requested_path, uint32_t now)
{ return start_navigation(id,requested_path,now,0); }
int navigation_host_start(uint32_t id, uint32_t requested_path, uint32_t now)
{ return start_navigation(id,requested_path,now,1); }
void navigation_stop(void)
{
    uint32_t key = board_lock();
    turn_stop(); clear_upload(); recovering = recovery_braking = 0;
    if (ready) update_pose();
    if (state == NAV_SUSPENDED) ready = 0;
    if (navigation_busy() || state == NAV_SUSPENDED) state = NAV_STOPPED;
    external_tracking = 0;
    motion_chassis_stop(); stable_count = 0; board_unlock(key);
}
/* Manual wheel motion/calibration invalidates pulse coordinates. */
void navigation_invalidate(void)
{ uint32_t key = board_lock(); if (state != NAV_SUSPENDED) { ready = 0; translating = 0; clear_upload(); } board_unlock(key); }
void navigation_clear_fault(void)
{
    uint32_t key = board_lock();
    navigation_invalidate(); state = NAV_STOPPED; fault = "none";
    stable_count = 0; micro_count = micro_steps = 0;
    board_unlock(key);
}
void navigation_tick(uint32_t now)
{
    unsigned i, complete = 1; axis_status_t axis; uint32_t key = board_lock();
    uint32_t paused, elapsed;
    unsigned waiting = turn_imu_waiting(now);
    paused = turn_imu_paused_ms(now); elapsed = paused-pause_seen; pause_seen = paused;
    if (navigation_busy() || state == NAV_SUSPENDED) {
        turn_started += elapsed; wait_started += elapsed; micro_started += elapsed;
    }
    if (ready) update_pose();
    if (uploading && (uint32_t)(now - upload_time) >= NAV_UPLOAD_TIMEOUT_MS) clear_upload();
    if ((navigation_busy() || state == NAV_SUSPENDED) && !motion_armed()) { navigation_stop(); board_unlock(key); return; }
    if (waiting) { stable_count = 0; board_unlock(key); return; }
    if (state == NAV_TURNING || state == NAV_ORIENTING) cruise_turn_tick(now);
    else if (state == NAV_MOVING && (host_mode || vector_travel)) {
        int32_t error = wrap(target_heading - wrap(heading-initial_heading));
        if ((uint32_t)(now-turn_started) >= NAV_TURN_TIMEOUT_MS) fail("move_timeout");
        else if (abs(error) > NAV_TRAVEL_TOLERANCE_CD) recover_heading(now);
        else if (motion_chassis_active()) motion_assist_correct(turn_heading_correction(error,speed));
        else {
            for (i=0;i<4;++i) { motion_status(i,&axis); if (axis.remaining || axis.command_id != command_id) complete=0; }
            if (!complete) fail("move_interrupted");
            else if (fabsf(x-host_x) <= .51f && fabsf(y-host_y) <= .51f) {
                x=host_x; y=host_y; translating=0;
                if (host_mode) state=NAV_CHECKPOINT;
                else { ++path_index; launch_segment(now); }
            } else if (!host_move_start()) fail("move_start");
        }
    } else if (state == NAV_MOVING) {
        int32_t error = wrap(target_heading - wrap(heading - initial_heading));
        if (error > NAV_TRAVEL_TOLERANCE_CD || error < -NAV_TRAVEL_TOLERANCE_CD) recover_heading(now);
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
    } else if (state == NAV_NUDGING) {
        int32_t error = wrap(target_heading - wrap(heading - initial_heading));
        if ((uint32_t)(now - micro_started) >= 60000U) fail("micro_timeout");
        else if (motion_chassis_active()) {
            motion_assist_correct(turn_heading_correction(error, micro_speed));
        } else {
            for (i = 0; i < 4; ++i) {
                motion_status(i, &axis);
                if (axis.remaining || axis.command_id != micro_id) complete = 0;
            }
            if (!complete) fail("move_interrupted");
            else {
                ++micro_count; begin_turn(NAV_ORIENTING, (host_mode ? host_hold_heading : attitudes[station_of(leg)]), now, 0);
            }
        }
    } else if (state == NAV_WAITING) {
        int32_t arrival = (host_mode ? host_hold_heading : attitudes[station_of(leg)]);
        int32_t error = wrap(arrival - wrap(heading - initial_heading));
        if (error > (int32_t)turn_tolerance_cd() || error < -(int32_t)turn_tolerance_cd()) begin_turn(NAV_ORIENTING, arrival, now, 0);
        /* A held leg waits for NAV GO instead of the dwell timer, so vision can
         * finish before the next leg starts. Attitude is still maintained above
         * while held, and the dwell runs after the release. */
        else if (holds[leg] && !released) { /* hold */ }
        else if (!host_mode && (uint32_t)(now - wait_started) >= dwell) {
            released = 0;
            if (++leg == leg_count) { leg = leg_count - 1; state = NAV_DONE; }
            else launch_leg(now);
        }
    }
    board_unlock(key);
}
void navigation_obstacle(unsigned blocked)
{
    uint32_t key = board_lock(); obstacle = blocked != 0;
    if (obstacle && (navigation_busy() || state == NAV_SUSPENDED)) { navigation_stop(); state = NAV_BLOCKED; }
    board_unlock(key);
}
int navigation_replan(void) { return 0; }
/* Task-only telemetry. Snapshot every mutable field together, then release
 * STEP IRQs before rounding/formatting; MicroLIB printf can exceed an edge's
 * deadline. Formatting must never reread live navigation state after unlock. */
void navigation_report(char *out, unsigned capacity, uint32_t now)
{
    struct {
        float x, y;
        int32_t tx, ty, heading, target_heading, arrival, wx, wy;
        uint32_t wait_ms, move_id, path_id;
        unsigned state, ready, station, leg, obstacle, imu;
        unsigned segment, segments, route_ready, legs, stations, held, paused;
        const char *fault;
    } saved;
    uint32_t elapsed, key = board_lock();
    unsigned target;
    if (ready) update_pose(); elapsed = now - wait_started;
    saved.wait_ms = state == NAV_WAITING && elapsed < dwell ? dwell - elapsed : 0;
    target = station_of(leg);
    saved.state = state; saved.ready = ready; saved.x = x; saved.y = y;
    saved.station = target; saved.tx = points[target][0]; saved.ty = points[target][1];
    saved.leg = leg + 1U; saved.move_id = command_id; saved.obstacle = obstacle;
    saved.heading = frame_set ? wrap(heading - initial_heading) : 0;
    saved.target_heading = target_heading; saved.arrival = host_mode ? host_hold_heading : attitudes[target];
    saved.imu = feedback_fresh(now); saved.fault = fault;
    saved.wx = host_mode ? host_x : path_index < path_count ? path[path_index][0] : points[target][0];
    saved.wy = host_mode ? host_y : path_index < path_count ? path[path_index][1] : points[target][1];
    saved.segment = path_index < path_count ? path_index + 1U : path_count;
    saved.segments = path_count; saved.path_id = path_id; saved.route_ready = committed;
    saved.legs = leg_count; saved.stations = station_count;
    saved.held = state == NAV_WAITING && leg_count && holds[leg] && !released;
    saved.paused = turn_imu_waiting(now);
    board_unlock(key);
    /* `station` is an index: names live in the host config, so the board never
     * stores or reports a label it could disagree with. */
    snprintf(out, capacity, "OK cmd=NAV_STAT state=%s ready=%u x=%ld y=%ld station=%u tx=%ld ty=%ld leg=%u wait_ms=%lu move_id=%lu obstacle=%u planner=host_grid heading_cd=%ld target_cd=%ld arrival_cd=%ld imu=%u fault=%s wx=%ld wy=%ld segment=%u segments=%u path_id=%lu route_ready=%u legs=%u stations=%u held=%u paused=%u",
        states[saved.state], saved.ready, (long)roundf(saved.x), (long)roundf(saved.y),
        saved.station, (long)saved.tx, (long)saved.ty, saved.leg,
        (unsigned long)saved.wait_ms, (unsigned long)saved.move_id, saved.obstacle, (long)saved.heading,
        (long)saved.target_heading, (long)saved.arrival, saved.imu, saved.fault,
        (long)saved.wx, (long)saved.wy, saved.segment, saved.segments,
        (unsigned long)saved.path_id, saved.route_ready, saved.legs, saved.stations, saved.held, saved.paused);
}
