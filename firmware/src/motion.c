/* TIM8 CH1..4 and TIM5 CH1..3 independently schedule STEP edges.
 * IRQs (priority 4) never call FreeRTOS. PRIMASK locks protect short shared-state
 * updates against those IRQs; FreeRTOS BASEPRI critical sections are insufficient.
 * Position is commanded pulses only, not encoder feedback. No EN/limit pins yet. */
#include "motion.h"
#include "motion_math.h"
#include "board.h"
#include <string.h>
typedef struct {
    TIM_TypeDef *timer;
    GPIO_TypeDef *step_port, *dir_port;
    unsigned channel, step_pin, dir_pin, af;
} axis_hw_t;
static const axis_hw_t hw[AXIS_COUNT] = {
    {TIM8, GPIOI, GPIOI, 0, 5, 8, 3}, {TIM8, GPIOI, GPIOI, 1, 6, 3, 3},
    {TIM8, GPIOI, GPIOI, 2, 7, 4, 3}, {TIM8, GPIOI, GPIOC, 3, 2, 13, 3},
    {TIM5, GPIOH, GPIOC, 0, 10, 2, 2}, {TIM5, GPIOH, GPIOG, 1, 11, 11, 2},
    {TIM5, GPIOH, GPIOB, 2, 12, 9, 2}
};
typedef struct {
    int32_t position, direction;
    uint32_t remaining, command_id, current_msps, target_sps, accel, fraction;
    unsigned active, high, finite, stopping;
} axis_t;
static volatile axis_t axes[AXIS_COUNT];
static volatile unsigned armed;
static volatile uint32_t faults;
static volatile unsigned lift_initialized;
static volatile int32_t lift_down_sign = M5_DOWN_SIGN_DEFAULT;
static volatile int32_t aux_minimum[2] = {0, 0};
static volatile int32_t aux_maximum[2] = {M5_TRAVEL_PULSES, 0};
static volatile unsigned rotation_referenced, rotation_configured;
/* TIM8 CH1 clocks all four DDA outputs, including when M1 has zero pulses.
 * Only CH1 interrupts; all selected outputs toggle at the same compare tick. */
static volatile struct {
    unsigned active, high, stopping, selected;
    uint32_t total, remaining, amount[4], error[4], msps, sps, accel, fraction;
} line;
/* Quarter-pulse translation progress is independent of differential steering.
 * ISR owns progress; tasks own targets. No wheel reverses during this mode. */
static volatile struct {
    unsigned active, sps, accel;
    uint32_t remaining_q;
    int32_t correction, mix[4];
} assist;

/* Caller masks IRQs (or is the timer ISR). Validate the whole requested target
 * against the original top, including raw MOVE commands and signed direction. */
static const char *lift_error_locked(int32_t raw_steps)
{
    int64_t target;
    if (!lift_initialized) return "M5_NOT_REFERENCED";
    if (lift_down_sign != 1 && lift_down_sign != -1) return "M5_DIRECTION_UNKNOWN";
    target = ((int64_t)axes[4].position + raw_steps) * lift_down_sign;
    if (target < aux_minimum[0] || target > aux_maximum[0]) return "M5_SOFT_LIMIT";
    return 0;
}

/* Host handshake only establishes the top once per MCU boot. Reconnects,
 * repeated HELLO and retries must never create additional downward travel. */
int motion_lift_init(void)
{
    uint32_t key = board_lock();
    if (lift_initialized) { board_unlock(key); return 1; }
    if (axes[4].active) { board_unlock(key); return 0; }
    axes[4].position = 0;
    lift_initialized = 1;
    board_unlock(key);
    return 1;
}

/* Set which raw step sign means down, only disarmed at the established top.
 * It never moves, re-zeros, or changes the direction of an active move. */
int motion_lift_direction(int32_t down_sign)
{
    uint32_t key;
    if (down_sign != 1 && down_sign != -1) return 0;
    key = board_lock();
    if (!lift_initialized || armed || axes[4].active || axes[4].position != 0) {
        board_unlock(key); return 0;
    }
    lift_down_sign = down_sign;
    board_unlock(key);
    return 1;
}

/* Protocol preflight for a readable rejection; motion_move repeats the same
 * check under its own lock so telemetry/command races cannot bypass limits. */
const char *motion_lift_error(int32_t raw_steps)
{
    const char *error;
    uint32_t key = board_lock();
    error = lift_error_locked(raw_steps);
    board_unlock(key);
    return error;
}

/* One coherent snapshot; position is positive downward, raw AXIS 5 position
 * retains its existing signed-MOVE convention for other clients. */
void motion_lift_status(lift_status_t *out)
{
    uint32_t key = board_lock();
    int32_t position = axes[4].position * lift_down_sign;
    out->position = position;
    out->down_sign = lift_down_sign;
    out->initialized = lift_initialized;
    out->active = axes[4].active;
    out->up_room = position > aux_minimum[0] ? (uint32_t)(position - aux_minimum[0]) : 0;
    out->down_room = position < aux_maximum[0] ? (uint32_t)(aux_maximum[0] - position) : 0;
    out->minimum = aux_minimum[0]; out->maximum = aux_maximum[0];
    board_unlock(key);
}

static volatile uint32_t *ccr(const axis_hw_t *h) { return &h->timer->CCR1 + h->channel; }

/* Mode 4 forces inactive; mode 3 toggles on compare. No CCR preload is used. */
static void compare_mode(const axis_hw_t *h, unsigned mode)
{
    volatile uint32_t *reg = h->channel < 2 ? &h->timer->CCMR1 : &h->timer->CCMR2;
    unsigned shift = (h->channel % 2) * 8;
    *reg = (*reg & ~((0xFFUL << shift) | (1UL << (shift + 16)))) | (mode << (shift + 4));
}
static void halt(unsigned i)
{
    const axis_hw_t *h = &hw[i];
    h->timer->DIER &= ~(2UL << h->channel);
    compare_mode(h, 4);
    axes[i].active = axes[i].high = 0;
    axes[i].current_msps = axes[i].target_sps = 0;
}

void motion_init(void)
{
    unsigned i;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
    (void)RCC->APB1LENR;
    TIM8->CR1 = TIM5->CR1 = 0;
    TIM8->PSC = TIM5->PSC = TIMER_HZ / STEP_TICK_HZ - 1;
    TIM8->ARR = TIM5->ARR = 0xFFFFU;
    TIM8->DIER = TIM5->DIER = 0;
    TIM8->CCER = TIM5->CCER = 0;
    TIM8->EGR = TIM5->EGR = TIM_EGR_UG;
    TIM8->SR = TIM5->SR = 0;
    for (i = 0; i < AXIS_COUNT; ++i) {
        const axis_hw_t *h = &hw[i];
        h->dir_port->BSRR = 1UL << (h->dir_pin + 16);
        board_gpio(h->dir_port, h->dir_pin, 1, 0);
        compare_mode(h, 4);
        h->timer->CCER |= 1UL << (h->channel * 4);
        board_gpio(h->step_port, h->step_pin, 2, h->af);
    }
    TIM8->BDTR = TIM_BDTR_MOE;
    TIM8->CR1 = TIM5->CR1 = TIM_CR1_CEN;
    NVIC_SetPriority(TIM8_CC_IRQn, 4);
    NVIC_SetPriority(TIM5_IRQn, 4);
    NVIC_EnableIRQ(TIM8_CC_IRQn);
    NVIC_EnableIRQ(TIM5_IRQn);
}

/* Used for fault containment only; normal STOP preserves the final high pulse. */
void motion_emergency(void)
{
    unsigned i;
    uint32_t key = board_lock();
    armed = 0;
    assist.active = 0;
    line.active = 0;
    for (i = 0; i < AXIS_COUNT; ++i) halt(i);
    board_unlock(key);
}
void motion_stop(void)
{
    unsigned i;
    uint32_t key = board_lock();
    armed = 0;
    assist.active = 0;
    if (line.active) {
        line.stopping = 1;
        if (!line.high) { line.active = 0; for (i = 0; i < 4; ++i) halt(i); }
    }
    for (i = 0; i < AXIS_COUNT; ++i) {
        if (i < 4 && line.active) continue;
        axes[i].stopping = 1;
        if (!axes[i].high) halt(i);
    }
    board_unlock(key);
}
int motion_arm(unsigned enable)
{
    unsigned i;
    uint32_t key;
    if (!enable) { motion_stop(); return 1; }
    key = board_lock();
    if (faults) { board_unlock(key); return 0; }
    for (i = 0; i < AXIS_COUNT; ++i) if (axes[i].active) { board_unlock(key); return 0; }
    armed = 1;
    board_unlock(key);
    return 1;
}

/* Called with IRQs masked. Initial low interval gives DIR >=200 us setup. */
static void start_axis(unsigned i, int32_t direction, unsigned sps, unsigned accel,
                       unsigned finite, uint32_t steps, uint32_t id, uint32_t first_compare)
{
    const axis_hw_t *h = &hw[i];
    axes[i].direction = direction;
    axes[i].target_sps = sps;
    axes[i].current_msps = (sps < 10U ? sps : 10U) * 1000U;
    axes[i].accel = accel;
    axes[i].remaining = steps;
    axes[i].finite = finite;
    axes[i].command_id = id;
    axes[i].fraction = axes[i].high = axes[i].stopping = 0;
    axes[i].active = 1;
    /* Invert the M5 pin only, keeping signed position/soft limits coherent. */
    h->dir_port->BSRR = 1UL << (h->dir_pin +
        (((direction > 0) ^ (i == 4U && M5_DIR_INVERT)) ? 0 : 16));
    *ccr(h) = first_compare;
    h->timer->SR = ~(2UL << h->channel);
    compare_mode(h, 3);
    h->timer->DIER |= 2UL << h->channel;
}
int motion_move(unsigned i, int32_t steps, unsigned sps, unsigned accel, uint32_t id)
{
    uint32_t key;
    if (i >= AXIS_COUNT || !steps || steps > MAX_MOVE_STEPS || steps < -MAX_MOVE_STEPS ||
        sps < 1 || sps > (i < 4 ? CHASSIS_SPS_MAX : AUX_SPS_MAX) || accel < 1 || accel > 20000) return 0;
    key = board_lock();
    if (!armed || (i < 4 && line.active) || axes[i].active || (i == 4U && lift_error_locked(steps)) ||
        (i == 5U && rotation_configured &&
         ((int64_t)axes[i].position + steps < aux_minimum[1] ||
          (int64_t)axes[i].position + steps > aux_maximum[1]))) {
        board_unlock(key); return 0;
    }
    start_axis(i, steps > 0 ? 1 : -1, sps, accel, 1, steps > 0 ? (uint32_t)steps : (uint32_t)-steps, id,
               (hw[i].timer->CNT + 10U) & 0xFFFFU);
    board_unlock(key);
    return 1;
}
/* Coherent logical positions, including whether M6 has an operator reference. */
void motion_aux_status(unsigned axis, aux_status_t *out)
{
    unsigned n = axis - 5U;
    uint32_t key = board_lock();
    if (n > 1U) { memset(out, 0, sizeof(*out)); board_unlock(key); return; }
    out->position = axes[axis - 1U].position * (axis == 5U ? lift_down_sign : 1);
    out->minimum = aux_minimum[n]; out->maximum = aux_maximum[n];
    out->referenced = axis == 5U ? lift_initialized : rotation_referenced;
    out->configured = axis == 5U ? 1U : rotation_configured;
    out->active = axes[axis - 1U].active;
    board_unlock(key);
}

/* Changing bounds never moves or re-zeros an axis. Refuse to exclude its pose. */
int motion_aux_config(unsigned axis, int32_t minimum, int32_t maximum)
{
    unsigned n = axis - 5U;
    int32_t position;
    uint32_t key;
    if (n > 1U || minimum < -AUX_POSITION_MAX || maximum > AUX_POSITION_MAX || minimum > maximum ||
        (axis == 5U && (minimum < 0 || minimum == maximum))) return 0;
    key = board_lock();
    position = axes[axis - 1U].position * (axis == 5U ? lift_down_sign : 1);
    if (axes[axis - 1U].active ||
        (axis == 5U ? !lift_initialized : !rotation_referenced) ||
        position < minimum || position > maximum) { board_unlock(key); return 0; }
    aux_minimum[n] = minimum; aux_maximum[n] = maximum;
    if (axis == 6U) rotation_configured = 1;
    board_unlock(key); return 1;
}

/* Explicit M6 reference at an operator-chosen physical zero; not homing. */
int motion_aux_zero(unsigned axis)
{
    uint32_t key = board_lock();
    if (axis != 6U || armed || axes[5].active ||
        (rotation_configured && (aux_minimum[1] > 0 || aux_maximum[1] < 0))) {
        board_unlock(key); return 0;
    }
    axes[5].position = 0; rotation_referenced = 1;
    board_unlock(key); return 1;
}

/* Absolute target/delta conversion and pulse launch share one IRQ lock.
 * A second click cannot use stale host telemetry to overshoot either bound. */
int motion_aux_move(unsigned axis, int32_t value, unsigned absolute,
                    unsigned sps, unsigned accel, uint32_t id)
{
    int32_t position, sign;
    int64_t target, steps;
    unsigned n = axis - 5U;
    int result;
    uint32_t key;
    if (n > 1U || !sps || sps > AUX_SPS_MAX || !accel || accel > 20000) return 0;
    key = board_lock();
    sign = axis == 5U ? lift_down_sign : 1;
    if (!armed || axes[axis - 1U].active ||
        (axis == 5U ? (!lift_initialized || (sign != 1 && sign != -1)) :
                     (!rotation_referenced || !rotation_configured))) {
        board_unlock(key); return 0;
    }
    position = axes[axis - 1U].position * sign;
    target = absolute ? value : (int64_t)position + value;
    if (target < aux_minimum[n] || target > aux_maximum[n]) { board_unlock(key); return 0; }
    steps = (target - position) * sign;
    result = !steps || motion_move(axis - 1U, (int32_t)steps, sps, accel, id);
    board_unlock(key); return result;
}

/* Finish an existing high pulse; stop only this auxiliary axis, retaining ARM. */
void motion_aux_stop(unsigned axis)
{
    uint32_t key = board_lock();
    if (axis == 5U || axis == 6U) {
        axes[axis - 1U].stopping = 1;
        if (!axes[axis - 1U].high) halt(axis - 1U);
    }
    board_unlock(key);
}
/* Atomic four-wheel target update. Reversal while moving is rejected: command
 * zero, wait for all wheels to stop, then submit the opposite direction. */
int motion_wheels(const int32_t sps[4], unsigned accel, uint32_t id)
{
    unsigned i;
    uint32_t key;
    if (accel < 1 || accel > 20000) return 0;
    for (i = 0; i < 4; ++i) if (sps[i] < -(int32_t)CHASSIS_SPS_MAX || sps[i] > (int32_t)CHASSIS_SPS_MAX) return 0;
    key = board_lock();
    if (!armed || line.active || assist.active) { board_unlock(key); return 0; }
    for (i = 0; i < 4; ++i) {
        if (axes[i].active && (axes[i].finite || axes[i].stopping ||
            (sps[i] != 0 && ((sps[i] > 0 ? 1 : -1) != axes[i].direction)))) {
            board_unlock(key); return 0;
        }
    }
    for (i = 0; i < 4; ++i) {
        unsigned magnitude = (unsigned)(sps[i] < 0 ? -sps[i] : sps[i]);
        if (!axes[i].active && magnitude) start_axis(i, sps[i] > 0 ? 1 : -1, magnitude, accel, 0, 0, id,
                                                   (TIM8->CNT + 10U) & 0xFFFFU);
        else { axes[i].target_sps = magnitude; axes[i].accel = accel; axes[i].command_id = id; }
    }
    board_unlock(key);
    return 1;
}

/* Group motion is all-or-nothing. Preflight every wheel before touching DIR.
 * All four compare registers receive the same launch tick, 1 ms in the future. */
int motion_chassis_start(const int32_t signs[4], uint32_t steps, unsigned sps, unsigned accel, uint32_t id)
{
    unsigned i;
    uint32_t launch, key;
    if (steps > MAX_MOVE_STEPS || !sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000) return 0;
    for (i = 0; i < 4; ++i) if (signs[i] != 1 && signs[i] != -1) return 0;
    key = board_lock();
    if (!armed || faults || line.active) { board_unlock(key); return 0; }
    for (i = 0; i < 4; ++i) if (axes[i].active) { board_unlock(key); return 0; }
    launch = (TIM8->CNT + 50U) & 0xFFFFU;
    for (i = 0; i < 4; ++i) start_axis(i, signs[i], sps, accel, steps != 0, steps, id, launch);
    board_unlock(key);
    return 1;
}

/* Launch continuous pulse generators under one lock, then attach an exact
 * aggregate distance budget. The caller supplies calibrated translation/yaw. */
int motion_assist_start(const int32_t signs[4], const int32_t yaw[4], uint32_t steps,
                        unsigned sps, unsigned accel, uint32_t id)
{
    unsigned i;
    int32_t sum = 0;
    uint32_t key;
    if (!steps || steps > MAX_MOVE_STEPS) return 0;
    for (i = 0; i < 4; ++i) {
        if ((signs[i] != 1 && signs[i] != -1) || (yaw[i] != 1 && yaw[i] != -1)) return 0;
        sum += signs[i] * yaw[i];
    }
    if (sum) return 0; /* Pure translation must be orthogonal to yaw. */
    key = board_lock();
    if (!motion_chassis_start(signs, 0, sps, accel, id)) { board_unlock(key); return 0; }
    assist.remaining_q = steps * 4U; assist.correction = 0;
    assist.sps = sps; assist.accel = accel;
    for (i = 0; i < 4; ++i) {
        assist.mix[i] = signs[i] * yaw[i]; axes[i].remaining = steps;
    }
    assist.active = 1;
    board_unlock(key); return 1;
}
void motion_assist_correct(int32_t correction_sps)
{
    uint32_t key = board_lock();
    if (assist.active) assist.correction = correction_sps;
    board_unlock(key);
}

/* Prepare the next common rising edge. Rounding error stays below one wheel
 * pulse along the whole segment, regardless of the acceleration profile. */
static void line_select(void)
{
    unsigned i;
    line.selected = 0;
    for (i = 0; i < 4; ++i) {
        line.error[i] += line.amount[i];
        if (line.error[i] >= line.total) {
            line.error[i] -= line.total;
            line.selected |= 1U << i;
        }
        compare_mode(&hw[i], (line.selected & (1U << i)) ? 3U : 4U);
    }
}

int motion_line_start(const int32_t steps[4], unsigned sps, unsigned accel, uint32_t id)
{
    unsigned i;
    uint32_t key, total = 0, launch;
    if (!sps || sps > CHASSIS_SPS_MAX || !accel || accel > 20000) return 0;
    for (i = 0; i < 4; ++i) {
        uint32_t amount;
        if (steps[i] < -MAX_MOVE_STEPS || steps[i] > MAX_MOVE_STEPS) return 0;
        amount = steps[i] < 0 ? (uint32_t)-steps[i] : (uint32_t)steps[i];
        if (amount > total) total = amount;
    }
    if (!total) return 0;
    key = board_lock();
    if (!armed || faults || line.active) { board_unlock(key); return 0; }
    for (i = 0; i < 4; ++i) {
        int64_t end = (int64_t)axes[i].position + steps[i];
        if (axes[i].active || end < INT32_MIN || end > INT32_MAX) { board_unlock(key); return 0; }
    }
    line.total = line.remaining = total;
    line.sps = sps; line.accel = accel;
    line.msps = (sps < 10U ? sps : 10U) * 1000U;
    line.fraction = line.high = line.stopping = 0;
    launch = (TIM8->CNT + 50U) & 0xFFFFU;
    TIM8->DIER &= ~0x1EU;
    for (i = 0; i < 4; ++i) {
        line.amount[i] = steps[i] < 0 ? (uint32_t)-steps[i] : (uint32_t)steps[i];
        line.error[i] = total / 2U;
        start_axis(i, steps[i] < 0 ? -1 : 1, sps, accel, 1, line.amount[i], id, launch);
        axes[i].active = line.amount[i] != 0;
    }
    line_select();
    TIM8->DIER = (TIM8->DIER & ~0x1EU) | 2U;
    TIM8->SR = ~0x1EU;
    line.active = 1;
    board_unlock(key);
    return 1;
}

/* Finish any current high pulse, suppress further steps, retain global ARM. */
void motion_chassis_stop(void)
{
    unsigned i;
    uint32_t key = board_lock();
    assist.active = 0;
    if (line.active) {
        line.stopping = 1;
        if (line.high) { board_unlock(key); return; }
        line.active = 0;
    }
    for (i = 0; i < 4; ++i) {
        axes[i].stopping = 1;
        if (!axes[i].high) halt(i);
    }
    board_unlock(key);
}

unsigned motion_chassis_active(void)
{
    unsigned i, mask = 0;
    uint32_t key = board_lock();
    for (i = 0; i < 4; ++i) if (axes[i].active) mask |= 1U << i;
    if (line.active && !mask) mask = 1U;
    board_unlock(key);
    return mask;
}

/* 1 kHz trajectory update. Braking envelope v^2 <= 2*a*remaining, with exact
 * pulse-count termination in IRQ. Final speed/ramp must be scoped on hardware. */
void motion_tick(void)
{
    unsigned i;
    /* One bounded snapshot/update prevents a common TIM8 edge from splitting
     * the four equal-profile updates between old and new velocities. */
    uint32_t outer_key = board_lock();
    if (assist.active) {
        uint32_t target = assist.sps, remaining = (assist.remaining_q + 3U) / 4U;
        int32_t correction = assist.correction, limit, base;
        if (remaining <= target * target / (2U * assist.accel)) {
            uint32_t brake = motion_isqrt(2U * assist.accel * remaining);
            if (brake < target) target = brake;
        }
        if (!target) target = 1;
        limit = (int32_t)(target / 3U);
        if (correction > limit) correction = limit;
        if (correction < -limit) correction = -limit;
        /* Requested speed is a per-wheel ceiling. Symmetric mix keeps mean
         * translation separate from yaw; ramp both sides without reversing. */
        base = (int32_t)target - (correction < 0 ? -correction : correction);
        for (i = 0; i < 4; ++i)
            axes[i].target_sps = (unsigned)(base + assist.mix[i] * correction);
    }
    if (line.active && !line.stopping) {
        uint32_t target = line.sps;
        if (line.remaining <= target * target / (2U * line.accel)) {
            uint32_t brake = motion_isqrt(2U * line.accel * line.remaining);
            if (brake < target) target = brake;
        }
        if (!target) target = 1;
        line.msps = motion_ramp(line.msps, target * 1000U, line.accel);
    }
    for (i = 0; i < AXIS_COUNT; ++i) {
        uint32_t target, remaining, brake, key = board_lock();
        if ((i < 4 && line.active) || !axes[i].active || axes[i].stopping) { board_unlock(key); continue; }
        target = axes[i].target_sps;
        remaining = axes[i].remaining;
        if (axes[i].finite) {
            if (remaining == 0) { axes[i].stopping = 1; board_unlock(key); continue; }
            /* Saturate before multiplication; no 32-bit overflow on long moves. */
            if (remaining <= (target * target) / (2U * axes[i].accel)) {
                brake = motion_isqrt(2U * axes[i].accel * remaining);
                if (brake < target) target = brake;
            }
            if (target == 0) target = 1;
        }
        axes[i].current_msps = motion_ramp(axes[i].current_msps, target * 1000U, axes[i].accel);
        if (axes[i].current_msps == 0) {
            axes[i].stopping = 1;
            if (!axes[i].high) halt(i);
        }
        board_unlock(key);
    }
    board_unlock(outer_key);
}

static void compare_irq(unsigned first, unsigned count)
{
    unsigned i;
    for (i = first; i < first + count; ++i) {
        const axis_hw_t *h = &hw[i];
        uint32_t flag = 2UL << h->channel;
        uint32_t next, interval, fraction, sps;
        if (!(h->timer->SR & h->timer->DIER & flag)) continue;
        h->timer->SR = ~flag;
        if (!axes[i].active) continue;
        axes[i].high ^= 1U;
        if (axes[i].high) {
            /* Avoid signed overflow after very long continuous operation. */
            if ((axes[i].direction > 0 && axes[i].position == INT32_MAX) ||
                (axes[i].direction < 0 && axes[i].position == INT32_MIN)) {
                ++faults; motion_emergency(); return;
            }
            axes[i].position += axes[i].direction;
            if (i < 4U && assist.active) {
                unsigned j;
                --assist.remaining_q;
                for (j = 0; j < 4; ++j) axes[j].remaining = (assist.remaining_q + 3U) / 4U;
                if (!assist.remaining_q) {
                    /* Stop all further rising edges exactly at sum=4*steps.
                     * Other pending high pulses still receive falling edges. */
                    assist.active = 0;
                    for (j = 0; j < 4; ++j) {
                        axes[j].stopping = 1;
                        if (!axes[j].high) halt(j);
                    }
                }
            }
            if (axes[i].finite && axes[i].remaining) --axes[i].remaining;
        } else if (axes[i].stopping || (axes[i].finite && !axes[i].remaining) ||
                   (i == 4U && lift_error_locked(axes[i].direction))) {
            /* At a lift boundary, finish this high pulse and prevent the next
             * rising edge even if a future caller miscounts remaining steps. */
            halt(i); continue;
        }
        sps = axes[i].current_msps / 1000U;
        if (!sps) sps = 1;
        fraction = axes[i].fraction;
        interval = motion_interval(sps, &fraction);
        axes[i].fraction = fraction;
        next = (*ccr(h) + interval) & 0xFFFFU;
        /* Signed modular distance works because half-period <=25000 ticks. */
        if ((int16_t)(uint16_t)(next - h->timer->CNT) <= 1) {
            ++faults; motion_emergency(); return;
        }
        *ccr(h) = next;
    }
}
/* All selected pins already changed electrically at the common compare.
 * Count that edge, finish highs on stop, then arm the next DDA selection. */
static void line_irq(void)
{
    unsigned i;
    uint32_t next, interval, fraction, sps;
    if (!(TIM8->SR & TIM8->DIER & 2U)) return;
    TIM8->SR = ~0x1EU;
    line.high ^= 1U;
    for (i = 0; i < 4; ++i) {
        axes[i].high = line.high && (line.selected & (1U << i));
        if (axes[i].high) {
            axes[i].position += axes[i].direction;
            if (axes[i].remaining) --axes[i].remaining;
        }
    }
    if (line.high) --line.remaining;
    else {
        if (line.stopping || !line.remaining) {
            line.active = 0;
            for (i = 0; i < 4; ++i) halt(i);
            return;
        }
        line_select();
    }
    sps = line.msps / 1000U;
    if (!sps) sps = 1;
    fraction = line.fraction;
    interval = motion_interval(sps, &fraction);
    line.fraction = fraction;
    next = (TIM8->CCR1 + interval) & 0xFFFFU;
    if ((int16_t)(uint16_t)(next - TIM8->CNT) <= 1) {
        ++faults; motion_emergency(); return;
    }
    for (i = 0; i < 4; ++i) *ccr(&hw[i]) = next;
}
void TIM8_CC_IRQHandler(void) { if (line.active) line_irq(); else compare_irq(0, 4); }
void TIM5_IRQHandler(void) { compare_irq(4, 3); }
void motion_status(unsigned i, axis_status_t *out)
{
    uint32_t key = board_lock();
    out->position = axes[i].position;
    out->sps = (int32_t)(axes[i].current_msps / 1000U) * axes[i].direction;
    if (i < 4 && line.active)
        out->sps = (int32_t)(((uint64_t)line.msps * line.amount[i]) / line.total / 1000U) * axes[i].direction;
    out->remaining = axes[i].remaining;
    out->command_id = axes[i].command_id;
    out->active = axes[i].active;
    board_unlock(key);
}
unsigned motion_armed(void) { return armed; }
uint32_t motion_faults(void) { return faults; }
