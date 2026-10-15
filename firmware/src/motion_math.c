#include "motion_math.h"
#include "app_config.h"
/* Integer floor square root without floating-point operations in the planner. */
uint32_t motion_isqrt(uint32_t value)
{
    uint32_t result = 0, bit = 1UL << 30;
    while (bit > value) bit >>= 2;
    while (bit) {
        if (value >= result + bit) { value -= result + bit; result = (result >> 1) + bit; }
        else result >>= 1;
        bit >>= 2;
    }
    return result;
}
/* One 1 ms update, velocity in milli-steps/s, acceleration in steps/s^2. */
uint32_t motion_ramp(uint32_t current_msps, uint32_t target_msps, uint32_t accel)
{
    if (current_msps < target_msps) return target_msps - current_msps <= accel ? target_msps : current_msps + accel;
    return current_msps - target_msps <= accel ? target_msps : current_msps - accel;
}
/* Fractional divider: average step rate is exact for constant integer sps.
 * 50 kHz timer => edge quantization <20 us; 1 sps half-period fits 16 bits. */
uint32_t motion_interval(uint32_t sps, uint32_t *fraction)
{
    uint32_t denom, ticks;
    if (sps == 0) return 0;
    denom = 2U * sps;
    ticks = STEP_TICK_HZ / denom;
    *fraction %= denom;
    *fraction += STEP_TICK_HZ % denom;
    if (*fraction >= denom) { ++ticks; *fraction -= denom; }
    return ticks;
}
