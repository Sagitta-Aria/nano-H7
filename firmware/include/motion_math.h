#ifndef MOTION_MATH_H
#define MOTION_MATH_H
#include <stdint.h>
/* Portable arithmetic shared by firmware and host-side C regression tests. */
uint32_t motion_isqrt(uint32_t value);
uint32_t motion_ramp(uint32_t current_msps, uint32_t target_msps, uint32_t accel);
uint32_t motion_interval(uint32_t sps, uint32_t *fraction);
#endif
