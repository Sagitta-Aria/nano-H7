#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>
/* RC servo PWM outputs on TIM15/PE4 and TIM12/PH9. Separate from the STEP/DIR axes: servos are
 * positional, hold no step counter and are not gated by ARM. A position is a
 * half-degree step (180 = 90.0 deg); the host speaks degrees to one decimal. */
typedef struct {
    uint16_t angle_tenths; /* e.g. 905 = 90.5 deg */
    uint16_t pulse_us;
    unsigned on;
} servo_status_t;

void servo_init(void);                                 /* 50 Hz output already running on return */
int servo_set_steps(unsigned channel, unsigned steps); /* 0 if past the travel limit or bad channel */
int servo_set_pulse_us(unsigned channel, unsigned us); /* bypasses the angle path; for calibration */
int servo_status(unsigned channel, servo_status_t *out);
unsigned servo_channels(void);
unsigned servo_pin(unsigned channel);
char servo_port(unsigned channel);                     /* GPIO letter for protocol metadata */
unsigned servo_timer(unsigned channel);                /* timer number for protocol metadata */
const char *servo_timer_suffix(unsigned channel);      /* N for a complementary physical output */
void servo_diagnostics(unsigned channel, char *out, unsigned capacity); /* read-only register snapshot */
unsigned servo_timer_channel(unsigned channel);
unsigned servo_steps_max(unsigned channel);           /* per-channel half-degree limit; 0 if invalid */
#endif
