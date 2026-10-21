#ifndef APP_CONFIG_H
#define APP_CONFIG_H
/* Board: Waiken STM32H743IIT6 CB V1.5, external VCORE supply, 25 MHz HSE.
 * Change these only with the board schematic and timer clock tree in hand. */
#define APP_VERSION "0.2.29"
#define CPU_HZ 400000000UL
#define APB_HZ 100000000UL
#define TIMER_HZ 200000000UL
#define UART_BAUD 115200UL
#define AXIS_COUNT 7U
#define CHASSIS_SPS_MAX 5000U
/* In-place yaw only: avoid crawling outside the angle tolerance. The task's
 * maximum speed still wins; translating yaw correction keeps its fine P. */
#define TURN_MIN_SPS 200U
#define AUX_SPS_MAX 3000U
#define STEP_TICK_HZ 50000UL
#define LINK_TIMEOUT_MS 1000U
/* User wheel assignment: M1=rear-left, M2=rear-right, M3=front-left,
 * M4=front-right. User verified mask 5 on 2026-10-11: invert M1/M3 DIR.
 * chassis.c applies this electrical polarity to its logical direction table. */
#define CHASSIS_INVERT_MASK 5U
#define CHASSIS_JOG_TIMEOUT_MS 400U
#define DEFAULT_ACCEL 2000U
#define MAX_MOVE_STEPS 10000000L
/* M5 is a lift: its first host connection is at the physical top.
 * Downward travel is cumulative, not a per-command allowance. Direction must
 * be configured explicitly (+1/-1 raw MOVE sign); 0 blocks M5 until known. */
#define M5_TRAVEL_PULSES 8000L
#define M5_DOWN_SIGN_DEFAULT 0
/* Reverse only M5 electrical DIR relative to v0.2.5. Logical step signs and
 * cumulative lift depth keep their meaning; do not also swap UP/DOWN in hosts. */
#define M5_DIR_INVERT 1U
/* Host-selected auxiliary travel bounds are RAM-only and count pulses. */
#define AUX_POSITION_MAX 1000000L
#define RTOS_HEAP_BYTES (48U * 1024U)
#define FRAME_MAX 384U
/* Positional servos: S1/PE4 retains 270 degrees; S2/PH9 is 180 degrees.
 * One TIM15/TIM12 tick is 1 us. Both use provisional 500..2500 us endpoints;
 * confirm them on the actual servos. Travel does not determine electrical
 * pulse limits. Recompute the corresponding servo.c lookup table if changed.
 * Steps are half degrees. S1 boots at 90 degrees, S2 at 90 degrees/1500 us.
 * These mappings are not suitable for continuous-rotation speed servos. */
#define SERVO_CHANNELS 2U
#define SERVO_PWM_HZ 50U
#define SERVO_PULSE_MIN_US 500U
#define SERVO_PULSE_MAX_US 2500U
#define SERVO_STEPS_MAX 540U
#define SERVO_HOME_STEPS 180U
#define SERVO2_STEPS_MAX 360U
#define SERVO2_HOME_STEPS 180U
#endif
