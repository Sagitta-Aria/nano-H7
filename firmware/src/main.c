/* Application ownership: comm is the only protocol writer; motion runs at 1 kHz;
 * supervisor expires the explicit heartbeat lease independently of serial load. */
#include "board.h"
#include "serial.h"
#include "motion.h"
#include "chassis.h"
#include "navigation.h"
#include "turn.h"
#include "servo.h"
#include "protocol.h"
#include "optical_flow.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

static TaskHandle_t comm_handle, motion_handle, supervisor_handle;
static volatile TickType_t heartbeat;
static volatile uint32_t link_timeouts;
static uint32_t bad_frames, replay_hits;
static char response[FRAME_MAX], output[FRAME_MAX];
typedef struct {
    char input[FRAME_MAX];
    unsigned used, discard;
    TickType_t last_byte;
} comm_rx_t;
static comm_rx_t comm_rx[SERIAL_PORT_COUNT];
typedef struct { uint32_t id; char command[128]; char response[FRAME_MAX]; } replay_t;
static replay_t replay[8];
static unsigned replay_next;

/* Fail closed for scheduler/memory exceptions. Debugger can inspect panic_line. */
volatile int panic_line;
void app_panic(const char *file, int line)  //报错记录急停函数
{
    (void)file;
    __disable_irq();
    panic_line = line;   //记录报错文件
    motion_emergency();  //急停函数
    for (;;) {}
}
void vApplicationMallocFailedHook(void) { app_panic(__FILE__, __LINE__); }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{ (void)task; (void)name; app_panic(__FILE__, __LINE__); }
void HardFault_Handler(void) { app_panic(__FILE__, __LINE__); }
void MemManage_Handler(void) { app_panic(__FILE__, __LINE__); }
void BusFault_Handler(void) { app_panic(__FILE__, __LINE__); }
void UsageFault_Handler(void) { app_panic(__FILE__, __LINE__); }

/* Parse a bounded argument list. Extra arguments are rejected, never ignored. */
static int split(char *body, char *argv[], unsigned capacity)  //解析小文本
{
    unsigned n = 0;
    char *p = body;
    while (*p) {
        while (*p == ' ') ++p;
        if (!*p) break;
        if (n == capacity) return -1;
        argv[n++] = p;   //每识别出一个单词，n 加 1
        while (*p && *p != ' ') ++p;
        if (*p) *p++ = 0;
    }
    return (int)n;
}

//strcmp:按字典序比较两个字符串，返回一个整数表示大小关系
// 返回 0：两个字符串完全相同
// 返回 负数：s1 小于 s2（按字符编码逐位比较）
// 返回 正数：s1 大于 s2

//strcpy:将源字符串复制到目标字符串，包括终止符'\0'，返回目标字符串的指针
// char *strcpy(char *dest, const char *src);
// 把 src 指向的字符串（包括结尾的 \0）复制到 dest 指向的内存里

/* Servo angle text: whole degrees ("90", "270", "13") or degrees with one
 * decimal ("13.5", "135.5"). The result is always tenths of a degree so the
 * caller does not repeat the unit. The decimal point is the only thing that
 * changes the scale: "135" is 135.0 degrees, never 13.5. Anything else is
 * rejected rather than guessed, because a misread angle moves the horn. */
static int parse_degrees(const char *text, int32_t *out)
{
    const char *p = text;
    int32_t value = 0;
    int digits = 0, fraction = 0;
    while (*p >= '0' && *p <= '9') { value = value * 10 + (*p++ - '0'); if (++digits > 3) return -1; }
    if (digits < 1) return -1;
    if (*p == '.') {
        /* Bound the read: p[2] may be uninitialised past the terminator. */
        if (p[1] < '0' || p[1] > '9' || p[2] != 0) return -1;
        value = value * 10 + (p[1] - '0');
        fraction = 1;
        p += 2;
    }
    /* The whole argument has to be consumed: "90x" and "90,5" are not 90. */
    if (*p) return -1;
    *out = fraction ? value : value * 10;
    return 0;
}

/* Report pulse-count depth below the original top; this is not an encoder. */
static void lift_report(void)
{
    lift_status_t status;
    motion_lift_status(&status);
    snprintf(response, sizeof(response), "OK cmd=LIFT_STAT axis=5 initialized=%u down_sign=%ld pos=%ld max=%ld up_room=%lu down_room=%lu active=%u min=%ld",
        status.initialized, (long)status.down_sign, (long)status.position,
        (long)status.maximum, (unsigned long)status.up_room,
        (unsigned long)status.down_room, status.active, (long)status.minimum);
}

/* Dedicated M5/M6 telemetry uses logical pulse positions, never host estimates. */
static void aux_report(unsigned axis)
{
    aux_status_t s;
    motion_aux_status(axis, &s);
    snprintf(response, sizeof(response), "OK cmd=AUX_STAT axis=%u pos=%ld min=%ld max=%ld referenced=%u configured=%u active=%u",
        axis, (long)s.position, (long)s.minimum, (long)s.maximum,
        s.referenced, s.configured, s.active);
}

/* Shared path for raw MOVE and semantic lift buttons; neither may bypass M5
 * limits. The motion layer repeats validation atomically before scheduling. */
static void dispatch_move(uint32_t id, unsigned axis, int32_t steps, unsigned sps, unsigned accel)
{
    const char *error = axis == 5U ? motion_lift_error(steps) : 0;
    if (axis <= 4U && chassis_busy()) strcpy(response, "ERR code=CHASSIS_BUSY");
    else if (error) snprintf(response, sizeof(response), "ERR code=%s", error);
    else if (motion_move(axis - 1U, steps, sps, accel, id)) {
        if (axis <= 4U) navigation_invalidate();
        snprintf(response, sizeof(response), "OK cmd=MOVE state=accepted axis=%u move_id=%lu", axis, (unsigned long)id);
    }
    else strcpy(response, "ERR code=DISARMED_BUSY_OR_RANGE");
}

static void dispatch(uint32_t id, char *body)
{
    char *arg[8];
    int n = split(body, arg, 8);
    int32_t a, b, c, d, e, wheels[4];
    unsigned i;
    strcpy(response, "ERR code=BAD_ARGUMENT");
    if (n < 1) return;
    if (strcmp(arg[0], "HELLO") == 0 && n == 1) {
        snprintf(response, sizeof(response), "OK cmd=HELLO fw=%s protocol=1 board=H743_CB_V1.5 led_mask=3 red=fixed chassis_max=%u aux_max=%u chassis=1 servo=%u servo_travel=%u servo1_travel=%u servo2_travel=%u serial=dual lift=1 lift_max=%ld nav=1 nav_heading=1 nav_grid=1 nav_path=1 turn=1 aux_control=1 flow=1 servo1_pin=PH%u servo2_pin=PH%u turn_tuning=1 aux_live=1 turn_precision=1 straight=1", APP_VERSION, CHASSIS_SPS_MAX, AUX_SPS_MAX, servo_channels(), servo_steps_max(0) * 5U, servo_steps_max(0) * 5U, servo_steps_max(1) * 5U, (long)M5_TRAVEL_PULSES, servo_pin(0), servo_pin(1));
    } else if (strcmp(arg[0], "PING") == 0 && n == 1) {
        heartbeat = xTaskGetTickCount();
        strcpy(response, "OK cmd=PING");
    } else if (strcmp(arg[0], "INFO") == 0 && n == 1) {
        snprintf(response, sizeof(response), "OK cmd=INFO cpu_hz=%lu apb_hz=%lu timer_hz=%lu flash_kb=%u uid=%08lX%08lX%08lX reset=0x%08lX rtos=11.2.0 baud=%lu",
            (unsigned long)SystemCoreClock, APB_HZ, TIMER_HZ, *(volatile uint16_t *)FLASHSIZE_BASE,
            *(volatile unsigned long *)(UID_BASE + 8), *(volatile unsigned long *)(UID_BASE + 4), *(volatile unsigned long *)UID_BASE,
            (unsigned long)board_reset_flags(), UART_BAUD);
    } else if (strcmp(arg[0], "STAT") == 0 && n == 1) {
        snprintf(response, sizeof(response), "OK cmd=STAT uptime_ms=%lu heap_free=%lu heap_min=%lu tasks=%lu stack_comm=%lu stack_motion=%lu stack_super=%lu led=%u armed=%u rx=%lu rx_err=%lu tx_err=%lu bad=%lu replay=%lu timeout=%lu motion_fault=%lu",
            (unsigned long)xTaskGetTickCount(), (unsigned long)xPortGetFreeHeapSize(), (unsigned long)xPortGetMinimumEverFreeHeapSize(),
            (unsigned long)uxTaskGetNumberOfTasks(), (unsigned long)uxTaskGetStackHighWaterMark(comm_handle) * sizeof(StackType_t),
            (unsigned long)uxTaskGetStackHighWaterMark(motion_handle) * sizeof(StackType_t),
            (unsigned long)uxTaskGetStackHighWaterMark(supervisor_handle) * sizeof(StackType_t), board_led_state(), motion_armed(),
            (unsigned long)serial_rx_bytes(), (unsigned long)serial_rx_errors(), (unsigned long)serial_tx_errors(),
            (unsigned long)bad_frames, (unsigned long)replay_hits, (unsigned long)link_timeouts, (unsigned long)motion_faults());
    } else if (strcmp(arg[0], "LED") == 0 && n == 3 && protocol_int(arg[1], 0, 1, &a) && protocol_int(arg[2], 0, 1, &b)) {
        board_led((unsigned)a, (unsigned)b);
        snprintf(response, sizeof(response), "OK cmd=LED led=%u", board_led_state());
    } else if (strcmp(arg[0], "ARM") == 0 && n == 2 && protocol_int(arg[1], 0, 1, &a)) {
        /* Only a recent explicit PING permits arming. Queries are not a lease. */
        if (a && (TickType_t)(xTaskGetTickCount() - heartbeat) >= pdMS_TO_TICKS(LINK_TIMEOUT_MS))
            strcpy(response, "ERR code=HEARTBEAT_REQUIRED");
        else if (motion_arm((unsigned)a)) {
            if (!a) chassis_halt();
            snprintf(response, sizeof(response), "OK cmd=ARM armed=%u", motion_armed());
        }
        else strcpy(response, "ERR code=BUSY_OR_FAULT");
    } else if (strcmp(arg[0], "STOP") == 0 && n == 1) {
        chassis_halt(); motion_stop(); strcpy(response, "OK cmd=STOP armed=0");
    } else if (strcmp(arg[0], "SERVO") == 0) {
        /* Servos are not armed and not stopped by STOP: they are positional
         * outputs, and a 50 Hz pulse train has to keep running to hold them. */
        servo_status_t status;
        unsigned ch;
        if (n == 2 && (strcmp(arg[1], "STAT") == 0 || strcmp(arg[1], "?") == 0)) {
            /* One line per channel: a fixed field list cannot describe a count
             * that grows, and the host should not have to guess. */
            response[0] = 0;
            for (ch = 0; ch < servo_channels(); ++ch) {
                char line[128];
                if (!servo_status(ch, &status)) break;
                snprintf(line, sizeof(line), "%sOK cmd=SERVO_STAT ch=%u angle=%u.%u us=%u on=%u pin=PH%u timer=TIM12_CH%u",
                    ch ? " " : "", ch + 1U, status.angle_tenths / 10U, status.angle_tenths % 10U,
                    status.pulse_us, status.on, servo_pin(ch), servo_timer_channel(ch));
                if (strlen(response) + strlen(line) >= sizeof(response)) break;
                strcat(response, line);
            }
        } else if (n == 3 && protocol_int(arg[1], 1, (int32_t)servo_channels(), &a) &&
                   parse_degrees(arg[2], &b) == 0) {
            ch = (unsigned)a - 1U;
            if (b > (int32_t)(servo_steps_max(ch) * 5U))
                strcpy(response, "ERR code=ANGLE_RANGE");
            else {
                /* The table is half-degree; the host may still ask for tenths. */
                servo_set_steps(ch, (unsigned)b / 5U);
                servo_status(ch, &status);
                snprintf(response, sizeof(response), "OK cmd=SERVO ch=%ld angle=%u.%u us=%u pin=PH%u timer=TIM12_CH%u",
                    (long)a, status.angle_tenths / 10U, status.angle_tenths % 10U, status.pulse_us, servo_pin(ch), servo_timer_channel(ch));
            }
        } else {
            /* Recognised verb, unusable arguments: say so instead of leaving the
             * generic parse error, which would suggest the command is unknown. */
            strcpy(response, "ERR code=SERVO_BAD_ARGUMENT");
        }
    } else if (strcmp(arg[0], "LIFT") == 0) {
        if (n == 2 && strcmp(arg[1], "STAT") == 0) lift_report();
        else if (n == 2 && strcmp(arg[1], "INIT") == 0) {
            if (motion_lift_init()) lift_report();
            else strcpy(response, "ERR code=M5_BUSY");
        } else if (n == 3 && strcmp(arg[1], "DIR") == 0 &&
                   protocol_int(arg[2], -1, 1, &a) && a != 0) {
            if (motion_lift_direction(a)) lift_report();
            else strcpy(response, "ERR code=M5_DIR_REQUIRES_TOP_DISARMED");
        } else if (n == 6 && strcmp(arg[1], "MOVE") == 0 &&
                   (strcmp(arg[2], "DOWN") == 0 || strcmp(arg[2], "UP") == 0) &&
                   protocol_int(arg[3], 1, MAX_MOVE_STEPS, &b) &&
                   protocol_int(arg[4], 1, AUX_SPS_MAX, &c) && protocol_int(arg[5], 1, 20000, &d)) {
            lift_status_t status;
            motion_lift_status(&status);
            a = strcmp(arg[2], "DOWN") == 0 ? 1 : -1;
            dispatch_move(id, 5, b * a * status.down_sign, (unsigned)c, (unsigned)d);
        }
    } else if (strcmp(arg[0], "AUX") == 0 && n >= 3 && protocol_int(arg[2], 5, 6, &a)) {
        if (n == 3 && strcmp(arg[1], "STAT") == 0) aux_report((unsigned)a);
        else if (n == 3 && strcmp(arg[1], "HALT") == 0) {
            motion_aux_stop((unsigned)a); aux_report((unsigned)a);
        } else if (n == 3 && strcmp(arg[1], "ZERO") == 0) {
            if (motion_aux_zero((unsigned)a)) aux_report((unsigned)a);
            else strcpy(response, "ERR code=AUX_ZERO_REQUIRES_M6_DISARMED_IDLE");
        } else if (n == 5 && strcmp(arg[1], "CONFIG") == 0 &&
                   protocol_int(arg[3], -AUX_POSITION_MAX, AUX_POSITION_MAX, &b) &&
                   protocol_int(arg[4], -AUX_POSITION_MAX, AUX_POSITION_MAX, &c)) {
            if (motion_aux_config((unsigned)a, b, c)) aux_report((unsigned)a);
            else strcpy(response, "ERR code=AUX_CONFIG_RANGE_REFERENCE_OR_BUSY");
        } else if (n == 6 && (strcmp(arg[1], "GOTO") == 0 || strcmp(arg[1], "STEP") == 0) &&
                   protocol_int(arg[3], -2 * AUX_POSITION_MAX, 2 * AUX_POSITION_MAX, &b) &&
                   protocol_int(arg[4], 1, AUX_SPS_MAX, &c) && protocol_int(arg[5], 1, 20000, &d)) {
            if (motion_aux_move((unsigned)a, b, strcmp(arg[1], "GOTO") == 0, (unsigned)c, (unsigned)d, id))
                snprintf(response, sizeof(response), "OK cmd=AUX_MOVE axis=%ld state=accepted move_id=%lu", (long)a, (unsigned long)id);
            else strcpy(response, "ERR code=AUX_NOT_READY_BUSY_OR_LIMIT");
        }
    } else if (strcmp(arg[0], "FLOW") == 0) {
        optical_flow_command(n, arg, response, sizeof(response), xTaskGetTickCount());
    } else if (strcmp(arg[0], "TURN") == 0) {
        if (n == 2 && strcmp(arg[1], "STAT") == 0)
            turn_report(response, sizeof(response), xTaskGetTickCount());
        else if (n == 2 && strcmp(arg[1], "ZERO") == 0) {
            if (turn_zero(xTaskGetTickCount())) turn_report(response, sizeof(response), xTaskGetTickCount());
            else strcpy(response, "ERR code=TURN_ZERO_REQUIRES_IDLE_IMU");
        } else if (n == 2 && strcmp(arg[1], "STOP") == 0) {
            turn_stop(); turn_report(response, sizeof(response), xTaskGetTickCount());
        } else if (n == 4 && strcmp(arg[1], "CONFIG") == 0 &&
                   protocol_int(arg[2], 0, INT32_MAX, &a) && protocol_int(arg[3], 1, 1000, &b)) {
            if (turn_configure((unsigned)a, (unsigned)b)) turn_report(response, sizeof(response), xTaskGetTickCount());
            else strcpy(response, "ERR code=TURN_CONFIG_RANGE_OR_BUSY");
        } else if (n == 3 && strcmp(arg[1], "CONFIG") == 0 && protocol_int(arg[2], 0, INT32_MAX, &a)) {
            if (turn_config((unsigned)a)) turn_report(response, sizeof(response), xTaskGetTickCount());
            else strcpy(response, "ERR code=TURN_CONFIG_RANGE_OR_BUSY");
        } else if (n == 6 && strcmp(arg[1], "MOVE") == 0 &&
                   (strcmp(arg[2], "REL") == 0 || strcmp(arg[2], "ABS") == 0) &&
                   protocol_int(arg[3], -18000, 18000, &a) &&
                   protocol_int(arg[4], 1, CHASSIS_SPS_MAX, &b) && protocol_int(arg[5], 1, 20000, &c)) {
            if (turn_start(strcmp(arg[2], "ABS") == 0, a, (unsigned)b, (unsigned)c, id, xTaskGetTickCount()))
                snprintf(response, sizeof(response), "OK cmd=TURN_MOVE state=accepted move_id=%lu", (unsigned long)id);
            else strcpy(response, "ERR code=TURN_NOT_READY_BUSY_DISARMED_OR_IMU");
        }
    } else if (strcmp(arg[0], "NAV") == 0) {
        if (n == 2 && strcmp(arg[1], "STAT") == 0)
            navigation_report(response, sizeof(response), xTaskGetTickCount());
        else if (n == 2 && strcmp(arg[1], "STOP") == 0) {
            navigation_stop(); strcpy(response, "OK cmd=NAV_STOP");
        } else if (n == 4 && strcmp(arg[1], "RESET") == 0 &&
                   protocol_int(arg[2], -NAV_COORD_MAX, NAV_COORD_MAX, &a) &&
                   protocol_int(arg[3], -NAV_COORD_MAX, NAV_COORD_MAX, &b)) {
            if (navigation_origin(a, b)) navigation_report(response, sizeof(response), xTaskGetTickCount());
            else strcpy(response, "ERR code=NAV_RESET_REQUIRES_DISARMED_IDLE");
        } else if (n == 6 && strcmp(arg[1], "GRID") == 0 &&
                   (strcmp(arg[2], "X") == 0 || strcmp(arg[2], "Y") == 0) &&
                   protocol_int(arg[3], -NAV_COORD_MAX, NAV_COORD_MAX, &a) &&
                   protocol_int(arg[4], -NAV_COORD_MAX, NAV_COORD_MAX, &b) &&
                   protocol_int(arg[5], -NAV_COORD_MAX, NAV_COORD_MAX, &c)) {
            strcpy(response, navigation_grid(strcmp(arg[2], "Y") == 0, a, b, c) ? "OK cmd=NAV_GRID" : "ERR code=NAV_GRID_INVALID_OR_BUSY");
        } else if (n == 5 && strcmp(arg[1], "POINT") == 0 &&
                   protocol_int(arg[2], 0, 4, &a) && protocol_int(arg[3], -NAV_COORD_MAX, NAV_COORD_MAX, &b) &&
                   protocol_int(arg[4], -NAV_COORD_MAX, NAV_COORD_MAX, &c)) {
            strcpy(response, navigation_point((unsigned)a, b, c) ? "OK cmd=NAV_POINT" : "ERR code=NAV_BUSY");
        } else if (n == 4 && strcmp(arg[1], "ATTITUDE") == 0 &&
                   protocol_int(arg[2], 0, 4, &a) && protocol_int(arg[3], -18000, 18000, &b)) {
            strcpy(response, navigation_attitude((unsigned)a, b) ? "OK cmd=NAV_ATTITUDE" : "ERR code=NAV_BUSY");
        } else if (n == 5 && strcmp(arg[1], "IMU") == 0 &&
                   protocol_int(arg[2], -18000, 18000, &a) && protocol_int(arg[3], 0, 1, &b) &&
                   protocol_int(arg[4], 1, INT32_MAX, &c)) {
            navigation_imu(a, (unsigned)b, (uint32_t)c, xTaskGetTickCount());
            optical_flow_feedback(a, (unsigned)b, (uint32_t)c, xTaskGetTickCount());
            strcpy(response, "OK cmd=NAV_IMU");
        } else if (n == 5 && strcmp(arg[1], "CONFIG") == 0 &&
                   protocol_int(arg[2], 1, CHASSIS_SPS_MAX, &a) && protocol_int(arg[3], 1, 20000, &b) &&
                   protocol_int(arg[4], 0, 60000, &c)) {
            strcpy(response, navigation_parameters((unsigned)a, (unsigned)b, (uint32_t)c) ? "OK cmd=NAV_CONFIG" : "ERR code=NAV_BUSY");
        } else if (n == 6 && strcmp(arg[1], "BEGIN") == 0 &&
                   protocol_int(arg[2], 1, INT32_MAX, &a) && protocol_int(arg[3], 5, NAV_PATH_MAX, &b) &&
                   protocol_int(arg[4], -NAV_COORD_MAX, NAV_COORD_MAX, &c) && protocol_int(arg[5], -NAV_COORD_MAX, NAV_COORD_MAX, &d)) {
            strcpy(response, navigation_begin((uint32_t)a, (unsigned)b, c, d, xTaskGetTickCount()) ? "OK cmd=NAV_BEGIN" : "ERR code=NAV_UPLOAD_NOT_READY");
        } else if (n == 7 && strcmp(arg[1], "WP") == 0 &&
                   protocol_int(arg[2], 1, INT32_MAX, &a) && protocol_int(arg[3], 0, NAV_PATH_MAX - 1, &b) &&
                   protocol_int(arg[4], -NAV_COORD_MAX, NAV_COORD_MAX, &c) && protocol_int(arg[5], -NAV_COORD_MAX, NAV_COORD_MAX, &d) &&
                   protocol_int(arg[6], 0, 4, &e)) {
            strcpy(response, navigation_waypoint((uint32_t)a, (unsigned)b, c, d, (unsigned)e, xTaskGetTickCount()) ? "OK cmd=NAV_WP" : "ERR code=NAV_UPLOAD_ORDER_OR_BUSY");
        } else if (n == 4 && strcmp(arg[1], "COMMIT") == 0 &&
                   protocol_int(arg[2], 1, INT32_MAX, &a) && protocol_int(arg[3], 0, 65535, &b)) {
            strcpy(response, navigation_commit((uint32_t)a, (unsigned)b, xTaskGetTickCount()) ? "OK cmd=NAV_COMMIT" : "ERR code=NAV_PATH_INVALID_OR_INCOMPLETE");
        } else if ((n == 2 || (n == 3 && protocol_int(arg[2], 1, INT32_MAX, &a))) && strcmp(arg[1], "START") == 0) {
            if (navigation_start(id, n == 3 ? (uint32_t)a : 0U, xTaskGetTickCount()))
                snprintf(response, sizeof(response), "OK cmd=NAV_START state=accepted move_id=%lu", (unsigned long)id);
            else strcpy(response, "ERR code=NAV_NOT_READY_BUSY_DISARMED_OR_IMU");
        } else if (n == 3 && strcmp(arg[1], "OBSTACLE") == 0 && protocol_int(arg[2], 0, 1, &a)) {
            navigation_obstacle((unsigned)a); navigation_report(response, sizeof(response), xTaskGetTickCount());
        } else if (n == 2 && strcmp(arg[1], "REPLAN") == 0) {
            strcpy(response, navigation_replan() ? "OK cmd=NAV_REPLAN" : "ERR code=NAV_PLANNER_NOT_IMPLEMENTED");
        }
    } else if (strcmp(arg[0], "CHASSIS") == 0) {
        // 处理 CHASSIS 命令
        if (n == 2 && strcmp(arg[1], "STAT") == 0) chassis_report(response, sizeof(response));
        else if (n == 2 && strcmp(arg[1], "HALT") == 0) {  
            chassis_halt(); strcpy(response, "OK cmd=CHASSIS_HALT");
        } else if (n == 3 && strcmp(arg[1], "CONFIG") == 0 && protocol_int(arg[2], 0, 15, &a)) {
            if (chassis_config((unsigned)a)) chassis_report(response, sizeof(response));
            else strcpy(response, "ERR code=CHASSIS_BUSY");
        } else if (n == 6 && strcmp(arg[1], "STRAIGHT") == 0 && (a = chassis_parse_direction(arg[2])) >= 0 &&
                   protocol_int(arg[3], 1, MAX_MOVE_STEPS, &b) && protocol_int(arg[4], 1, CHASSIS_SPS_MAX, &c) &&
                   protocol_int(arg[5], 1, 20000, &d)) {
            if (chassis_straight((unsigned)a, (uint32_t)b, (unsigned)c, (unsigned)d, id, xTaskGetTickCount()))
                snprintf(response, sizeof(response), "OK cmd=CHASSIS_STRAIGHT state=accepted move_id=%lu", (unsigned long)id);
            else strcpy(response, "ERR code=STRAIGHT_NOT_READY_DIRECTION_BUSY_OR_IMU");
        } else if (n == 6 && strcmp(arg[1], "MOVE") == 0 && (a = chassis_parse_direction(arg[2])) >= 0 &&
                   protocol_int(arg[3], 1, MAX_MOVE_STEPS, &b) && protocol_int(arg[4], 1, CHASSIS_SPS_MAX, &c) &&
                   protocol_int(arg[5], 1, 20000, &d)) {
            if (chassis_move((unsigned)a, (uint32_t)b, (unsigned)c, (unsigned)d, id))
                snprintf(response, sizeof(response), "OK cmd=CHASSIS_MOVE state=accepted move_id=%lu", (unsigned long)id);
            else strcpy(response, "ERR code=DISARMED_BUSY_OR_FAULT");
        } else if (n == 5 && strcmp(arg[1], "JOG") == 0 && (a = chassis_parse_direction(arg[2])) >= 0 &&
                   protocol_int(arg[3], 1, CHASSIS_SPS_MAX, &b) && protocol_int(arg[4], 1, 20000, &c)) {
            if (chassis_jog((unsigned)a, (unsigned)b, (unsigned)c, id, xTaskGetTickCount()))
                strcpy(response, "OK cmd=CHASSIS_JOG state=accepted");
            else strcpy(response, "ERR code=DISARMED_BUSY_OR_FAULT");
        }
    } else if (strcmp(arg[0], "MOVE") == 0 && n == 5 && protocol_int(arg[1], 1, 7, &a) &&
               protocol_int(arg[2], -MAX_MOVE_STEPS, MAX_MOVE_STEPS, &b) &&
               protocol_int(arg[3], 1, a <= 4 ? CHASSIS_SPS_MAX : AUX_SPS_MAX, &c) && protocol_int(arg[4], 1, 20000, &d)) {
        dispatch_move(id, (unsigned)a, b, (unsigned)c, (unsigned)d);
    } else if (strcmp(arg[0], "WHEELS") == 0 && n == 6 && protocol_int(arg[5], 1, 20000, &a)) {
        for (i = 0; i < 4; ++i) if (!protocol_int(arg[i + 1], -(int32_t)CHASSIS_SPS_MAX, CHASSIS_SPS_MAX, &wheels[i])) return;
        if (chassis_busy()) strcpy(response, "ERR code=CHASSIS_BUSY");
        else if (motion_wheels(wheels, (unsigned)a, id)) {
            navigation_invalidate(); strcpy(response, "OK cmd=WHEELS state=accepted");
        }
        else strcpy(response, "ERR code=DISARMED_BUSY_OR_REVERSE");
    } else if (strcmp(arg[0], "AXIS") == 0 && n == 2 && protocol_int(arg[1], 1, 7, &a)) {
        axis_status_t status;
        motion_status((unsigned)a - 1, &status);
        snprintf(response, sizeof(response), "OK cmd=AXIS axis=%ld pos=%ld sps=%ld remaining=%lu active=%u move_id=%lu",
            (long)a, (long)status.position, (long)status.sps, (unsigned long)status.remaining, status.active, (unsigned long)status.command_id);
    } else strcpy(response, "ERR code=UNKNOWN_OR_BAD_ARGUMENT");
}

/* Exact-body replay cache prevents repeated MOVE after a lost reply.
 * Eight recent requests are retained; the host has one in-flight request. */
static void process_line(char *line)    //把收到的一行原始数据解码、查重（防重放）、执行命令、记录结果，最后编码发回
{
    protocol_request_t req;     //解码后的请求结构体
    char original[128];
    unsigned i;
    /* Drain between individual host requests as well as task iterations:
     * a batch of long UART replies must not starve the sensor receive ring. */
    optical_flow_poll(xTaskGetTickCount());
    if (!protocol_decode(line, &req)) { ++bad_frames; return; }
    if (strlen(req.body) >= sizeof(original)) {
        strcpy(response, "ERR code=COMMAND_TOO_LONG");   
    } else {
        strcpy(original, req.body);
        for (i = 0; i < 8; ++i) {
            if (replay[i].id == req.id) {
                if (strcmp(replay[i].command, original) != 0) strcpy(response, "ERR code=SEQUENCE_REUSE");
                else { strcpy(response, replay[i].response); ++replay_hits; }
                if (protocol_encode(req.id, response, output, sizeof(output))) serial_write(output);
                return;
            }
        }
        dispatch(req.id, req.body);
        replay[replay_next].id = req.id;
        strcpy(replay[replay_next].command, original);
        strcpy(replay[replay_next].response, response);
        replay_next = (replay_next + 1) % 8;
    }
    if (protocol_encode(req.id, response, output, sizeof(output))) serial_write(output);
}

/* Bounded round-robin service of independent line assemblers. Shared replay
 * IDs intentionally deduplicate an identical request sent over both links.
 * This is one logical controller with two transports, not two motion owners. */

// app_comm_poll() 是一个周期性被调用的通信轮询函数（通常放在一个任务或定时器里跑），
// 遍历所有串口，把收到的字节攒成一行，攒满一行就交给 process_line() 处理
void app_comm_poll(void)
{
    unsigned port, budget;
    int ch;
    for (port = 0; port < SERIAL_PORT_COUNT; ++port) {
        comm_rx_t *r = &comm_rx[port];
        if ((r->used || r->discard) && (TickType_t)(xTaskGetTickCount() - r->last_byte) > pdMS_TO_TICKS(200)) {
            r->used = r->discard = 0; ++bad_frames;
        }
        budget = 256;
        while (budget-- && (ch = serial_read(port)) >= 0) {
            r->last_byte = xTaskGetTickCount();
            if (ch == '\n') {
                if (!r->discard && r->used) { r->input[r->used] = 0; process_line(r->input); }
                r->used = r->discard = 0;
            } else if (!r->discard) {
                if (ch == 0 || r->used == sizeof(r->input) - 1) { r->discard = 1; ++bad_frames; }
                else r->input[r->used++] = (char)ch;
            }
        }
    }
}

static void comm_task(void *arg)
{
    (void)arg;
    for (;;) {  //无限循环
        optical_flow_poll(xTaskGetTickCount());
        app_comm_poll();   //调用通信轮询函数，处理串口数据
        vTaskDelay(pdMS_TO_TICKS(1));  //相对延时cpu 1 毫秒，给其他任务机会运行
    }
}
static void motion_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;) { chassis_tick(xTaskGetTickCount()); navigation_tick(xTaskGetTickCount()); turn_tick(xTaskGetTickCount()); motion_tick(); xTaskDelayUntil(&wake, pdMS_TO_TICKS(1)); }//固定1ms更新底盘、坐标巡航和运动状态
}
static void supervisor_task(void *arg)  //看门狗
{
    (void)arg;
    for (;;) {
        if (motion_armed() && (TickType_t)(xTaskGetTickCount() - heartbeat) >= pdMS_TO_TICKS(LINK_TIMEOUT_MS)) {
            chassis_halt(); motion_stop(); ++link_timeouts;   //底盘急停；运动停止；超时计数加1
        }
        vTaskDelay(pdMS_TO_TICKS(10));   //相对延时cpu 10 毫秒，给其他任务机会运行
    }
}
int main(void)
{
    board_init();  //底层硬件：时钟，GPIO，串口，定时器等初始化
    motion_init();  //PID,电机初始化
    servo_init();   //舵机 PWM，上电即输出中位，不受 ARM 门控
    serial_init();  //串口初始化
    optical_flow_init();
    heartbeat = (TickType_t)-pdMS_TO_TICKS(LINK_TIMEOUT_MS); //初始化心跳计数器为负值，确保在启动时不会立即触发超时
    configASSERT(xTaskCreate(comm_task, "comm", 1536, 0, 2, &comm_handle) == pdPASS);  //任务创建
    configASSERT(xTaskCreate(motion_task, "motion", 512, 0, 3, &motion_handle) == pdPASS);
    configASSERT(xTaskCreate(supervisor_task, "supervisor", 512, 0, 4, &supervisor_handle) == pdPASS);
    vTaskStartScheduler();  //启动调度器
    app_panic(__FILE__, __LINE__);  // 只要执行到这里，就是致命错误，记录文件和行号后停机。
    return 0;
}
