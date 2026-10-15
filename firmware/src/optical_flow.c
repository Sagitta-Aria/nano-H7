/* Independent binary sensor stream. Never routes sensor bytes into motor
 * command parsing. comm task owns totals; IRQ owns the bounded RX producer.
 * Integer nanometre totals retain small increments over long runs; wire uses um. */
#include "optical_flow.h"
#include "board.h"
#include "app_config.h"
#include "protocol.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#define FLOW_RING_SIZE 1024U
#define FLOW_IMU_TIMEOUT 300U
#define FLOW_RAD_CD (3.14159265358979323846f / 18000.0f)
static uint8_t ring[FLOW_RING_SIZE], frame[14];
static volatile unsigned head, tail;
static volatile uint32_t rx_errors;
static uint32_t seen_errors, last_byte, last_packet, packets, accepted, bad, low_quality;
static unsigned used, quality, dt_us;
static int32_t last_x, last_y;
static int32_t height_um = 30000, gain_x = 1000000, gain_y = 1000000;
static unsigned mapping, min_quality = 1, config_revision;
static int32_t mount_cd;
static float coefficient_x = 3000.0f, coefficient_y = 3000.0f;
static float mount_sin, mount_cos = 1.0f, yaw_sin, yaw_cos = 1.0f;
/* Internal values below are nm despite the telemetry-oriented variable names. */
static long long x_um, y_um, wx_um, wy_um, base_x, base_y, base_wx, base_wy;
static unsigned recording, marked, saturated;
static long long stopped_x, stopped_y, stopped_wx, stopped_wy;
static uint32_t stopped_gap;
static uint32_t mark_ms, end_ms, world_gap, mark_gap;
static unsigned imu_valid, world_set, world_locked;
static int32_t initial_yaw;
static uint32_t imu_ms, imu_epoch, world_epoch, world_since;

/* Clockwise yaw, +Y forward and +X right; wrap before trigonometry. */
static int32_t wrap(int32_t value)
{ while (value > 18000) value -= 36000; while (value < -18000) value += 36000; return value; }

/* Initialise only the dedicated UART; existing USART1/UART7 clock source
 * remains PCLK1 for USART3. TX is configured for future sensor commands but
 * this driver never writes to it or changes the sensor's settings. */
void optical_flow_init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
    (void)RCC->APB1LENR;
    board_gpio(GPIOB, 11, 2, 7);
    board_gpio(GPIOB, 14, 2, 7);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << 22)) | (1UL << 22);
    USART3->CR1 = USART3->CR2 = USART3->CR3 = 0;
    USART3->PRESC = 0;
    USART3->BRR = (APB_HZ + UART_BAUD / 2) / UART_BAUD;
    USART3->ICR = 0xFFFFFFFFUL;
    USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART3_IRQn, 6);
    NVIC_EnableIRQ(USART3_IRQn);
}

/* Single producer, bounded work; do not perform floating point or RTOS calls. */
void USART3_IRQHandler(void)
{
    uint32_t flags = USART3->ISR;
    if (flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        ++rx_errors;
    }
    if (flags & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)USART3->RDR;
        unsigned next = (head + 1U) & (FLOW_RING_SIZE - 1U);
        if (next == tail) ++rx_errors;
        else { ring[head] = byte; __DMB(); head = next; }
    }
}

/* Forward verified host/H723 feedback even when no navigation mission runs.
 * First fresh yaw establishes the ground frame. Epoch change locks that frame
 * until MCU reboot: a new sensor yaw origin must never silently rotate history. */
void optical_flow_feedback(int32_t yaw_cd, unsigned valid, uint32_t epoch, uint32_t now_ms)
{
    imu_valid = valid != 0; imu_ms = now_ms; imu_epoch = epoch;
    if (world_set && epoch != world_epoch) world_locked = 1;
    if (!world_set && imu_valid) {
        world_set = 1; initial_yaw = yaw_cd; world_epoch = epoch; world_since = now_ms;
    }
    if (world_set && !world_locked && imu_valid) {
        float angle = wrap(yaw_cd - initial_yaw) * FLOW_RAD_CD;
        yaw_sin = sinf(angle); yaw_cos = cosf(angle);
    }
}

/* Explicit little-endian signed conversion also works on host replay tests. */
static int32_t signed_word(unsigned offset)
{
    uint32_t v = frame[offset] | ((uint32_t)frame[offset + 1] << 8);
    return v >= 32768U ? (int32_t)v - 65536L : (int32_t)v;
}

/* A validated frame contributes signed vectors, never scalar path lengths.
 * Reserved distance bytes are ignored. Sensor integration time is diagnostic;
 * increments already cover that time and must not be multiplied by dt again. */
static void integrate(uint32_t now_ms)
{
    float a, b, dx, dy;
    long long nx, ny, nwx, nwy;
    ++packets; last_packet = now_ms;
    last_x = signed_word(2); last_y = signed_word(4);
    dt_us = frame[6] | ((unsigned)frame[7] << 8);
    quality = frame[10];
    if (!dt_us) { ++bad; return; }
    if (quality < min_quality) { ++low_quality; return; }
    if (saturated) return;
    a = (mapping & 4U) ? last_y : last_x;
    b = (mapping & 4U) ? last_x : last_y;
    a *= coefficient_x * ((mapping & 1U) ? -1.0f : 1.0f);
    b *= coefficient_y * ((mapping & 2U) ? -1.0f : 1.0f);
    /* Round each increment to nm, not um. The minimum configurable raw unit
     * is 1 nm. Bounded coefficients keep float-to-int64 conversions safe. */
    dx = mount_cos * a + mount_sin * b;
    dy = -mount_sin * a + mount_cos * b;
    nx = (long long)(dx + (dx < 0 ? -.5f : .5f));
    ny = (long long)(dy + (dy < 0 ? -.5f : .5f));
    if (x_um > 4000000000000000000LL || x_um < -4000000000000000000LL ||
        y_um > 4000000000000000000LL || y_um < -4000000000000000000LL ||
        wx_um > 4000000000000000000LL || wx_um < -4000000000000000000LL ||
        wy_um > 4000000000000000000LL || wy_um < -4000000000000000000LL) {
        saturated = 1; return;
    }
    x_um += nx; y_um += ny; ++accepted;
    if (world_set && !world_locked && imu_valid && (uint32_t)(now_ms - imu_ms) < FLOW_IMU_TIMEOUT) {
        a = yaw_cos * dx + yaw_sin * dy;
        b = -yaw_sin * dx + yaw_cos * dy;
        nwx = (long long)(a + (a < 0 ? -.5f : .5f));
        nwy = (long long)(b + (b < 0 ? -.5f : .5f));
        wx_um += nwx; wy_um += nwy;
    } else ++world_gap;
}

/* Sliding resynchronisation survives half frames, garbage, wrong XOR and lost
 * bytes. This entry point is task-only and also supports captured-data replay. */
void optical_flow_byte(unsigned byte, uint32_t now_ms)
{
    unsigned i, check;
    if (used && (uint32_t)(now_ms - last_byte) > 50U) { used = 0; ++bad; }
    last_byte = now_ms;
    frame[used++] = (uint8_t)byte;
    for (;;) {
        if (!used) return;
        if (frame[0] != 0xFE || (used >= 2 && frame[1] != 0x0A)) {
            --used; memmove(frame, frame + 1, used); continue;
        }
        if (used < 14) return;
        check = 0; for (i = 2; i < 12; ++i) check ^= frame[i];
        if (frame[13] == 0x55 && frame[12] == check) { integrate(now_ms); used = 0; return; }
        ++bad; --used; memmove(frame, frame + 1, used);
    }
}

/* Bounded drain before command processing. An RX overrun discards queued
 * bytes and the partial frame; it cannot create an unchecked displacement. */
void optical_flow_poll(uint32_t now_ms)
{
    unsigned budget = 512;
    if (seen_errors != rx_errors) {
        uint32_t key = board_lock();
        tail = head; seen_errors = rx_errors; used = 0;
        board_unlock(key);
    }
    while (budget-- && tail != head) {
        uint8_t byte = ring[tail]; __DMB(); tail = (tail + 1U) & (FLOW_RING_SIZE - 1U);
        optical_flow_byte(byte, now_ms);
    }
}

/* Round only wire telemetry; accumulation retains fractional um. */
static long long rounded(long long value)
{ return (value + (value < 0 ? -500 : 500)) / 1000; }

/* Split diagnostics/ground-frame/config into bounded protocol replies. The
 * comm task owns all fields; no trigonometry runs with interrupts disabled. */
static void report(char *out, unsigned capacity, unsigned kind, uint32_t now)
{
    if (kind == 1) {
        snprintf(out, capacity, "OK cmd=FLOW_WORLD wx_um=%lld wy_um=%lld wrx_um=%lld wry_um=%lld heading=%u locked=%u gaps=%lu rgaps=%lu world_ms=%lu epoch=%lu",
            rounded(wx_um), rounded(wy_um), rounded(marked && !recording ? stopped_wx : wx_um - base_wx), rounded(marked && !recording ? stopped_wy : wy_um - base_wy),
            world_set && !world_locked && imu_valid && (uint32_t)(now - imu_ms) < FLOW_IMU_TIMEOUT,
            world_locked, (unsigned long)world_gap, (unsigned long)(marked && !recording ? stopped_gap : world_gap - mark_gap),
            (unsigned long)world_since, (unsigned long)imu_epoch);
    } else if (kind == 2) {
        snprintf(out, capacity, "OK cmd=FLOW_CONFIG height_um=%ld gx_ppm=%ld gy_ppm=%ld map=%u angle_cd=%ld minq=%u rev=%u",
            (long)height_um, (long)gain_x, (long)gain_y, mapping, (long)mount_cd, min_quality, config_revision);
    } else {
        snprintf(out, capacity, "OK cmd=FLOW_STAT x_um=%lld y_um=%lld rx_um=%lld ry_um=%lld marked=%u recording=%u start_ms=%lu end_ms=%lu now_ms=%lu online=%u valid=%u dt_us=%u fx=%ld fy=%ld frames=%lu accepted=%lu bad=%lu low=%lu uart=%lu sat=%u rev=%u",
            rounded(x_um), rounded(y_um), rounded(marked && !recording ? stopped_x : x_um - base_x), rounded(marked && !recording ? stopped_y : y_um - base_y),
            marked, recording, (unsigned long)mark_ms, (unsigned long)end_ms, (unsigned long)now,
            packets && (uint32_t)(now - last_packet) < 200U, quality, dt_us, (long)last_x, (long)last_y,
            (unsigned long)packets, (unsigned long)accepted, (unsigned long)bad, (unsigned long)low_quality,
            (unsigned long)rx_errors, saturated, config_revision);
    }
}

/* Stop snapshots freeze only the recording vector. Boot totals keep growing.
 * Frozen vectors are independent snapshots, unaffected by later calibration. */
void optical_flow_command(int argc, char **argv, char *out, unsigned capacity, uint32_t now_ms)
{
    int32_t v[6]; unsigned i;
    static const int32_t lo[] = {1000, 10000, 10000, 0, -18000, 1};
    static const int32_t hi[] = {1000000, 10000000, 10000000, 7, 18000, 255};
    strcpy(out, "ERR code=FLOW_ARGUMENT");
    if (argc == 2 && strcmp(argv[1], "START") == 0) {
        base_x = x_um; base_y = y_um; base_wx = wx_um; base_wy = wy_um;
        mark_gap = world_gap; mark_ms = now_ms; end_ms = 0; recording = marked = 1;
    } else if (argc == 2 && strcmp(argv[1], "STOP") == 0) {
        if (recording) {
            stopped_x = x_um - base_x; stopped_y = y_um - base_y;
            stopped_wx = wx_um - base_wx; stopped_wy = wy_um - base_wy;
            stopped_gap = world_gap - mark_gap; end_ms = now_ms; recording = 0;
        }
    } else if (argc == 8 && strcmp(argv[1], "CONFIG") == 0) {
        for (i = 0; i < 6; ++i) if (!protocol_int(argv[i + 2], lo[i], hi[i], &v[i])) return;
        height_um = v[0]; gain_x = v[1]; gain_y = v[2]; mapping = (unsigned)v[3]; mount_cd = v[4]; min_quality = (unsigned)v[5];
        coefficient_x = (float)height_um * gain_x / 1e7f;
        coefficient_y = (float)height_um * gain_y / 1e7f;
        mount_sin = sinf(mount_cd * FLOW_RAD_CD); mount_cos = cosf(mount_cd * FLOW_RAD_CD); ++config_revision;
        report(out, capacity, 2, now_ms); return;
    } else if (argc != 2 || (strcmp(argv[1], "STAT") && strcmp(argv[1], "WORLD") && strcmp(argv[1], "CONFIG"))) return;
    report(out, capacity, strcmp(argv[1], "WORLD") == 0 ? 1 : strcmp(argv[1], "CONFIG") == 0 ? 2 : 0, now_ms);
}
