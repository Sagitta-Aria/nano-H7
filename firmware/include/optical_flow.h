#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H
#include <stdint.h>
/* UP-F7-QX01 only: USART3 PB11 RX / PB14 TX, 115200 8N1. All parsing,
 * calibration and accumulation belong to comm task, never the byte IRQ.
 * Body totals survive host reconnect; only MCU reset clears them. */
void optical_flow_init(void);
void optical_flow_poll(uint32_t now_ms);
void optical_flow_byte(unsigned byte, uint32_t now_ms);
void optical_flow_feedback(int32_t yaw_cd, unsigned valid, uint32_t epoch, uint32_t now_ms);
void optical_flow_command(int argc, char **argv, char *out, unsigned capacity, uint32_t now_ms);
#endif
