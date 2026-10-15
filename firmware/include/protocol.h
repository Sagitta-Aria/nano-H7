#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include "app_config.h"
typedef struct { uint32_t id; char *body; } protocol_request_t;
uint16_t protocol_crc(const char *data, size_t size);
int protocol_decode(char *line, protocol_request_t *out);
int protocol_encode(uint32_t id, const char *body, char *out, size_t capacity);
int protocol_int(const char *text, int32_t min, int32_t max, int32_t *out);
#endif
