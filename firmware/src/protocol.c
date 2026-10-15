/* ASCII frames: @<u32 sequence> <body>*<CRC16-CCITT-FALSE>\n.
 * CRC covers sequence + space + body, excluding @, *, CRC and newline.
 * Pure C: no MCU/RTOS dependencies; exercised directly by native tests. */
#include "protocol.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <ctype.h>
uint16_t protocol_crc(const char *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    unsigned bit;
    while (size--) {
        crc ^= (uint16_t)(uint8_t)*data++ << 8;
        for (bit = 0; bit < 8; ++bit) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0));
    }
    return crc;
}
int protocol_decode(char *line, protocol_request_t *out)
{
    char *star, *space, *end, *p;
    unsigned long id, crc;
    size_t n = strlen(line);
    if (n && line[n - 1] == '\r') line[--n] = 0;
    if (n < 9 || n >= FRAME_MAX || line[0] != '@') return 0;
    star = strchr(line, '*');
    space = strchr(line, ' ');
    if (!star || !space || space >= star || space == line + 1 || strlen(star + 1) != 4) return 0;
    for (p = line + 1; p < space; ++p) if (*p < '0' || *p > '9') return 0;
    for (p = star + 1; *p; ++p) if (!isxdigit((unsigned char)*p)) return 0;
    for (p = space + 1; p < star; ++p) if (*p < 32 || *p > 126) return 0;
    crc = strtoul(star + 1, &end, 16);
    if (crc != protocol_crc(line + 1, (size_t)(star - line - 1))) return 0;
    errno = 0;
    id = strtoul(line + 1, &end, 10);
    if (errno || end != space || id == 0 || id > UINT32_MAX) return 0;
    *star = 0;
    out->id = (uint32_t)id;
    out->body = space + 1;
    return *out->body != 0;
}
int protocol_encode(uint32_t id, const char *body, char *out, size_t capacity)
{
    int n = snprintf(out, capacity, "@%lu %s", (unsigned long)id, body);
    uint16_t crc;
    if (n < 0 || (size_t)n + 7U > capacity) return 0;
    crc = protocol_crc(out + 1, (size_t)n - 1U);
    snprintf(out + n, capacity - (size_t)n, "*%04X\n", crc);
    return 1;
}
int protocol_int(const char *text, int32_t min, int32_t max, int32_t *out)
{
    char *end;
    long value;
    const char *p = text;
    if (!text || !*text) return 0;
    if (*p == '-') ++p;
    if (!*p) return 0;
    for (; *p; ++p) if (*p < '0' || *p > '9') return 0;
    errno = 0;
    value = strtol(text, &end, 10);
    if (errno || *end || value < min || value > max) return 0;
    *out = (int32_t)value;
    return 1;
}
