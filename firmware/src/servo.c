/* S1: PH9 = TIM12_CH2 (270 degrees), S2: PH6 = TIM12_CH1 (180 degrees).
 * PCB connector routing is swapped; logical S1/S2 travel and tables stay tied
 * to the servo number, while both the GPIO and compare channel change here.
 * TIM12 is general purpose and has no BDTR/MOE, so each compare channel drives
 * its pin directly with no break or dead-time logic in the way. The prescaler is
 * chosen so one counter tick is exactly one microsecond: the compare value is
 * then the pulse width in microseconds, and nothing between the command and the
 * electrical output can round it. Both channels share the 20 ms period, so S1
 * and S2 stay phase-locked without any extra work.
 *
 * Angle is a half-degree step and the translation is the table below rather
 * than a division, because a divide by the travel constant is the one place a
 * silent arithmetic mistake would move the horn without any diagnostic. The
 * tables are linear: pulse[i] = PULSE_MIN + (PULSE_MAX - PULSE_MIN) * i /
 * channel_steps_max, rounded, spanning 500..2500 us. S1 has 541 entries,
 * S2 has 361. Recompute the respective table when its configuration changes. */
#include "servo.h"
#include "board.h"
#include "app_config.h"
typedef struct {
    GPIO_TypeDef *port;
    unsigned pin, channel, af;
} servo_hw_t;
static const servo_hw_t hw[SERVO_CHANNELS] = {
    {GPIOH, 9, 1, 2},   /* S1: PH9 = TIM12_CH2 */
    {GPIOH, 6, 0, 2}    /* S2: PH6 = TIM12_CH1 */
};
static const uint16_t pulse_us[SERVO_STEPS_MAX + 1] = {
    500, 504, 507, 511, 515, 519, 522, 526, 530, 533, 537, 541,
    544, 548, 552, 556, 559, 563, 567, 570, 574, 578, 581, 585,
    589, 593, 596, 600, 604, 607, 611, 615, 619, 622, 626, 630,
    633, 637, 641, 644, 648, 652, 656, 659, 663, 667, 670, 674,
    678, 681, 685, 689, 693, 696, 700, 704, 707, 711, 715, 719,
    722, 726, 730, 733, 737, 741, 744, 748, 752, 756, 759, 763,
    767, 770, 774, 778, 781, 785, 789, 793, 796, 800, 804, 807,
    811, 815, 819, 822, 826, 830, 833, 837, 841, 844, 848, 852,
    856, 859, 863, 867, 870, 874, 878, 881, 885, 889, 893, 896,
    900, 904, 907, 911, 915, 919, 922, 926, 930, 933, 937, 941,
    944, 948, 952, 956, 959, 963, 967, 970, 974, 978, 981, 985,
    989, 993, 996, 1000, 1004, 1007, 1011, 1015, 1019, 1022, 1026, 1030,
    1033, 1037, 1041, 1044, 1048, 1052, 1056, 1059, 1063, 1067, 1070, 1074,
    1078, 1081, 1085, 1089, 1093, 1096, 1100, 1104, 1107, 1111, 1115, 1119,
    1122, 1126, 1130, 1133, 1137, 1141, 1144, 1148, 1152, 1156, 1159, 1163,
    1167, 1170, 1174, 1178, 1181, 1185, 1189, 1193, 1196, 1200, 1204, 1207,
    1211, 1215, 1219, 1222, 1226, 1230, 1233, 1237, 1241, 1244, 1248, 1252,
    1256, 1259, 1263, 1267, 1270, 1274, 1278, 1281, 1285, 1289, 1293, 1296,
    1300, 1304, 1307, 1311, 1315, 1319, 1322, 1326, 1330, 1333, 1337, 1341,
    1344, 1348, 1352, 1356, 1359, 1363, 1367, 1370, 1374, 1378, 1381, 1385,
    1389, 1393, 1396, 1400, 1404, 1407, 1411, 1415, 1419, 1422, 1426, 1430,
    1433, 1437, 1441, 1444, 1448, 1452, 1456, 1459, 1463, 1467, 1470, 1474,
    1478, 1481, 1485, 1489, 1493, 1496, 1500, 1504, 1507, 1511, 1515, 1519,
    1522, 1526, 1530, 1533, 1537, 1541, 1544, 1548, 1552, 1556, 1559, 1563,
    1567, 1570, 1574, 1578, 1581, 1585, 1589, 1593, 1596, 1600, 1604, 1607,
    1611, 1615, 1619, 1622, 1626, 1630, 1633, 1637, 1641, 1644, 1648, 1652,
    1656, 1659, 1663, 1667, 1670, 1674, 1678, 1681, 1685, 1689, 1693, 1696,
    1700, 1704, 1707, 1711, 1715, 1719, 1722, 1726, 1730, 1733, 1737, 1741,
    1744, 1748, 1752, 1756, 1759, 1763, 1767, 1770, 1774, 1778, 1781, 1785,
    1789, 1793, 1796, 1800, 1804, 1807, 1811, 1815, 1819, 1822, 1826, 1830,
    1833, 1837, 1841, 1844, 1848, 1852, 1856, 1859, 1863, 1867, 1870, 1874,
    1878, 1881, 1885, 1889, 1893, 1896, 1900, 1904, 1907, 1911, 1915, 1919,
    1922, 1926, 1930, 1933, 1937, 1941, 1944, 1948, 1952, 1956, 1959, 1963,
    1967, 1970, 1974, 1978, 1981, 1985, 1989, 1993, 1996, 2000, 2004, 2007,
    2011, 2015, 2019, 2022, 2026, 2030, 2033, 2037, 2041, 2044, 2048, 2052,
    2056, 2059, 2063, 2067, 2070, 2074, 2078, 2081, 2085, 2089, 2093, 2096,
    2100, 2104, 2107, 2111, 2115, 2119, 2122, 2126, 2130, 2133, 2137, 2141,
    2144, 2148, 2152, 2156, 2159, 2163, 2167, 2170, 2174, 2178, 2181, 2185,
    2189, 2193, 2196, 2200, 2204, 2207, 2211, 2215, 2219, 2222, 2226, 2230,
    2233, 2237, 2241, 2244, 2248, 2252, 2256, 2259, 2263, 2267, 2270, 2274,
    2278, 2281, 2285, 2289, 2293, 2296, 2300, 2304, 2307, 2311, 2315, 2319,
    2322, 2326, 2330, 2333, 2337, 2341, 2344, 2348, 2352, 2356, 2359, 2363,
    2367, 2370, 2374, 2378, 2381, 2385, 2389, 2393, 2396, 2400, 2404, 2407,
    2411, 2415, 2419, 2422, 2426, 2430, 2433, 2437, 2441, 2444, 2448, 2452,
    2456, 2459, 2463, 2467, 2470, 2474, 2478, 2481, 2485, 2489, 2493, 2496,
    2500,
};
/* S2: 0..180 degrees in half-degree steps, provisional 500..2500 us. */
static const uint16_t pulse_us_s2[SERVO2_STEPS_MAX + 1] = {
    500, 506, 511, 517, 522, 528, 533, 539, 544, 550, 556, 561,
    567, 572, 578, 583, 589, 594, 600, 606, 611, 617, 622, 628,
    633, 639, 644, 650, 656, 661, 667, 672, 678, 683, 689, 694,
    700, 706, 711, 717, 722, 728, 733, 739, 744, 750, 756, 761,
    767, 772, 778, 783, 789, 794, 800, 806, 811, 817, 822, 828,
    833, 839, 844, 850, 856, 861, 867, 872, 878, 883, 889, 894,
    900, 906, 911, 917, 922, 928, 933, 939, 944, 950, 956, 961,
    967, 972, 978, 983, 989, 994, 1000, 1006, 1011, 1017, 1022, 1028,
    1033, 1039, 1044, 1050, 1056, 1061, 1067, 1072, 1078, 1083, 1089, 1094,
    1100, 1106, 1111, 1117, 1122, 1128, 1133, 1139, 1144, 1150, 1156, 1161,
    1167, 1172, 1178, 1183, 1189, 1194, 1200, 1206, 1211, 1217, 1222, 1228,
    1233, 1239, 1244, 1250, 1256, 1261, 1267, 1272, 1278, 1283, 1289, 1294,
    1300, 1306, 1311, 1317, 1322, 1328, 1333, 1339, 1344, 1350, 1356, 1361,
    1367, 1372, 1378, 1383, 1389, 1394, 1400, 1406, 1411, 1417, 1422, 1428,
    1433, 1439, 1444, 1450, 1456, 1461, 1467, 1472, 1478, 1483, 1489, 1494,
    1500, 1506, 1511, 1517, 1522, 1528, 1533, 1539, 1544, 1550, 1556, 1561,
    1567, 1572, 1578, 1583, 1589, 1594, 1600, 1606, 1611, 1617, 1622, 1628,
    1633, 1639, 1644, 1650, 1656, 1661, 1667, 1672, 1678, 1683, 1689, 1694,
    1700, 1706, 1711, 1717, 1722, 1728, 1733, 1739, 1744, 1750, 1756, 1761,
    1767, 1772, 1778, 1783, 1789, 1794, 1800, 1806, 1811, 1817, 1822, 1828,
    1833, 1839, 1844, 1850, 1856, 1861, 1867, 1872, 1878, 1883, 1889, 1894,
    1900, 1906, 1911, 1917, 1922, 1928, 1933, 1939, 1944, 1950, 1956, 1961,
    1967, 1972, 1978, 1983, 1989, 1994, 2000, 2006, 2011, 2017, 2022, 2028,
    2033, 2039, 2044, 2050, 2056, 2061, 2067, 2072, 2078, 2083, 2089, 2094,
    2100, 2106, 2111, 2117, 2122, 2128, 2133, 2139, 2144, 2150, 2156, 2161,
    2167, 2172, 2178, 2183, 2189, 2194, 2200, 2206, 2211, 2217, 2222, 2228,
    2233, 2239, 2244, 2250, 2256, 2261, 2267, 2272, 2278, 2283, 2289, 2294,
    2300, 2306, 2311, 2317, 2322, 2328, 2333, 2339, 2344, 2350, 2356, 2361,
    2367, 2372, 2378, 2383, 2389, 2394, 2400, 2406, 2411, 2417, 2422, 2428,
    2433, 2439, 2444, 2450, 2456, 2461, 2467, 2472, 2478, 2483, 2489, 2494,
    2500,
};
static volatile uint16_t current_us[SERVO_CHANNELS];
static volatile unsigned hold[SERVO_CHANNELS];

/* Nearest table index for a pulse; only the calibration path can produce a
 * value that is not already an exact table entry. */
static unsigned nearest_step(unsigned channel, unsigned us)
{
    const uint16_t *table = channel == 1U ? pulse_us_s2 : pulse_us;
    unsigned limit = servo_steps_max(channel);
    unsigned best = 0, low = 0, high = limit;
    if (us <= table[0]) return 0;
    if (us >= table[limit]) return limit;
    while (low <= high) {
        unsigned mid = (low + high) / 2U;
        if (table[mid] <= us) { best = mid; low = mid + 1U; }
        else { if (mid == 0) break; high = mid - 1U; }
    }
    if (best < limit && us - table[best] > table[best + 1U] - us) ++best;
    return best;
}

/* Caller holds the lock; the compare channel is a direct index from CCR1. */
static void apply(unsigned channel, unsigned us)
{
    (&TIM12->CCR1)[hw[channel].channel] = us;
    current_us[channel] = (uint16_t)us;
    hold[channel] = 1;
}

void servo_init(void)
{
    unsigned i;
    /* APB1 timers run at 2x PCLK1 (200 MHz); PSC=199 gives one tick per microsecond. */
    RCC->APB1LENR |= RCC_APB1LENR_TIM12EN;
    (void)RCC->APB1LENR;
    TIM12->CR1 = 0;
    TIM12->PSC = TIMER_HZ / 1000000UL - 1U;
    TIM12->ARR = 1000000UL / SERVO_PWM_HZ - 1U;
    TIM12->CCMR1 = 0;
    TIM12->CCMR2 = 0;
    TIM12->CCER = 0;
    TIM12->DIER = 0;
    TIM12->EGR = TIM_EGR_UG; /* Load PSC/ARR/CCR once, before any output goes live. */
    TIM12->SR = 0;
    for (i = 0; i < SERVO_CHANNELS; ++i) {
        const servo_hw_t *h = &hw[i];
        const uint16_t *table = i == 1U ? pulse_us_s2 : pulse_us;
        unsigned home = i == 1U ? SERVO2_HOME_STEPS : SERVO_HOME_STEPS;
        unsigned shift = (h->channel % 2U) * 8U;
        volatile uint32_t *reg = h->channel < 2U ? &TIM12->CCMR1 : &TIM12->CCMR2;
        if (home > servo_steps_max(i)) home = servo_steps_max(i);
        /* Every channel is held low from reset until its compare output is
         * enabled below, so a booting board cannot twitch a horn. */
        (&TIM12->CCR1)[h->channel] = 0;
        /* PWM1 is OCxM=0b0110: both mode bits are required. OCxM=0b0010
         * only forces inactive on compare and cannot generate servo pulses.
         * Preload applies a late CCR write at the next period boundary. */
        *reg |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE) << shift;
        board_gpio(h->port, h->pin, 2, h->af);
        TIM12->CCER |= TIM_CCER_CC1E << (h->channel * 4U);
        apply(i, table[home]);
    }
    TIM12->CR1 = TIM_CR1_CEN;
}

int servo_set_pulse_us(unsigned channel, unsigned us)
{
    uint32_t key;
    if (channel >= SERVO_CHANNELS) return 0;
    /* Never silently clamp: a value outside the electrical range is a bug or a
     * servo wired to the wrong constants, and the mechanical stop is the only
     * limit left once the pulse is out of spec. */
    if (us < SERVO_PULSE_MIN_US || us > SERVO_PULSE_MAX_US) return 0;
    key = board_lock();
    apply(channel, us);
    board_unlock(key);
    return 1;
}

int servo_set_steps(unsigned channel, unsigned steps)
{
    const uint16_t *table = channel == 1U ? pulse_us_s2 : pulse_us;
    if (channel >= SERVO_CHANNELS || steps > servo_steps_max(channel)) return 0;
    return servo_set_pulse_us(channel, table[steps]);
}

int servo_status(unsigned channel, servo_status_t *out)
{
    uint32_t key;
    if (channel >= SERVO_CHANNELS || !out) return 0;
    key = board_lock();
    out->pulse_us = current_us[channel];
    out->on = hold[channel];
    out->angle_tenths = (uint16_t)(nearest_step(channel, out->pulse_us) * 5U);
    board_unlock(key);
    return 1;
}

unsigned servo_channels(void) { return SERVO_CHANNELS; }

/* Query a positional channel's limit for table bounds and protocol validation.
 * Channel indices are zero-based; this is not a mechanical feedback reading. */
unsigned servo_steps_max(unsigned channel)
{
    if (channel >= SERVO_CHANNELS) return 0;
    return channel == 1U ? SERVO2_STEPS_MAX : SERVO_STEPS_MAX;
}

/* Physical identity comes from the same table that writes GPIO/CCR. */
unsigned servo_pin(unsigned channel) { return channel < SERVO_CHANNELS ? hw[channel].pin : 0; }
unsigned servo_timer_channel(unsigned channel) { return channel < SERVO_CHANNELS ? hw[channel].channel + 1U : 0; }
