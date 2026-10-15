/* Register-only board support. No external SDRAM, LCD, USB or camera is enabled.
 * The RGB LED has a permanently powered red die; only green/blue are writable. */
#include "board.h"
#include "app_config.h"
static uint32_t reset_flags;
static unsigned led_state;

uint32_t board_lock(void) { uint32_t key = __get_PRIMASK(); __disable_irq(); return key; }
void board_unlock(uint32_t key) { __set_PRIMASK(key); }

/* Configure one pin; the caller must enable its GPIO clock first. */
void board_gpio(GPIO_TypeDef *port, unsigned pin, unsigned mode, unsigned af)
{
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << (pin * 2))) | (2UL << (pin * 2));
    port->PUPDR &= ~(3UL << (pin * 2));
    port->AFR[pin / 8] = (port->AFR[pin / 8] & ~(15UL << ((pin % 8) * 4))) | (af << ((pin % 8) * 4));
    port->MODER = (port->MODER & ~(3UL << (pin * 2))) | (mode << (pin * 2));
}

/* Bounded boot wait; failures remain on HSI with motor GPIOs unconfigured. */
static void wait_ready(volatile uint32_t *reg, uint32_t mask, uint32_t value)
{
    uint32_t count = 10000000;
    while ((*reg & mask) != value) {
        if (--count == 0) { __disable_irq(); for (;;) {} }
    }
}

void board_init(void)
{
    reset_flags = RCC->RSR;
    RCC->RSR |= RCC_RSR_RMVF;
    /* This board supplies VCAP externally: use bypass, as in its vendor BSP.
     * 400 MHz / VOS1 avoids depending on revision-V-only 480 MHz operation. */
    PWR->CR3 = (PWR->CR3 & ~((1UL << 1) | (1UL << 2))) | 1UL;
    wait_ready(&PWR->CSR1, 1UL << 13, 1UL << 13);
    PWR->D3CR = (PWR->D3CR & ~(3UL << 14)) | (3UL << 14);
    wait_ready(&PWR->D3CR, 1UL << 13, 1UL << 13);
    RCC->CR |= RCC_CR_HSEON;
    wait_ready(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);
    /* HSE / 5 * 160 / 2 = 400 MHz. AHB=200 MHz; all APBs=100 MHz. */
    RCC->PLLCKSELR = 2UL | (5UL << 4);
    RCC->PLLCFGR = (2UL << 2) | (1UL << 16);
    RCC->PLL1DIVR = 159UL | (1UL << 9) | (3UL << 16) | (1UL << 24);
    FLASH->ACR = (FLASH->ACR & ~0x3FUL) | 4UL | (2UL << 4);
    wait_ready(&FLASH->ACR, 15UL, 4UL);
    RCC->D1CFGR = 8UL | (4UL << 4);
    RCC->D2CFGR = (4UL << 4) | (4UL << 8);
    RCC->D3CFGR = 4UL << 4;
    RCC->CR |= RCC_CR_PLL1ON;
    wait_ready(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY);
    RCC->CFGR = (RCC->CFGR & ~7UL) | 3UL;
    wait_ready(&RCC->CFGR, 7UL << 3, 3UL << 3);
    SystemCoreClockUpdate();
    SCB->VTOR = FLASH_BANK1_BASE;
    NVIC_SetPriorityGrouping(3); /* Four preemption bits, no subpriority. */
    SCB_EnableICache();
    /* D-cache deliberately off until a DMA/cache ownership policy is needed. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55UL; /* Unlock Cortex-M7 cycle counter for bounded waits. */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN |
                      RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIOGEN |
                      RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOIEN;
    (void)RCC->AHB4ENR;
    GPIOB->BSRR = 3UL;
    board_gpio(GPIOB, 0, 1, 0);
    board_gpio(GPIOB, 1, 1, 0);
    board_led(0, 0);
}

/* LEDs are common-anode: logical on drives the corresponding pin low. */
void board_led(unsigned green, unsigned blue)
{
    unsigned on = (green ? 1U : 0U) | (blue ? 2U : 0U);
    GPIOB->BSRR = ((uint32_t)on << 16) | ((~on) & 3U);
    led_state = on;
}
unsigned board_led_state(void) { return led_state; }
uint32_t board_reset_flags(void) { return reset_flags; }

/* Only short peripheral setup waits; never use for motion pulse scheduling. */
void board_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < us * (CPU_HZ / 1000000UL)) {}
}
