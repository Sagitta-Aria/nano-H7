/* USART1 (COM/CH340) plus UART7 (wireless PA8 RX / PA15 TX), both 115200 8N1.
 * Each IRQ owns a separate SPSC ring. Only comm reads rings and broadcasts TX;
 * raw printf or echo must not share these framed protocol streams. */
#include "serial.h"
#include "board.h"
#include "app_config.h"
#define RX_SIZE 1024U
typedef struct {
    uint8_t bytes[RX_SIZE];
    volatile unsigned head, tail;
    volatile uint32_t received, errors;
} serial_rx_t;
static serial_rx_t rx[SERIAL_PORT_COUNT];
static USART_TypeDef *const uart[SERIAL_PORT_COUNT] = {USART1, UART7};
static uint32_t tx_errors;

void serial_init(void)
{
    unsigned i;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1LENR |= RCC_APB1LENR_UART7EN;
    (void)RCC->APB1LENR;
    /* USART1 kernel=PCLK2, UART7 kernel=PCLK1; both APBs run at APB_HZ. */
    RCC->D2CCIP2R &= ~(RCC_D2CCIP2R_USART16SEL | RCC_D2CCIP2R_USART28SEL);
    board_gpio(GPIOA, 9, 2, 7);
    board_gpio(GPIOA, 10, 2, 7);
    board_gpio(GPIOA, 8, 2, 11);
    board_gpio(GPIOA, 15, 2, 11);
    GPIOA->PUPDR |= (1UL << 20) | (1UL << 16); /* Both RX pins idle high. */
    for (i = 0; i < SERIAL_PORT_COUNT; ++i) {
        uart[i]->CR1 = 0;
        uart[i]->CR2 = 0;
        uart[i]->CR3 = 0;
        uart[i]->PRESC = 0;
        uart[i]->BRR = (APB_HZ + UART_BAUD / 2) / UART_BAUD;
        uart[i]->ICR = 0xFFFFFFFFUL;
        uart[i]->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;
    }
    NVIC_SetPriority(USART1_IRQn, 6);
    NVIC_SetPriority(UART7_IRQn, 6);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(UART7_IRQn);
}

/* One producer per ring; equal IRQ priorities also avoid nested UART writers. */
static void receive_irq(unsigned port)
{
    serial_rx_t *r = &rx[port];
    uint32_t flags = uart[port]->ISR;
    if (flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        uart[port]->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        ++r->errors;
    }
    if (flags & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)uart[port]->RDR;
        unsigned next = (r->head + 1) & (RX_SIZE - 1);
        ++r->received;
        if (next == r->tail) ++r->errors;
        else { r->bytes[r->head] = byte; __DMB(); r->head = next; }
    }
}
void USART1_IRQHandler(void) { receive_irq(SERIAL_COM); }
void UART7_IRQHandler(void) { receive_irq(SERIAL_WIRELESS); }

int serial_read(unsigned port)
{
    serial_rx_t *r;
    uint8_t byte;
    if (port >= SERIAL_PORT_COUNT) return -1;
    r = &rx[port];
    if (r->tail == r->head) return -1;
    byte = r->bytes[r->tail];
    __DMB();
    r->tail = (r->tail + 1) & (RX_SIZE - 1);
    return byte;
}

/* Advance both transmitters independently, with a 10 ms no-progress deadline
 * per port. A stalled UART cannot suppress the other copy. IRQs stay enabled;
 * higher-priority motion/supervisor tasks can preempt this polling sender. */
int serial_write(const char *data)
{
    const char *cursor[SERIAL_PORT_COUNT] = {data, data};
    uint32_t progress[SERIAL_PORT_COUNT];
    unsigned i, pending = 3U, ok = 1;
    progress[0] = progress[1] = DWT->CYCCNT;
    while (pending) {
        for (i = 0; i < SERIAL_PORT_COUNT; ++i) {
            if (!(pending & (1U << i))) continue;
            if (!*cursor[i]) { pending &= ~(1U << i); continue; }
            if (uart[i]->ISR & USART_ISR_TXE_TXFNF) {
                uart[i]->TDR = (uint8_t)*cursor[i]++;
                progress[i] = DWT->CYCCNT;
            } else if ((uint32_t)(DWT->CYCCNT - progress[i]) > CPU_HZ / 100) {
                ++tx_errors; ok = 0; pending &= ~(1U << i);
            }
        }
    }
    return (int)ok;
}
uint32_t serial_rx_bytes(void) { return rx[0].received + rx[1].received; }
uint32_t serial_rx_errors(void) { return rx[0].errors + rx[1].errors; }
uint32_t serial_tx_errors(void) { return tx_errors; }
