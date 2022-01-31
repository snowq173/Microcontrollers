#include <gpio.h>
#include <irq.h>
#include <stm32.h>
#include <string.h>

#define BAUD_RATE 9600U
#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ

#define CONTROLLER_BUTTONS_NUMBER       7

typedef struct {
    GPIO_TypeDef *gpio;
    uint32_t reg;
    char *message_press;
    char *message_release;
    uint32_t neg;
} button_t;

#define STR_LEFT "LEFT"
#define STR_RIGHT "RIGHT"
#define STR_UP "UP"
#define STR_DOWN "DOWN"
#define STR_FIRE "FIRE"
#define STR_USER "USER"
#define STR_MODE "MODE"

#define PRESSED " PRESSED\r\n"
#define RELEASED " RELEASED\r\n"

#define MSG(BUTTON, TYPE) BUTTON ## #TYPE

static
button_t controller_buttons[CONTROLLER_BUTTONS_NUMBER] = {
        {GPIOB, 3, MSG(STR_LEFT, RELEASED),  "LEFT RELEASED\r\n",  0},
        {GPIOB, 4, MSG(STR_RIGHT, RELEASED), "RIGHT RELEASED\r\n", 0},
        {GPIOB, 5,  "UP PRESSED\r\n",        "UP RELEASED\r\n",    0},
        {GPIOB, 6,  "DOWN PRESSED\r\n",      "DOWN RELEASED\r\n",  0},
        {GPIOB, 10, "FIRE PRESSED\r\n",      "FIRE RELEASED\r\n",  0},
        {GPIOC, 13, "USER PRESSED\r\n",      "USER RELEASED\r\n",  0},
        {GPIOA, 0,  "MODE PRESET\r\n",       "MODE RELEASED\r\n",  1}
};

#define MESSAGES_QUEUE_SIZE             512

static struct {
    char *buffer[MESSAGES_QUEUE_SIZE];
    int32_t read_pos;
    int32_t insert_pos;
    int32_t used;
} messages;

static
void clear_queue(void) {
    messages.read_pos = 0;
    messages.insert_pos = 0;
    messages.used = 0;
}

static
int32_t is_queue_empty(void) {
    return messages.used == 0;
}

static
int32_t is_queue_full(void) {
    return messages.used == MESSAGES_QUEUE_SIZE;
}

static
char *queue_poll(void) {
    char *ptr = messages.buffer[messages.read_pos];
    messages.read_pos = (messages.read_pos + 1) % MESSAGES_QUEUE_SIZE;
    messages.used--;
    return ptr;
}

static
void queue_push(char *ptr) {
    messages.buffer[messages.insert_pos] = ptr;
    messages.insert_pos = (messages.insert_pos + 1) % MESSAGES_QUEUE_SIZE;
    messages.used++;
}

static
void configure_button(button_t *button) {
    GPIOinConfigure(button->gpio,
                    button->reg,
                    GPIO_PuPd_UP,
                    EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);
}

static
void RCC_configure(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_DMA1EN;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

static
void UART_configure(void) {
    GPIOafConfigure(GPIOA,
                    2,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_USART2);

    GPIOafConfigure(GPIOA,
                    3,
                    GPIO_OType_PP,
                    GPIO_Fast_Speed,
                    GPIO_PuPd_UP,
                    GPIO_AF_USART2);

    USART2->CR1 = USART_CR1_RE | USART_CR1_TE;
    USART2->CR2 = 0;
    USART2->BRR = (PCLK1_HZ + (BAUD_RATE / 2U)) / BAUD_RATE;
    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
}

static
void DMA_configure(void) {
    DMA1_Stream6->CR = 4U << 25 |
                       DMA_SxCR_PL_1 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_DIR_0 |
                       DMA_SxCR_TCIE;

    DMA1_Stream6->PAR = (uint32_t) & USART2->DR;

    DMA1_Stream5->CR = 4U << 25 |
                       DMA_SxCR_PL_1 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_TCIE;

    DMA1_Stream5->PAR = (uint32_t) & USART2->DR;

    DMA1->HIFCR = DMA_HIFCR_CTCIF6 |
                  DMA_HIFCR_CTCIF5;
}

static
void NVIC_configure(void) {
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static
uint32_t is_pressed(button_t *button) {
    return ((button->gpio->IDR >> button->reg) & 1) ^ button->neg;
}

static
void send_to_DMA1(char *message) {
    DMA1_Stream6->M0AR = (uint32_t) message;
    DMA1_Stream6->NDTR = strlen(message);
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

static
void interrupt_handler(uint32_t EXTI_PR_STATE,
                       uint32_t LINE_INTERRUPT_STATE,
                       button_t *button) {
    if (EXTI_PR_STATE & LINE_INTERRUPT_STATE) {
        char *message = is_pressed(button)
                        ? button->message_release
                        : button->message_press;

        if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
            (DMA1->HISR & DMA_HISR_TCIF6) == 0) {

            send_to_DMA1(message);
        } else if (!is_queue_full()) {
            queue_push(message);
        }

        EXTI->PR = LINE_INTERRUPT_STATE;
    }
}

void DMA1_Stream6_IRQHandler(void) {
    uint32_t isr = DMA1->HISR;

    if (isr & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        if (!is_queue_empty()) {
            send_to_DMA1(queue_poll());
        }
    }
}

void EXTI0_IRQHandler(void) {
    uint32_t interrupt_state = EXTI->PR;
    interrupt_handler(interrupt_state, EXTI_PR_PR0, &controller_buttons[6]);
}

void EXTI3_IRQHandler(void) {
    uint32_t interrupt_state = EXTI->PR;
    interrupt_handler(interrupt_state, EXTI_PR_PR3, &controller_buttons[0]);
}

void EXTI4_IRQHandler(void) {
    uint32_t interrupt_state = EXTI->PR;
    interrupt_handler(interrupt_state, EXTI_PR_PR4, &controller_buttons[1]);
}

void EXTI9_5_IRQHandler(void) {
    uint32_t interrupt_state = EXTI->PR;
    interrupt_handler(interrupt_state, EXTI_PR_PR5, &controller_buttons[2]);
    interrupt_handler(interrupt_state, EXTI_PR_PR6, &controller_buttons[3]);
}

void EXTI15_10_IRQHandler(void) {
    uint32_t interrupt_state = EXTI->PR;
    interrupt_handler(interrupt_state, EXTI_PR_PR10, &controller_buttons[4]);
    interrupt_handler(interrupt_state, EXTI_PR_PR13, &controller_buttons[5]);
}

int main(void) {
    clear_queue();

    RCC_configure();

    __NOP();

    UART_configure();
    DMA_configure();
    NVIC_configure();

    for (int i = 0; i < CONTROLLER_BUTTONS_NUMBER; ++i) {
        configure_button(controller_buttons + i);
    }

    USART2->CR1 |= USART_CR1_UE;

    for (;;) {}

    return 0;
}
