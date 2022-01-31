#include <gpio.h>
#include <stm32.h>
#include <string.h>
#include "configuration.h"
#include "consts.h"
#include "messages_queue.h"


#define     BUFFER_SIZE                          11
#define     BUFFER_POSITION_X                     0
#define     BUFFER_POSITION_Y                     4
#define     BUFFER_POSITION_CR                    8
#define     BUFFER_POSITION_LF                    9
#define     REGISTER_VALUE_DECIMAL_LENGTH         3


/* Enum representing the states of accelerometer register
 * value read operation
 */
typedef enum {
    IDLE,
    WRITING,
    READING
} accelerometer_read_state_t;


/* Integer value representing the number of accelerometer register from
 * which the value should be received
 */
static uint8_t target_register;


/* Integer value representing the state of accelerometer register read operation */
static accelerometer_read_state_t read_state;


/* Integer value representing the number of step of communication between
 * the program and accelerometer
 */
static uint32_t communication_step;


/* Integer value for reading acceleration from accelerometer register */
static uint8_t value_from_register;


/* Buffer for sending messages with acceleration values in format
 * Xacc_xYacc_y, where acc_x, acc_y are zero-padded integers
 * corresponding to acceleration on X and Y axes, respectively
 */
static char buffer[BUFFER_SIZE];


/* Static queue for queueing messages */
static messages_queue_t messages_queue;


static
void initiate_read_from_accelerometer_register(uint8_t register_number) {
    target_register = register_number;

    read_state = WRITING;
    communication_step = 0;

    I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
    I2C1->CR1 |= I2C_CR1_START;
}


static
void send_with_DMA(char *message_text) {
    DMA1_Stream6->M0AR = (uint32_t) message_text;
    DMA1_Stream6->NDTR = strlen(message_text);
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}


static
void send(char *message_text) {
    if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
        (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
        send_with_DMA(message_text);
    } else if (!is_queue_full(&messages_queue)) {
        enqueue(&messages_queue, message_text);
    }
}


static
void write_value_to_buffer(uint8_t register_number) {
    int buffer_offset = (register_number == REGISTER_X) ? BUFFER_POSITION_X
                                                        : BUFFER_POSITION_Y;

    uint8_t value = value_from_register;

    for (int i = REGISTER_VALUE_DECIMAL_LENGTH; i > 0; --i) {
        char char_to_buffer = (value % 10) + '0';
        buffer[buffer_offset + i] = char_to_buffer;
        value /= 10;
    }
}


void DMA1_Stream6_IRQHandler(void) {
    uint32_t isr = DMA1->HISR;

    if (isr & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        if (!is_queue_empty(&messages_queue)) {
            send_with_DMA(poll_queue(&messages_queue));
        }
    }
}


void I2C1_EV_IRQHandler() {
    if (read_state == WRITING) {
        if (communication_step == 0 && (I2C1->SR1 & I2C_SR1_SB)) {
            communication_step = 1;
            I2C1->DR = LIS35DE_ADDR << 1;
        } else if (communication_step == 1 && (I2C1->SR1 & I2C_SR1_ADDR)) {
            communication_step = 2;
            I2C1->SR2;

            I2C1->DR = target_register;
            __NOP();
            read_state = READING;
        } else {
            I2C1->CR1 |= I2C_CR1_STOP;
        }
    } else if (read_state == READING) {
        if (communication_step == 2 && (I2C1->SR1 & I2C_SR1_BTF)) {
            I2C1->CR1 |= I2C_CR1_START;
            communication_step = 3;
        } else if (communication_step == 3 && (I2C1->SR1 & I2C_SR1_SB)) {
            I2C1->DR = (LIS35DE_ADDR << 1) | 1U;
            I2C1->CR1 &= ~I2C_CR1_ACK;
            communication_step = 4;
        }
        if (communication_step == 4 && (I2C1->SR1 & I2C_SR1_ADDR)) {
            I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            communication_step = 5;
        }
        if (communication_step == 5 && (I2C1->SR1 & I2C_SR1_RXNE)) {
            value_from_register = I2C1->DR;
            __NOP();
            read_state = IDLE;
        }
    } else {
        communication_step = 0;
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C1->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    }
}


void TIM3_IRQHandler(void) {
    uint32_t interrupt_status = TIM3->SR & TIM3->DIER;

    if (interrupt_status & TIM_SR_UIF) {
        initiate_read_from_accelerometer_register(REGISTER_X);
        write_value_to_buffer(REGISTER_X);

        TIM3->SR = ~TIM_SR_UIF;
    }

    if (interrupt_status & TIM_SR_CC1IF) {
        initiate_read_from_accelerometer_register(REGISTER_Y);
        write_value_to_buffer(REGISTER_Y);

        TIM3->SR = ~TIM_SR_CC1IF;

        send(buffer);
    }
}


static
void init_buffer() {
    buffer[BUFFER_POSITION_X] = 'X';
    buffer[BUFFER_POSITION_Y] = 'Y';
    buffer[BUFFER_POSITION_CR] = '\r';
    buffer[BUFFER_POSITION_LF] = '\n';
}


int main(void) {
    init_buffer();

    RCC_configure();
    USART_configure();
    DMA_configure();
    NVIC_configure();
    I2C_configure();
    TIM_configure();

    USART_enable();

    for (;;) {}

    return 0;
}
