#include <gpio.h>
#include <stm32.h>
#include <delay.h>
#include "consts.h"
#include "configuration.h"



/* Macros for USART configuration    */

#define    BAUD_RATE              9600U
#define    HSI_HZ             16000000U
#define    PCLK1_HZ              HSI_HZ



/* Macros for I2C configuration       */

#define    I2C_SPEED_HZ           100000
#define    PCLK1_MHZ                  16
#define    CTRL_REG1_VALUE    0b01000111
#define    CTRL_REG3_VALUE    0b00000100



/* Macro for condition awaiting       */

#define     WAIT_MAX             1000000



/* Macros for TIM configuration       */

#define     PSC_VALUE                400
#define     ARR_VALUE               1000


void USART_configure(void) {
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


void DMA_configure() {
    DMA1_Stream6->CR = 4U << 25 |
                       DMA_SxCR_PL_1 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_DIR_0 |
                       DMA_SxCR_TCIE;

    DMA1_Stream6->PAR = (uint32_t) & USART2->DR;

    DMA1->HIFCR = DMA_HIFCR_CTCIF6;
}


void NVIC_configure() {
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
}


static
void wait_for_condition(uint16_t condition) {
    volatile uint32_t counter = 0;

    while (!(I2C1->SR1 & condition)) {
        ++counter;

        if (counter > WAIT_MAX) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return;
        }
    }
}


static
void I2C_configure_partial(uint8_t slave_register_number, uint8_t value) {
    I2C1->CR1 |= I2C_CR1_START;
    wait_for_condition(I2C_SR1_SB);

    I2C1->DR = LIS35DE_ADDR << 1;

    wait_for_condition(I2C_SR1_ADDR);

    I2C1->SR2;
    I2C1->DR = slave_register_number;

    wait_for_condition(I2C_SR1_TXE);

    I2C1->DR = value;

    wait_for_condition(I2C_SR1_BTF);

    I2C1->CR1 |= I2C_CR1_STOP;
}


void I2C_configure() {
    GPIOafConfigure(GPIOB,
                    8,
                    GPIO_OType_OD,
                    GPIO_Low_Speed,
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_I2C1);

    GPIOafConfigure(GPIOB,
                    9,
                    GPIO_OType_OD,
                    GPIO_Low_Speed,
                    GPIO_PuPd_NOPULL,
                    GPIO_AF_I2C1);

    I2C1->CR1 = 0;
    I2C1->CR2 = PCLK1_MHZ;

    I2C1->CCR = (PCLK1_MHZ * 1000000) /
                (I2C_SPEED_HZ << 1);

    I2C1->TRISE = PCLK1_MHZ + 1;

    I2C1->CR1 |= I2C_CR1_PE;

    __NOP();

    I2C_configure_partial(I2C_CTRL_REG1, CTRL_REG1_VALUE);

    Delay(100000);

    I2C_configure_partial(I2C_CTRL_REG3, CTRL_REG3_VALUE);
}


void TIM_configure() {
    TIM3->CR1 = 0;
    TIM3->PSC = 400;
    TIM3->ARR = 1000;

    TIM3->EGR = TIM_EGR_UG;

    TIM3->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
    TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;

    TIM3->CCR1 = 500;

    TIM3->CR1 |= TIM_CR1_CEN;
}


void RCC_configure() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_DMA1EN;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN |
                    RCC_APB1ENR_I2C1EN |
                    RCC_APB1ENR_TIM3EN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}


void USART_enable() {
    USART2->CR1 |= USART_CR1_UE;
}
