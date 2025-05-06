/**
 * @file: main.c
 * @author: Jaden Bouah
 * @student-number: BHXJAD001
 * @ps-no:1893647
 *
 * Student solution for practical 6.
 */

// Includes - must come before any defines
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdint.h>

// Define target processor
#define STM32F051

// Global Variables
volatile uint8_t SW0_PRESSED = 0;

//Function Declerations
void ResetClockTo48Mhz(void);
void init_student(void);
void init_ADC(void);
void init_GPIOB(void);
void init_GPIOA(void);
void init_TIM3 (void);

//Main Function
int main (void);
{
    ResetClockTo48Mhz();
    init_LCD();
    init_student();
    init_GPIOB();
    init_GPIOA();
    init_ADC();

    uint8_t flag = 0;

    while (1){

        if (!(flag) && (SW0_PRESSED))
        {
            init_TIM3();
            GPIOB->ODR = 0;  // clear leds
            flag = 1;

        }

    }
}

//Function Definitions
/**
 * Function resets the STM32 Clocks to 48 MHz
 */
void ResetClockTo48Mhz(void)
{
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
    {
        RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    }
    RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
    while ((RCC->CR & RCC_CR_PLLRDY) != 0);
    RCC->CFGR = ((RCC->CFGR & (~0x003C0000)) | 0x00280000);
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void init_student(void){

    lcd_command(CLEAR);  //creating character array for string
    lcd_putstring("BHXJAD001"); //placing string on LCD

}

void init_ADC(void){

   //initialising ADC
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER6;       // SeT to analogue mode

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;          // Enable ADC clock 
    ADC1->CHSELR = ADC_CHSELR_CHSEL6;          // Channel 6
    ADC1->CFGR1 |= ADC_CFGR1_RES_1;             // 8 bit resolution
    ADC1->CFGR1 |= ADC_CFGR1_ALIGN;             // left aligned data in adc register
    ADC1->CFGR1 |= ADC_CFGR1_WAIT;              // set to wait mode
    ADC1->CFGR1 |= ADC_CFGR1_CONT;              // set to continuous mode

    ADC1->IER |= ADC_IER_EOCIE;                 // enable interupts
    NVIC_EnableIRQ(ADC1_IRQn);                  // enable interupt in NVIC


    ADC1->CR |= ADC_CR_ADEN; // config to start
    while((ADC1 -> ISR & ADC_ISR_ADRDY) == 0); // Wait for ADC to be ready to start converting
    ADC1->CR |= ADC_CR_ADSTART;                // start conversion
}

void init_GPIOB(void)
{

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;          // Enable GPIOB clock
    GPIOB->MODER |= GPIO_MODER_MODER0_0;        //Setting led pin to output mode
    GPIOB->MODER |= GPIO_MODER_MODER1_0; 
    GPIOB->MODER |= GPIO_MODER_MODER2_0; 
    GPIOB->MODER |= GPIO_MODER_MODER3_0; 
    GPIOB->MODER |= GPIO_MODER_MODER4_0; 
    GPIOB->MODER |= GPIO_MODER_MODER5_0; 
    GPIOB->MODER |= GPIO_MODER_MODER6_0;   
    GPIOB->MODER |= GPIO_MODER_MODER7_0; 

}

void init_GPIOA(void){

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          // enable GPIOA clock
    GPIOA->MODER &= ~GPIO_MODER_MODER0;          // setting pin PA0 to input mode
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;        // enable pull-up resistor

}

void init_TIM3 (void) 
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // enable timer clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;      // enable GPIOB clock
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);  // clear bits

    GPIOB->MODER |= GPIO_MODER_MODER0_1;    // set pb0 to alternate function
    GPIOB->MODER |= GPIO_MODER_MODER1_1;    // set pb1 to alternate function

    GPIOB->AFR[0] |= 0x1 << (0*4);         // set pwm for PB0
    GPIOB->AFR[0] |= 0x1 << (1*4);         // set pwm for PB1

    TIM3->PSC = 47;                        // setting PSC value
    TIM3->ARR = 100;                       // setting ARR value

    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

    TIM3->CCER |= TIM_CCER_CC3E; // enable channel 3 output
    TIM3->CCER |= TIM_CCER_CC4E; // enable channel 4 output
    TIM3->CCER |= TIM_CCER_CC3P; //inverting channel 3

    TIM3->CCR3 = 0; // initial duty 
    TIM3->CCR4 = 0;

    TIM3->CR1 |= TIM_CR1_CEN;  // start the timer

}

void ADC1_COMP_IRQHandler(void)
{
    if(ADC1->ISR & ADC_ISR_EOC) {
        uint8_t ADC_DR = 0;
        ADC_DR = (ADC1->DR >> 8);    // reads result of adc conversion and lowers flag

        if ((!(GPIOA->IDR & GPIO_IDR_0))) // checking whether button has been pressed and hasnt been pressed before
            {
                SW0_PRESSED = 1;        // changing SW0 to true
            }

        if (SW0_PRESSED)  // checking if button SW0 has been pressed
        {
            uint8_t ADC_VAL = (ADC1->DR >> 8);
            uint8_t duty_cycle = (ADC_VAL * 100) / 255; // 0-100 range
            TIM3->CCR3 = duty_cycle;
            TIM3->CCR4 = duty_cycle;  //anti phase
        }
        else
        {
            GPIOB->ODR = ADC_DR;  // if not pressed, sets the LEDs to the value read from the ADC
        }

        ADC1->ISR |= ADC_ISR_EOC; // clear EOC flag
}
}

