/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 3, 2024
  * @brief   ECE 362 Lab 6 Student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "kdtaneja";
/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
//void autotest();

void nano_wait(int);
uint32_t temperature = 0;
uint32_t brightness = 0;
int fan_status = 0;
int led_status = 1;
int channel_no = 1;

//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void) {
    //  GPIO for port A
    //  1-Input temperature sensor
    //  2-Input brightness sensor
    //  3-Output fan
    //  4-Output LED
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

    GPIOC->MODER |= 0x00055500; //set PC4-PC9 as O/P
    GPIOC->PUPDR |= 0x000000AA; //Set PC0-PC3 as O/P w/ pull-down resistors   

    //resetting all ports and setting 2-3 for output
    GPIOA->MODER &= ~0x00ff03fc;
    GPIOA->MODER |= 0x0015017c;
    //Pin 0 -> 00
    //Pin 1 -> 11
    //Pin 2 -> 11
    //Pin 3 -> 01
    //Pin 4 -> 01
    //Pin 8 -> 01
    //Pin 9 -> 01
    //Pin 10 -> 01
    //  ADC FOR PORTS 1-2
    //enabling high speed clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    //RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    //waiting for clock
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {
    }
    //enable ADC and wait for clock
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
    }
    //selecting channel 1
    //ADC1->CHSELR = ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2; 
    //ADC1->CFGR1 &= ~ADC_CFGR1_RES; //NOT SURE IF WE NEED THIS 

}


#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

void handle_fan(int temp, int fan){
    if(fan){
        if(temp < 20){
            setn(3,0);
            fan = 0;
        }
    }
    else{
        if(temp > 25){
            setn(3,1);
            fan = 1;
        }
    }
}

void handle_led(int bright, int led){
    if(led){
        if(bright > 300){
            setn(4,0);
            led = 0;
        }
    }
    else{
        if(bright < 200){
            setn(4,1);
            led = 1;
        }
    }
}

void setup_tim1(void) {

    // Generally the steps are similar to those in setup_tim3

    // except we will need to set the MOE bit in BDTR. 

    // Be sure to do so ONLY after enabling the RCC clock to TIM1.

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11);

    GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

    

    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);

    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL8_Pos) | (0x2 << GPIO_AFRH_AFSEL9_Pos) | (0x2 << GPIO_AFRH_AFSEL10_Pos) | (0x2 << GPIO_AFRH_AFSEL11_Pos);


    TIM1->BDTR |= TIM_BDTR_MOE;


    TIM1->PSC = 0;

    TIM1->ARR = 2399;


    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    TIM1->CCMR2 |= (0x6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;    

    TIM1->CCMR2 |= (0x6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;    

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;


    TIM1->CR1 |= TIM_CR1_CEN;
}

uint8_t bcd2dec(uint8_t bcd) {

    // Lower digit

    uint8_t dec = bcd & 0xF;


    // Higher digit

    dec += 10 * (bcd >> 4);

    return dec;

}


void setrgb(int rgb) {

    uint8_t b = bcd2dec(rgb & 0xFF);

    uint8_t g = bcd2dec((rgb >> 8) & 0xFF);

    uint8_t r = bcd2dec((rgb >> 16) & 0xFF);
    

    // TODO: Assign values to TIM1->CCRx registers

    // Remember these are all percentages

    // Also, LEDs are on when the corresponding PWM output is low

    // so you might want to invert the numbers.


    //uint32_t ARR = TIM2->ARR + 1; //was originally tim1


    TIM1->CCR1 = (100 - rgb) / 100; 
    //TIM2->CCR1 = ARR * (100 - rgb) / 100; 

    //TIM1->CCR2 = ARR * (100 - g) / 100;

    //TIM1->CCR3 = ARR * (100 - b) / 100;

}

void togglexn(GPIO_TypeDef *port, int n) {
  if (port->ODR & (1 << n)){
    port->ODR &= ~(1 << n);
  }
  else{
    port->ODR |= 1 << n;
  }
}

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;  
    ADC1->CR |= ADC_CR_ADSTART;
    //DO STUFF
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
    }
    
    if (channel_no == 1) {
        ADC1->CHSELR = ADC_CHSELR_CHSEL1;
        brightness = ADC1->DR; 
        //brightness = brightness / 1000;
        //if (brightness < 0) brightness = 0;
        if (brightness >= 1000) brightness = 999;
        
        if (brightness < 600){
            setn(3, 1);
            setrgb(0x000000);
        }
        else{
            setn(3, 0);
            setrgb(~0xf);
        }

        char str_2[12];
        itoa(brightness, str_2, 10);
        char temp_str_2[50] = "Bright: "; //temp_str_2 == temporary string 2
        strcat(temp_str_2, str_2);
        spi1_display1(temp_str_2);
    }

    else if (channel_no == 2) {
        ADC1->CHSELR = ADC_CHSELR_CHSEL2;
        temperature = ADC1->DR;
        char str[12];
        itoa(temperature, str, 10);
        char temp_str[50] = "Temp: ";
        strcat(temp_str, str);
        spi1_display2(temp_str); 
    }

    // ADC1->CHSELR = ADC_CHSELR_CHSEL2;
    // temperature = ADC1->DR;
    // // handle_fan(temperature, fan_status);
    // // handle_led(brightness, led_status);    
}

void setn(int32_t pin_num, int32_t val) {
  if (val == 0){
    GPIOA->BRR = (1 << pin_num);
  }
  else{
    GPIOA->BSRR = (1 << pin_num);
  }
}

//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 47999;
    TIM2->ARR = 499; //CURRENTLY 10 HZ
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

void init_spi1(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;  

    GPIOA->MODER &= ~((3 << (2 * 15)) | (3 << (2 * 5)) | (3 << (2 * 7)));  
    GPIOA->MODER |=  ((2 << (2 * 15)) | (2 << (2 * 5)) | (2 << (2 * 7)));  

    GPIOA->AFR[1] &= ~(0xF << (4 * (15 - 8)));  
    GPIOA->AFR[1] |=  (0x0 << (4 * (15 - 8)));  

    GPIOA->AFR[0] &= ~((0xF << (4 * 5)) | (0xF << (4 * 7)));  
    GPIOA->AFR[0] |=  ((0x0 << (4 * 5)) | (0x0 << (4 * 7)));  

    SPI1->CR1 &= ~SPI_CR1_SPE;

    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    SPI1->CR1 |= SPI_CR1_MSTR;

    SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | (0x9 << SPI_CR2_DS_Pos);  
    //SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_cmd(unsigned int data) {
    while ((SPI1->SR & SPI_SR_TXE) == 0)
        ;

    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);  

    spi_cmd(0x38);  
    spi_cmd(0x08); 
    spi_cmd(0x01);  
    nano_wait(2000000);  

    spi_cmd(0x06);  
    spi_cmd(0x02); 
    spi_cmd(0x0c);  
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);

    while (*string) {
        spi_data(*string++);
    }
}
void spi1_display2(const char *string) {
    spi_cmd(0xC0);  
    while (*string) {
        spi_data(*string++);
    }
}

//=============================================================================

// Part 2: Debounced keypad scanning.

//=============================================================================
uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()
//============================================================================

// The Timer 7 ISR

//============================================================================

// Write the Timer 7 ISR here.  Be sure to give it the right name.

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 0x03;
    drive_column(col);
}


//============================================================================

// init_tim7()

//============================================================================

void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 47999;
    TIM7->ARR = 1 - 1;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->SR &= ~TIM_SR_UIF;
    TIM7->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM7_IRQn);
}

//===========================================================================
// Main function
//===========================================================================
int main(void) {
    internal_clock();
    // GPIO enable
    enable_ports();
    // setup keyboard
    for (;;){
        char key = get_keypress();
        if (key == 'A'){
            channel_no = 1;
        }
        else if (key == 'B'){
            channel_no = 2;
        }
    }
    init_tim2();
    init_tim7();

    //sprintf("%d", temperature); 
    setup_tim1();
    init_spi1();
    spi1_init_oled();
    //spi1_display1("Hello again,");
    //const char temp_str = "test";
    // char temp_str = "test";
    // if (temperature == 35){
    //   temp_str = "35";
    // }
    //spi1_display2("50");

    //uint32_t x;

}



