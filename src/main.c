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
//void autotest();

uint32_t temperature = 2048;
int fan_status = 0;

//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void) {
    //  GPIO for port A
    //  1-Input temperature sensor
    //  2-Input brightness sensor
    //  3-Output fan
    //  4-Output LED
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    //resetting all ports and setting 2-3 for output
    GPIOA->MODER &= ~0x000003fc;
    GPIOA->MODER |= 0x00000140;
    
    //  ADC FOR PORTS 1-2
    //enabling high speed clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    //waiting for clock
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {
    }
    //enable ADC and wait for clock
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
    }
    //selecting channel 1
    ADC1->CHSELR = ADC_CHSELR_CHSEL1; 
    ADC1->CFGR1 &= ~ADC_CFGR1_RES; //NOT SURE IF WE NEED THIS 

}


#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;  
    ADC1->CR |= ADC_CR_ADSTART;
    //DO STUFF
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
    }

    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR; 
    bcn += 1;
    if (bcn >= BCSIZE) {
        bcn = 0; 
    }
    temperature = bcsum / BCSIZE;
    togglexn(GPIOA, 3);
    togglexn(GPIOA, 4);
}

void togglexn(GPIO_TypeDef *port, int n) {
  if (port->ODR & (1 << n)){
    port->ODR &= ~(1 << n);
  }
  else{
    port->ODR |= 1 << n;
  }
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

//===========================================================================
// Main function
//===========================================================================
void handle_fan(){
    if(fan_status){
        if(temperature < 20){
            setn(3,0);
            fan_status = 0;
        }
    }
    else{
        if(temperature > 25){
            setn(3,1);
            fan_status = 1;
        }
    }
}
int main(void) {
    internal_clock();


    // GPIO enable
    enable_ports();
    // setup keyboard
    init_tim2();

    printf("%d", temperature); 

    
}



