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
#include <stdio.h>
//void autotest();

uint32_t temperature = 0;
uint32_t brightness = 0;
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
    GPIOA->MODER |= 0x00000030;
    
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
    ADC1->CHSELR = ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2; 
    //ADC1->CFGR1 &= ~ADC_CFGR1_RES; //NOT SURE IF WE NEED THIS 

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

    temperature = ADC1->DR;
    brightness = ADC1->DR; //1970 is default, 2400 is max
    
    handle_fan();

    char str[12];
    itoa(temperature, str, 10);
    char temp_str[50] = "Temperature: ";
    strcat(temp_str, str);
    spi1_display2(temp_str); 

    char str_2[12];
    itoa(brightness, str_2, 10);
    char temp_str_2[50] = "Brightness: ";
    strcat(temp_str_2, str_2);
    spi1_display1(temp_str_2);
    //if (temperature > 1) temperature--;
    //printf("Temperature: %d", fan_status);
    //setn(4, 1);
    
    // togglexn(GPIOA, 3);
    // togglexn(GPIOA, 4);
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
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
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

  //sprintf("%d", temperature); 

  init_spi1();
  spi1_init_oled();
  //spi1_display1("Hello again,");
  //const char temp_str = "test";
  // char temp_str = "test";
  // if (temperature == 35){
  //   temp_str = "35";
  // }
  //spi1_display2("50");

  uint32_t x;
  

  // char str[12];
  // itoa(temperature, str, 10);
  // char temp_str[50] = "Temperature: ";
  // strcat(temp_str, str);
  // spi1_display2(temp_str); 

}



