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
int led_status = 1;
uint8_t mode = 'C';
uint8_t col = 0;
char keymap;
char* keymap_arr = &keymap;
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

    //--------------------------
    // Configure PA8, PA9, PA10 as general-purpose output
    GPIOA->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)) | (3 << (10 * 2))); // Clear mode bits
    GPIOA->MODER |= ((1 << (8 * 2)) | (1 << (9 * 2)) | (1 << (10 * 2)));  // Set output mode

    // Configure PA8, PA9, PA10 as push-pull (default state)
    GPIOA->OTYPER &= ~((1 << 8) | (1 << 9) | (1 << 10));

    // Configure PA8, PA9, PA10 for low speed (default state)
    GPIOA->OSPEEDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)) | (3 << (10 * 2)));

    // Disable pull-up/pull-down resistors for PA8, PA9, PA10
    GPIOA->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)) | (3 << (10 * 2)));
    //--------------------------
    
    GPIOC->MODER &= ~(0x000000FF); //Clear mode bits for PC0-PC3 (input mode is 00)
    GPIOC->PUPDR &= ~(0x000000FF); //Clear the pull-up/pull-down bits for PC0-PC3
    GPIOC->PUPDR |= 0x000000AA; //Set PC0-PC3 to be internally pulled down (10)
    GPIOC->MODER &= ~(0x0000FF00); //Clear mode bits for PC4-PC7
    GPIOC->MODER |= 0x00005500; //Set PC4-PC7 as outputs
    //GPIOC->PUPDR |= (0x00000055);      

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
}

uint8_t bcd2dec(uint8_t bcd) {

    // Lower digit

    uint8_t dec = bcd & 0xF;


    // Higher digit

    dec += 10 * (bcd >> 4);

    return dec;

}


#define BCSIZE 6
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

void write_display() {
    char str_2[12];
    char str_3[12];
    setn(3, 0);    
    switch (mode){
    case('1'):
    
    spi1_display1("                ");
    spi1_display2("                ");
      setn(8,0);
      setn(9,0);
      setn(10,0);
      spi1_display1("MANUAL");
      break;
    case('2'):
    spi1_display1("                ");
    spi1_display2("                ");
      setn(8,1);
      setn(9,0);
      setn(10,0);
      spi1_display1("MANUAL");
      break;
    case('3'):
    spi1_display1("                ");
    spi1_display2("                ");
      setn(8,0);
      setn(9,1);
      setn(10,0);
      spi1_display1("MANUAL");
      break;
    

    case ('C'):
      setn(8,1);
      setn(9,1);
      setn(10,1);
      spi1_display1("Press A- Bright");
      spi1_display2("Press B- Temp");
  
      break;    

    case('A'):
    ADC1->CHSELR &= ~ADC_CHSELR_CHSEL2;
    ADC1->CHSELR = ADC_CHSELR_CHSEL1;
    brightness = ADC1->DR;
    
    //BOXCAR AVERAGING********************************* 
    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    brightness = bcsum / BCSIZE;
    //BOXCAR AVERAGING********************************* 


    if (brightness < 400){ //purple if at night and turns on fan
      setn(3, 1);
      setn(8,0);
      setn(9,1);
      setn(10,0);
    }
    else{ // blue during the day
        setn(3, 0);
        setn(8,1);
        setn(9,0);
        setn(10,0);
    }
    spi1_display1("                ");
    spi1_display2("                ");
    itoa(brightness, str_2, 10);
    char temp_str_2[50] = "Bright: "; //temp_str_2 == temporary string 2
    strcat(temp_str_2, str_2);
    spi1_display1(temp_str_2);
    break;

    case('B'):
    ADC1->CHSELR &= ~ADC_CHSELR_CHSEL1;
    ADC1->CHSELR = ADC_CHSELR_CHSEL2;
    temperature = ADC1->DR;

    //BOXCAR AVERAGING********************************* 
    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    temperature = bcsum / BCSIZE;
    //BOXCAR AVERAGING********************************* 
    
    spi1_display1("                ");
    spi1_display2("                ");
    float calc_temp = (temperature * 5.0) / 100.0;
    //calc_temp /= 100;
    float tempF = calc_temp * 1.8 + 32; //TEMPERATURE IN FAHRENHEIT
    //tempF = 150-tempF; 
    //int display_temp = 150 - tempF;
    if (tempF < 78){ //red hot
      setn(3, 1);
      setn(8,0);
      setn(9,1);
      setn(10,1);
    }
    else{ // blue cold
        setn(3, 0);
        setn(8,1);
        setn(9,1);
        setn(10,0);
    }
    itoa(tempF, str_3, 10); //should be temperature
    char temp_str_3[50] = "Temp: "; //temp_str_2 == temporary string 2
    strcat(temp_str_3, str_3);
    spi1_display1(temp_str_3);
    break;
  }
}

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;  
    ADC1->CR |= ADC_CR_ADSTART;
    //DO STUFF
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
    }
}

void setn(int32_t pin_num, int32_t val) {
  if (val == 0){
    GPIOA->BRR |= (1 << pin_num);
  }
  else{
    GPIOA->BSRR |= (1 << pin_num);
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

char rows_to_key(int rows) {

  int row = -1;
  if (rows & 0x01) row = 0;
  else if (rows & 0x02) row = 1;  
  else if (rows & 0x04) row = 2;  
  else if (rows & 0x08) row = 3;  
  if (row == -1) return 0;
  int column = col & 0x03;  
  int offset = column * 4 + row;
  return keymap_arr[offset];

}

int read_rows() {

  return GPIOC->IDR & 0x0F;

}

void handle_key(char key) {
    mode = key;  
}

void drive_column(int c) {

  c = c & 0x03;  

  GPIOC->BSRR = (0xF << 4) << 16;  

  GPIOC->BSRR = (1 << (c + 4));  

}


char disp[9] = "Hello...";

void TIM7_IRQHandler(void) {

    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();
    if (rows != 0) {
        char key = rows_to_key(rows);
        handle_key(key);
    }

    col = (col + 1) % 8;
    drive_column(col);
}


/**

 * @brief Setup timer 7 as described in lab handout

 * 

 */

void setup_tim7() {

    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 47;   
    TIM7->ARR = 999;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;
  }

void TIM14_IRQHandler(void) {
  TIM14->SR &= ~TIM_SR_UIF;
  //update_variables();
  write_display();
}

/**
 * @brief Setup timer 14 as described in lab
 *        handout
 * 
 */

void setup_tim14() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
  TIM14->PSC = 47999; 
  TIM14->ARR = 499;    
  TIM14->DIER |= TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM14_IRQn);
  TIM14->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// Main function
//===========================================================================

int main(void) {
  internal_clock();
  // GPIO enable
  enable_ports();

  // setup keyboard
  setup_tim14();
  setup_tim7();
  init_tim2();

  //setup_tim1();
  init_spi1();
  spi1_init_oled();
}


