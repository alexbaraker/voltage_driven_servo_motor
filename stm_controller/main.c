#include "stm32l476xx.h"
#include "SysClock.h"
//#include "LED.h"
#include "UART.h"
#include <stdlib.h>

#include <string.h>
#include <stdio.h>

// Function prototypes ------------------
void setup_board(void);
void read_string_from_USART(uint8_t*);
void print_string(const char*);
// --------------------------------------

// Global declarations ------------------
uint8_t RxComByte = 0;
const uint32_t master_clk_freq = 80000000;
const uint16_t TIM6_prescaler = 4;
// --------------------------------------


// **************************************
// **************** Main ****************
// **************************************
int main(void)
{
  setup_board();
	
	return 0;
}


// **************************************
// ************** End main **************
// **************************************
void print_string(const char* str) { USART_Write(USART2, (uint8_t *)str, strlen(str)); }


void read_string_from_USART(uint8_t* buffer)
{
	int32_t counter = -1;
	for (int j = 0; j <= BufferSize; j++)
		buffer[j] = '\0';

	do
	{
		counter++;
		buffer[counter] = USART_Read(USART2);
		buffer[counter + 1] = '\0';
		
		print_string((const char*)(buffer + counter));
		
	} while ( (buffer[counter] != '\r' ) || ( counter+1 >= BufferSize ) );
	print_string("\r\n");
}

void setup_board(void)
{
	System_Clock_Init(); // Switch System Clock = 80 MHz
	UART2_Init();
	
	// set up port A for input
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(0x2 << 0);   // Set A0 as input
	GPIOA->PUPDR &= ~(0x3 << 0);   // Disable pull-ups and pull-downs on A0
	GPIOA->MODER &= ~(0x3 << 1*2); // Set A1 as output
    GPIOA->PUPDR &= ~(0x3 << 1*2); // Disable pull-ups and pull-downs on A1
	
	/*
	// set up a timer for 1MHz
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;  // enable TIM6 clock
	TIM6->CR1 |= 0x1;					   // Enable counter
	TIM6->PSC = TIM6_prescaler;			   // Timer_freq = master_clk / (prescaler + 1)
  */
}












