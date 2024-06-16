
/********************************************
*			STM32F439 Main (C Startup File)  			*
*			Developed for the STM32								*
*			Author: 															*
*			Source File														*
********************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"
#include "stm32f4xx.h"
#include "gpioControl.h"       

// Function prototypes
void delay(volatile uint32_t);

//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{
	
    // Enable GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Initialize GPIO pins for LD2 LED on PB7
    GPIO_Config ledConfig;
    ledConfig.port = GPIOB;
    ledConfig.pin = Pin7;
    ledConfig.mode = GPIO_Output;
    ledConfig.outputType = GPIO_Output_PushPull;
    ledConfig.speed = GPIO_50MHz;
    ledConfig.pullUpDown = GPIO_No_Pull;

    gpio_configureGPIO(&ledConfig);    // Configure GPIO pin for LD2 LED
	
    while (1)
    {
        // Toggle LD2 LED on/off
        GPIOB->BSRR = (1 << Pin7);      // Set pin high (turn on LD2 LED)
        delay(500);                     // Delay 500ms

        GPIOB->BSRR = (1 << (Pin7 + 16));   // Reset pin low (turn off LD2 LED)
        delay(500);                     // Delay 500ms
    }
}

// delay function
void delay(volatile uint32_t ms)
{
    ms *= (SystemCoreClock / 1000 / 3);  // Approximate loops for 1ms delay in Cortex-M3
    while (ms--)
        __NOP();
}
