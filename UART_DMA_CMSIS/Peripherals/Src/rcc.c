/*
 * rcc.c
 *
 *  Created on: Aug 16, 2024
 *      Author: hlqth
 */


#include "rcc.h"

// ms Tick
static __IO uint32_t msTicks = 0;

/*
 * @brief HSE configuration
 */
void rcc_HSE_config(void) {
  /*
   * configuration parameters --> STM32F103 Clock tree
   * HSE = 8 MHz
   * PLL = x9 (72 MHz system clock)
   * USB divider = /1.5
   * AHB divider = /1
   * Cortex divider (i.e. SysTick) = /1
   *
   * APB1 divider = /2 (36 Mhz, 72 MHz)
   * APB2 divider = /1 (72 MHz)
   * ADC divider = /6 (12 MHz)
   */

  // PLL configuration
  // Set PLL multiplier to x9
  //RCC->CFGR &= ~(0xFUL << 18); // Clear bitfields [21:18] 0xF = 1111
  RCC->CFGR &= ~(RCC_CFGR_PLLMULL); // same as above
  RCC->CFGR |= (7UL << 18); // setting PLL multiplier to 9

  // USB divider (/1.5)
  RCC->CFGR &= ~(RCC_CFGR_USBPRE);

  // HSE oscillator (enable external oscillator, wait for it to become stable, use it as source for PLL, wait until PLL kicks in)
  RCC->CR |= (RCC_CR_HSEON);
  // wait for it to stabilize
  while((RCC->CR & RCC_CR_HSERDY) == 0);
  // select HSE as PLL source
  RCC->CFGR |= (RCC_CFGR_PLLSRC);
  // Enable PLL
  RCC->CR |= (RCC_CR_PLLON);
  // wait for it to stabilize
  while((RCC->CR & RCC_CR_PLLRDY) == 0);

  // Flash prefetch enable / wait state (2 WS for 72 MHz)
  FLASH->ACR |= FLASH_ACR_PRFTBE;
  FLASH->ACR &= ~(FLASH_ACR_LATENCY); // clear latency fields
  FLASH->ACR |= (FLASH_ACR_LATENCY_1);

  // select PLL as main system clock source
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= RCC_CFGR_SW_1;
  // wait for it to stabilize
   while((RCC->CFGR & RCC_CFGR_SWS_1) == 0); // waiting until switch status becomes 1


   // AHB prescaler /1
   RCC->CFGR &= ~(RCC_CFGR_HPRE);
   // APB1 prescaler /1
   RCC->CFGR &= ~(RCC_CFGR_PPRE1);
   RCC->CFGR |= RCC_CFGR_PPRE1_2;
   // APB2 prescaler
   RCC->CFGR &= ~(RCC_CFGR_PPRE2);
   // ADC prescaler
   RCC->CFGR &= ~(RCC_CFGR_ADCPRE);
   RCC->CFGR |= (RCC_CFGR_ADCPRE_1);

}


/*
 * @brief SysTick configuration
 * @param[in] reload value
 */
void rcc_SysTick_config(uint32_t arr) {
  // Reset control register
  SysTick->CTRL = 0;
  // Set the reload value
  SysTick->LOAD = arr - 1;
  // Set SysTick interrupt priority (from NVIC)
  NVIC_SetPriority(SysTick_IRQn,0);
  // Reset SysTick value to 0
  SysTick->VAL = 0;
  // Enable SysTick from control register
  SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


/*
 * @brief Increment ms Ticks
 */
void rcc_msIncTicks(void) {
  msTicks++;
}

/*
 * @brief Get current ms Ticks
 */
uint32_t rcc_msGetTicks(void) {
  return msTicks;
}

/*
 * @brief Delay ms Ticks
 * @param[in] delay ticks
 */
void rcc_msDelayTicks(uint32_t ms) {
  uint32_t startTicks = rcc_msGetTicks();
  while((rcc_msGetTicks() - startTicks) < ms);
}


/*
 * @brief SysTick interrupt callback
 */
//void SysTick_Handler(void) {
//  NVIC_ClearPendingIRQ(SysTick_IRQn);
//  rcc_msIncTicks();
//}

