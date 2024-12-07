/*
 * BasicTimer.cpp
 *
 *  Created on: Dec 1, 2024
 *      Author: emrullah
 */

#include "BasicTimer.h"

BasicTimer::BasicTimer(){

  // Enable TIM6 Clock
  __HAL_RCC_TIM6_CLK_ENABLE();

  // Set default period (100 ms)
  TIM6->PSC = 47999;   // Prescaler: 48 MHz / 48000 = 1 kHz
  TIM6->ARR = 499;     // Auto-reload: 1 kHz / 500 = 2 Hz (100 ms period)

  // Enable Timer Update Interrupt
  TIM6->DIER |= TIM_DIER_UIE;

  // Enable the timer interrupt in NVIC
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  NVIC_SetPriority(TIM6_DAC_IRQn, 2);
}
