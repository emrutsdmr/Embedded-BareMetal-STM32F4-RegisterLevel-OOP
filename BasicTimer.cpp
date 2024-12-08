/*
 * BasicTimer.cpp
 *
 *  Created on: Dec 1, 2024
 *      Author: emrullah
 */

#include "BasicTimer.h"

BasicTimer::BasicTimer(TIM_TypeDef* timer)
  : _timer(timer) {
  // Enable the clock for the selected timer
  if (_timer == TIM6) {
    __HAL_RCC_TIM6_CLK_ENABLE();
  } else if (_timer == TIM7) {
    __HAL_RCC_TIM7_CLK_ENABLE();
  }

  // Set default period (100 ms)
  _timer->PSC = 47999;   // Prescaler: 48 MHz / 48000 = 1 kHz
  _timer->ARR = 499;     // Auto-reload: 1 kHz / 500 = 2 Hz (100 ms period)

  // Enable Timer Update Interrupt
  _timer->DIER |= TIM_DIER_UIE;

  // Enable the timer interrupt in NVIC
  if (_timer == TIM6) {
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
  } else if (_timer == TIM7) {
    NVIC_EnableIRQ(TIM7_IRQn);
    NVIC_SetPriority(TIM7_IRQn, 2);
  }
}

void BasicTimer::setPeriod(uint16_t period_ms) {
  _timer->ARR = period_ms - 1;
}

void BasicTimer::enable() {
  _timer->CR1 |= TIM_CR1_CEN;
}

void BasicTimer::disable() {
  _timer->CR1 &= ~TIM_CR1_CEN;
}

uint16_t BasicTimer::getCounterValue() const {
  return _timer->CNT;
}
