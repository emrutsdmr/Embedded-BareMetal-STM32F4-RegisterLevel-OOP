/*
 * BasicTimer.cpp
 *
 *  Created on: Dec 1, 2024
 *      Author: emrullah
 */

#include "BasicTimer.h"
#include "LedDriver.h"

// Global instance of LedDriver for demonstration
LedDriver led(GPIOA, 5);

BasicTimer::BasicTimer(TIM_TypeDef* timer, uint32_t prescaler, uint32_t period_ms)
  : _timer(timer) {

  if (_timer != TIM6 && _timer != TIM7) {
    // Invalid timer
    return;
  }

  // Enable the clock for the selected timer
  if (_timer == TIM6) {
    __HAL_RCC_TIM6_CLK_ENABLE();
  } else if (_timer == TIM7) {
    __HAL_RCC_TIM7_CLK_ENABLE();
  }

  // Configure the timer with default settings
  configureTimer(prescaler, period_ms);

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

void BasicTimer::configureTimer(uint32_t prescaler, uint32_t period_ms) {
  _timer->PSC = prescaler - 1;
  _timer->ARR = period_ms - 1;
}

void BasicTimer::setPeriod(uint16_t period_ms) {
  _timer->ARR = period_ms - 1;
}

void BasicTimer::setPrescaler(uint32_t prescaler) {
  _timer->PSC = prescaler - 1;
}

void BasicTimer::enable() {
  _timer->CR1 |= TIM_CR1_CEN;
}

void BasicTimer::disable() {
  _timer->CR1 &= ~TIM_CR1_CEN;
}

bool BasicTimer::isEnabled() const {
  return _timer->CR1 & TIM_CR1_CEN;
}

uint16_t BasicTimer::getCounterValue() const {
  return _timer->CNT;
}

void BasicTimer::IRQHandler(TIM_TypeDef* timer) {
  if (timer->SR & TIM_SR_UIF) {
    timer->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag

    // Example: Toggle an LED when the interrupt occurs
    led.toggle();
  }
}

// Interrupt handler for TIM6
extern "C" void TIM6_DAC_IRQHandler() {
  BasicTimer::IRQHandler(TIM6);
}

// Interrupt handler for TIM7
extern "C" void TIM7_IRQHandler() {
  BasicTimer::IRQHandler(TIM7);
}

