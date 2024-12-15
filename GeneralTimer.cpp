/*
 * GeneralTimer.cpp
 *
 *  Created on: Dec 14, 2024
 *      Author: emrullah
 */

#include "GeneralTimer.h"

GeneralTimer::GeneralTimer(TIM_TypeDef* timer, Mode mode, uint32_t prescaler, uint32_t period)
  : _timer(timer), _mode(mode) {

  enableClock();

  // Basic timer configuration
  _timer->PSC = prescaler;
  _timer->ARR = period - 1;

  // Configure mode-specific settings
  if (_mode == Mode::UpCounter) {
    _timer->CR1 &= ~TIM_CR1_DIR; // Up-counter
  } else if (_mode == Mode::DownCounter) {
    _timer->CR1 |= TIM_CR1_DIR; // Down-counter
  }

  // Enable Timer Update Interrupt if necessary
  _timer->DIER |= TIM_DIER_UIE;
}

void GeneralTimer::enableClock() {
    if (_timer == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (_timer == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (_timer == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (_timer == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
}

void GeneralTimer::configurePWM(uint32_t channel, uint32_t pulse) {
  // Set mode to PWM
  _timer->CCMR1 |= TIM_CCMR1_OC1M_PWM1; // PWM mode 1 for Channel 1
  _timer->CCR1 = pulse;                 // Set pulse width
  _timer->CCER |= TIM_CCER_CC1E;        // Enable channel 1 output
}

void GeneralTimer::configureInputCapture(uint32_t channel) {
  if (channel == 1) {
    _timer->CCMR1 |= TIM_CCMR1_CC1S_0;  // Input capture on TI1
    _timer->CCER |= TIM_CCER_CC1E;      // Enable capture
  }
}

void GeneralTimer::configureOutputCompare(uint32_t channel, uint32_t compareValue) {
  if (channel == 1) {
    _timer->CCMR1 |= TIM_CCMR1_OC1M;    // Output compare mode
    _timer->CCR1 = compareValue;        // Set compare value
    _timer->CCER |= TIM_CCER_CC1E;      // Enable output
  }
}

void GeneralTimer::setPeriod(uint32_t period) {
  _timer->ARR = period - 1;
}

void GeneralTimer::setPrescaler(uint32_t prescaler) {
  _timer->PSC = prescaler;
}

void GeneralTimer::setCounterValue(uint32_t value) {
  _timer->CNT = value;
}

void GeneralTimer::start() {
  _timer->CR1 |= TIM_CR1_CEN; // Enable the timer
}

void GeneralTimer::stop() {
  _timer->CR1 &= ~TIM_CR1_CEN; // Disable the timer
}

void GeneralTimer::enableInterrupt() {
  _timer->DIER |= TIM_DIER_UIE; // Enable update interrupt
}

void GeneralTimer::disableInterrupt() {
  _timer->DIER &= ~TIM_DIER_UIE; // Disable update interrupt
}

void GeneralTimer::IRQHandler(TIM_TypeDef* timer) {
  if (timer->SR & TIM_SR_UIF) {
    timer->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    // Handle interrupt event here
  }
}

extern "C" void TIM2_IRQHandler() {
  GeneralTimer::IRQHandler(TIM2);
}
