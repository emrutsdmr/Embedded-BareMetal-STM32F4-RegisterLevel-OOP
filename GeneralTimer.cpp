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

  // Enable Timer Update Interrupt if necessary
  _timer->DIER |= TIM_DIER_UIE;
}

void GeneralTimer::enableClock() {
  if (_timer == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  else if (_timer == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  else if (_timer == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
  else if (_timer == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
}

void GeneralTimer::enableChannelOutput(uint32_t channel) {
  // Enable output for the given channel (CCER register)
  _timer->CCER |= (1 << ((channel - 1) * 4));  // Enable output for the selected channel
}

void GeneralTimer::configureOutputCompare(uint32_t channel, uint32_t compareValue) {
  uint32_t* ccmrRegister = nullptr;
  uint32_t ccmrMask = 0;
  uint32_t* ccrRegister = nullptr;
  uint32_t ocMode = 0b001 << 4;  // Toggle mode for OCxM

  switch (channel) {
    case 1:
      ccmrRegister = &(_timer->CCMR1);      // Channel 1 is in CCMR1
      ccmrMask = TIM_CCMR1_OC1M;            // OC1M is in bits 4-6 of CCMR1
      ccrRegister = &(_timer->CCR1);        // Channel 1 uses CCR1
      _timer->CCMR1 &= ~TIM_CCMR1_CC1S;     // Clear CC1S to select output compare mode
      //ocMode = (0b001 << 4);                // OC1M bits are 4-6, so shift by 4
      break;
    case 2:
      ccmrRegister = &(_timer->CCMR1);      // Channel 2 is also in CCMR1
      ccmrMask = TIM_CCMR1_OC2M;            // OC2M is in bits 12-14 of CCMR1
      ccrRegister = &(_timer->CCR2);        // Channel 2 uses CCR2
      _timer->CCMR1 &= ~TIM_CCMR1_CC2S;     // Clear CC2S to select output compare mode
      ocMode = (0b001 << 12);               // OC2M bits are 12-14, so shift by 12
      break;
    case 3:
      ccmrRegister = &(_timer->CCMR2);      // Channel 3 is in CCMR2
      ccmrMask = TIM_CCMR2_OC3M;            // OC3M is in bits 4-6 of CCMR2
      ccrRegister = &(_timer->CCR3);        // Channel 3 uses CCR3
      _timer->CCMR2 &= ~TIM_CCMR2_CC3S;     // Clear CC3S to select output compare mode
      //ocMode = (0b001 << 4);                // OC3M bits are 4-6, so shift by 4
      break;
    case 4:
      ccmrRegister = &(_timer->CCMR2);      // Channel 4 is in CCMR2
      ccmrMask = TIM_CCMR2_OC4M;            // OC4M is in bits 12-14 of CCMR2
      ccrRegister = &(_timer->CCR4);        // Channel 4 uses CCR4
      _timer->CCMR2 &= ~TIM_CCMR2_CC4S;     // Clear CC4S to select output compare mode
      ocMode = (0b001 << 12);               // OC4M bits are 12-14, so shift by 12
      break;
    default:
      // Invalid channel, return or handle error
      return;
  }

  // Set the compare value for the specified channel
  *ccrRegister = compareValue;

  // Clear existing mode and set the new output compare mode (OCxM)
  *ccmrRegister &= ~ccmrMask;  // Clear OCxM bits for the selected channel
  *ccmrRegister |= ocMode;     // Set OCxM to toggle mode

  // Enable the channel output (CCER register)
  enableChannelOutput(channel);
}

void GeneralTimer::configurePWM(uint32_t channel, uint32_t pulse) {
  // Set PWM mode (Mode 1) for the given channel
  if (channel == 1) {
	  _timer->CCMR1 &= ~TIM_CCMR1_CC1S;
    _timer->CCMR1 &= ~TIM_CCMR1_OC1M;  // Clear previous mode
    _timer->CCMR1 |= TIM_CCMR1_OC1M_PWM1;  // Set PWM Mode 1 for Channel 1
    _timer->CCR1 = pulse;  // Set the pulse width for Channel 1
  } else if (channel == 2) {
	  _timer->CCMR1 &= ~TIM_CCMR1_CC2S;
    _timer->CCMR1 &= ~TIM_CCMR1_OC2M;  // Clear previous mode
    _timer->CCMR1 |= TIM_CCMR1_OC2M_PWM1;  // Set PWM Mode 1 for Channel 2
    _timer->CCR2 = pulse;  // Set the pulse width for Channel 2
  } else if (channel == 3) {
	  _timer->CCMR2 &= ~TIM_CCMR1_CC1S;
    _timer->CCMR2 &= ~TIM_CCMR2_OC3M;  // Clear previous mode
    _timer->CCMR2 |= TIM_CCMR2_OC3M_PWM1;  // Set PWM Mode 1 for Channel 3
    _timer->CCR3 = pulse;  // Set the pulse width for Channel 3
  } else if (channel == 4) {
	  _timer->CCMR2 &= ~TIM_CCMR1_CC2S;
    _timer->CCMR2 &= ~TIM_CCMR2_OC4M;  // Clear previous mode
    _timer->CCMR2 |= TIM_CCMR2_OC4M_PWM1;  // Set PWM Mode 1 for Channel 4
    _timer->CCR4 = pulse;  // Set the pulse width for Channel 4
  }

  // Enable output for the channel
  enableChannelOutput(channel);
}

void GeneralTimer::configureInputCapture(uint32_t channel) {
  if (channel == 1) {
    _timer->CCMR1 |= TIM_CCMR1_CC1S_0;  // Input capture on TI1
    _timer->CCER |= TIM_CCER_CC1E;      // Enable capture
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
