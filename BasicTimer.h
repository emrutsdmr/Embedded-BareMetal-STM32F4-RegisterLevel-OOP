/*
 * BasicTimer.h
 *
 *  Created on: Dec 1, 2024
 *      Author: emrullah
 */

#ifndef SRC_BASICTIMER_H_
#define SRC_BASICTIMER_H_

#include "stm32f4xx_hal.h"

class BasicTimer {
public:
  // Constructor to initialize TIM6 or TIM7
  BasicTimer(TIM_TypeDef* timer, uint32_t prescaler = DEFAULT_PSC, uint32_t period_ms = DEFAULT_PERIOD_MS);

  // Set the timer period in milliseconds
  void setPeriod(uint16_t period_ms);

  // Set the prescaler value
  void setPrescaler(uint32_t prescaler);

  // Enable the timer
  void enable();

  // Disable the timer
  void disable();

  // Check if the timer is enabled
  bool isEnabled() const;

  // Get the current counter value
  uint16_t getCounterValue() const;

  // Static interrupt handler for the timer
  static void IRQHandler(TIM_TypeDef* timer);

private:
  TIM_TypeDef* _timer;// Pointer to the selected timer (TIM6 or TIM7)

  static constexpr uint32_t DEFAULT_PSC = 48000;     // Default prescaler (48 MHz -> 1 kHz)
  static constexpr uint32_t DEFAULT_PERIOD_MS = 100; // Default period (100 ms)

  void configureTimer(uint32_t prescaler, uint32_t period_ms); // Internal configuration helper
};

#endif /* SRC_BASICTIMER_H_ */
