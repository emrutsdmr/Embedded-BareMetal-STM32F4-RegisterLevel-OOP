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
  BasicTimer(TIM_TypeDef* timer);               // Constructor to initialize TIM6 or TIM7
//  void setPeriod(uint16_t period_ms);         // Set the timer period
//  void enable();                              // Enable the timer
//  void disable();                             // Disable the timer
//  uint16_t getCounterValue() const;           // Get the current counter value
//
//  static void IRQHandler(TIM_TypeDef* timer); // Interrupt handler for the timer

private:
  TIM_TypeDef* _timer;                      // Pointer to the selected timer (TIM6 or TIM7)
};

#endif /* SRC_BASICTIMER_H_ */
