/*
 * GeneralTimer.h
 *
 *  Created on: Dec 14, 2024
 *      Author: emrullah
 */

#ifndef SRC_GENERALTIMER_H_
#define SRC_GENERALTIMER_H_

#include "stm32f4xx_hal.h"

class GeneralTimer {
public:
  // Timer Modes
//  enum class Mode {
//    OutputCompare,
//    InputCapture,
//    UpCounter,
//    DownCounter,
//    PWM,
//    Encoder
//  };

  // Output Compare Modes
  enum class OCMode {
    Frozen = 0b000,
    SetActive = 0b001,
    SetInactive = 0b010,
    Toggle = 0b011,
    ForceInactive = 0b100,
    ForceActive = 0b101,
    PWMMode1 = 0b110,
    PWMMode2 = 0b111
  };

  GeneralTimer(TIM_TypeDef* timer, uint32_t prescaler = DEFAULT_PSC, uint32_t period = DEFAULT_PERIOD);

  // Configure timer channels for specific functionality
  void configureInputCapture(uint32_t channel);
  void configureOutputCompare(uint32_t channel, uint32_t compareValue, OCMode ocMode);

  // Setters
  void setPeriod(uint32_t period);
  void setPrescaler(uint32_t prescaler);
  void setCounterValue(uint32_t value);

  // Control Functions
  void start();
  void stop();

  // Interrupts
  void enableInterrupt();
  void disableInterrupt();
  static void IRQHandler(TIM_TypeDef* timer);

private:
  TIM_TypeDef* _timer;
  Mode _mode;

  static constexpr uint32_t DEFAULT_PSC    = 48000; // Default prescaler (48 MHz -> 1 kHz)
  static constexpr uint32_t DEFAULT_PERIOD = 1000;  // Default period (1 second)

  void enableClock();              // Enable the timer's clock
  void configureTimer();           // Internal helper to configure the timer
};

#endif /* SRC_GENERALTIMER_H_ */
