/*
 * LedDriver.h
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#ifndef SRC_LEDDRIVER_H_
#define SRC_LEDDRIVER_H_

#include "GPIODevice.h"

class LedDriver: public GPIODevice {
public:
  enum class LedState { ON, OFF };

  LedDriver(GPIO_TypeDef* port, uint16_t pin = 5);   // Constructor

  void toggle();                            // Toggle LED state
  void toggle(uint16_t pin);                            // Toggle LED state
  void set(LedState state);                 // Set LED ON or OFF
  void set(uint16_t pin, LedState state);
};

#endif /* SRC_LEDDRIVER_H_ */
