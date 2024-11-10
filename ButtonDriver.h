/*
 * ButtonDriver.h
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#ifndef SRC_BUTTONDRIVER_H_
#define SRC_BUTTONDRIVER_H_

#include "GPIODevice.h"

class ButtonDriver: public GPIODevice {
public:
  enum class ButtonState { ON, OFF };

  ButtonDriver(GPIO_TypeDef* port = GPIOC);                   // Constructor
  ButtonDriver(GPIO_TypeDef* port, uint16_t pin);             // Constructor

  ButtonState getState() const;                               // Get the current state of the button
  ButtonState getState(uint16_t pin) const;                   // Get the current state of the button
  void enableInterrupt(uint16_t pin, uint32_t priority = 1);  // Enable EXTI interrupt for the button

};

#endif /* SRC_BUTTONDRIVER_H_ */
