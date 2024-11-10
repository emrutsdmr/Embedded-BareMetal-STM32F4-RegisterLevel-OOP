/*
 * LedDriver.cpp
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#include "LedDriver.h"

LedDriver::LedDriver(GPIO_TypeDef* port, uint16_t pin)
    : GPIODevice(port)
{
  configurePin(Mode::Output, OutputType::PushPull, Speed::High, Pull::NoPull); // Configure pin for LED output
}

void LedDriver::toggle()
{
  _port->ODR ^= (1 << _currentPin);
}

void LedDriver::toggle(uint16_t pin)
{
  _port->ODR ^= (1 << pin);
}

void LedDriver::set(LedState state)
{
  if (state == LedState::ON)
  {
    _port->BSRR = (1 << _currentPin); // Set pin
  }
  else
  {
    _port->BSRR = (1 << (_currentPin + 16));  // Reset pin
  }
}

void LedDriver::set(uint16_t pin, LedState state)
{
  if (state == LedState::ON)
  {
    _port->BSRR = (1 << pin); // Set pin
  }
  else
  {
    _port->BSRR = (1 << (pin + 16));  // Reset pin
  }
}
