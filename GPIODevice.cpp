/*
 * GPIODevice.cpp
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#include "GPIODevice.h"
//#include <stdexcept>
#include "stm32f4xx_hal.h"

GPIODevice::GPIODevice(GPIO_TypeDef* port)
{
  // TODO: Write isValid function to validate that the port is valid
  //if (port != GPIOA && port != GPIOB && port != GPIOC && port != GPIOD && port != GPIOE) {
  //  throw std::invalid_argument("Invalid GPIO port!");
  //}
  _port = port;  // Assign if valid
  enableClock();
}

GPIODevice::GPIODevice(GPIO_TypeDef* port, uint16_t pin)
{
  //if (port != GPIOA && port != GPIOB && port != GPIOC && port != GPIOD && port != GPIOE) {
  //  throw std::invalid_argument("Invalid GPIO port!");
  //}
  _port = port;  // Assign if valid
  enableClock();
  currentPin = pin;
}

void GPIODevice::enableClock()
{
  if (_port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (_port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (_port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (_port == GPIOD)
  {
      __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (_port == GPIOE)
  {
      __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  //TODO: add an acceptable exception
  //else {
  //    // Handle invalid port or throw an exception
  //  throw std::invalid_argument("Invalid GPIO port!");
  //}

}

void GPIODevice::configurePin(uint16_t pin, Mode mode, OutputType outputType, Speed speed, Pull pull)
{
  currentPin = pin;
  if (mode != Mode::Unchanged) setMode(mode);
  if (outputType != OutputType::Unchanged) setOutputType(outputType);
  if (speed != Speed::Unchanged) setSpeed(speed);
  if (pull != Pull::Unchanged) setPull(pull);

  // Configure alternate function if the mode is AlternateFunction
  if (mode == Mode::AlternateFunction) {
    if (pin < 8) {
      // Set alternate function in AFRL (low register for pins 0-7)
      _port->AFR[0] &= ~(0xF << (pin * 4));             // Clear bits
      _port->AFR[0] |= (alternateFunction << (pin * 4)); // Set alternate function
    } else {
      // Set alternate function in AFRH (high register for pins 8-15)
      _port->AFR[1] &= ~(0xF << ((pin - 8) * 4));             // Clear bits
      _port->AFR[1] |= (alternateFunction << ((pin - 8) * 4)); // Set alternate function
    }
  }
}

void GPIODevice::setMode(Mode mode)
{
  if (mode == Mode::Output) {
    _port->MODER &= ~(3UL << (currentPin * 2));    // Clear both bits for the pin
    _port->MODER |= (1UL << (currentPin * 2));     // Set to output mode
  }
  else if (mode == Mode::Input) {
    _port->MODER &= ~(3UL << (currentPin * 2));    // Clear both bits to set input mode
  }
  else if (mode == Mode::AlternateFunction) {
    _port->MODER &= ~(3UL << (currentPin * 2));    // Clear both bits for the pin
    _port->MODER |= (2UL << (currentPin * 2));     // Set to alternate function mode
  }
}

void GPIODevice::setOutputType(OutputType outputType)
{
  if (outputType == OutputType::PushPull)
  {
    _port->OTYPER &= ~(1UL << currentPin); // Push-pull
  }
  else if (outputType == OutputType::OpenDrain)
  {
    _port->OTYPER |= (1UL << currentPin);  // Open-drain
  }
}

void GPIODevice::setSpeed(Speed speed)
{
  if (speed == Speed::Low)
  {
    _port->OSPEEDR &= ~(3UL << (currentPin * 2)); // Low speed
  }
  else if (speed == Speed::Medium)
  {
     _port->OSPEEDR &= ~(1UL << (currentPin * 2 + 1));
     _port->OSPEEDR |= (1UL << (currentPin * 2)); // Medium speed
  }
  else if (speed == Speed::High)
  {
     _port->OSPEEDR |= (3UL << (currentPin * 2)); // High speed
  }
}

void GPIODevice::setPull(Pull pull)
{
  if (pull == Pull::NoPull)
  {
    _port->PUPDR &= ~(3UL << (currentPin * 2)); // No pull-up, no pull-down
  }
  else if (pull == Pull::PullUp)
  {
    _port->PUPDR &= ~(1UL << (currentPin * 2 + 1));
    _port->PUPDR |= (1UL << (currentPin * 2)); // Pull-up
  }
  else if (pull == Pull::PullDown)
  {
    _port->PUPDR |= (3UL << (currentPin * 2)); // Pull-down
  }
}
//GPIODevice::GPIODevice() {
//	// TODO Auto-generated constructor stub
//
//}
//
//GPIODevice::~GPIODevice() {
//	// TODO Auto-generated destructor stub
//}
//
//GPIODevice::GPIODevice(const GPIODevice &other) {
//	// TODO Auto-generated constructor stub
//
//}
//
//GPIODevice::GPIODevice(GPIODevice &&other) {
//	// TODO Auto-generated constructor stub
//
//}
//
//GPIODevice& GPIODevice::operator=(const GPIODevice &other) {
//	// TODO Auto-generated method stub
//
//}
//
//GPIODevice& GPIODevice::operator=(GPIODevice &&other) {
//	// TODO Auto-generated method stub
//
//}

