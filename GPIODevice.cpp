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
  _port = port;
  enableClock();
}

GPIODevice::GPIODevice(GPIO_TypeDef* port, uint16_t pin)
{
  _port = port; 
  enableClock();
  _currentPin = pin;
}

void GPIODevice::enableClock()
{
  // TODO: Write isValid function to validate that the port is valid
  //if (_port == nullptr || (_port != GPIOA && _port != GPIOB && _port != GPIOC && _port != GPIOD && _port != GPIOE)) {
  //    throw std::invalid_argument("Invalid GPIO port!");
  //}
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

void GPIODevice::configurePin(Mode mode, OutputType outputType, Speed speed, Pull pull, uint8_t alternateFunction)
{
  if (mode != Mode::Unchanged) setMode(mode);
  if (outputType != OutputType::Unchanged) setOutputType(outputType);
  if (speed != Speed::Unchanged) setSpeed(speed);
  if (pull != Pull::Unchanged) setPull(pull);

  // Configure alternate function if the mode is AlternateFunction
  if (mode == Mode::AlternateFunction && alternateFunction < 16) {
    if (_currentPin < 8) {
      // Set alternate function in AFRL (low register for _currentPin 0-7)
      _port->AFR[0] &= ~(0xF << (_currentPin * 4));             // Clear bits
      _port->AFR[0] |= (alternateFunction << (_currentPin * 4)); // Set alternate function
    } else {
      // Set alternate function in AFRH (high register for _currentPin 8-15)
      _port->AFR[1] &= ~(0xF << ((_currentPin - 8) * 4));             // Clear bits
      _port->AFR[1] |= (alternateFunction << ((_currentPin - 8) * 4)); // Set alternate function
    }
  }
}

void GPIODevice::setMode(Mode mode)
{
  if (mode == Mode::Output) {
    _port->MODER &= ~(3UL << (_currentPin * 2));    // Clear both bits for the pin
    _port->MODER |= (1UL << (_currentPin * 2));     // Set to output mode
  }
  else if (mode == Mode::Input) {
    _port->MODER &= ~(3UL << (_currentPin * 2));    // Clear both bits to set input mode
  }
  else if (mode == Mode::AlternateFunction) {
    _port->MODER &= ~(3UL << (_currentPin * 2));    // Clear both bits for the pin
    _port->MODER |= (2UL << (_currentPin * 2));     // Set to alternate function mode
  }
}

void GPIODevice::setOutputType(OutputType outputType)
{
  if (outputType == OutputType::PushPull)
  {
    _port->OTYPER &= ~(1UL << _currentPin); // Push-pull
  }
  else if (outputType == OutputType::OpenDrain)
  {
    _port->OTYPER |= (1UL << _currentPin);  // Open-drain
  }
}

void GPIODevice::setSpeed(Speed speed)
{
  if (speed == Speed::Low)
  {
    _port->OSPEEDR &= ~(3UL << (_currentPin * 2)); // Low speed
  }
  else if (speed == Speed::Medium)
  {
     _port->OSPEEDR &= ~(1UL << (_currentPin * 2 + 1));
     _port->OSPEEDR |= (1UL << (_currentPin * 2)); // Medium speed
  }
  else if (speed == Speed::High)
  {
     _port->OSPEEDR |= (3UL << (_currentPin * 2)); // High speed
  }
}

void GPIODevice::setPull(Pull pull)
{
  if (pull == Pull::NoPull)
  {
    _port->PUPDR &= ~(3UL << (_currentPin * 2)); // No pull-up, no pull-down
  }
  else if (pull == Pull::PullUp)
  {
    _port->PUPDR &= ~(1UL << (_currentPin * 2 + 1));
    _port->PUPDR |= (1UL << (_currentPin * 2)); // Pull-up
  }
  else if (pull == Pull::PullDown)
  {
    _port->PUPDR |= (3UL << (_currentPin * 2)); // Pull-down
  }
}

void GPIODevice::write(bool value)
{
  if (value) {
    _port->ODR |= (1UL << _currentPin); // Set the pin high
  } else {
    _port->ODR &= ~(1UL << _currentPin); // Set the pin low
  }
}

bool GPIODevice::read() const
{
  return (_port->IDR & (1UL << _currentPin)) != 0; // Return true if the pin is high, false otherwise
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

