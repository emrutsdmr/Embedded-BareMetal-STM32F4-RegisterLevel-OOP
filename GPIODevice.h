/*
 * GPIODevice.h
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#ifndef SRC_GPIODEVICE_H_
#define SRC_GPIODEVICE_H_

#include "stm32f4xx_hal.h"

class GPIODevice {
public:
  static constexpr uint8_t GPIOA_EXTI = 0x00;
  static constexpr uint8_t GPIOB_EXTI = 0x01;
  static constexpr uint8_t GPIOC_EXTI = 0x02;
  static constexpr uint8_t GPIOD_EXTI = 0x03;
  static constexpr uint8_t GPIOE_EXTI = 0x04;
  // Other members...

  enum class Mode { Output, Input, AlternateFunction, Unchanged };
  enum class OutputType { PushPull, OpenDrain, Unchanged };
  enum class Speed { Low, Medium, High, Unchanged };
  enum class Pull { NoPull, PullUp, PullDown, Unchanged };

  GPIODevice(GPIO_TypeDef* port);  
  GPIODevice(GPIO_TypeDef* port, uint16_t pin);
  virtual ~GPIODevice() = default;          // Virtual destructor for proper cleanup
  void configurePin(uint16_t pin, Mode mode, OutputType outputType, Speed speed, Pull pull, uint8_t alternateFunction = 0); // Generalized configuration


protected:
  GPIO_TypeDef* _port;
  uint16_t currentPin;

  void enableClock();                       // Method to enable clock for the GPIO port

private:
  void setMode(Mode mode);                  // Configure GPIO mode (input/output)
  void setOutputType(OutputType outputType); // Configure output type (push-pull, open-drain)
  void setSpeed(Speed speed);               // Configure GPIO speed (low, medium, high)
  void setPull(Pull pull);                  // Configure pull-up/pull-down resistors
//public:
//	GPIODevice();
//	virtual ~GPIODevice();
//	GPIODevice(const GPIODevice &other);
//	GPIODevice(GPIODevice &&other);
//	GPIODevice& operator=(const GPIODevice &other);
//	GPIODevice& operator=(GPIODevice &&other);
};

#endif /* SRC_GPIODEVICE_H_ */
