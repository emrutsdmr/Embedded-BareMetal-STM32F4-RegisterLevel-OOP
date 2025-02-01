/*
 * DistanceSensor.cpp
 *
 *  Created on: Jan 26, 2025
 *      Author: emrullah
 */

#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(GPIO_TypeDef* trigPort, uint16_t trigPin, GPIO_TypeDef* echoPort, uint16_t echoPin)
  : trigPin(trigPort, trigPin), echoPin(echoPort, echoPin) {
  // Configure TRIG pin as output
  this->trigPin.configurePin(GPIODevice::Mode::Output, GPIODevice::OutputType::PushPull, GPIODevice::Speed::Medium, GPIODevice::Pull::NoPull);

  // Configure ECHO pin as input
  this->echoPin.configurePin(GPIODevice::Mode::Input, GPIODevice::OutputType::Unchanged, GPIODevice::Speed::Unchanged, GPIODevice::Pull::NoPull);

  //TODO Write AdvancedTimer Class to replace the following initialisation
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM1 clock
  TIM1->PSC = 16 - 1;                 // Prescaler (16 MHz / 16 = 1 MHz)
  TIM1->ARR = 1;                      // Auto-reload value (1 MHz clock)
  TIM1->CNT = 0;
  TIM1->CR1 = 1;                      // Enable TIM1
}

void DistanceSensor::delayMilliseconds(uint32_t ms) {
  SysTick->LOAD = 16000 - 1; // 1 ms delay with 16 MHz clock
  SysTick->VAL = 0;
  SysTick->CTRL = 0x5; // Enable SysTick with processor clock
  for (uint32_t i = 0; i < ms; i++) {
    while (!(SysTick->CTRL & 0x10000)) {
    } // Wait until COUNTFLAG is set
  }
  SysTick->CTRL = 0;
}

void DistanceSensor::delayMicroseconds(uint32_t us) {
  for (uint32_t i = 0; i < us; i++) {
    while (!(TIM1->SR & 1)) {
    } // Wait for UIF flag to be set
    TIM1->SR &= ~1; // Clear UIF flag
  }
}

uint32_t DistanceSensor::readEcho(uint32_t timeout) {
  uint32_t duration = 0;

  // Wait for echo to go high
  while (!echoPin.read()) {
    duration++;
    delayMicroseconds(1);
    if (duration > timeout) {
      return 0; // Timeout
    }
  }

  duration = 0;

  // Wait for echo to go low
  while (echoPin.read()) {
    duration++;
    delayMicroseconds(1);
    if (duration > timeout) {
      return 0; // Timeout
    }
  }

  return duration;
}

float DistanceSensor::measureDistance(uint32_t timeout) {
  // Trigger the sensor
  trigPin.write(true);  // Trigger high
  delayMicroseconds(10);   // 10 µs pulse
  trigPin.write(false); // Trigger low

  // Read echo and calculate distance
  uint32_t duration = readEcho(timeout);
  if (duration == 0) {
    return -1; // Indicate timeout
  }
  return duration / 58.0f; // Distance in cm
}

void DistanceSensor::wait(uint32_t ms) {
  delayMilliseconds(ms);
}

