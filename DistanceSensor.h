/*
 * DistanceSensor.h
 *
 *  Created on: Jan 26, 2025
 *      Author: emrullah
 */

#ifndef SRC_DISTANCESENSOR_H_
#define SRC_DISTANCESENSOR_H_

#include "stm32f4xx.h"
#include "GPIODevice.h"

//HC-SR04 Ultrasonic Sensor
class DistanceSensor {
private:
  GPIODevice trigPin;
  GPIODevice echoPin;

  void delayMilliseconds(uint32_t ms);
  void delayMicroseconds(uint32_t us);
  uint32_t readEcho(uint32_t timeout);

public:
  DistanceSensor(GPIO_TypeDef* trigPort, uint16_t trigPin, GPIO_TypeDef* echoPort, uint16_t echoPin);
  float measureDistance(uint32_t timeout = 400000);
  void wait(uint32_t ms);
};

#endif /* SRC_DISTANCESENSOR_H_ */
