/*
 * CanDriver.cpp
 *
 *  Created on: Dec 28, 2024
 *      Author: emrullah
 */

#include "CanDriver.h"

// Static map initialization
const std::map<CAN_TypeDef*, std::vector<CanDriver::CanPinConfig>> CanDriver::canPinMap = {
  {CAN1, {
    {GPIOA, GPIO_PIN_11, GPIO_AF9_CAN1}, // RX
    {GPIOA, GPIO_PIN_12, GPIO_AF9_CAN1}  // TX
  }},
  {CAN2, {
    {GPIOB, GPIO_PIN_12, GPIO_AF9_CAN2}, // RX
    {GPIOB, GPIO_PIN_13, GPIO_AF9_CAN2}  // TX
  }}
};

CanDriver::CanDriver(CAN_TypeDef* canInstance, Mode mode, uint32_t baudRate)
  : _canInstance(canInstance), _mode(mode), _baudRate(baudRate) {

  enableClock();
}

void CanDriver::enableClock() {
  if (_canInstance == CAN1) {
    __HAL_RCC_CAN1_CLK_ENABLE();
  } else if (_canInstance == CAN2) {
    __HAL_RCC_CAN2_CLK_ENABLE();
  } else {
    return; // Unsupported CAN instance
  }
}

void CanDriver::configurePins() {
  if (canPinMap.find(_canInstance) == canPinMap.end()) {
    return; // No pins available for this CAN instance
  }

  for (const auto& pinConfig : canPinMap.at(_canInstance)) {
    GPIODevice gpioDevice(pinConfig.port, pinConfig.pin);
    gpioDevice.configurePin(
        GPIODevice::Mode::AlternateFunction,
        GPIODevice::OutputType::PushPull,
        GPIODevice::Speed::High,
        GPIODevice::Pull::NoPull,
        pinConfig.alternateFunction
    );
  }
}

void CanDriver::configureCAN(uint32_t prescaler, uint32_t mode, uint32_t sjw, uint32_t bs1, uint32_t bs2) {
  _canInstance->MCR |= CAN_MCR_INRQ; // Request initialization
  while ((_canInstance->MSR & CAN_MSR_INAK) == 0);

  _canInstance->BTR = (sjw << 24) | (bs1 << 16) | (bs2 << 20) | (mode << 30) | (prescaler - 1);

  _canInstance->MCR &= ~CAN_MCR_INRQ; // Exit initialization mode
  while ((_canInstance->MSR & CAN_MSR_INAK) != 0);
}
