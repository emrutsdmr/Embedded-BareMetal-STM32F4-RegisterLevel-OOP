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
  configurePins();
}

void CanDriver::enableClock() {
  if      (_canInstance == CAN1) __HAL_RCC_CAN1_CLK_ENABLE();
  else if (_canInstance == CAN2) __HAL_RCC_CAN2_CLK_ENABLE();
  else return; // Unsupported CAN instance
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

void CanDriver::configureCAN(uint32_t prescaler, Mode mode, uint32_t sjw, uint32_t bs1, uint32_t bs2) {
  // Request initialization mode
  _canInstance->MCR |= CAN_MCR_INRQ;
  while ((_canInstance->MSR & CAN_MSR_INAK) == 0);

  // Reset BTR configuration
  _canInstance->BTR = 0;

  // Enable CAN1 peripheral
  _canInstance->BTR |= ((sjw - 1) << CAN_BTR_SJW_Pos) |
                       ((bs1 - 1) << CAN_BTR_TS1_Pos) |
                       ((bs2 - 1) << CAN_BTR_TS2_Pos) |
                       ((prescaler - 1) << CAN_BTR_BRP_Pos) |
                       (static_cast<uint32_t>(mode) << CAN_BTR_LBKM_Pos);

  // Enable CAN1 peripheral
//  _canInstance->MCR &= ~CAN_MCR_SLEEP;
//  while ((CAN1->MSR & CAN_MSR_SLAK)!=0U);

  // Exit initialization mode
  _canInstance->MCR &= ~CAN_MCR_INRQ;
  while ((_canInstance->MSR & CAN_MSR_INAK) != 0);
}
