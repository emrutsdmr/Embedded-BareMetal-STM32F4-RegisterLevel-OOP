/*
 * CanDriver.h
 *
 *  Created on: Dec 28, 2024
 *      Author: emrullah
 */

#ifndef SRC_CANDRIVER_H_
#define SRC_CANDRIVER_H_

#include "stm32f4xx_hal.h"
#include "GPIODevice.h"
#include <map>
#include <vector>
#include <functional>

class CanDriver {
public:
  // CAN Modes
  enum class Mode {
    Normal = 0b00,
    Loopback = 0b01,
    Silent = 0b10,
    SilentLoopback = 0b11
  };

  struct CanPinConfig {
    GPIO_TypeDef* port;        // GPIO Port name
    uint16_t pin;              // GPIO Pin number
    uint8_t alternateFunction; // Alternate Function
  };

  struct TxFrame {
    uint32_t identifier;
    uint8_t length;
    uint8_t data[8];
  };

  struct RxFrame {
    uint32_t identifier;
    uint8_t length;
    uint8_t data[8];
  };

  using RxCallback = std::function<void(const RxFrame&)>;

  CanDriver(CAN_TypeDef* canInstance);

  void configurePins();
  void configureCAN(uint32_t prescaler, Mode mode, uint32_t sjw, uint32_t bs1, uint32_t bs2);
  void configureFilter(uint8_t filterBank, uint8_t startBank, uint32_t id, uint32_t mask,
                       uint8_t fifo = 0, bool isIdentifierList = false, bool is32Bit = true);
  void start();
  void receiveMessage(RxFrame *frame);
  void sendMessage(TxFrame *frame );
  void enableInterrupt(RxCallback cb, uint8_t fifo = 0, uint32_t priority = 5);
  void handleInterrupt();  // Call from ISR

private:
  CAN_TypeDef* _canInstance;
  Mode _mode;
  RxCallback _rxCallback;
  uint8_t _rxFifo = 0;

  void enableClock(); // Enable the CAN peripheral's clock

  uint32_t encode32BitFilter(uint32_t value) const;
  std::pair<uint32_t, uint32_t> encode16BitFilter(uint32_t id, uint32_t mask) const;

  // Static map for CAN GPIO pin mapping
  static const std::map<CAN_TypeDef*, std::vector<CanPinConfig>> canPinMap;
};

#endif /* SRC_CANDRIVER_H_ */
