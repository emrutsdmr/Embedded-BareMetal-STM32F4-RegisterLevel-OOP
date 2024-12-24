/*
 * GeneralTimer.cpp
 *
 *  Created on: Dec 14, 2024
 *      Author: emrullah
 */

#include "GeneralTimer.h"

// Static map initialization
const std::map<TIM_TypeDef*, std::map<std::string, std::vector<GeneralTimer::TimerChannelPin>>> GeneralTimer::timerChannelMap = {
  {TIM2, {
    {"CH1", {{ GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2 }, { GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2 }}},
    {"CH2", {{ GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2 }, { GPIOB, GPIO_PIN_3,  GPIO_AF1_TIM2 }}},
    {"CH3", {{ GPIOA, GPIO_PIN_2, GPIO_AF1_TIM2 }, { GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2 }}},
    {"CH4", {{ GPIOA, GPIO_PIN_3, GPIO_AF1_TIM2 }, { GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2 }}}
  }},
  {TIM3, {
    {"CH1", {{ GPIOA, GPIO_PIN_6, GPIO_AF2_TIM3 }, { GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3 }, { GPIOC, GPIO_PIN_6, GPIO_AF2_TIM3}}},
    {"CH2", {{ GPIOA, GPIO_PIN_7, GPIO_AF2_TIM3 }, { GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3 }, { GPIOC, GPIO_PIN_7, GPIO_AF2_TIM3}}},
    {"CH3", {{ GPIOB, GPIO_PIN_0, GPIO_AF2_TIM3 }, { GPIOC, GPIO_PIN_8, GPIO_AF2_TIM3 }}},
    {"CH4", {{ GPIOB, GPIO_PIN_1, GPIO_AF2_TIM3 }, { GPIOC, GPIO_PIN_9, GPIO_AF2_TIM3 }}}
  }},
  {TIM4, {
    {"CH1", {{ GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4 }, { GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4 }}},
    {"CH2", {{ GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4 }, { GPIOD, GPIO_PIN_13, GPIO_AF2_TIM4 }}},
    {"CH3", {{ GPIOB, GPIO_PIN_8, GPIO_AF2_TIM4 }, { GPIOD, GPIO_PIN_14, GPIO_AF2_TIM4 }}},
    {"CH4", {{ GPIOB, GPIO_PIN_9, GPIO_AF2_TIM4 }, { GPIOD, GPIO_PIN_15, GPIO_AF2_TIM4 }}}
  }},
  {TIM5, {
    {"CH1", {{ GPIOA, GPIO_PIN_0,  GPIO_AF2_TIM5 }}},
    {"CH2", {{ GPIOA, GPIO_PIN_1,  GPIO_AF2_TIM5 }}},
    {"CH3", {{ GPIOA, GPIO_PIN_2,  GPIO_AF2_TIM5 }}},
    {"CH4", {{ GPIOA, GPIO_PIN_3,  GPIO_AF2_TIM5 }}}
  }}
};

GeneralTimer::GeneralTimer(TIM_TypeDef* timer, uint32_t prescaler, uint32_t period)
  : _timer(timer) {

  enableClock();

  // Basic timer configuration
  _timer->PSC = prescaler - 1;
  _timer->ARR = period - 1;

  // Enable Timer Update Interrupt if necessary
  //_timer->DIER |= TIM_DIER_UIE;
}

void GeneralTimer::enableClock() {
  if (_timer == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  else if (_timer == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  else if (_timer == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
  else if (_timer == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
  else return; // Unsupported timer
}

void GeneralTimer::configureChannelPins() {

  // Ensure the timer exists in the map
  if (timerChannelMap.find(_timer) == timerChannelMap.end()) {
    return;
  }

  // Iterate over each channel for the timer
  for (const auto& channel : timerChannelMap.at(_timer)) {
    //const std::string& channelName = channel.first;
    const auto& pinConfigs = channel.second;

    // Use only the first pin configuration for the channel
    if (!pinConfigs.empty()) {
      const auto& pinConfig = pinConfigs.front(); // Select the first pin configuration
      GPIO_TypeDef* gpioPort = pinConfig.port;
      uint16_t gpioPin = pinConfig.pin;
      uint8_t alternateFunction = pinConfig.alternateFunction;

      GPIODevice gpioDevice(gpioPort, gpioPin);
      gpioDevice.configurePin(
        GPIODevice::Mode::AlternateFunction,
        GPIODevice::OutputType::PushPull,
        GPIODevice::Speed::High,
        GPIODevice::Pull::NoPull,
        alternateFunction
      );
    }
  }
}

void GeneralTimer::configureOutputCompare(uint32_t channel, uint32_t compareValue, OCMode ocMode) {
  volatile uint32_t* ccmrRegister = nullptr;
  uint32_t ccmrMask = 0;
  volatile uint32_t* ccrRegister = nullptr;
  uint32_t shiftValue = 0;

  switch (channel) {
    case 1:
      ccmrRegister = &(_timer->CCMR1);      // Channel 1 is in CCMR1
      ccmrMask = TIM_CCMR1_OC1M;            // OC1M is in bits 4-6 of CCMR1
      ccrRegister = &(_timer->CCR1);        // Channel 1 uses CCR1
      _timer->CCMR1 &= ~TIM_CCMR1_CC1S;     // Clear CC1S to select output compare mode
      shiftValue = 4;                       // OC1M bits are 4-6, so shift by 4
      break;
    case 2:
      ccmrRegister = &(_timer->CCMR1);      // Channel 2 is also in CCMR1
      ccmrMask = TIM_CCMR1_OC2M;            // OC2M is in bits 12-14 of CCMR1
      ccrRegister = &(_timer->CCR2);        // Channel 2 uses CCR2
      _timer->CCMR1 &= ~TIM_CCMR1_CC2S;     // Clear CC2S to select output compare mode
      shiftValue = 12;                      // OC2M bits are 12-14, so shift by 12
      break;
    case 3:
      ccmrRegister = &(_timer->CCMR2);      // Channel 3 is in CCMR2
      ccmrMask = TIM_CCMR2_OC3M;            // OC3M is in bits 4-6 of CCMR2
      ccrRegister = &(_timer->CCR3);        // Channel 3 uses CCR3
      _timer->CCMR2 &= ~TIM_CCMR2_CC3S;     // Clear CC3S to select output compare mode
      shiftValue = 4;                       // OC3M bits are 4-6, so shift by 4
      break;
    case 4:
      ccmrRegister = &(_timer->CCMR2);      // Channel 4 is in CCMR2
      ccmrMask = TIM_CCMR2_OC4M;            // OC4M is in bits 12-14 of CCMR2
      ccrRegister = &(_timer->CCR4);        // Channel 4 uses CCR4
      _timer->CCMR2 &= ~TIM_CCMR2_CC4S;     // Clear CC4S to select output compare mode
      shiftValue = 12;                      // OC4M bits are 12-14, so shift by 12
      break;
    default:
      return; // Invalid channel, return or handle error
  }

  // Set the compare value for the specified channel
  *ccrRegister = compareValue;

  // Clear existing mode and set the new output compare mode (OCxM)
  *ccmrRegister &= ~ccmrMask;  // Clear OCxM bits for the selected channel
  *ccmrRegister |= (static_cast<uint32_t>(ocMode) << shiftValue);  // Set OCxM to the selected mode

  // Enable output for the given channel (CCER register)
  _timer->CCER |= (1 << ((channel - 1) * 4));
}

void GeneralTimer::configureInputCapture(uint32_t channel) {
//TODO write it later
  if (channel == 1) {
    _timer->CCMR1 |= TIM_CCMR1_CC1S_0;  // Input capture on TI1
    _timer->CCER |= TIM_CCER_CC1E;      // Enable capture
  }
}

void GeneralTimer::setPeriod(uint32_t period) {
  _timer->ARR = period - 1;
}

void GeneralTimer::setPrescaler(uint32_t prescaler) {
  _timer->PSC = prescaler - 1;
}

void GeneralTimer::setCounterValue(uint32_t value) {
  _timer->CNT = value;
}

void GeneralTimer::start() {
  _timer->CR1 |= TIM_CR1_CEN; // Enable the timer
}

void GeneralTimer::stop() {
  _timer->CR1 &= ~TIM_CR1_CEN; // Disable the timer
}

void GeneralTimer::enableInterrupt() {
  _timer->DIER |= TIM_DIER_UIE; // Enable update interrupt
}

void GeneralTimer::disableInterrupt() {
  _timer->DIER &= ~TIM_DIER_UIE; // Disable update interrupt
}

void GeneralTimer::IRQHandler(TIM_TypeDef* timer) {
  if (timer->SR & TIM_SR_UIF) {
    timer->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    // Handle interrupt event here
  }
}

extern "C" void TIM2_IRQHandler() {
  GeneralTimer::IRQHandler(TIM2);
}
