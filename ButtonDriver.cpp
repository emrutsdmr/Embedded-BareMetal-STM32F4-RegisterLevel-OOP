/*
 * ButtonDriver.cpp
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#include "ButtonDriver.h"

ButtonDriver::ButtonDriver(GPIO_TypeDef* port)
    : GPIODevice(port, 13) {}
ButtonDriver::ButtonDriver(GPIO_TypeDef* port, uint16_t pin)
    : GPIODevice(port, pin)
{
  configurePin(Mode::Input, OutputType::PushPull, Speed::Low, Pull::NoPull);  // Configure pin for button input
}

ButtonDriver::ButtonState ButtonDriver::getState(uint16_t pin) const
{
  // Read the state of the button
  if ((_port->IDR & (1 << pin)) != 0)
  {
      return ButtonState::OFF;
  }
  else
  {
      return ButtonState::ON;
  }
}

ButtonDriver::ButtonState ButtonDriver::getState() const
{
  // Read the state of the button
  if ((_port->IDR & (1 << _currentPin)) != 0)
  {
      return ButtonState::OFF;
  }
  else
  {
      return ButtonState::ON;
  }
}

void ButtonDriver::enableInterrupt(uint16_t pin, uint32_t priority)
{
  //configurePin(pin, Mode::Input, OutputType::Unchanged, Speed::Unchanged, Pull::NoPull);
  // Enable SYSCFG clock
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  if (pin <= 15) {
    uint8_t portCode;
    if (_port == GPIOA) portCode = GPIODevice::GPIOA_EXTI;
    else if (_port == GPIOB) portCode = GPIODevice::GPIOB_EXTI;
    else if (_port == GPIOC) portCode = GPIODevice::GPIOC_EXTI;
    else if (_port == GPIOD) portCode = GPIODevice::GPIOD_EXTI;
    else if (_port == GPIOE) portCode = GPIODevice::GPIOE_EXTI;
    else return;  // Unsupported port

    // Clear previous config and set EXTI source
    SYSCFG->EXTICR[pin / 4] &= ~(0xF << (4 * (pin % 4)));  // Clear bits
    SYSCFG->EXTICR[pin / 4] |= (portCode << (4 * (pin % 4)));

    // Enable interrupt mask for the pin and configure trigger
    EXTI->IMR |= (1UL << pin);
    EXTI->FTSR |= (1UL << pin);  // Falling edge trigger

    // Clear any pending interrupt
    EXTI->PR = (1UL << pin);

    // Set priority and enable NVIC interrupt based on pin range
    if (pin >= 5 && pin <= 9) {
      NVIC_SetPriority(EXTI9_5_IRQn, priority);
      NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    else if (pin >= 10 && pin <= 15) {
      NVIC_SetPriority(EXTI15_10_IRQn, priority);
      NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    else if (pin <= 4) {
      NVIC_SetPriority((IRQn_Type)(EXTI0_IRQn + pin), priority);
      NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn + pin));
    }
  }
}
// TODO: Implement interrupt handler for button press/release.
//extern "C" void EXTI15_10_IRQHandler(void)
//{
//    if ((EXTI->PR & EXTI_PR_PR13) != 0) {
//        EXTI->PR = EXTI_PR_PR13;    // Clear interrupt pending bit for pin 13
//
//        for (int i = 0; i < 5; i++) {
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Set LED ON
//            HAL_Delay(500);                                      // Delay for 500 ms
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);// Set LED OFF
//            HAL_Delay(500);                                      // Delay for 500 ms
//        }
//    }
//}
//extern "C" void EXTI15_10_IRQHandler(void)
//{
//    if ((EXTI->PR & EXTI_PR_PR13) != 0) {
//        EXTI->PR = EXTI_PR_PR13;    // Clear interrupt pending bit for pin 13
//        led.toggle();                                // Toggle the LED
//    }
//    // Add conditions here for other pins as needed
//}

