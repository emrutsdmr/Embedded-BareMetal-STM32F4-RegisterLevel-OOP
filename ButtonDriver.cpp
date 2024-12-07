/*
 * ButtonDriver.cpp
 *
 *  Created on: Oct 16, 2024
 *      Author: emrullah
 */

#include "ButtonDriver.h"
#include "LedDriver.h"

LedDriver led(GPIOA, 5);

static ButtonDriver* instance = nullptr;

ButtonDriver::ButtonDriver(GPIO_TypeDef* port)
    : GPIODevice(port, 13) {}  // Default pin 13

ButtonDriver::ButtonDriver(GPIO_TypeDef* port, uint16_t pin)
    : GPIODevice(port, pin)
{
  configurePin(Mode::Input, OutputType::PushPull, Speed::Low, Pull::NoPull);  // Configure pin for button input
  instance = this;  // Assign the instance pointer
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
  // Read the state of the button (using _currentPin)
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
  // Enable SYSCFG clock to allow configuration of EXTI
  __HAL_RCC_SYSCFG_CLK_ENABLE();

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

// Static function that handles the interrupt for the button press
void ButtonDriver::BTN_IRQ_Handler()
{
  // Check which pin triggered the interrupt (e.g., pin 13 for the button)
  if (EXTI->PR & EXTI_PR_PR13) {
    EXTI->PR = EXTI_PR_PR13;  // Clear the interrupt flag

    // Debouncing: Check if the button is still pressed after a small delay
    static uint32_t lastPressTime = 0;
    uint32_t currentTime = HAL_GetTick();

    if (currentTime - lastPressTime > 50) {  // 50ms debounce threshold
      lastPressTime = currentTime;  // Update last press time

      if (instance->getState() == ButtonState::ON) {  // Check the button state after debounce

        led.toggle();  // Toggle the LED or perform other actions

      }
    }
  }
}
// The EXTI15_10 interrupt handler for EXTI pins 10 to 15
extern "C" void EXTI15_10_IRQHandler(void)
{
    // Call the ButtonDriver's static IRQ handler for the button interrupt
    ButtonDriver::BTN_IRQ_Handler();
}

//void ButtonDriver::BTN_IRQ_Handler()
//{
//  // Check which pin triggered the interrupt (e.g., pin 13 for the button)
//  if (EXTI->PR & EXTI_PR_PR13) {
//    EXTI->PR = EXTI_PR_PR13;  // Clear the interrupt flag for pin 13
//
//    // Action: Toggle LED or perform some other action (like debouncing)
//    led.toggle(); // Toggle the LED
//
//    // Optional: Add a software debounce mechanism or other handling logic
//    // For example, debounce with a small delay:
//    HAL_Delay(50); // 50ms debounce delay (adjustable)
//  }
//}

