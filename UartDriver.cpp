/*
 * UartDriver.cpp
 *
 *  Created on: Nov 10, 2024
 *      Author: emrullah
 */

#include "UartDriver.h"

UartDriver::UartDriver(GPIO_TypeDef* txPort, uint16_t txPin, GPIO_TypeDef* rxPort,
                       uint16_t rxPin, USART_TypeDef* uartInstance)
  : _txPin(txPort, txPin), _rxPin(rxPort, rxPin)
{
  /* Enable USARTx clock */
  __HAL_RCC_USART2_CLK_ENABLE(); 

  // Configure UART hardware instance
  _uartHandle.Instance = uartInstance;
}

void UartDriver::initialize(uint32_t baudRate)
{
  // GPIO Pin Configuration for TX
  _txPin.configurePin(GPIODevice::Mode::AlternateFunction,
                      GPIODevice::OutputType::PushPull,
                      GPIODevice::Speed::High,
                      GPIODevice::Pull::PullUp,
                      GPIO_AF7_USART2);

  // GPIO Pin Configuration for RX
  _rxPin.configurePin(GPIODevice::Mode::AlternateFunction,
                      GPIODevice::OutputType::PushPull,
                      GPIODevice::Speed::High,
                      GPIODevice::Pull::PullUp,
                      GPIO_AF7_USART2);

  // UART Configuration
  _uartHandle.Init.BaudRate = baudRate;
  _uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  _uartHandle.Init.StopBits = UART_STOPBITS_1;
  _uartHandle.Init.Parity = UART_PARITY_NONE;
  _uartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  _uartHandle.Init.Mode = UART_MODE_TX_RX;

  if (HAL_UART_Init(&_uartHandle) != HAL_OK) {
    //TODO: errorHandler();
  }

  //TODO: enableRXInterrupt();
}
