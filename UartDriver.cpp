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
    errorHandler();
  }

  enableRXInterrupt();
}

void UartDriver::enableRXInterrupt()
{
  SET_BIT(_uartHandle.Instance->CR1, USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void UartDriver::errorHandler()
{
  while (1) {
    // TODO: Loop to indicate an error
  }
}

void UartDriver::sendByte(uint8_t data)
{
  _txBuffer.buffer[_txBuffer.head++] = data;
  if (_txBuffer.head == BUFFER_SIZE) {
    _txBuffer.head = 0;
  }
  SET_BIT(_uartHandle.Instance->CR1, USART_CR1_TXEIE);
}

void UartDriver::sendByteArray(uint8_t* buffer, uint32_t size)
{
  for (uint32_t i = 0; i < size; i++) {
    sendByte(buffer[i]);
  }
}

int32_t UartDriver::readByte()
{
  if (isBufferEmpty(&_rxBuffer)) {
    return -1;
  }
  uint8_t data = _rxBuffer.buffer[_rxBuffer.tail++];
  if (_rxBuffer.tail == BUFFER_SIZE) {
    _rxBuffer.tail = 0;
  }
  return data;
}

uint32_t UartDriver::bytesToRead()
{
  if (_rxBuffer.head >= _rxBuffer.tail) {
    return _rxBuffer.head - _rxBuffer.tail;
  }
  else {
    return (BUFFER_SIZE + _rxBuffer.head - _rxBuffer.tail);
  }
}

bool UartDriver::isBufferEmpty(volatile UART_Buffer* buffer) const
{
  return buffer->head == buffer->tail;
}

void UartDriver::UART_IRQ_Handler(UartDriver* driver)
{
  uint32_t srFlags = driver->_uartHandle.Instance->SR;
  uint32_t cr1 = driver->_uartHandle.Instance->CR1;

  // Handle RX interrupt
  if (((srFlags & USART_SR_RXNE) != 0) && ((cr1 & USART_CR1_RXNEIE) != 0)) {
    driver->_rxBuffer.buffer[driver->_rxBuffer.head++] = driver->_uartHandle.Instance->DR;
    if (driver->_rxBuffer.head == BUFFER_SIZE) {
      driver->_rxBuffer.head = 0;
    }
  }

  // Handle TX interrupt
  if (((srFlags & USART_SR_TXE) != 0) && ((cr1 & USART_CR1_TXEIE) != 0)) {
    if (driver->_txBuffer.head != driver->_txBuffer.tail) {
      driver->_uartHandle.Instance->DR = driver->_txBuffer.buffer[driver->_txBuffer.tail++];
      if (driver->_txBuffer.tail == BUFFER_SIZE) {
        driver->_txBuffer.tail = 0;
      }
    }
    else {
      CLEAR_BIT(driver->_uartHandle.Instance->CR1, USART_CR1_TXEIE);
    }
  }
}

// Extern C function to call from IRQ
extern "C" void USART2_IRQHandler()
{
  UartDriver::UART_IRQ_Handler(/* pass an instance of UartDriver */);
}

