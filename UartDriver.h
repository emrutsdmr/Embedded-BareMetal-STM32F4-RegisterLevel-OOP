/*
 * UartDriver.h
 *
 *  Created on: Nov 10, 2024
 *      Author: emrullah
 */

#ifndef SRC_UARTDRIVER_H_
#define SRC_UARTDRIVER_H_

#include "GPIODevice.h"

#define BUFFER_SIZE 256

class UartDriver {
  public:
    UartDriver(GPIO_TypeDef* txPort, uint16_t txPin, GPIO_TypeDef* rxPort,
               uint16_t rxPin, USART_TypeDef* uartInstance);

    void initialize(uint32_t baudRate = 9600);
    void enableRXInterrupt();
    void sendByte(uint8_t data);
    void sendByteArray(uint8_t* buffer, uint32_t size);
    int32_t readByte();
    uint32_t bytesToRead();

    // Buffer structures
    struct UART_Buffer {
      uint32_t buffer[BUFFER_SIZE];
      uint32_t head = 0;
      uint32_t tail = 0;
    };

  private:
    GPIODevice _txPin;
    GPIODevice _rxPin;
    UART_HandleTypeDef _uartHandle;
    UART_Buffer _rxBuffer;
    UART_Buffer _txBuffer;

    bool isBufferEmpty(UART_Buffer* buffer) const;
    void errorHandler();

    static void UART_IRQ_Handler(UartDriver* driver);
};

#endif /* SRC_UARTDRIVER_H_ */

