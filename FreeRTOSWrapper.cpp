/*
 * FreeRTOSWrapper.cpp
 *
 *  Created on: Feb 16, 2025
 *      Author: emrullah
 */

#include "FreeRTOSWrapper.h"

namespace FreeRTOS {

// Task Implementation
Task::Task(const char* name, uint16_t stackSize, UBaseType_t priority)
  : taskName(name), stackSize(stackSize), priority(priority) {}

Task::~Task() {
  if (handle != nullptr) {
      vTaskDelete(handle);
      handle = nullptr;
  }
}

void Task::start() {
  if (xTaskCreate(taskFunction, taskName, stackSize, this, priority, &handle) != pdPASS) {
      handle = nullptr; // Indicate failure
  }
}

void Task::taskFunction(void* pvParameters) {
  auto* instance = static_cast<Task*>(pvParameters);
  instance->run();
}

// Semaphore Implementation
Semaphore::Semaphore() {
  handle = xSemaphoreCreateBinary();
}

Semaphore::~Semaphore() {
  if (handle != nullptr) {
    vSemaphoreDelete(handle);
  }
}

bool Semaphore::take(TickType_t ticksToWait) {
  return (handle != nullptr) && (xSemaphoreTake(handle, ticksToWait) == pdTRUE);
}

void Semaphore::give() {
  if (handle != nullptr) {
    xSemaphoreGive(handle);
  }
}

// Mutex Implementation
Mutex::Mutex() {
  handle = xSemaphoreCreateMutex();
}

Mutex::~Mutex() {
  if (handle != nullptr) {
    vSemaphoreDelete(handle);
  }
}

bool Mutex::lock(TickType_t ticksToWait) {
  return (handle != nullptr) && (xSemaphoreTake(handle, ticksToWait) == pdTRUE);
}

void Mutex::unlock() {
  if (handle != nullptr) {
    xSemaphoreGive(handle);
  }
}

// Queue Implementation
template <typename T>
Queue<T>::Queue(size_t length) {
  handle = xQueueCreate(length, sizeof(T));
}

template <typename T>
Queue<T>::~Queue() {
  if (handle != nullptr) {
    vQueueDelete(handle);
  }
}

template <typename T>
bool Queue<T>::send(const T& item, TickType_t ticksToWait) {
  return (handle != nullptr) && (xQueueSend(handle, &item, ticksToWait) == pdPASS);
}

template <typename T>
bool Queue<T>::receive(T& item, TickType_t ticksToWait) {
  return (handle != nullptr) && (xQueueReceive(handle, &item, ticksToWait) == pdPASS);
}

