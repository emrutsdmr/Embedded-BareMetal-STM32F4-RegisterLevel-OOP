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

