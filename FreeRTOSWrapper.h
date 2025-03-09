/*
 * FreeRTOSWrapper.h
 *
 *  Created on: Feb 16, 2025
 *      Author: emrullah
 */

#ifndef SRC_FREERTOSWRAPPER_H_
#define SRC_FREERTOSWRAPPER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

namespace FreeRTOS {

// Task Wrapper
class Task {
public:
  Task(const char* name, uint16_t stackSize, UBaseType_t priority);
  virtual ~Task();

  void start();

protected:
  virtual void run() = 0;

private:
  static void taskFunction(void* pvParameters);

  TaskHandle_t handle = nullptr;
  const char* taskName;
  uint16_t stackSize;
  UBaseType_t priority;
};

// Semaphore Wrapper
class Semaphore {
public:
  Semaphore();
  ~Semaphore();

  bool take(TickType_t ticksToWait = portMAX_DELAY);
  void give();

private:
  SemaphoreHandle_t handle;
};

// Mutex Wrapper
class Mutex {
public:
  Mutex();
  ~Mutex();

  bool lock(TickType_t ticksToWait = portMAX_DELAY);
  void unlock();

private:
  SemaphoreHandle_t handle;
};

}
#endif /* SRC_FREERTOSWRAPPER_H_ */
