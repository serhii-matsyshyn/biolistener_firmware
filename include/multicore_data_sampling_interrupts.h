#ifndef MULTICORE_DATA_SAMPLING_INTERRUPTS_H
#define MULTICORE_DATA_SAMPLING_INTERRUPTS_H

// These define's must be placed at the beginning before #include "_TIMERINTERRUPT_LOGLEVEL_.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// #define _TIMERINTERRUPT_LOGLEVEL_ 4

#include <Arduino.h>
#include <ESP32TimerInterrupt.hpp> //https://github.com/khoih-prog/ESP32TimerInterrupt
// FIXME: change to new version of ESP32TimerInterrupt library
#include "tcp_server_client.h"
#include <SPI.h>

#include "config_esp32_biosignals.h"

#define I_TIMER0_INTERVAL_MS 1L

// Something should be outside of class to be able to use it in ISR
bool IRAM_ATTR ITimerHandler0(void *timerNo);
void IRAM_ATTR InterruptHandlerADC_DRDY();
void IRAM_ATTR addTaskIdToQueueUniversal(void *taskId);
int get_irqTasksMapSize();

typedef void (*irqCallback)();        // no arguments
typedef void (*irqCallbackP)(void *); // one argument

struct irqUniversalTaskStruct
{
  short taskId;
  String taskType;
  irqCallbackP irqCallbackFunc;
  uint32_t irqInterval;
  short irqInternalId;
  bool active;
  bool stop;
  bool runFromBegin;
};

extern QueueHandle_t interruptsTasksQueue;
extern ESP32Timer ITimer0;        // Init ESP32 timer 0
extern ESP32_ISR_Timer ISR_Timer; // Init ESP32_ISR_Timer
extern irqUniversalTaskStruct irqTasksMap[];

extern SPIClass spi;
extern uint32_t adc_raw_array[];

#if (ADC_USED == ADC_ADS131M08)
extern ADS131M08 adc;
#elif (ADC_USED == ADC_AD7771)
extern AD7779 adc;
#else
#error "ADC_USED is not defined or is not valid"
#endif


extern Esp32TcpServerCLient esp32Tcp;

class multicoreDataSamplingInterrupts
{
private:
  bool _init{false};

  short interruptsTasksQueueLength{100};
  int numberOfTasks{0};

public:
  static TaskHandle_t quickTasksProcessingThreadHandle;
  multicoreDataSamplingInterrupts();
  virtual ~multicoreDataSamplingInterrupts() {};

  bool begin(short interruptsTasksQueueLengthValue, int numberOfTasks);
  void startMainInterrupt();
  bool runTask(short i);
  void deleteTaskByForce(short i);
  void deleteTaskFriendly(short i);
  bool manuallyAddTaskIdToTasksQueue(short taskId);

  void quickTasksProcessingThread(void *parameter);

  void dogFeeder(void *parameter);
};

void processTask(u_int8_t taskCodeId);


#endif // MULTICORE_DATA_SAMPLING_INTERRUPTS_H