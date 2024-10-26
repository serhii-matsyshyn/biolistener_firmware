#include "multicore-data-sampling-interrupts.h"

SemaphoreHandle_t taskSemaphore;

ESP32Timer ITimer0(0);     // Init ESP32 timer 0
ESP32_ISR_Timer ISR_Timer; // Init ESP32_ISR_Timer
QueueHandle_t interruptsTasksQueue;
TaskHandle_t multicoreDataSamplingInterrupts::quickTasksProcessingThreadHandle;

// CAN BE USED IN ISR ONLY!!! DO NOT USE!!!
// With core v2.0.0+, you can't use Serial.print/println in ISR or crash.
// and you can't use float calculation inside ISR
// Only OK in core v1.0.6-
bool IRAM_ATTR ITimerHandler0(void *timerNo)
{
    ISR_Timer.run();
    return true;
}

// CAN BE USED IN ISR ONLY!!! DO NOT USE!!!
// In ESP32, avoid doing something fancy in ISR, for example complex Serial.print with String() argument
// The pure simple Serial.prints here are just for demonstration and testing. Must be eliminate in working environment
// Or you can get this run-time error / crash
void IRAM_ATTR addTaskIdToQueueUniversal(void *taskId)
{
    // Variable to check if a higher priority task needs to be woken
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Add task id to queue (from ISR)
    if (xQueueSendFromISR(interruptsTasksQueue, (short *)taskId, &xHigherPriorityTaskWoken) == pdPASS)
    {
        // Successfully added task to queue, now give semaphore to unblock the processing task
        xSemaphoreGiveFromISR(taskSemaphore, &xHigherPriorityTaskWoken);

        // If xHigherPriorityTaskWoken was set to true, yield to ensure the higher-priority task is executed
        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        // Failed to post the message (optional logging or error handling)
    }
}

multicoreDataSamplingInterrupts::multicoreDataSamplingInterrupts()
{
}

bool multicoreDataSamplingInterrupts::init()
{
    interruptsTasksQueue = xQueueCreate(interruptsTasksQueueLength, sizeof(u_int8_t));
    taskSemaphore = xSemaphoreCreateBinary();

    if (interruptsTasksQueue == NULL)
    {
        printf("Error creating the queue");
        return false;
    }

    _init = true;

    return true;
}

bool multicoreDataSamplingInterrupts::begin(short interruptsTasksQueueLengthValue, int numberOfTasks)
{
    interruptsTasksQueueLength = interruptsTasksQueueLengthValue;
    this->numberOfTasks = numberOfTasks;

    if (!_init && interruptsTasksQueueLength)
    {
        return init();
    }
    else
    {
        return false;
    }
}

bool multicoreDataSamplingInterrupts::runTask(short i)
{
    int num_timer;

    if (irqTasksMap[i].active)
    {
        Serial.print("Task already active: ");
        Serial.println(i);
        return false;
    }

    if (irqTasksMap[i].taskType == "setInterval")
    {
        num_timer = ISR_Timer.setInterval(
            irqTasksMap[i].irqInterval,
            irqTasksMap[i].irqCallbackFunc,
            (void *)&irqTasksMap[i].taskId);
    }
    else if (irqTasksMap[i].taskType == "setTimeout")
    {
        num_timer = ISR_Timer.setTimeout(
            irqTasksMap[i].irqInterval,
            irqTasksMap[i].irqCallbackFunc,
            (void *)&irqTasksMap[i].taskId);
    }
    else
    {
        Serial.print("Unknown task type OR not implemented yet -> ");
        Serial.println(irqTasksMap[i].taskType);
        return false;
    }
    Serial.print(F("Starting  ISR_Timer OK, num_timer = "));
    Serial.print(num_timer);
    Serial.print(F(", millis() = "));
    Serial.println(millis());

    irqTasksMap[i].irqInternalId = num_timer;
    irqTasksMap[i].active = true;
    irqTasksMap[i].stop = false;

    return true;
}

void multicoreDataSamplingInterrupts::deleteTaskByForce(short i)
{
    ISR_Timer.deleteTimer(irqTasksMap[i].irqInternalId);
    irqTasksMap[i].active = false;
    irqTasksMap[i].stop = false;
}

void multicoreDataSamplingInterrupts::deleteTaskFriendly(short i)
{
    if (irqTasksMap[i].active)
    {
        irqTasksMap[i].stop = true;
    }
    else
    {
        deleteTaskByForce(i); // if not active, delete it directly
    }
}

void multicoreDataSamplingInterrupts::startMainInterrupt()
{
    // Interval in microsecs
    if (ITimer0.attachInterruptInterval(I_TIMER0_INTERVAL_MS * 1000, ITimerHandler0))
    {
        Serial.print(F("Starting  ITimer0 OK, millis() = "));
        Serial.println(millis());
    }
    else
    {
        Serial.println(F("Can't set ITimer0. Select another Timer, freq. or timer"));
    }
}

bool multicoreDataSamplingInterrupts::manuallyAddTaskIdToTasksQueue(short taskId)
{
    if (xQueueSend(interruptsTasksQueue, &taskId, 0) != pdPASS)
    {
        // Failed to post the message
        return false;
    }
    xSemaphoreGive(taskSemaphore);

    return true;
}

void multicoreDataSamplingInterrupts::quickTasksProcessingThread(void *parameter)
{
    startMainInterrupt();

    for (int i = 0; i < numberOfTasks; i++)
    {
        runTask(i);
    }

    for (;;)
    {
        if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE)
        {
            u_int8_t taskCodeId;

            if (xQueueReceive(interruptsTasksQueue, &taskCodeId, 0) == pdPASS)
            {
                processTask(taskCodeId);
            }
        }

        delay(0);
    }

    // Delete this task if ever needed (unlikely reached)
    vTaskDelete(NULL);
}