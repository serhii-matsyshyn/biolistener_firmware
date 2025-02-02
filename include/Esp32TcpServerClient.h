#ifndef ESP32_TCP_SERVER_CLIENT_H
#define ESP32_TCP_SERVER_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "config_esp32_biosignals.h"
#include <ESP32TimerInterrupt.hpp> //https://github.com/khoih-prog/ESP32TimerInterrupt


typedef void (*irqCallback)();        // no arguments
typedef void (*irqCallbackP)(void *); // one argument

extern QueueHandle_t interruptsTasksQueue;
extern SingleNeoPixel led;
extern ConfigManager config_manager;

class Esp32TcpServerCLient
{
public:
    Esp32TcpServerCLient();
    void begin();                    // Initialize WiFi, timer, and TCP task
    void tcpTask();                  // Task to handle TCP communication (to run on second core)
    static TaskHandle_t tcpTaskHandle;
    static QueueHandle_t messageQueue;
    static QueueHandle_t commandQueue;
    
    void tcpSendTask(void *parameter);
    void tcpConnectionTask(void *parameter);
    void receiveTask(void *parameter);
    EventGroupHandle_t tcpEvents;

    static void addCommandToCommandQueue(const String &command);
    static WiFiClient client;

private:
    static void reconnect();                           // Reconnect if TCP connection is lost
};

#endif // ESP32_TCP_SERVER_CLIENT_H
