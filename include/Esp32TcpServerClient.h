#ifndef ESP32_TCP_TIMER_H
#define ESP32_TCP_TIMER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESP32TimerInterrupt.hpp> //https://github.com/khoih-prog/ESP32TimerInterrupt

// Configuration
#define WIFI_SSID "..."
#define WIFI_PASSWORD "..."
#define SERVER_IP "192.168.1.5"
#define SERVER_PORT 12345

typedef void (*irqCallback)();        // no arguments
typedef void (*irqCallbackP)(void *); // one argument

#define EVERY_DOING_NOTHING_DO_DELAY 50

class Esp32TcpServerCLient
{
public:
    Esp32TcpServerCLient();
    void begin();                    // Initialize WiFi, timer, and TCP task
    void tcpTask();                  // Task to handle TCP communication (to run on second core)
    static TaskHandle_t tcpTaskHandle;
    static QueueHandle_t messageQueue;
    
    void tcpSendTask(void *parameter);
    void tcpConnectionTask(void *parameter);
    void receiveTask(void *parameter);
    EventGroupHandle_t tcpEvents;

private:
    static void reconnect();                           // Reconnect if TCP connection is lost
    static void processCommand(const String &command); // Process commands received from the server

    static WiFiClient client;
};

#endif
