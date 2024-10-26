#include "Esp32TcpServerClient.h"

WiFiClient Esp32TcpServerCLient::client;
TaskHandle_t Esp32TcpServerCLient::tcpTaskHandle;
QueueHandle_t Esp32TcpServerCLient::messageQueue;

Esp32TcpServerCLient::Esp32TcpServerCLient() {}

void Esp32TcpServerCLient::begin()
{
    messageQueue = xQueueCreate(10, 256);
    tcpEvents = xEventGroupCreate();
}

void Esp32TcpServerCLient::processCommand(const String &command)
{
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, command);
    if (error)
    {
        Serial.println("Failed to parse command");
        return;
    }
    const char *cmd = doc["command"];
    Serial.printf("Received command: %s\n", cmd);
}

// void Esp32TcpServerCLient::tcpTask()
// {
//     while (true)
//     {
//         if (!client.connected())
//         {
//             reconnect();
//         }

//         // Send data from the queue
//         char message[256];
//         if (xQueueReceive(messageQueue, &message, 0) == pdPASS)
//         {
//             client.print(message);
//         }

//         // Receive data from Python server
//         if (client.available())
//         {
//             String command = client.readStringUntil('\n');
//             processCommand(command);
//         }

//         delay(1); // Avoid busy loop
//     }
// }

// Task to send data when a new message is added to the queue
void Esp32TcpServerCLient::tcpSendTask(void *parameter) {
    char message[256];
    while (true) {
        // Block until a message is received
        if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdPASS) {
            if (client.connected()) {
                client.print(message);
            }
        }
    }
}

void Esp32TcpServerCLient::tcpConnectionTask(void *parameter) {
    while (true) {
        if (!client.connected()) {
            reconnect();
        }
        delay(1000);
    }
}

void Esp32TcpServerCLient::reconnect()
{
    while (!client.connect(SERVER_IP, SERVER_PORT))
    {
        Serial.println("Reconnecting to server...");
        delay(1000);
    }
    Serial.println("Connected to server");
}

void Esp32TcpServerCLient::receiveTask(void *parameter) {
    while (true) {
        if (client.available()) {
            String command = client.readStringUntil('\n');
            processCommand(command);
        }
        delay(10);
    }
}
