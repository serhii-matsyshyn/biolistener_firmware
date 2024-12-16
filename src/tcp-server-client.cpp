#include "Esp32TcpServerClient.h"

WiFiClient Esp32TcpServerCLient::client;
TaskHandle_t Esp32TcpServerCLient::tcpTaskHandle;
QueueHandle_t Esp32TcpServerCLient::messageQueue;
QueueHandle_t Esp32TcpServerCLient::commandQueue;

Esp32TcpServerCLient::Esp32TcpServerCLient() {}

void Esp32TcpServerCLient::begin()
{
    messageQueue = xQueueCreate(1000, sizeof(data_packet));
    commandQueue = xQueueCreate(20, 256);
}

void Esp32TcpServerCLient::addCommandToCommandQueue(const String &command)
{
    xQueueSend(commandQueue, command.c_str(), 0);

    short taskId = TaskId_ProcessCommand;
    if (xQueueSend(interruptsTasksQueue, &taskId, 50) != pdPASS)  // FIXME: replace 50 with define
    {
        // Failed to post the message
        Serial.println("Failed to post the message");
    }
}

// Task to send data when a new message is added to the queue
void Esp32TcpServerCLient::tcpSendTask(void *parameter)
{
    static uint8_t message[sizeof(data_packet)];
    size_t messageNum = 0;

    static uint8_t tcp_transfer_buffer[50 * sizeof(data_packet)];

    while (true)
    {
        // Block until a message is received
        // if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdPASS) {
        //     // if (client.connected()) {
        //         // Serial.printf("Sending message: %s\n", message);
        //         client.write(message);
        //     // }
        // }

        if (xQueueReceive(messageQueue, &message, 0) == pdPASS)
        {
            memcpy(tcp_transfer_buffer + messageNum * sizeof(data_packet), &message, sizeof(data_packet));
            messageNum++;
        }

        if (messageNum == 50)
        {
            if (client.connected())
            {
                client.write(tcp_transfer_buffer, sizeof(data_packet) * 50);
            }
            messageNum = 0;
        }
    }
}

void Esp32TcpServerCLient::tcpConnectionTask(void *parameter)
{
    while (true)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi disconnected");
            led.setColor(SingleNeoPixel::RED);
            WiFi.reconnect();
            while (WiFi.status() != WL_CONNECTED)
            {
                delay(500);
                Serial.println("Connecting to WiFi...");
            }
        }
        if (!client.connected())
        {
            led.setColor(SingleNeoPixel::BLUE);
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
    client.setNoDelay(false);
}

void Esp32TcpServerCLient::receiveTask(void *parameter)
{
    while (true)
    {
        if (client.available())
        {
            String command = client.readStringUntil('\n');
            Serial.printf("Received command in receiveTask: %s\n", command.c_str());
            addCommandToCommandQueue(command);
        }
        delay(10);
    }
}
