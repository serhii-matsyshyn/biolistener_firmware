#include "Esp32TcpServerClient.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <ESP32TimerInterrupt.h> //https://github.com/khoih-prog/ESP32TimerInterrupt
#include "multicore-data-sampling-interrupts.h"

Esp32TcpServerCLient esp32Tcp;
multicoreDataSamplingInterrupts multicoreDataSamplingInterruptsModule;

irqUniversalTaskStruct irqTasksMap[] =
    {
        {0, "setInterval", addTaskIdToQueueUniversal, 5000L, -1, false, false}, // sampling heart rate i2c
     // {1, "setTimeout", addTaskIdToQueueUniversal, 10000L, -1, false, false}, // switch off after 10 seconds of sampling
};

void processTask(u_int8_t taskCodeId)
{
    if (taskCodeId == 0)
    {
        StaticJsonDocument<200> doc;
        doc["message"] = "Hello from Core 1 (Timer IRQ)";
        char buffer[256];
        serializeJson(doc, buffer);

        // xQueueSendFromISR(esp32Tcp.messageQueue, buffer, NULL);
        xQueueSend(esp32Tcp.messageQueue, buffer, 0);
    }
    else if (taskCodeId == 1)
    {
        /* code */
    }
    else
    {
        Serial.println("Unknown task code");
    }
}

    void onReceive() {
    xEventGroupSetBits(esp32Tcp.tcpEvents, 0x01);
}

void launchThreads(void)
{
    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<multicoreDataSamplingInterrupts *>(param)->quickTasksProcessingThread(param); }, // Task function (lambda)
        "quickTasksProcessingThread",                                                                  /* name of task. */
        10000,                                                                                         /* Stack size of task */
        &multicoreDataSamplingInterruptsModule,                                                        /* parameter of the task */
        10,                                                                                            /* priority of the task */
        &multicoreDataSamplingInterruptsModule.quickTasksProcessingThreadHandle,                        /* Task handle to keep track of created task */
        0                                                                                              /* pin task to core 0 */
    );

    // xTaskCreatePinnedToCore(
    //     [](void *param)
    //     { static_cast<Esp32TcpServerCLient *>(param)->tcpTask(); }, // Task function (lambda)
    //     "tcpTask",                                                  // Name of the task
    //     4096,                                                       // Stack size
    //     &esp32Tcp,                                                  // Task parameter
    //     1,                                                          // Priority
    //     &esp32Tcp.tcpTaskHandle,                                    // Task handle
    //     1                                                           // Run on core 0 (second core)
    // );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->tcpSendTask(param); }, // Task function (lambda)
        "tcpSendTask",                                                      // Name of the task
        4096,                                                               // Stack size
        &esp32Tcp,                                                          // Task parameter
        1,                                                                  // Priority
        NULL,                                                               // Task handle
        1                                                                   // Run on core 0 (second core)
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->tcpConnectionTask(param); }, // Task function (lambda)
        "tcpConnectionTask",                                                      // Name of the task
        4096,                                                                     // Stack size
        &esp32Tcp,                                                                // Task parameter
        1,                                                                        // Priority
        NULL,                                                                     // Task handle
        1                                                                         // Run on core 0 (second core)
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->receiveTask(param); }, // Task function (lambda)
        "receiveTask",                                                      // Name of the task
        4096,                                                               // Stack size
        &esp32Tcp,                                                          // Task parameter
        1,                                                                  // Priority
        NULL,                                                               // Task handle
        1                                                                   // Run on core 0 (second core)
    );

}

void setup()
{
    Serial.begin(115200);

    // WiFi connection
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Print local IP address
    Serial.print("Local IP address: ");
    Serial.println(WiFi.localIP());

    esp32Tcp.begin();
    multicoreDataSamplingInterruptsModule.begin(100, sizeof(irqTasksMap) / sizeof(irqTasksMap[0]));

    Serial.println("Starting threads");
    launchThreads();    
}

void loop()
{
    // Main loop does nothing; everything is handled on the second core
}
