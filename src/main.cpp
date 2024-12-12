#include "Esp32TcpServerClient.h"
#include <SPI.h>

#if (ADC_USED == ADC_ADS131M08)
#include "ADS131M08.h"
#elif (ADC_USED == ADC_AD7771)
// #include "AD7771.h"
#else
#error "ADC_USED is not defined or is not valid"
#endif

#include "config_esp32_biosignals.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <ESP32TimerInterrupt.h> //https://github.com/khoih-prog/ESP32TimerInterrupt
#include "multicore-data-sampling-interrupts.h"

#include <WiFiUdp.h>
WiFiUDP udpClient;

SPIClass spi(VSPI);
#if (ADC_USED == ADC_ADS131M08)
ADS131M08 adc;
#elif (ADC_USED == ADC_AD7771)
// AD7771 adc;
#else
#error "ADC_USED is not defined or is not valid"
#endif

int32_t adc_raw_array[ADS131M08_NUM_CHANNELS] = {0};

Esp32TcpServerCLient esp32Tcp;
multicoreDataSamplingInterrupts multicoreDataSamplingInterruptsModule;

irqUniversalTaskStruct irqTasksMap[] =
    {
        {TaskId_SendDebugMessage, "setInterval", addTaskIdToQueueUniversal, 50L, -1, false, false, false}, // SendDebugMessage
        {TaskId_SendBiosignalData, "setInterval", addTaskIdToQueueUniversal, 2L, -1, false, false, false}, // SendBiosignalData
};

void processTask(u_int8_t taskCodeId)
{
    static size_t counter = 0;
    if (taskCodeId == TaskId_SendDebugMessage)
    {

        // {
        // "ts": 111111111, // timestamp in milliseconds
        // "n": 0, // number of the message
        // "type": 1, // type 1 - data from sensor
        // "s_id": 0, // sensor id; 0 - ADS131M08, 1 - AD7771
        // "data": [111, 222, 333, 444, 555, 666, 777, 888]
        // }

        static size_t counter = 0;

        StaticJsonDocument<200> doc;
        doc["ts"] = millis();
        doc["type"] = 1;
        doc["n"] = counter;
        doc["s_id"] = 0;
        JsonArray data = doc.createNestedArray("data");
        uint16_t data_arr[8] = {1, 2, 3, 4, 5, 6, 7, 8};

        for (int i = 0; i < 8; i++)
        {
            data.add(data_arr[i]);
        }

        counter++;

        char buffer[256];
        serializeJson(doc, buffer);
        // serializeMsgPack(doc, buffer);
        strcat(buffer, "\n"); // add /n to the end of the message

        xQueueSend(esp32Tcp.messageQueue, buffer, 0);
    }
    else if (taskCodeId == TaskId_SendBiosignalData)
    {
        static size_t last_millis = millis();
        static data_packet listener_packet;
        static size_t counter = 0;
        adc.do_read_adc(adc_raw_array);
        Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", adc_raw_array[0], adc_raw_array[1], adc_raw_array[2], adc_raw_array[3], adc_raw_array[4], adc_raw_array[5], adc_raw_array[6], adc_raw_array[7]);

        // StaticJsonDocument<1024> doc;
        // doc["ts"] = millis();
        // doc["type"] = 1;
        // doc["n"] = counter;
        // doc["s_id"] = ADC_USED;
        // JsonArray data = doc.createNestedArray("data");
        // for (int i = 0; i < 8; i++)
        // {
        //     data.add(adc_raw_array[i]);
        //     // Serial.printf("Adding i=%d, value=%d\n", i, adc_raw_array[i]);
        // }

        // char buffer[128];
        // // serializeJson(doc, buffer);
        // serializeMsgPack(doc, buffer);
        // strcat(buffer, "\n"); // add /n to the end of the message

        counter++;
        listener_packet = {0xA0, millis(), 1, counter, ADC_USED, {adc_raw_array[0], adc_raw_array[1], adc_raw_array[2], adc_raw_array[3], adc_raw_array[4], adc_raw_array[5], adc_raw_array[6], adc_raw_array[7]}, 0xC0};

        if (millis() - last_millis >= 1000)
        {
            // Serial.printf("FPS: %f\n", (float)counter / ((millis() - last_millis) / 1000));
            counter = 0;
            last_millis = millis();
        }

        if (xQueueSend(esp32Tcp.messageQueue, &listener_packet, 0) != pdPASS)
        {
            // Failed to post the message
            Serial.println("Failed to post the message - Queue is full");
        }
    }
    else if (taskCodeId == TaskId_ProcessCommand)
    {
        Serial.println("TaskId_ProcessCommand");
        char message[256];
        // while there are messages in the queue, process them
        while (xQueueReceive(esp32Tcp.commandQueue, &message, 0) == pdPASS)
        {
            // Parse JSON
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, message);
            if (error)
            {
                Serial.println("Failed to parse command");
                return;
            }
            int command = doc["command"];
            Serial.printf("Received command: %d\n", command);

            if (command == BIOLISTENER_COMMAND_SET_ADC_DATA_RATE)
            {
                // {"command": X, "data_rate": 500}
                int data_rate = doc["data_rate"];
                adc.set_data_rate(data_rate);
            }
            else if (command == BIOLISTENER_COMMAND_SET_ADC_CHANNEL_ENABLE)
            {
                // {"command": X, "channel": 0, "enable": 1}
                int channel = doc["channel"];
                int enable = doc["enable"];
                adc.set_channel_enable(channel, enable);
            }
            else if (command == BIOLISTENER_COMMAND_SET_ADC_CHANNEL_PGA)
            {
                // {"command": X, "channel": 0, "pga": 32}
                int channel = doc["channel"];
                int pga = doc["pga"];
                adc.set_channel_pga(channel, pga);
            }
            else if (command == BIOLISTENER_COMMAND_RESET_ADC)
            {
                // {"command": X}
                detachInterrupt(ESP_GPIO_ANALOG_DRDY);

                adc.reset();
                delay(200);

                adc.set_data_rate(500); // Set data rate in Hz

                // Configure the channels
                for (uint8_t i = 0; i < 8; i++)
                {
                    adc.set_channel_enable(i, true);
                    adc.set_channel_pga(i, 1); // Set PGA Gain as gain number
                }

                delay(200);

#if (ADC_USED == ADC_ADS131M08)

                Serial.printf("ID: %d\n", adc.getId());
                Serial.printf("MODE: %d\n", adc.getModeReg());
                Serial.printf("CLOCK: %d\n", adc.getClockReg());
                Serial.printf("CFG: %d\n", adc.getCfgReg());
                Serial.println("ADS131M08 ready\n");

#elif (ADC_USED == ADC_AD7771)
                Serial.println("AD7771 ready\n");
#endif
            }
            else if (command == BIOLISTENER_COMMAND_START_SAMPLING)
            {
                // erase the message queue
                xQueueReset(esp32Tcp.messageQueue);

                attachInterrupt(ESP_GPIO_ANALOG_DRDY, InterruptHandlerADC_DRDY, FALLING);
            }
            else if (command == BIOLISTENER_COMMAND_STOP_SAMPLING)
            {
                // {"command": X}
                detachInterrupt(ESP_GPIO_ANALOG_DRDY);
            }
            else
            {
                Serial.println("Unknown TaskId_ProcessCommand");
            }
        }
    }
    else
    {
        Serial.println("Unknown task code");
    }
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
        &multicoreDataSamplingInterruptsModule.quickTasksProcessingThreadHandle,                       /* Task handle to keep track of created task */
        0                                                                                              /* pin task to core 0 */
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->tcpSendTask(param); }, // Task function (lambda)
        "tcpSendTask",                                                       // Name of the task
        4096,                                                                // Stack size
        &esp32Tcp,                                                           // Task parameter
        1,                                                                   // Priority
        NULL,                                                                // Task handle
        1                                                                    // Run on core 0 (second core)
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->tcpConnectionTask(param); }, // Task function (lambda)
        "tcpConnectionTask",                                                       // Name of the task
        4096,                                                                      // Stack size
        &esp32Tcp,                                                                 // Task parameter
        1,                                                                         // Priority
        NULL,                                                                      // Task handle
        1                                                                          // Run on core 0 (second core)
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->receiveTask(param); }, // Task function (lambda)
        "receiveTask",                                                       // Name of the task
        4096,                                                                // Stack size
        &esp32Tcp,                                                           // Task parameter
        1,                                                                   // Priority
        NULL,                                                                // Task handle
        1                                                                    // Run on core 0 (second core)
    );
}

void setup()
{
    Serial.begin(1000000);

    spi.begin(ESP_SCLK, ESP_MISO, ESP_MOSI, ESP_GPIO_CS_ANALOG_1);

    // WiFi connection
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setSleep(false);
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
    multicoreDataSamplingInterruptsModule.begin(1000, sizeof(irqTasksMap) / sizeof(irqTasksMap[0]));

    Serial.println("Starting threads");
    launchThreads();
}

void loop()
{
    // Main loop does nothing; everything is handled on the second core
}
