#include "Esp32TcpServerClient.h"
#include "ADS131M08.h"
#include <SPI.h>

#include "config_esp32_biosignals.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <ESP32TimerInterrupt.h> //https://github.com/khoih-prog/ESP32TimerInterrupt
#include "multicore-data-sampling-interrupts.h"



SPIClass spi(VSPI);
ADS131M08 adc;
int32_t adc_raw_array[ADS131M08_NUM_CHANNELS] = {0};

Esp32TcpServerCLient esp32Tcp;
multicoreDataSamplingInterrupts multicoreDataSamplingInterruptsModule;

irqUniversalTaskStruct irqTasksMap[] =
    {
        // {0, "setInterval", addTaskIdToQueueUniversal, 50L, -1, false, false}, // sampling heart rate i2c
     {1, "setInterval", addTaskIdToQueueUniversal, 2L, -1, false, false}, // switch off after 10 seconds of sampling
};

void processTask(u_int8_t taskCodeId)
{
    if (taskCodeId == 0)
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
    else if (taskCodeId == 1)
    {
        if (adc.isDataReady())
        {
            // Get the data as float
            // res = adc.readAdcFloat();
            // Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", res.ch[0].f, res.ch[1].f, res.ch[2].f, res.ch[3].f, res.ch[4].f, res.ch[5].f, res.ch[6].f, res.ch[7].f);
            static size_t counter = 0;

            adc.do_read_adc(adc_raw_array);
            // Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", adc_raw_array[0], adc_raw_array[1], adc_raw_array[2], adc_raw_array[3], adc_raw_array[4], adc_raw_array[5], adc_raw_array[6], adc_raw_array[7]);
        
            StaticJsonDocument<1024> doc;
            doc["ts"] = millis();
            doc["type"] = 1;
            doc["n"] = counter;
            doc["s_id"] = 0;
            JsonArray data = doc.createNestedArray("data");
            for (int i = 0; i < 8; i++)
            {
                data.add(adc_raw_array[i]);
                // Serial.printf("Adding i=%d, value=%d\n", i, adc_raw_array[i]);
            }

            counter++;

            char buffer[1024];
            serializeJson(doc, buffer);
            // serializeMsgPack(doc, buffer);
            strcat(buffer, "\n"); // add /n to the end of the message

            xQueueSend(esp32Tcp.messageQueue, buffer, 0);
       
        }

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
    Serial.begin(1000000);

    spi.begin(ESP_SCLK, ESP_MISO, ESP_MOSI, ESP_GPIO_CS_ANALOG_1);

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

// FIXME: attachInterrupt causes crash
    // attachInterrupt(ESP_GPIO_ANALOG_DRDY, InterruptHandlerADC_DRDY, FALLING);
   
}

void loop()
{
    // Main loop does nothing; everything is handled on the second core
}
