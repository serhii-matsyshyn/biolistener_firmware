#include "Esp32TcpServerClient.h"
#include <SPI.h>

#if (ADC_USED == ADC_ADS131M08)
#include "ADS131M08.h"
#elif (ADC_USED == ADC_AD7771)
// #include "ad7779.h"
#else
#error "ADC_USED is not defined or is not valid"
#endif

#include "config_esp32_biosignals.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <ESP32TimerInterrupt.h> //https://github.com/khoih-prog/ESP32TimerInterrupt
#include "multicore-data-sampling-interrupts.h"

#include <WiFiUdp.h>
WiFiUDP udpClient;

SPIClass spi(HSPI);
#if (ADC_USED == ADC_ADS131M08)
ADS131M08 adc;
int32_t adc_raw_array[ADS131M08_NUM_CHANNELS] = {0};
#elif (ADC_USED == ADC_AD7771)
AD7779 adc(true, {7, 6, 5, 4, 0, 1, 2, 3});
uint32_t adc_raw_array[AD777x_NUM_CHANNELS] = {0};
#else
#error "ADC_USED is not defined or is not valid"
#endif


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
        int32_t ret = adc.do_read_adc(adc_raw_array);

        if (ret != 0)
        {
            // Serial.printf("Error reading ADC: %d\n", ret);
            return;
        }
        // Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", adc_raw_array[0], adc_raw_array[1], adc_raw_array[2], adc_raw_array[3], adc_raw_array[4], adc_raw_array[5], adc_raw_array[6], adc_raw_array[7]);

        // // Debugging errors on SPI line
        // double ref_mV = 2500.0; // 2.5V reference voltage
        // double ref_microV = ref_mV * 1000.0;
        // double pga_gain = 8.0;  // PGA gain of 1
        // double result_mV[AD777x_NUM_CHANNELS];
        // for (int i = 0; i < AD777x_NUM_CHANNELS; i++)
        // {
        //     result_mV[i] = adc.data_to_millivolts(ref_microV, adc_raw_array[i], pga_gain);
        // }

        // bool send = true;
        // bool same = true;
        // uint32_t first = adc_raw_array[0];
        // for (int i = 0; i < 7; i++)
        // {
        //     if (result_mV[i] < -500 || result_mV[i] > 500 ) // & adc_raw_array[i] != first)
        //     {
        //         send = false;
        //         break;
        //     }
        // }

        // for (int i = 0; i < 8; i++)
        // {
        //     if (adc_raw_array[i] != first)
        //     {
        //         same = false;
        //         break;
        //     }
        // }

        // if (!send) {
        //     digitalWrite(25, HIGH);
        //     Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        //           result_mV[0],
        //           result_mV[1],
        //           result_mV[2],
        //           result_mV[3],
        //           result_mV[4],
        //           result_mV[5],
        //           result_mV[6],
        //           result_mV[7]);
            
        // }

        // if (!send || same)
        // {
        //     return;
        // }

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
            // Serial.println("Failed to post the message - Queue is full");
        }

        // digitalWrite(25, LOW);
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

#if (ADC_USED == ADC_ADS131M08)
                adc.reset();
                delay(200);

                adc.set_data_rate(500); // Set data rate in Hz

                // Configure the channels
                for (uint8_t i = 0; i < 8; i++)
                {
                    adc.set_channel_enable(i, true);
                    adc.set_channel_pga(i, 1); // Set PGA Gain as gain number
                }
#elif (ADC_USED == ADC_AD7771)
// FIXME: set reset
                int32_t ret = adc.configure_sd_data_convertion_mode();

                Serial.println("2 - AD7779 initialized");
                if (SUCCESS != ret)
                {
                    delay(50);
                    uint8_t reg = 0;
                    ret = adc.spi_int_reg_read(AD7779_REG_GEN_ERR_REG_1_EN, &reg);
                    while (1)
                    {
                        Serial.printf("PANIC: Failed to initialize ad777x! %d. Reg hex: %x\r\n", ret, reg);
                        delay(5000);
                    }
                }

#endif

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

    // pinMode(25, OUTPUT);
    // digitalWrite(25, LOW);

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
