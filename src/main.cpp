#include "tcp_server_client.h"
#include <SPI.h>
#include "config_esp32_biosignals.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <ESP32TimerInterrupt.h> //https://github.com/khoih-prog/ESP32TimerInterrupt
#include "multicore_data_sampling_interrupts.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

SPIClass spi(HSPI);
#if (ADC_USED == ADC_ADS131M08)
ADS131M08 adc;
uint32_t adc_raw_array[ADS131M08_NUM_CHANNELS] = {0};
#elif (ADC_USED == ADC_AD7771)
AD7779 adc(true, {7, 6, 5, 4, 0, 1, 2, 3});
uint32_t adc_raw_array[AD777x_NUM_CHANNELS] = {0};
#else
#error "ADC_USED is not defined or is not valid"
#endif

SingleNeoPixel led(ESP_GPIO_WS2812B, 50);
Esp32TcpServerCLient esp32Tcp;
multicoreDataSamplingInterrupts multicoreDataSamplingInterruptsModule;
Adafruit_MPU6050 mpu;
ConfigManager config_manager;

float batteryVoltage = 999.0;

irqUniversalTaskStruct irqTasksMap[] =
    {
        {TaskId_SendDebugMessage, "setInterval", addTaskIdToQueueUniversal, 50L, -1, false, false, false}, // SendDebugMessage
        {TaskId_SendBiosignalData, "setInterval", addTaskIdToQueueUniversal, 2L, -1, false, false, false}, // SendBiosignalData without interrupt data ready
        {TaskId_ProcessCommand, "setInterval", addTaskIdToQueueUniversal, 100L, -1, false, false, false}, // ProcessCommand
        {TaskId_ReadBatteryVoltage, "setInterval", addTaskIdToQueueUniversal, 2000L, -1, false, false, true}, // ReadBatteryVoltage
        {TaskId_ReadIMUData, "setInterval", addTaskIdToQueueUniversal, 20L, -1, false, false, false}, // ReadIMUData
};

void processTask(u_int8_t taskCodeId)
{
    static size_t counter = 0;
    if (taskCodeId == TaskId_SendDebugMessage)
    {
        static size_t counter = 0;

        data_packet listener_packet = {BIOLISTENER_DATA_PACKET_HEADER, millis(), BIOLISTENER_DATA_PACKET_DEBUG, counter, ADC_USED, {999, 999, 999, 999, 999, 999, 999, 999}, BIOLISTENER_DATA_PACKET_FOOTER};
        counter++;

        xQueueSend(esp32Tcp.messageQueue, &listener_packet, 0);
    }
    else if (taskCodeId == TaskId_SendBiosignalData)
    {
        static size_t last_millis = millis();
        static data_packet listener_packet;
        static size_t counter = 0;
        bool ret = adc.do_read_adc(adc_raw_array);

        if (ret != SUCCESS)
        {
            // Serial.printf("Error reading ADC: %d\n", ret);
            return;
        }
        // Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", adc_raw_array[0], adc_raw_array[1], adc_raw_array[2], adc_raw_array[3], adc_raw_array[4], adc_raw_array[5], adc_raw_array[6], adc_raw_array[7]);

        counter++;
        listener_packet = {BIOLISTENER_DATA_PACKET_HEADER, millis(), BIOLISTENER_DATA_PACKET_BIOSIGNALS, counter, ADC_USED, {adc_raw_array[0], adc_raw_array[1], adc_raw_array[2], adc_raw_array[3], adc_raw_array[4], adc_raw_array[5], adc_raw_array[6], adc_raw_array[7]}, BIOLISTENER_DATA_PACKET_FOOTER};

        if (xQueueSend(esp32Tcp.messageQueue, &listener_packet, 0) != pdPASS)
        {
            // Failed to post the message
            // Serial.println("Failed to post the message - Queue is full");
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

                // erase the message queue
                xQueueReset(esp32Tcp.messageQueue);

                adc.reset();
                delay(200);

#if (ADC_USED == ADC_ADS131M08)
                adc.set_data_rate(500); // Set data rate in Hz

                // Configure the channels
                for (uint8_t i = 0; i < 8; i++)
                {
                    adc.set_channel_enable(i, true);
                    adc.set_channel_pga(i, 1); // Set PGA Gain as gain number
                }

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
                detachInterrupt(ESP_GPIO_ANALOG_DRDY);

                // erase the message queue
                xQueueReset(esp32Tcp.messageQueue);

#if (ADC_USED == ADC_AD7771)
                bool ret = adc.configure_sd_data_convertion_mode();

                Serial.println("AD777X configure_sd_data_convertion_mode done.");
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

                delay(10);
#endif
                led.setColor(SingleNeoPixel::GREEN);

                multicoreDataSamplingInterruptsModule.runTask(TaskId_ReadIMUData);

                attachInterrupt(ESP_GPIO_ANALOG_DRDY, InterruptHandlerADC_DRDY, FALLING);
            }
            else if (command == BIOLISTENER_COMMAND_STOP_SAMPLING)
            {
                led.setColor(SingleNeoPixel::BLUE);
                // {"command": X}
                detachInterrupt(ESP_GPIO_ANALOG_DRDY);
                multicoreDataSamplingInterruptsModule.deleteTaskByForce(TaskId_ReadIMUData);
            }
            else
            {
                Serial.println("Unknown TaskId_ProcessCommand");
            }
        }
    }
    else if (taskCodeId == TaskId_ReadBatteryVoltage)
    {
        // Serial.println("TaskId_ReadBatteryVoltage");
        uint16_t analogReadVal = analogRead(ESP_BAT_VOLTAGE);
        batteryVoltage = (analogReadVal * 4.73) / 4096.0; // 4.73 = 1.1 * (330.0 + 100.0) / 100.0;
        // Serial.printf("Battery voltage: %f\n", batteryVoltage);

        if (batteryVoltage < 3.3)
        {
            led.setColor(SingleNeoPixel::YELLOW);
        }
    } 
    else if (taskCodeId == TaskId_ReadIMUData)
    {
        // Serial.println("TaskId_ReadIMUData");

        static size_t counter = 0;

        sensors_event_t a, g, temp;
#if (!IMU_DISABLE)
        mpu.getEvent(&a, &g, &temp);
#endif
        data_packet listener_packet = {BIOLISTENER_DATA_PACKET_HEADER, millis(), BIOLISTENER_DATA_PACKET_IMU, counter, ADC_USED,
        {
            FLOAT_TO_UINT32(a.acceleration.x), FLOAT_TO_UINT32(a.acceleration.y), FLOAT_TO_UINT32(a.acceleration.z),
            FLOAT_TO_UINT32(g.gyro.x), FLOAT_TO_UINT32(g.gyro.y), FLOAT_TO_UINT32(g.gyro.z),
            FLOAT_TO_UINT32(temp.temperature), FLOAT_TO_UINT32(batteryVoltage)
        }, BIOLISTENER_DATA_PACKET_FOOTER};
        counter++;

        if (xQueueSend(esp32Tcp.messageQueue, &listener_packet, 0) != pdPASS)
        {
            // Failed to post the message
            Serial.println("Failed to post the message - Queue is full");
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
        "quickTasksProcessingThread",                                                                  // Name of the task
        10000,                                                                                         // Stack size
        &multicoreDataSamplingInterruptsModule,                                                        // Task parameter
        10,                                                                                            // Priority
        &multicoreDataSamplingInterruptsModule.quickTasksProcessingThreadHandle,                       // Task handle
        1                                                                                              // Run on core
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<multicoreDataSamplingInterrupts *>(param)->dogFeeder(param); }, // Task function (lambda)
        "dogFeeder",                                                                  // Name of the task
        1000,                                                                         // Stack size
        &multicoreDataSamplingInterruptsModule,                                       // Task parameter
        1,                                                                            // Priority
        NULL,                                                                         // Task handle
        1                                                                             // Run on core
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->tcpSendTask(param); }, // Task function (lambda)
        "tcpSendTask",                                                       // Name of the task
        4096,                                                                // Stack size
        &esp32Tcp,                                                           // Task parameter
        1,                                                                   // Priority
        NULL,                                                                // Task handle
        0                                                                    // Run on core
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->tcpConnectionTask(param); }, // Task function (lambda)
        "tcpConnectionTask",                                                       // Name of the task
        4096,                                                                      // Stack size
        &esp32Tcp,                                                                 // Task parameter
        1,                                                                         // Priority
        NULL,                                                                      // Task handle
        0                                                                          // Run on core
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<Esp32TcpServerCLient *>(param)->receiveTask(param); }, // Task function (lambda)
        "receiveTask",                                                       // Name of the task
        4096,                                                                // Stack size
        &esp32Tcp,                                                           // Task parameter
        1,                                                                   // Priority
        NULL,                                                                // Task handle
        0                                                                    // Run on core
    );

    xTaskCreatePinnedToCore(
        [](void *param)
        { static_cast<ConfigManager *>(param)->configPortalTask(param); },   // Task function (lambda)
        "tcpSendTask",                                                       // Name of the task
        4096,                                                                // Stack size
        &config_manager,                                                     // Task parameter
        1,                                                                   // Priority
        NULL,                                                                // Task handle
        0                                                                    // Run on core
    );
}

void setup()
{
    Serial.begin(1000000);

    Wire.begin();
    Wire.setClock(400000L);

    spi.begin(ESP_SCLK, ESP_MISO, ESP_MOSI, ESP_GPIO_CS_ANALOG_1);

    led.begin();
    led.setColor(SingleNeoPixel::RED);

    analogSetPinAttenuation(ESP_BAT_VOLTAGE, ADC_0db);
    analogSetWidth(12);

#if (!IMU_DISABLE)
#pragma message("IMU is enabled")
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
        delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
#endif

    // WiFi connection
    // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    config_manager.begin();

    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    led.setColor(SingleNeoPixel::BLUE);

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
