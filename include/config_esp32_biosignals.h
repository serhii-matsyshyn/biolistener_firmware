#ifndef CONFIG_ESP32_BIOSIGNALS_H
#define CONFIG_ESP32_BIOSIGNALS_H

#define ESP_GPIO_CS_SD 26
#define ESP_STIMULATION_VOLTAGE 39
#define ESP_SCLK 18
#define ESP_GPIO_ANALOG_DRDY 35
#define ESP_MISO 19
#define ESP_GPIO_RESET 33
#define ESP_MOSI 23
#define ESP_BAT_VOLTAGE 34
#define ESP_GPIO_CS_ANALOG_1 5
#define ESP_GPIO_WS2812B 13
#define ESP_SCL 22
#define ESP_SDA 21
#define ESP_IMU_INT 32
#define ESP_GPIO_CS_ANALOG_2 4

#define TaskId_SendDebugMessage 0
#define TaskId_SendBiosignalData 1
#define TaskId_ProcessCommand 2

#define BIOLISTENER_COMMAND_UNDEFINED 0
#define BIOLISTENER_COMMAND_SET_ADC_DATA_RATE 1
#define BIOLISTENER_COMMAND_SET_ADC_CHANNEL_ENABLE 2
#define BIOLISTENER_COMMAND_SET_ADC_CHANNEL_PGA 3
#define BIOLISTENER_COMMAND_RESET_ADC 4
#define BIOLISTENER_COMMAND_START_SAMPLING 5
#define BIOLISTENER_COMMAND_STOP_SAMPLING 6

#define BIOLISTENER_DATA_PACKET_DEBUG 0
#define BIOLISTENER_DATA_PACKET_BIOSIGNALS 1

#define BIOLISTENER_DATA_PACKET_HEADER 0xA0
#define BIOLISTENER_DATA_PACKET_FOOTER 0xC0

#define ADC_ADS131M08 0
#define ADC_AD7771 1

#define COMBINED_DATA_PACKET_NUM 50

#define SUCCESS 0
#define FAILURE -1

#if (ADC_USED == ADC_ADS131M08)
#include "ADS131M08.h"
#elif (ADC_USED == ADC_AD7771)
#include "ad7779.h"
#else
#error "ADC_USED is not defined or is not valid"
#endif


typedef struct data_packet
{
    uint8_t header;

    uint32_t ts;

    uint8_t type;

    uint32_t n;

    uint8_t s_id;

    uint32_t data[8];

    uint8_t footer;
} __attribute__ ((packed)) data_packet;

#endif // CONFIG_ESP32_BIOSIGNALS_H