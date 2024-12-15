extern "C"
{
#include "ad7779_c.h"
}

#include <Arduino.h>
#include <SPI.h>

#define ad777x_spi_settings SPISettings(20000000, MSBFIRST, SPI_MODE3)

struct ChannelMap {
    uint8_t channels[AD777x_NUM_CHANNELS];

    ChannelMap() {
        for (int i = 0; i < AD777x_NUM_CHANNELS; i++) {
            channels[i] = i;
        }
    }

    ChannelMap(std::initializer_list<uint8_t> init) {
        if (init.size() != sizeof(channels)) {
            throw std::invalid_argument("Initializer list must have exactly 8 elements.");
        }
        memcpy(channels, init.begin(), sizeof(channels));
    }

    void apply_channel_mapping_to_array(uint32_t *adc_raw_array, uint32_t *mapped_adc_raw_array) {
        for (int i = 0; i < AD777x_NUM_CHANNELS; i++) {
            mapped_adc_raw_array[i] = adc_raw_array[channels[i]];
        }
    }

    uint8_t& operator[](size_t i) {
        return channels[i];
    }
};

class AD7779
{
public:
    AD7779(bool all_channels_map = false, const ChannelMap& ch_map_in = ChannelMap()) : all_channels_map(all_channels_map) {
        ch_map = ch_map_in;
    }
    ~AD7779();

    bool begin(uint8_t cs_pin, uint8_t drdy_pin, uint8_t reset_pin, SPIClass &spi_ads131m08);
    bool configure_sd_data_convertion_mode();
    bool set_data_rate(uint16_t frequency_Hz);
    bool set_channel_enable(uint8_t ch, uint8_t enable);
    bool set_channel_pga(uint8_t ch, uint8_t gain);
    bool spi_int_reg_read(uint8_t reg_addr, uint8_t *reg_data);
    int32_t do_read_adc(uint32_t *adc_raw_array);
    double data_to_millivolts(double ref_mV, uint32_t raw_code, double pga_gain);

    bool reset();

private:
    ad7779_dev ad777x;
    SPIClass spi;

    bool all_channels_map = false;
    ChannelMap ch_map;

    uint8_t cs_pin;
    uint8_t drdy_pin;
    uint8_t reset_pin;
};
