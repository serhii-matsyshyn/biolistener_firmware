#include "ad7779.h"

AD7779::~AD7779()
{
}

bool AD7779::begin(uint8_t cs_pin, uint8_t drdy_pin, uint8_t reset_pin, SPIClass &spi_ad7779)
{
    this->cs_pin = cs_pin;
    this->drdy_pin = drdy_pin;
    this->reset_pin = reset_pin;

    // DRDY pin
    pinMode(drdy_pin, INPUT);

    spi = spi_ad7779;
    ad7779_init_param init_param = {
        .spi_init = {
            .chip_select = cs_pin,
            .spi = &spi_ad7779,
        },
        .gpio_reset = {.number = reset_pin},
        .ctrl_mode = AD7779_SPI_CTRL,
        .spi_crc_en = AD7779_DISABLE,
        .state = {AD7779_ENABLE, AD7779_ENABLE, AD7779_ENABLE, AD7779_ENABLE, AD7779_ENABLE, AD7779_ENABLE, AD7779_ENABLE, AD7779_ENABLE},
        .gain = {AD7779_GAIN_8, AD7779_GAIN_8, AD7779_GAIN_8, AD7779_GAIN_8, AD7779_GAIN_8, AD7779_GAIN_8, AD7779_GAIN_8, AD7779_GAIN_8},
        .dec_rate_int = 0xfa0, // 0.512 kHz
        .dec_rate_dec = 0x0000,
        .ref_type = AD7779_INT_REF,
        .pwr_mode = AD7779_HIGH_RES,
        .dclk_div = AD7779_DCLK_DIV_1,
        .sync_offset = {0, 0, 0, 0, 0, 0, 0, 0},
        .offset_corr = {0, 0, 0, 0, 0, 0, 0, 0},
        .gain_corr = {1, 1, 1, 1, 1, 1, 1, 1},
        .ref_buf_op_mode = {AD7779_REF_BUF_ENABLED, AD7779_REF_BUF_ENABLED},
        .sinc5_state = AD7779_DISABLE,
    };
    delay(200);
    return ad7779_init(&ad777x, init_param);
}

bool AD7779::configure_sd_data_convertion_mode()
{
    return ad7779_configure_sd_data_convertion_mode(&ad777x);
}

bool AD7779::reset()
{
    ad777x = ad7779_dev();

    // call begin to re-initialize
    return begin(cs_pin, drdy_pin, reset_pin, spi);
}

/*
AD7779::set_data_rate(uint16_t int_val, uint16_t dec_val)
As an example, if an output data rate of 2.8 kHz is required, the
decimation rate equates to
• High resolution mode = 2048/2.8 = 731.428
• Low power mode = 512/2.8 = 182.857
Minimum data rate in High resolution mode is 512 Hz.
Switch to Low power mode if the data rate lower than 512 Hz is needed.
*/
bool AD7779::set_data_rate(uint16_t frequency_Hz)
{
    uint16_t int_val = 2048000 / frequency_Hz;
    if (ad777x.pwr_mode == AD7779_LOW_PWR)
    {
        int_val = 512000 / frequency_Hz;
    }

    if (int_val < 1)
    {
        Serial.println("Invalid data rate - negative value");
        return false;
    }

    if (int_val > 4000) {
        Serial.println("Invalid data rate - it may be too low if you are in high resolution mode of ADC. Switch to low power mode. Setting to 512Hz.");
        int_val = 4000;
    }
   
    return ad7779_set_dec_rate(&ad777x, int_val, 0);
}

bool AD7779::set_channel_enable(uint8_t ch, uint8_t enable)
{
    if (all_channels_map)
    {
        ch = ch_map[ch];
    }

    return ad7779_set_state(&ad777x, (ad7779_ch)ch, (ad7779_state)enable);
}

/* Supported gain values: 1, 2, 4, 8 */
bool AD7779::set_channel_pga(uint8_t ch, uint8_t gain)
{
    if (all_channels_map)
    {
        ch = ch_map[ch];
    }

    uint16_t pga = (gain >= 1 && gain <= 8 && (gain & (gain - 1)) == 0) 
           ? __builtin_ctz(gain) 
           : 0;

    return ad7779_set_gain(&ad777x, (ad7779_ch)ch, (ad7779_gain)pga);
}

bool AD7779::spi_int_reg_read(uint8_t reg_addr, uint8_t *reg_data)
{
    return ad7779_spi_int_reg_read(&ad777x, reg_addr, reg_data);
}

int32_t AD7779::do_read_adc(uint32_t *adc_raw_array)
{
    uint32_t adc_raw_array_in[AD777x_NUM_CHANNELS] = {0};

    int32_t ret = ad7779_do_read_adc(&ad777x, adc_raw_array_in);

    if (all_channels_map)
    {
        ch_map.apply_channel_mapping_to_array(adc_raw_array_in, adc_raw_array);
    }
    else
    {
        memcpy(adc_raw_array, adc_raw_array_in, sizeof(adc_raw_array_in));
    }
    return ret;
}

double AD7779::data_to_millivolts(double ref_mV, uint32_t raw_code, double pga_gain)
{
    // Calculate the resolution in millivolts
    double resolution = ref_mV / (16777216.0 * pga_gain);

    // Compute the voltage in millivolts based on the raw ADC code
    if (raw_code <= 0x7FFFFF)
    { // Positive range
        return raw_code * resolution;
    }
    else
    { // Negative range (two's complement)
        return (raw_code * resolution) - (ref_mV / pga_gain);
    }
}

/* delay.h */
void mdelay(uint32_t msecs)
{
    // Serial.print("delay ");
    // Serial.println(msecs);
    delay(msecs);
}

/* gpio.h */

int32_t gpio_get(struct gpio_desc **desc,
                 const struct gpio_init_param *param)
{
    *desc = (struct gpio_desc *)malloc(sizeof(struct gpio_desc));
    if (NULL == *desc)
    {
        Serial.println("Failed to allocate memory for gpio_desc");
        *desc = NULL;
        return FAILURE;
    }

    (*desc)->number = param->number;

    return SUCCESS;
}

int32_t gpio_set_value(struct gpio_desc *desc,
                       uint8_t value)
{
    if (NULL == desc)
    {
        return ERR_INVALID_GPIO;
    }

    if (desc->number == 0)
    {
        // Serial.println("gpio_set_value: GPIO 0 is not supported");
        return SUCCESS;
    }

    // Serial.printf("gpio_set_value %d %d\n", desc->number, value);

    digitalWrite(desc->number, value);

    return SUCCESS;
}

int32_t gpio_direction_output(struct gpio_desc *desc,
                              uint8_t value)
{
    if (NULL == desc)
    {
        return SUCCESS;
    }

    if (desc->number == 0)
    {
        // Serial.println("gpio_direction_output: GPIO 0 is not supported");
        return SUCCESS;
    }

    pinMode(desc->number, OUTPUT);
    return gpio_set_value(desc, value);
}

/* spi.h */

struct spi_desc spi_dev;

int32_t spi_init(struct spi_desc **desc,
                 const struct spi_init_param *param)
{
    spi_dev.chip_select = param->chip_select;
    spi_dev.spi = param->spi;
    *desc = &spi_dev;

    // Enable CS pin
    pinMode(param->chip_select, OUTPUT);
    digitalWrite(param->chip_select, HIGH);

    return SUCCESS;
}

int32_t spi_write_and_read(struct spi_desc *desc,
                           uint8_t *data,
                           uint16_t bytes_number)
{
    digitalWrite(desc->chip_select, LOW);
    // Serial.print("TX: [ ");
    // for (int i = 0; i < bytes_number; i++)
    // {
    //     Serial.print("0x");
    //     Serial.print(data[i], HEX);
    //     Serial.print(", ");
    // }
    // Serial.println();

    // spi.beginTransaction(ad777x_spi_settings);
    // spi.transfer(data, bytes_number);
    // spi.endTransaction();

    ((SPIClass *)desc->spi)->beginTransaction(ad777x_spi_settings);
    ((SPIClass *)desc->spi)->transfer(data, bytes_number);
    ((SPIClass *)desc->spi)->endTransaction();

    // Serial.print("RX: [ ");
    // for (int i = 0; i < bytes_number; i++)
    // {
    //     // Serial.print("0x");
    //     Serial.print(data[i], HEX);
    //     Serial.print(",");
    // }
    // Serial.println("]");
    digitalWrite(desc->chip_select, HIGH);

    return SUCCESS;
}


