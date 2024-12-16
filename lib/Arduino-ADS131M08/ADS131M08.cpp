/*
The original library: https://github.com/tpcorrea/Arduino-ADS131M08
ADS131M08 Arduino Library implemented based on
ADS131M04 Arduino Library (https://github.com/LucasEtchezuri/Arduino-ADS131M04)

Arduino/C++ Driver for the ADS131M08 24-bit Analog to Digital Converter.

Datasheet = https://www.ti.com/lit/ds/symlink/ads131m08.pdf
*/

#include "Arduino.h"
#include "ADS131M08.h"

ADS131M08::ADS131M08() : csPin(0), drdyPin(0), resetPin(0)
{
  for (uint16_t i = 0U; i < 8; i++)
  {
    fullScale.ch[i].f = 1.2; // +-1.2V
    pgaGain[i] = ADS131M08_PgaGain::PGA_1;
    resultFloat.ch[i].f = 0.0;
  }
}

uint8_t ADS131M08::writeRegister(uint8_t address, uint16_t value)
{
  uint16_t res;
  uint8_t addressRcv;
  uint8_t bytesRcv;
  uint16_t cmd = 0;

  spi.beginTransaction(ads131m08_spi_settings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  cmd = (CMD_WRITE_REG) | (address << 7) | 0;

  // res = spi.transfer16(cmd);
  spi.transfer16(cmd);
  spi.transfer(0x00);

  spi.transfer16(value);
  spi.transfer(0x00);

  for (int i = 0; i < 8; i++)
  {
    spi.transfer16(0x0000);
    spi.transfer(0x00);
  }

  res = spi.transfer16(0x0000);
  spi.transfer(0x00);

  for (int i = 0; i < 9; i++)
  {
    spi.transfer16(0x0000);
    spi.transfer(0x00);
  }

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  spi.endTransaction();

  addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
  bytesRcv = (res & REGMASK_CMD_READ_REG_BYTES);

  if (addressRcv == address)
  {
    return bytesRcv + 1;
  }
  return 0;
}

void ADS131M08::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask)
{
  // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

  // Leo el contenido actual del registro
  uint16_t register_contents = readRegister(address);

  // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | value;

  // Escribo nuevamente el registro
  writeRegister(address, register_contents);
}

uint16_t ADS131M08::readRegister(uint8_t address)
{
  uint16_t cmd;
  uint16_t data;

  cmd = CMD_READ_REG | (address << 7 | 0);

  spi.beginTransaction(ads131m08_spi_settings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  // data = spi.transfer16(cmd);
  spi.transfer16(cmd);
  spi.transfer(0x00);

  for (int i = 0; i < 9; i++)
  {
    spi.transfer16(0x0000);
    spi.transfer(0x00);
  }

  data = spi.transfer16(0x0000);
  spi.transfer(0x00);

  for (int i = 0; i < 9; i++)
  {
    spi.transfer16(0x0000);
    spi.transfer(0x00);
  }

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  spi.endTransaction();

  return data;
}

void ADS131M08::begin(uint8_t cs_pin, uint8_t drdy_pin, uint8_t reset_pin, SPIClass &spi_ads131m08)
{
  // Set pins up
  csPin = cs_pin;
  drdyPin = drdy_pin;
  resetPin = reset_pin;

  spi = spi_ads131m08;

  // Configure chip select as an output
  pinMode(csPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  // Configure DRDY as an input
  pinMode(drdyPin, INPUT);
}

void ADS131M08::begin(uint8_t sclk, uint8_t miso, uint8_t mosi, uint8_t cs_pin, uint8_t drdy_pin, uint8_t reset_pin, SPIClass &spi_ads131m08)
{
  begin(cs_pin, drdy_pin, reset_pin, spi_ads131m08);
  spi.begin(sclk, miso, mosi, cs_pin);
}

int8_t ADS131M08::isDataReadySoft(byte channel)
{
  if (channel == 0)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
  }
  else if (channel == 1)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
  }
  else if (channel == 2)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY2);
  }
  else if (channel == 3)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY3);
  }
  else if (channel == 4)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY4);
  }
  else if (channel == 5)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY5);
  }
  else if (channel == 6)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY6);
  }
  else if (channel == 7)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY7);
  }
  else
  {
    return -1;
  }
}

bool ADS131M08::isResetStatus(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_RESET);
}

bool ADS131M08::isLockSPI(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_LOCK);
}

bool ADS131M08::setDrdyFormat(uint8_t drdyFormat)
{
  if (drdyFormat > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
    return true;
  }
}

bool ADS131M08::setDrdyStateWhenUnavailable(uint8_t drdyState)
{
  if (drdyState > 1)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
    return true;
  }
}

bool ADS131M08::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
    return true;
  }
}

bool ADS131M08::setOsr(uint16_t osr)
{
  if (osr > 7)
  {
    return false;
  }
  else
  {
    writeRegisterMasked(REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR);
    return true;
  }
}

/* Set OSR based on frequency HZ */
bool ADS131M08::set_data_rate(uint16_t frequency_Hz)
{
    uint16_t osr = 0;

    // if power mode is POWER_MODE_HIGH_RESOLUTION - do not scale frequency_hz
    // if POWER_MODE_LOW_POWER - scale frequency_hz by 2
    // if POWER_MODE_VERY_LOW_POWER - scale frequency_hz by 4

    // Serial.printf("Frequency: %d\n", frequency_Hz);

    u_int8_t powerMode = (readRegister(REG_CLOCK) & REGMASK_CLOCK_PWR);

    // Serial.printf("Power Mode: %d\n", powerMode);
    // Serial.printf("REG_CLOCK: %d\n", readRegister(REG_CLOCK));

    if (powerMode == POWER_MODE_LOW_POWER) {
        frequency_Hz = frequency_Hz * 2;
    } else if (powerMode == POWER_MODE_VERY_LOW_POWER) {
        frequency_Hz = frequency_Hz * 4;
    }

    // Serial.printf("Frequency after power mode: %d\n", frequency_Hz);

    if (frequency_Hz <= 250) {
        osr = OSR_16384;
    } else if (frequency_Hz <= 500) {
        osr = OSR_8192;
    } else if (frequency_Hz <= 1000) {
        osr = OSR_4096;
    } else if (frequency_Hz <= 2000) {
        osr = OSR_2048;
    } else if (frequency_Hz <= 4000) {
        osr = OSR_1024;
    } else if (frequency_Hz <= 8000) {
        osr = OSR_512;
    } else if (frequency_Hz <= 16000) {
        osr = OSR_256;
    } else {
        osr = OSR_128;
    }

    return setOsr(osr);
}

void ADS131M08::setFullScale(uint8_t channel, float scale)
{
  if (channel > 7)
  {
    return;
  }

  this->fullScale.ch[channel].f = scale;
}

float ADS131M08::getFullScale(uint8_t channel)
{
  if (channel > 7)
  {
    return 0.0;
  }

  return this->fullScale.ch[channel].f;
}

void ADS131M08::reset()
{
  digitalWrite(this->resetPin, LOW);
  delay(10);
  digitalWrite(this->resetPin, HIGH);
}

bool ADS131M08::set_channel_enable(uint8_t channel, uint16_t enable)
{
  if (channel > 7)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
    return true;
  }
  else if (channel == 4)
  {
    writeRegisterMasked(REG_CLOCK, enable << 12, REGMASK_CLOCK_CH4_EN);
    return true;
  }
  else if (channel == 5)
  {
    writeRegisterMasked(REG_CLOCK, enable << 13, REGMASK_CLOCK_CH5_EN);
    return true;
  }
  else if (channel == 6)
  {
    writeRegisterMasked(REG_CLOCK, enable << 14, REGMASK_CLOCK_CH6_EN);
    return true;
  }
  else if (channel == 7)
  {
    writeRegisterMasked(REG_CLOCK, enable << 15, REGMASK_CLOCK_CH7_EN);
    return true;
  }
  return false;
}

bool ADS131M08::setChannelPGA(uint8_t channel, ADS131M08_PgaGain pga)
{
  uint16_t pgaCode = (uint16_t)pga;

  if (channel > 7)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_GAIN1, pgaCode, REGMASK_GAIN_PGAGAIN0);
    this->pgaGain[0] = pga;
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_GAIN1, pgaCode << 4, REGMASK_GAIN_PGAGAIN1);
    this->pgaGain[1] = pga;
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_GAIN1, pgaCode << 8, REGMASK_GAIN_PGAGAIN2);
    this->pgaGain[2] = pga;
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_GAIN1, pgaCode << 12, REGMASK_GAIN_PGAGAIN3);
    this->pgaGain[3] = pga;
    return true;
  }
  if (channel == 4)
  {
    writeRegisterMasked(REG_GAIN2, pgaCode, REGMASK_GAIN_PGAGAIN4);
    this->pgaGain[4] = pga;
    return true;
  }
  else if (channel == 5)
  {
    writeRegisterMasked(REG_GAIN2, pgaCode << 4, REGMASK_GAIN_PGAGAIN5);
    this->pgaGain[5] = pga;
    return true;
  }
  else if (channel == 6)
  {
    writeRegisterMasked(REG_GAIN2, pgaCode << 8, REGMASK_GAIN_PGAGAIN6);
    this->pgaGain[6] = pga;
    return true;
  }
  else if (channel == 7)
  {
    writeRegisterMasked(REG_GAIN2, pgaCode << 12, REGMASK_GAIN_PGAGAIN7);
    this->pgaGain[7] = pga;
    return true;
  }
  return false;
}

bool ADS131M08::setChannelPGA(uint8_t channel, uint16_t pga)
{
  return setChannelPGA(channel, (ADS131M08_PgaGain)pga);
}

/* Supported gain values: 1, 2, 4, 8, 16, 32, 64, 128 */
bool ADS131M08::set_channel_pga(uint8_t channel, uint16_t gain)
{
  uint16_t pga = (gain >= 1 && gain <= 128 && (gain & (gain - 1)) == 0) 
           ? __builtin_ctz(gain) 
           : ADS131M08_PgaGain::PGA_INVALID; 

  // Serial.printf("Setting channel %d PGA to %d\n", channel, pga);

  return setChannelPGA(channel, pga);
}

ADS131M08_PgaGain ADS131M08::getChannelPGA(uint8_t channel)
{
  if (channel > 7)
  {
    return ADS131M08_PgaGain::PGA_INVALID;
  }
  return this->pgaGain[channel];
}

void ADS131M08::setGlobalChop(uint16_t global_chop)
{
  writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

void ADS131M08::setGlobalChopDelay(uint16_t delay)
{
  writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M08::setInputChannelSelection(uint8_t channel, uint8_t input)
{
  if (channel > 7)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 4)
  {
    writeRegisterMasked(REG_CH4_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 5)
  {
    writeRegisterMasked(REG_CH5_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 6)
  {
    writeRegisterMasked(REG_CH6_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 7)
  {
    writeRegisterMasked(REG_CH7_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  return false;
}

bool ADS131M08::setAllInputChannelSelection(uint8_t input)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    setInputChannelSelection(i, input);
  }
  return true;
}

bool ADS131M08::setChannelOffsetCalibration(uint8_t channel, int32_t offset)
{

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

  if (channel > 7)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH2_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH3_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 4)
  {
    writeRegisterMasked(REG_CH4_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH4_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 5)
  {
    writeRegisterMasked(REG_CH5_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH5_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 6)
  {
    writeRegisterMasked(REG_CH6_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH6_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 7)
  {
    writeRegisterMasked(REG_CH7_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH7_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  return false;
}

bool ADS131M08::setChannelGainCalibration(uint8_t channel, uint32_t gain)
{

  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  if (channel > 7)
  {
    return false;
  }
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 2)
  {
    writeRegisterMasked(REG_CH2_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH2_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 3)
  {
    writeRegisterMasked(REG_CH3_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH3_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 4)
  {
    writeRegisterMasked(REG_CH4_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH4_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 5)
  {
    writeRegisterMasked(REG_CH5_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH5_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 6)
  {
    writeRegisterMasked(REG_CH6_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH6_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 7)
  {
    writeRegisterMasked(REG_CH7_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH7_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  return false;
}

bool ADS131M08::isDataReady()
{
  // DRDY on high to low falling edge
  if (digitalRead(drdyPin) == HIGH)
  {
    return false;
  }
  return true;
}

uint16_t ADS131M08::getId()
{
  return readRegister(REG_ID);
}

uint16_t ADS131M08::getModeReg()
{
  return readRegister(REG_MODE);
}

uint16_t ADS131M08::getClockReg()
{
  return readRegister(REG_CLOCK);
}

uint16_t ADS131M08::getCfgReg()
{
  return readRegister(REG_CFG);
}

//*****************************************************************************
//
//! Calculates the 16-bit CRC for the selected CRC polynomial.
//! Source: https://github.com/TexasInstruments/precision-adc-examples
//!
//! \fn uint16_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use 0xFFFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 16-bit calculated CRC word
//
//*****************************************************************************
uint16_t ADS131M08::calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
{
	/* Check that "dataBytes" is not a null pointer */
	assert(dataBytes != 0x00);

	int         bitIndex, byteIndex;
	bool        dataMSb;						/* Most significant bit of data byte */
	bool        crcMSb;						    /* Most significant bit of crc byte  */
	uint8_t     bytesPerWord = 3;

	/*
     * Initial value of crc register
     * NOTE: The ADS131M0x defaults to 0xFFFF,
     * but can be set at function call to continue an on-going calculation
     */
    uint16_t crc = initialValue;

    #ifdef CRC_CCITT
    /* CCITT CRC polynomial = x^16 + x^12 + x^5 + 1 */
    const uint16_t poly = 0x1021;
    #endif

    #ifdef CRC_ANSI
    /* ANSI CRC polynomial = x^16 + x^15 + x^2 + 1 */
    const uint16_t poly = 0x8005;
    #endif

    //
    // CRC algorithm
    //

    // Loop through all bytes in the dataBytes[] array
	for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
	{
	    // Point to MSb in byte
	    bitIndex = 0x80u;

	    // Loop through all bits in the current byte
	    while (bitIndex > 0)
	    {
	        // Check MSB's of data and crc
	        dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
	        crcMSb  = (bool) (crc & 0x8000u);

	        crc <<= 1;              /* Left shift CRC register */

	        // Check if XOR operation of MSBs results in additional XOR operations
	        if (dataMSb ^ crcMSb)
	        {
	            crc ^= poly;        /* XOR crc with polynomial */
	        }

	        /* Shift MSb pointer to the next data bit */
	        bitIndex >>= 1;
	    }
	}

	return crc;
}


bool ADS131M08::readAdcRaw(void)
{
  uint8_t x = 0;
  uint8_t x2 = 0;
  uint8_t x3 = 0;
  uint8_t x_arr[3] = {0};
  uint32_t aux;
  AdcOutput res;
  uint16_t crcWord = 0xFFFF;  // Initial value for CRC calculation

  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  // Read the status register
  for (int i=0; i<3; i++) {
    x_arr[i] = spi.transfer(0x00);
  }

  crcWord = calculateCRC(&x_arr[0], 3, crcWord);

  this->resultRaw.status = ((x_arr[0] << 8) | x_arr[1]);

  for (int i = 0; i < 8; i++)
  {
    for (int j=0; j<3; j++) {
      x_arr[j] = spi.transfer(0x00);
    }

    crcWord = calculateCRC(&x_arr[0], 3, crcWord);

    aux = ((x_arr[0] << 16) | (x_arr[1] << 8) | x_arr[2]);
    this->resultRaw.ch[i].u_i32 = aux;
  }

  // Read CRC word (3 bytes)
  for (int i=0; i<3; i++) {
    x_arr[i] = spi.transfer(0x00);
  }

  crcWord = calculateCRC(&x_arr[0], 3, crcWord);

  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);

  // Returns true when a CRC error occurs
  return ((bool) crcWord);
}

bool ADS131M08::do_read_adc(uint32_t *adc_raw_array) {
  bool result = this->readAdcRaw();
  for (int i = 0; i < ADS131M08_NUM_CHANNELS; i++) {
    adc_raw_array[i] = this->resultRaw.ch[i].u_i32;
  }
  return result;
}


// FIXME: Fix this function based on external calculations approach
float ADS131M08::scaleResult(uint8_t num)
{
  if (num >= 8)
  {
    return 0.0;
  }

  // aux = (((x_arr[0] << 16) | (x_arr[1] << 8) | x_arr[2]) & 0x00FFFFFF);
  // convert aux to uint32_t
  // if (aux > 0x7FFFFF)
  // {
  //   this->resultRaw.ch[i].i = ((~(aux) & 0x00FFFFFF) + 1) * -1;
  // }
  // else
  // {
  //   this->resultRaw.ch[i].i = aux;
  // }

  return this->resultFloat.ch[num].f = (float)(this->resultRaw.ch[num].i * rawToVolts * this->fullScale.ch[num].f);
}

AdcOutput ADS131M08::scaleResult(void)
{
  // update status
  this->resultFloat.status = this->resultRaw.status;
  // Scale all channels
  for (int i = 0; i < 8; i++)
  {
    this->scaleResult(i);
  }

  return this->resultFloat;
}

AdcOutput ADS131M08::readAdcFloat(void)
{
  this->readAdcRaw();
  return this->scaleResult();
}