#include <SX127x_driver.h>

SPIClass* sx127x_spi = &SX127X_SPI;
uint32_t sx127x_spiFrequency = SX127X_SPI_FREQUENCY;
int8_t sx127x_nss = SX127X_PIN_NSS;

void sx127x_setSPI(SPIClass &SpiObject, uint32_t frequency)
{
    sx127x_spi = &SpiObject;
    sx127x_spiFrequency = frequency ? frequency : sx127x_spiFrequency;
}

void sx127x_setPins(int8_t nss)
{
    sx127x_nss = nss;
}

void sx127x_reset(int8_t reset)
{
    pinMode(reset, OUTPUT);
    digitalWrite(reset, LOW);
    delay(1);
    digitalWrite(reset, HIGH);
    delay(5);
}

void sx127x_begin()
{
    pinMode(sx127x_nss, OUTPUT);
    sx127x_spi->begin();
}

void sx127x_writeBits(uint8_t address, uint8_t data, uint8_t position, uint8_t length)
{
    uint8_t read = sx127x_transfer(address & 0x7F, 0x00);
    uint8_t mask = (0xFF >> (8 - length)) << position;
    uint8_t write = (data << position) | (read & ~mask);
    sx127x_transfer(address | 0x80, write);
}

void sx127x_writeRegister(uint8_t address, uint8_t data)
{
    sx127x_transfer(address | 0x80, data);
}

uint8_t sx127x_readRegister(uint8_t address)
{
    return sx127x_transfer(address & 0x7F, 0x00);
}

uint8_t sx127x_transfer(uint8_t address, uint8_t data)
{
    digitalWrite(sx127x_nss, LOW);

    sx127x_spi->beginTransaction(SPISettings(sx127x_spiFrequency, MSBFIRST, SPI_MODE0));
    sx127x_spi->transfer(address);
    uint8_t response = sx127x_spi->transfer(data);
    sx127x_spi->endTransaction();

    digitalWrite(sx127x_nss, HIGH);

    return response;
}
