#include <SX127x.h>

SPIClass* SX127x::_spiStatic;

uint32_t SX127x::_spiFrequency;

int8_t SX127x::_nssStatic;

SX127x::SX127x()
{
    setSPI(SX127X_SPI);
    setPins(SX127X_PIN_NSS, SX127X_PIN_RESET);
}

bool SX127x::begin()
{
    // set pins as input or output
    pinMode(_nss, OUTPUT);
    if (_irq != -1) pinMode(_irq, INPUT);
    if (_txen != -1) pinMode(_txen, OUTPUT);
    if (_rxen != -1) pinMode(_rxen, OUTPUT);

    // begin spi and perform device reset
    _spi->begin();
    SX127x::reset();

    // check if device connect and set modem to LoRa
    uint8_t version = readRegister(SX127X_REG_VERSION);
    if (version != 0x12 && version != 0x22) return false;
    setModem(SX127X_LORA_MODEM);
    return true;
}

bool SX127x::begin(int8_t nss, int8_t reset, int8_t irq, int8_t txen, int8_t rxen)
{
    setPins(nss, reset, irq, txen, rxen);
    return begin();
}

void SX127x::end()
{
    sleep();
    _spi->end();
}

bool SX127x::reset()
{
    // put reset pin to low then wait 10 ms
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, LOW);
    delay(1);
    digitalWrite(_reset, HIGH);
    delay(10);
}

void SX127x::sleep()
{
    // put device in sleep mode
    writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_SLEEP);
}

void SX127x::wake()
{
    // wake device by put in standby mode
    writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_STDBY);
}

void SX127x::standby()
{
    writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_STDBY);
}

void SX127x::setActive()
{
    // override spi and nss static property with object property
    _nssStatic = _nss;
    _spiStatic = _spi;
}

void SX127x::setSPI(SPIClass &SpiObject, uint32_t frequency)
{
    _spi = &SpiObject;
    _spiStatic = &SpiObject;
    _spiFrequency = frequency;
}

void SX127x::setPins(int8_t nss, int8_t reset, int8_t irq, int8_t txen, int8_t rxen)
{
    _nss = nss;
    _nssStatic = nss;
    _reset = reset;
    _irq = irq;
    _txen = txen;
    _rxen = rxen;
}

void SX127x::setModem(uint8_t modem)
{
    _modem = modem;
    sleep();
    writeRegister(SX127X_REG_OP_MODE, modem | SX127X_MODE_STDBY);
}

void SX127x::writeBits(uint8_t address, uint8_t data, uint8_t position, uint8_t length)
{
    uint8_t read = _transfer(address & 0x7F, 0x00);
    uint8_t mask = (0xFF >> (8 - length)) << position;
    uint8_t write = (data << position) | (read & ~mask);
    _transfer(address | 0x80, write);
}

void SX127x::writeRegister(uint8_t address, uint8_t data)
{
    _transfer(address | 0x80, data);
}

uint8_t SX127x::readRegister(uint8_t address)
{
    return _transfer(address & 0x7F, 0x00);
}

uint8_t SX127x::_transfer(uint8_t address, uint8_t data)
{
    digitalWrite(_nssStatic, LOW);

    _spiStatic->beginTransaction(SPISettings(_spiFrequency, MSBFIRST, SPI_MODE0));
    _spiStatic->transfer(address);
    uint8_t response = _spiStatic->transfer(data);
    _spiStatic->endTransaction();

    digitalWrite(_nssStatic, HIGH);

    return response;
}
