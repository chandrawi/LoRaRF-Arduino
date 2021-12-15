#include <SX127x.h>

SPIClass* SX127x::_spiStatic = &SX127X_SPI;

uint32_t SX127x::_spiFrequency = SX127X_SPI_FREQUENCY;

int8_t SX127x::_nssStatic = SX127X_PIN_NSS;

uint8_t SX127x::_statusIrq = 0x00;

uint32_t SX127x::_transmitTime = 0;

uint8_t SX127x::_payloadTxRx = 0;

int8_t SX127x::_pinToLow = -1;

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
    if (!SX127x::reset()) return false;

    // set modem to LoRa
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
    // wait until device connected, return false when device too long to respond
    uint32_t t = millis();
    uint8_t version = 0x00;
    while (version != 0x12 && version != 0x22) {
        version = readRegister(SX127X_REG_VERSION);
        if (millis() - t > 1000) return false;
    }
    return true;
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

void SX127x::setFrequency(uint32_t frequency)
{
    _frequency = frequency;
    // calculate frequency
    uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
    writeRegister(SX127X_REG_FRF_MSB, (uint8_t) (frf >> 16));
    writeRegister(SX127X_REG_FRF_MID, (uint8_t) (frf >> 8));
    writeRegister(SX127X_REG_FRF_LSB, (uint8_t) frf);
}

void SX127x::beginPacket()
{
    // clear IRQ flag from last TX or RX operation
    writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);

    // reset TX buffer base address, FIFO address pointer and payload length
    writeRegister(SX127X_REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(SX127X_REG_FIFO_ADDR_PTR, 0);
    _payloadTxRx = 0;

    // set txen pin to high and rxen pin to low
    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, LOW);
        digitalWrite(_txen, HIGH);
        _pinToLow = _txen;
    }
}

void SX127x::endPacket()
{
    // set packet payload length
    writeRegister(SX127X_REG_PAYLOAD_LENGTH, _payloadTxRx);

    // set status to TX wait
    _statusWait = SX127X_STATUS_TX_WAIT;
    _statusIrq = 0x0000;

    // set device to transmit mode
    writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_TX);
    _transmitTime = millis();

}

void SX127x::write(uint8_t data)
{
    // write single byte of package to be transmitted
    write(&data, 1);
}

void SX127x::write(uint8_t* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted in FIFO buffer
    for (uint8_t i = 0; i < length; i++) {
        writeRegister(SX127X_REG_FIFO, data[i]);
    }
    // increasing payload length
    _payloadTxRx += length;
}

void SX127x::write(char* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted for char type
    uint8_t* data_ = (uint8_t*) data;
    write(data_, length);
}

void SX127x::request()
{
    // clear IRQ flag from last TX or RX operation
    writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);

    // set txen pin to low and rxen pin to high
    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, HIGH);
        digitalWrite(_txen, LOW);
        _pinToLow = _rxen;
    }

    // set status to RX wait
    _statusWait = SX127X_STATUS_RX_WAIT;
    _statusIrq = 0x0000;

    // set device to receive mode
    writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_RX_CONTINUOUS);
}

uint8_t SX127x::available()
{
    // get size of package still available to read
    return _payloadTxRx;
}

uint8_t SX127x::read()
{
    // read single byte of received package
    uint8_t data;
    read(&data, 1);
    return data;
}

uint8_t SX127x::read(uint8_t* data, uint8_t length)
{
    // calculate actual read length and remaining payload length
    if (_payloadTxRx > length) {
        _payloadTxRx -= length;
    }
    else {
        length = _payloadTxRx;
        _payloadTxRx = 0;
    }
    // read multiple bytes of received package in FIFO buffer
    for (uint8_t i = 0; i < length; i++) {
        data[i] = readRegister(SX127X_REG_FIFO);
    }
    return length;
}

uint8_t SX127x::read(char* data, uint8_t length)
{
    // read multiple bytes of received package for char type
    uint8_t* data_ = (uint8_t*) data;
    return read(data_, length);
}

void SX127x::purge(uint8_t length)
{
    // subtract or reset received payload length
    _payloadTxRx = (_payloadTxRx > length) && length ? _payloadTxRx - length : 0;
}

bool SX127x::wait(uint32_t timeout)
{
    // immediately return when currently not waiting transmit or receive process
    if (_statusIrq) return false;

    // wait transmit or receive process finish by checking IRQ status
    uint8_t irqFlag = 0x00;
    uint8_t irqFlagMask = _statusWait == SX127X_STATUS_TX_WAIT ? SX127X_IRQ_TX_DONE : SX127X_IRQ_RX_DONE | SX127X_IRQ_CRC_ERR;
    uint32_t t = millis();
    while (!(irqFlag & irqFlagMask)) {
        irqFlag = readRegister(SX127X_REG_IRQ_FLAGS);
        // return when timeout reached
        if (millis() - t > timeout && timeout != 0) return false;
    }

    if (_statusWait == SX127X_STATUS_TX_WAIT) {
        // calculate transmit time and set back txen pin to low
        _transmitTime = millis() - _transmitTime;
        if (_txen != -1) digitalWrite(_txen, LOW);

    } else if (_statusWait == SX127X_STATUS_RX_WAIT) {
        // terminate receive mode by setting mode to standby
        standby();
        // set pointer to RX buffer base address and get packet payload length
        writeRegister(SX127X_REG_FIFO_ADDR_PTR, readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR));
        _payloadTxRx = readRegister(SX127X_REG_RX_NB_BYTES);
        // set back rxen pin to low
        if (_rxen != -1) digitalWrite(_rxen, LOW);

    }

    // store IRQ status
    _statusIrq = irqFlag;
    return true;
}

uint8_t SX127x::status()
{
    // get status for transmit and receive operation based on status IRQ
    if (_statusIrq & SX127X_IRQ_RX_TIMEOUT) return SX127X_STATUS_TX_TIMEOUT;
    else if (!(_statusIrq & SX127X_IRQ_HEADER_VALID)) return SX127X_STATUS_HEADER_ERR;
    else if (_statusIrq & SX127X_IRQ_CRC_ERR) return SX127X_STATUS_CRC_ERR;
    else if (_statusIrq & SX127X_IRQ_TX_DONE) return SX127X_STATUS_TX_DONE;
    else if (_statusIrq & SX127X_IRQ_RX_DONE) return SX127X_STATUS_RX_DONE;

    // return TX or RX wait status
    return _statusWait;
}

uint32_t SX127x::transmitTime()
{
    // get transmit time in millisecond (ms)
    return _transmitTime;
}

float SX127x::dataRate()
{
    // get data rate last transmitted package in kbps
    return 1000.0 * _payloadTxRx / _transmitTime;
}

int16_t SX127x::packetRssi()
{
    // get relative signal strength index (RSSI) of last incoming package
    int16_t offset = _frequency < SX127X_BAND_THRESHOLD ? SX127X_RSSI_OFFSET_LF : SX127X_RSSI_OFFSET_HF;
    if (readRegister(SX127X_REG_VERSION) == 0x22) {
        offset = SX1272_RSSI_OFFSET;
    }
    return (int16_t) readRegister(SX127X_REG_PKT_RSSI_VALUE) - offset;
}

int16_t SX127x::rssi()
{
    int16_t offset = _frequency < SX127X_BAND_THRESHOLD ? SX127X_RSSI_OFFSET_LF : SX127X_RSSI_OFFSET_HF;
    if (readRegister(SX127X_REG_VERSION) == 0x22) {
        offset = SX1272_RSSI_OFFSET;
    }
    return (int16_t) readRegister(SX127X_REG_RSSI_VALUE) - offset;
}

float SX127x::snr()
{
    // get signal to noise ratio (SNR) of last incoming package
    return readRegister(SX127X_REG_PKT_SNR_VALUE) / 4.0;
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
