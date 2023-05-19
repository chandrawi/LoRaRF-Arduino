#include <SX127x.h>

void (*SX127x::_onTransmit)();

void (*SX127x::_onReceive)();

volatile uint8_t SX127x::_statusIrq = 0xFF;

uint32_t SX127x::_transmitTime = 0;

uint8_t SX127x::_payloadTxRx = 0;

int8_t SX127x::_irqStatic = -1;

int8_t SX127x::_pinToLow = -1;

SX127x::SX127x()
{
    _spi = &SX127X_SPI;
    setPins(SX127X_PIN_NSS, SX127X_PIN_RESET);
}

bool SX127x::begin()
{
    // set pins as input or output
    if (_irq != -1) pinMode(_irq, INPUT);
    if (_txen != -1) pinMode(_txen, OUTPUT);
    if (_rxen != -1) pinMode(_rxen, OUTPUT);

    // begin spi and perform device reset
    sx127x_begin();
    if (!SX127x::reset()) return false;

    // set modem to LoRa
    setModem(SX127X_LORA_MODEM);
    // set tx power and rx gain
    setTxPower(17, SX127X_TX_POWER_PA_BOOST);
    setRxGain(SX127X_RX_GAIN_BOOSTED, SX127X_RX_GAIN_AUTO);
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
    sx127x_reset(_reset);
    // wait until device connected, return false when device too long to respond
    uint32_t t = millis();
    uint8_t version = 0x00;
    while (version != 0x12 && version != 0x22) {
        version = sx127x_readRegister(SX127X_REG_VERSION);
        if (millis() - t > 1000) return false;
    }
    return true;
}

void SX127x::sleep()
{
    // put device in sleep mode
    sx127x_writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_SLEEP);
}

void SX127x::wake()
{
    // wake device by put in standby mode
    sx127x_writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_STDBY);
}

void SX127x::standby()
{
    sx127x_writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_STDBY);
}

void SX127x::setActive()
{
    // override spi, nss, and busy static property in SX127x driver with object property
    sx127x_setSPI(*_spi, 0);
    sx127x_setPins(_nss);
}

void SX127x::setSPI(SPIClass &SpiObject, uint32_t frequency)
{
    sx127x_setSPI(SpiObject, frequency);

    _spi = &SpiObject;
}

void SX127x::setPins(int8_t nss, int8_t reset, int8_t irq, int8_t txen, int8_t rxen)
{
    sx127x_setPins(nss);

    _nss = nss;
    _reset = reset;
    _irq = irq;
    _txen = txen;
    _rxen = rxen;
    _irqStatic = digitalPinToInterrupt(_irq);
}

void SX127x::setCurrentProtection(uint8_t current)
{
    // calculate ocp trim
    uint8_t ocpTrim = 27;
    if (current <= 120) {
        ocpTrim = (current - 45) / 5;
    } else if (current <=240) {
        ocpTrim = (current + 30) / 10;
    }
    // set over current protection config
    sx127x_writeRegister(SX127X_REG_OCP, 0x20 | ocpTrim);
}

void SX127x::setOscillator(uint8_t option)
{
    uint8_t cfg = option == SX127X_OSC_TCXO ? SX127X_OSC_TCXO : SX127X_OSC_CRYSTAL;
    sx127x_writeRegister(SX127X_REG_TCXO, cfg);
}

uint8_t SX127x::getModem()
{
    uint8_t modem = sx127x_readRegister(SX127X_REG_OP_MODE);
    if (modem & 0x20) {
        return SX127X_OOK_MODEM;
    }
    return modem >> 7;
}

void SX127x::setModem(uint8_t modem)
{
    if (modem == SX127X_LORA_MODEM) _modem = SX127X_LONG_RANGE_MODE;
    else if (modem == SX127X_FSK_MODEM) _modem = SX127X_MODULATION_FSK;
    else _modem = SX127X_MODULATION_OOK;
    sleep();
    sx127x_writeRegister(SX127X_REG_OP_MODE, _modem);
    standby();
}

void SX127x::setFrequency(uint32_t frequency)
{
    _frequency = frequency;
    // calculate frequency
    uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
    sx127x_writeRegister(SX127X_REG_FRF_MSB, (uint8_t) (frf >> 16));
    sx127x_writeRegister(SX127X_REG_FRF_MID, (uint8_t) (frf >> 8));
    sx127x_writeRegister(SX127X_REG_FRF_LSB, (uint8_t) frf);
}

void SX127x::setTxPower(uint8_t txPower, uint8_t paPin)
{
    // maximum TX power is 20 dBm and 14 dBm for RFO pin
    if (txPower > 20) txPower = 20;
    else if (txPower > 14 && paPin == SX127X_TX_POWER_RFO) txPower = 14;

    uint8_t paConfig, outputPower;
    if (paPin == SX127X_TX_POWER_RFO) {
        // txPower = Pmax - (15 - outputPower)
        if (txPower == 14) {
            // max power (Pmax) 14.4 dBm
            paConfig = 0x60;
            outputPower = txPower + 1;
        } else {
            // max power (Pmax) 13.2 dBm
            paConfig = 0x40;
            outputPower = txPower + 2;
        }
    } else {
        paConfig = 0xC0;
        uint8_t paDac = 0x04;
        // txPower = 17 - (15 - outputPower)
        if (txPower > 17) {
            outputPower = 15;
            paDac = 0x07;
            setCurrentProtection(100);  // max current 100 mA
        } else {
            if (txPower < 2) txPower = 2;
            outputPower = txPower - 2;
            setCurrentProtection(140);  // max current 140 mA
        }
        // enable or disable +20 dBm option on PA_BOOST pin
        sx127x_writeRegister(SX127X_REG_PA_DAC, paDac);
    }
    // set PA config
    sx127x_writeRegister(SX127X_REG_PA_CONFIG, paConfig | outputPower);
}

void SX127x::setRxGain(uint8_t boost, uint8_t level)
{
    // valid RX gain level 0 - 6 (0 -> AGC on)
    if (level > 6) level = 6;
    // boost LNA and automatic gain controller config
    uint8_t LnaBoostHf = boost ? 0x03 : 0x00;
    uint8_t AgcOn = level == SX127X_RX_GAIN_AUTO ? 0x01 : 0x00;
    // set gain and boost LNA config
    sx127x_writeRegister(SX127X_REG_LNA, LnaBoostHf | (level << 5));
    // enable or disable AGC
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_3, AgcOn, 2, 1);
}

void SX127x::setLoRaModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro)
{
    setSpreadingFactor(sf);
    setBandwidth(bw);
    setCodeRate(cr);
    setLdroEnable(ldro);
}

void SX127x::setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq)
{
    setHeaderType(headerType);
    setPreambleLength(preambleLength);
    setPayloadLength(payloadLength);
    setCrcEnable(crcType);
    // setInvertIq(invertIq);
}

void SX127x::setSpreadingFactor(uint8_t sf)
{
    _sf = sf;
    // valid spreading factor is 6 - 12
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;
    // set appropriate signal detection optimize and threshold
    uint8_t optimize = sf == 6 ? 0x05 : 0x03;
    uint8_t threshold = sf == 6 ? 0x0C : 0x0A;
    sx127x_writeRegister(SX127X_REG_DETECTION_OPTIMIZE, optimize);
    sx127x_writeRegister(SX127X_REG_DETECTION_THRESHOLD, threshold);
    // set spreading factor config
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_2, sf, 4, 4);
}

void SX127x::setBandwidth(uint32_t bw)
{
    _bw = bw;
    uint8_t bwCfg;
    if (bw < 9100) bwCfg = 0;           // 7.8 kHz
    else if (bw < 13000) bwCfg = 1;     // 10.4 kHz
    else if (bw < 18200) bwCfg = 2;     // 15.6 kHz
    else if (bw < 26000) bwCfg = 3;     // 20.8 kHz
    else if (bw < 36500) bwCfg = 4;     // 31.25 kHz
    else if (bw < 52100) bwCfg = 5;     // 41.7 kHz
    else if (bw < 93800) bwCfg = 6;     // 62.5 kHz
    else if (bw < 187500) bwCfg = 7;    // 125 kHz
    else if (bw < 375000) bwCfg = 8;    // 250 kHz
    else bwCfg = 9;                     // 500 kHz
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_1, bwCfg, 4, 4);
}

void SX127x::setCodeRate(uint8_t cr)
{
    // valid code rate denominator is 5 - 8
    if (cr < 5) cr = 4;
    else if (cr > 8) cr = 8;
    uint8_t crCfg = cr - 4;
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_1, crCfg, 1, 3);
}

void SX127x::setLdroEnable(bool ldro)
{
    uint8_t ldroCfg = ldro ? 0x01 : 0x00;
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_3, ldroCfg, 3, 1);
}

void SX127x::setHeaderType(uint8_t headerType)
{
    _headerType = headerType;
    uint8_t headerTypeCfg = headerType == SX127X_HEADER_IMPLICIT ? SX127X_HEADER_IMPLICIT : SX127X_HEADER_EXPLICIT;
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_1, headerTypeCfg, 0, 1);
}

void SX127x::setPreambleLength(uint16_t preambleLength)
{
    sx127x_writeRegister(SX127X_REG_PREAMBLE_MSB, (uint8_t) (preambleLength >> 8));
    sx127x_writeRegister(SX127X_REG_PREAMBLE_LSB, (uint8_t) preambleLength);
}

void SX127x::setPayloadLength(uint8_t payloadLength)
{
    _payloadLength = payloadLength;
    sx127x_writeRegister(SX127X_REG_PAYLOAD_LENGTH, payloadLength);
}

void SX127x::setCrcEnable(bool crcType)
{
    uint8_t crcTypeCfg = crcType ? 0x01 : 0x00;
    sx127x_writeBits(SX127X_REG_MODEM_CONFIG_2, crcTypeCfg, 2, 1);
}

void SX127x::setInvertIq(bool invertIq)
{
    uint8_t invertIqCfg1 = invertIq ? 0x01 : 0x00;
    uint8_t invertIqCfg2 = invertIq ? 0x19 : 0x1D;
    sx127x_writeBits(SX127X_REG_INVERTIQ, invertIqCfg1, 0, 1);
    sx127x_writeBits(SX127X_REG_INVERTIQ, invertIqCfg1, 6, 1);
    sx127x_writeRegister(SX127X_REG_INVERTIQ2, invertIqCfg2);
}

void SX127x::setSyncWord(uint16_t syncWord)
{
    uint8_t sw = syncWord;
    // keep compatibility between 1 and 2 bytes synchronize word
    if (syncWord > 0xFF) {
        sw = ((syncWord >> 8) & 0xF0) | (syncWord & 0x0F);
    }
    sx127x_writeRegister(SX127X_REG_SYNC_WORD, sw);
}

void SX127x::beginPacket()
{
    // reset TX buffer base address, FIFO address pointer and payload length
    sx127x_writeRegister(SX127X_REG_FIFO_TX_BASE_ADDR, sx127x_readRegister(SX127X_REG_FIFO_ADDR_PTR));
    _payloadTxRx = 0;

    // set txen pin to high and rxen pin to low
    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, LOW);
        digitalWrite(_txen, HIGH);
        _pinToLow = _txen;
    }
}

bool SX127x::endPacket(uint32_t timeout)
{
    // skip to enter TX mode when previous TX operation incomplete
    if (sx127x_readRegister(SX127X_REG_OP_MODE) & 0x07 == SX127X_MODE_TX) return false;

    // clear IRQ flag from last TX or RX operation
    sx127x_writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);

    // set packet payload length
    sx127x_writeRegister(SX127X_REG_PAYLOAD_LENGTH, _payloadTxRx);

    // set status to TX wait
    _statusWait = SX127X_STATUS_TX_WAIT;
    _statusIrq = 0x00;

    // set device to transmit mode
    sx127x_writeRegister(SX127X_REG_OP_MODE, _modem | SX127X_MODE_TX);
    _transmitTime = millis();

    // set TX done interrupt on DIO0 and attach TX interrupt handler
    if (_irq != -1) {
        sx127x_writeRegister(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_TX_DONE);
        attachInterrupt(_irqStatic, SX127x::_interruptTx, RISING);
    }
    return true;
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
        sx127x_writeRegister(SX127X_REG_FIFO, data[i]);
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

bool SX127x::request(uint32_t timeout)
{
    // skip to enter RX mode when previous RX operation incomplete
    uint8_t rxMode = sx127x_readRegister(SX127X_REG_OP_MODE) & 0x07;
    if (rxMode == SX127X_MODE_RX_SINGLE || rxMode == SX127X_MODE_RX_CONTINUOUS) return false;

    // clear IRQ flag from last TX or RX operation
    sx127x_writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);

    // set txen pin to low and rxen pin to high
    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, HIGH);
        digitalWrite(_txen, LOW);
        _pinToLow = _rxen;
    }

    // set status to RX wait
    _statusWait = SX127X_STATUS_RX_WAIT;
    _statusIrq = 0x00;
    // select RX mode to RX continuous mode for RX single and continuos operation
    rxMode = SX127X_MODE_RX_CONTINUOUS;
    if (timeout == SX127X_RX_CONTINUOUS) {
        _statusWait = SX127X_STATUS_RX_CONTINUOUS;
    } else if (timeout > 0) {
        // Select RX mode to single mode for RX operation with timeout
        rxMode = SX127X_MODE_RX_SINGLE;
        // calculate and set symbol timeout
        uint16_t symbTimeout = (timeout * _bw / 1000) >> _sf; // devided by 1000, ms to s
        symbTimeout = symbTimeout < 0x03FF ? symbTimeout : 0x03FF;
        sx127x_writeBits(SX127X_REG_MODEM_CONFIG_2, (symbTimeout >> 8) & 0x03, 0, 2);
        sx127x_writeRegister(SX127X_REG_SYMB_TIMEOUT, symbTimeout);
    }

    // set device to receive mode
    sx127x_writeRegister(SX127X_REG_OP_MODE, _modem | rxMode);

    // set RX done interrupt on DIO0 and attach RX interrupt handler
    if (_irq != -1) {
        sx127x_writeRegister(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_RX_DONE);
        if (timeout == SX127X_RX_CONTINUOUS) {
            attachInterrupt(_irqStatic, SX127x::_interruptRxContinuous, RISING);
        } else {
            attachInterrupt(_irqStatic, SX127x::_interruptRx, RISING);
        }
    }
    return true;
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
        data[i] = sx127x_readRegister(SX127X_REG_FIFO);
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
    if (_statusIrq) return true;

    // wait transmit or receive process finish by checking interrupt status or IRQ status
    uint8_t irqFlag = 0x00;
    uint8_t irqFlagMask = _statusWait == SX127X_STATUS_TX_WAIT 
        ? SX127X_IRQ_TX_DONE 
        : SX127X_IRQ_RX_DONE | SX127X_IRQ_RX_TIMEOUT | SX127X_IRQ_CRC_ERR
    ;
    uint32_t t = millis();
    while (!(irqFlag & irqFlagMask) && _statusIrq == 0x00) {
        // only check IRQ status register for non interrupt operation
        if (_irq == -1) irqFlag = sx127x_readRegister(SX127X_REG_IRQ_FLAGS);
        // return when timeout reached
        if (millis() - t > timeout && timeout != 0) return false;
        yield();
    }

    if (_statusIrq) {
        // immediately return when interrupt signal hit
        return true;

    } else if (_statusWait == SX127X_STATUS_TX_WAIT) {
        // calculate transmit time and set back txen pin to low
        _transmitTime = millis() - _transmitTime;
        if (_txen != -1) digitalWrite(_txen, LOW);

    } else if (_statusWait == SX127X_STATUS_RX_WAIT) {
        // terminate receive mode by setting mode to standby
        standby();
        // set pointer to RX buffer base address and get packet payload length
        sx127x_writeRegister(SX127X_REG_FIFO_ADDR_PTR, sx127x_readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR));
        _payloadTxRx = sx127x_readRegister(SX127X_REG_RX_NB_BYTES);
        // set back rxen pin to low
        if (_rxen != -1) digitalWrite(_rxen, LOW);

    } else if (_statusWait == SX127X_STATUS_RX_CONTINUOUS) {
        // set pointer to RX buffer base address and get packet payload length
        sx127x_writeRegister(SX127X_REG_FIFO_ADDR_PTR, sx127x_readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR));
        _payloadTxRx = sx127x_readRegister(SX127X_REG_RX_NB_BYTES);
        // clear IRQ flag
        sx127x_writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);
    }

    // store IRQ status
    _statusIrq = irqFlag;
    return true;
}

uint8_t SX127x::status()
{
    // set back status IRQ for RX continuous operation
    uint8_t statusIrq = _statusIrq;
    if (_statusWait == SX127X_STATUS_RX_CONTINUOUS) {
        _statusIrq = 0x00;
    }

    // get status for transmit and receive operation based on status IRQ
    if (statusIrq & SX127X_IRQ_RX_TIMEOUT) return SX127X_STATUS_RX_TIMEOUT;
    else if (statusIrq & SX127X_IRQ_CRC_ERR) return SX127X_STATUS_CRC_ERR;
    else if (statusIrq & SX127X_IRQ_TX_DONE) return SX127X_STATUS_TX_DONE;
    else if (statusIrq & SX127X_IRQ_RX_DONE) return SX127X_STATUS_RX_DONE;

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
    if (sx127x_readRegister(SX127X_REG_VERSION) == 0x22) {
        offset = SX1272_RSSI_OFFSET;
    }
    return (int16_t) sx127x_readRegister(SX127X_REG_PKT_RSSI_VALUE) - offset;
}

int16_t SX127x::rssi()
{
    int16_t offset = _frequency < SX127X_BAND_THRESHOLD ? SX127X_RSSI_OFFSET_LF : SX127X_RSSI_OFFSET_HF;
    if (sx127x_readRegister(SX127X_REG_VERSION) == 0x22) {
        offset = SX1272_RSSI_OFFSET;
    }
    return (int16_t) sx127x_readRegister(SX127X_REG_RSSI_VALUE) - offset;
}

float SX127x::snr()
{
    // get signal to noise ratio (SNR) of last incoming package
    return (int8_t) sx127x_readRegister(SX127X_REG_PKT_SNR_VALUE) / 4.0;
}

uint32_t SX127x::random()
{
    // generate random number from register and previous random number
    uint32_t n = sx127x_readRegister(SX127X_REG_RSSI_WIDEBAND);
    uint32_t number = (n << 24) | ((~n & 0xFF) << 16) | (n << 8) | (~n & 0xFF);
    n = _random;
    number = number ^ ((~n << 16) | n);
    // combine random number with random number seeded by time
    srand(millis());
    number = number ^ (((uint32_t) rand() << 16) | rand());
    _random = number >> 8;
    return number;
}

void SX127x::_interruptTx()
{
    // calculate transmit time
    _transmitTime = millis() - _transmitTime;

    // store IRQ status as TX done
    _statusIrq = SX127X_IRQ_TX_DONE;

    // set back txen pin to low and detach interrupt
    if (_pinToLow != -1) digitalWrite(_pinToLow, LOW);
    detachInterrupt(_irqStatic);

    // call onTransmit function
    if (_onTransmit) {
        _onTransmit();
    }
}

void SX127x::_interruptRx()
{
    // store IRQ status
    _statusIrq = sx127x_readRegister(SX127X_REG_IRQ_FLAGS);
    // set IRQ status to RX done when interrupt occured before register updated
    if (!(_statusIrq & 0xF0)) _statusIrq = SX127X_IRQ_RX_DONE;

    // terminate receive mode by setting mode to standby
    sx127x_writeBits(SX127X_REG_OP_MODE, SX127X_MODE_STDBY, 0, 3);

    // set back rxen pin to low and detach interrupt
    if (_pinToLow != -1) digitalWrite(_pinToLow, LOW);
    detachInterrupt(_irqStatic);

    // set pointer to RX buffer base address and get packet payload length
    sx127x_writeRegister(SX127X_REG_FIFO_ADDR_PTR, sx127x_readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR));
    _payloadTxRx = sx127x_readRegister(SX127X_REG_RX_NB_BYTES);

    // call onReceive function
    if (_onReceive) {
        _onReceive();
    }
}

void SX127x::_interruptRxContinuous()
{
    // store IRQ status
    _statusIrq = sx127x_readRegister(SX127X_REG_IRQ_FLAGS);
    // set IRQ status to RX done when interrupt occured before register updated
    if (!(_statusIrq & 0xF0)) _statusIrq = SX127X_IRQ_RX_DONE;

    // clear IRQ flag from last TX or RX operation
    sx127x_writeRegister(SX127X_REG_IRQ_FLAGS, 0xFF);

    // set pointer to RX buffer base address and get packet payload length
    sx127x_writeRegister(SX127X_REG_FIFO_ADDR_PTR, sx127x_readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR));
    _payloadTxRx = sx127x_readRegister(SX127X_REG_RX_NB_BYTES);

    // call onReceive function
    if (_onReceive) {
        _onReceive();
    }
}

void SX127x::onTransmit(void(&callback)())
{
    // register onTransmit function to call every transmit done
    _onTransmit = &callback;
}

void SX127x::onReceive(void(&callback)())
{
    // register onReceive function to call every receive done
    _onReceive = &callback;
}
