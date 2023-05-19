#include <SX126x.h>

void (*SX126x::_onTransmit)();

void (*SX126x::_onReceive)();

volatile uint16_t SX126x::_statusIrq = 0xFFFF;

uint32_t SX126x::_transmitTime = 0;

uint8_t SX126x::_bufferIndex = 0;

uint8_t SX126x::_payloadTxRx = 0;

int8_t SX126x::_irqStatic = -1;

int8_t SX126x::_pinToLow = -1;

SX126x::SX126x()
{
    _spi = &SX126X_SPI;
    _dio = SX126X_PIN_RF_IRQ;
    setPins(SX126X_PIN_NSS, SX126X_PIN_RESET, SX126X_PIN_BUSY);
}

bool SX126x::begin()
{
    // set pins as input or output
    if (_irq != -1) pinMode(_irq, INPUT);
    if (_txen != -1) pinMode(_txen, OUTPUT);
    if (_rxen != -1) pinMode(_rxen, OUTPUT);

    // begin spi and perform device reset
    sx126x_begin();
    sx126x_reset(_reset);

    // check if device connect and set modem to LoRa
    sx126x_setStandby(SX126X_STANDBY_RC);
    if (getMode() != SX126X_STATUS_MODE_STDBY_RC) return false;
    sx126x_setPacketType(SX126X_LORA_MODEM);
    
    sx126x_fixResistanceAntenna();
    return true;
}

bool SX126x::begin(int8_t nss, int8_t reset, int8_t busy, int8_t irq, int8_t txen, int8_t rxen)
{
    setPins(nss, reset, busy, irq, txen, rxen);
    return begin();
}

void SX126x::end()
{
    sleep(SX126X_SLEEP_COLD_START);
    _spi->end();
}

bool SX126x::reset()
{
    // put reset pin to low then wait busy pin to low
    sx126x_reset(_reset);
    return !sx126x_busyCheck();
}

void SX126x::sleep(uint8_t option)
{
    // put device in sleep mode, wait for 500 us to enter sleep mode
    standby();
    sx126x_setSleep(option);
    delayMicroseconds(500);
}

void SX126x::wake()
{
    // wake device by set nss to low and put device in standby mode
    digitalWrite(_nss, LOW);
    standby();
    sx126x_fixResistanceAntenna();
}

void SX126x::standby(uint8_t option)
{
    sx126x_setStandby(option);
}

void SX126x::setActive()
{
    // override spi, nss, and busy static property in SX126x driver with object property
    sx126x_setSPI(*_spi, 0);
    sx126x_setPins(_nss, _busy);
}

bool SX126x::busyCheck(uint32_t timeout)
{
    return sx126x_busyCheck(timeout);
}

void SX126x::setFallbackMode(uint8_t fallbackMode)
{
    sx126x_setRxTxFallbackMode(fallbackMode);
}

uint8_t SX126x::getMode()
{
    uint8_t mode;
    sx126x_getStatus(&mode);
    return mode & 0x70;
}

void SX126x::setSPI(SPIClass &SpiObject, uint32_t frequency)
{
    sx126x_setSPI(SpiObject, frequency);

    _spi = &SpiObject;
}

void SX126x::setPins(int8_t nss, int8_t reset, int8_t busy, int8_t irq, int8_t txen, int8_t rxen)
{
    sx126x_setPins(nss, busy);

    _nss = nss;
    _reset = reset;
    _busy = busy;
    _irq = irq;
    _txen = txen;
    _rxen = rxen;
    _irqStatic = digitalPinToInterrupt(_irq);
}

void SX126x::setRfIrqPin(int8_t dioPinSelect)
{
    if ((dioPinSelect == 2) || (dioPinSelect == 3)) _dio = dioPinSelect;
    else _dio = 1;
}

void SX126x::setDio2RfSwitch(bool enable)
{
    if (enable) sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);
    else sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_IRQ);
}

void SX126x::setDio3TcxoCtrl(uint8_t tcxoVoltage, uint32_t delayTime)
{
    sx126x_setDio3AsTcxoCtrl(tcxoVoltage, delayTime);
    sx126x_setStandby(SX126X_STANDBY_RC);
    sx126x_calibrate(0xFF);
}

void SX126x::setXtalCap(uint8_t xtalA, uint8_t xtalB)
{
    sx126x_setStandby(SX126X_STANDBY_XOSC);
    uint8_t buf[2] = {xtalA, xtalB};
    sx126x_writeRegister(SX126X_REG_XTA_TRIM, buf, 2);
    sx126x_setStandby(SX126X_STANDBY_RC);
    sx126x_calibrate(0xFF);
}

void SX126x::setRegulator(uint8_t regMode)
{
    sx126x_setRegulatorMode(regMode);
}

void SX126x::setCurrentProtection(uint8_t current)
{
    uint8_t currentmA = current * 2 / 5;
    sx126x_writeRegister(SX126X_REG_OCP_CONFIGURATION, &currentmA, 1);
}

uint8_t SX126x::getModem()
{
    uint8_t modem;
    sx126x_getPacketType(&modem);
    return modem;
}

void SX126x::setModem(uint8_t modem)
{
    _modem = modem;
    sx126x_setStandby(SX126X_STANDBY_RC);
    sx126x_setPacketType(modem);
}

void SX126x::setFrequency(uint32_t frequency)
{
    uint8_t calFreq[2];
    if (frequency < 446000000) {        // 430 - 440 Mhz
        calFreq[0] = SX126X_CAL_IMG_430;
        calFreq[1] = SX126X_CAL_IMG_440;
    }
    else if (frequency < 734000000) {   // 470 - 510 Mhz
        calFreq[0] = SX126X_CAL_IMG_470;
        calFreq[1] = SX126X_CAL_IMG_510;
    }
    else if (frequency < 828000000) {   // 779 - 787 Mhz
        calFreq[0] = SX126X_CAL_IMG_779;
        calFreq[1] = SX126X_CAL_IMG_787;
    }
    else if (frequency < 877000000) {   // 863 - 870 Mhz
        calFreq[0] = SX126X_CAL_IMG_863;
        calFreq[1] = SX126X_CAL_IMG_870;
    }
    else if (frequency < 1100000000) {  // 902 - 928 Mhz
        calFreq[0] = SX126X_CAL_IMG_902;
        calFreq[1] = SX126X_CAL_IMG_928;
    }
    // calculate frequency for setting configuration
    uint32_t rfFreq = ((uint64_t) frequency << SX126X_RF_FREQUENCY_SHIFT) / SX126X_RF_FREQUENCY_XTAL;

    // perform image calibration before set frequency
    sx126x_calibrateImage(calFreq[0], calFreq[1]);
    sx126x_setRfFrequency(rfFreq);
}

void SX126x::setTxPower(uint8_t txPower, uint8_t version)
{
    // maximum TX power is 22 dBm and 15 dBm for SX1261
    if (txPower > 22) txPower = 22;
    else if (txPower > 15 && version == SX126X_TX_POWER_SX1261) txPower = 15;

    uint8_t paDutyCycle = 0x00;
    uint8_t hpMax = 0x00;
    uint8_t deviceSel = version == SX126X_TX_POWER_SX1261 ? 0x01 : 0x00;
    uint8_t power = 0x0E;
    // set parameters for PA config and TX params configuration
    if (txPower == 22) {
        paDutyCycle = 0x04;
        hpMax = 0x07;
        power = 0x16;
    } else if (txPower >= 20) {
        paDutyCycle = 0x03;
        hpMax = 0x05;
        power = 0x16;
    } else if (txPower >= 17) {
        paDutyCycle = 0x02;
        hpMax = 0x03;
        power = 0x16;
    } else if (txPower >= 14 && version == SX126X_TX_POWER_SX1261) {
        paDutyCycle = 0x04;
        hpMax = 0x00;
        power = 0x0E;
    } else if (txPower >= 14 && version == SX126X_TX_POWER_SX1262) {
        paDutyCycle = 0x02;
        hpMax = 0x02;
        power = 0x16;
    } else if (txPower >= 14 && version == SX126X_TX_POWER_SX1268) {
        paDutyCycle = 0x04;
        hpMax = 0x06;
        power = 0x0F;
    } else if (txPower >= 10 && version == SX126X_TX_POWER_SX1261) {
        paDutyCycle = 0x01;
        hpMax = 0x00;
        power = 0x0D;
    } else if (txPower >= 10 && version == SX126X_TX_POWER_SX1268) {
        paDutyCycle = 0x00;
        hpMax = 0x03;
        power = 0x0F;
    } else {
        return;
    }

    // set power amplifier and TX power configuration
    sx126x_setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01);
    sx126x_setTxParams(power, SX126X_PA_RAMP_800U);
}

void SX126x::setRxGain(uint8_t boost)
{
    // set power saving or boosted gain in register
    uint8_t gain = boost ? SX126X_BOOSTED_GAIN : SX126X_POWER_SAVING_GAIN;
    sx126x_writeRegister(SX126X_REG_RX_GAIN, &gain, 1);
    if (boost){
        // set certain register to retain configuration after wake from sleep mode
        uint8_t buf[3] = {0x01, 0x08, 0xAC};
        sx126x_writeRegister(0x029F, buf, 3);
    }
}

void SX126x::setLoRaModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro)
{
    _sf = sf;
    _bw = bw;
    _cr = cr;
    _ldro = ldro;

    // valid spreading factor is between 5 and 12
    if (sf > 12) sf = 12;
    else if (sf < 5) sf = 5;
    // select bandwidth options
    if (bw < 9100) bw = SX126X_BW_7800;             // 7.8 kHz
    else if (bw < 13000) bw = SX126X_BW_10400;      // 10.4 kHz
    else if (bw < 18200) bw = SX126X_BW_15600;      // 15.6 kHz
    else if (bw < 26000) bw = SX126X_BW_20800;      // 20.8 kHz
    else if (bw < 36500) bw = SX126X_BW_31250;      // 31.25 kHz
    else if (bw < 52100) bw = SX126X_BW_41700;      // 41.7 kHz
    else if (bw < 93800) bw = SX126X_BW_62500;      // 62.5 kHz
    else if (bw < 187500) bw = SX126X_BW_125000;    // 125 kHz
    else if (bw < 375000) bw = SX126X_BW_250000;    // 250 kHz
    else bw = SX126X_BW_500000;                     // 500 kHz
    // valid code rate denominator is between 4 and 8
    cr -= 4;
    if (cr > 4) cr = 0;

    sx126x_setModulationParamsLoRa(sf, (uint8_t) bw, cr, (uint8_t) ldro);
}

void SX126x::setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq)
{
    _headerType = headerType;
    _preambleLength = preambleLength;
    _payloadLength = payloadLength;
    _crcType = crcType;
    _invertIq = invertIq;

    // filter valid header type config
    if (headerType != SX126X_HEADER_IMPLICIT) headerType = SX126X_HEADER_EXPLICIT;

    sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, (uint8_t) crcType, (uint8_t) invertIq);
    sx126x_fixInvertedIq((uint8_t) invertIq);
}

void SX126x::setSpreadingFactor(uint8_t sf)
{
    setLoRaModulation(sf, _bw, _cr, _ldro);
}

void SX126x::setBandwidth(uint32_t bw)
{
    setLoRaModulation(_sf, bw, _cr, _ldro);
}

void SX126x::setCodeRate(uint8_t cr)
{
    setLoRaModulation(_sf, _bw, cr, _ldro);
}

void SX126x::setLdroEnable(bool ldro)
{
    setLoRaModulation(_sf, _bw, _cr, ldro);
}

void SX126x::setHeaderType(uint8_t headerType)
{
    setLoRaPacket(headerType, _preambleLength, _payloadLength, _crcType, _invertIq);
}

void SX126x::setPreambleLength(uint16_t preambleLength)
{
    setLoRaPacket(_headerType, preambleLength, _payloadLength, _crcType, _invertIq);
}

void SX126x::setPayloadLength(uint8_t payloadLength)
{
    setLoRaPacket(_headerType, _preambleLength, payloadLength, _crcType, _invertIq);
}

void SX126x::setCrcEnable(bool crcType)
{
    setLoRaPacket(_headerType, _preambleLength, _payloadLength, crcType, _invertIq);
}

void SX126x::setInvertIq(bool invertIq)
{
    setLoRaPacket(_headerType, _preambleLength, _payloadLength, _crcType, invertIq);
}

void SX126x::setSyncWord(uint16_t syncWord)
{
    uint8_t buf[2];
    buf[0] = syncWord >> 8;
    buf[1] = syncWord & 0xFF;
    if (syncWord <= 0xFF) {
        buf[0] = (syncWord & 0xF0) | 0x04;
        buf[1] = (syncWord << 4) | 0x04;
    }
    sx126x_writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);
}

void SX126x::setFskModulation(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{
    sx126x_setModulationParamsFSK(br, pulseShape, bandwidth, Fdev);
}

void SX126x::setFskPacket(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    sx126x_setPacketParamsFSK(preambleLength, preambleDetector, syncWordLength, addrComp, packetType, payloadLength, crcType, whitening);
}

void SX126x::setFskSyncWord(uint8_t* sw, uint8_t swLen)
{
    sx126x_writeRegister(SX126X_REG_FSK_SYNC_WORD_0, sw, swLen);
}

void SX126x::setFskAdress(uint8_t nodeAddr, uint8_t broadcastAddr)
{
    uint8_t buf[2] = {nodeAddr, broadcastAddr};
    sx126x_writeRegister(SX126X_REG_FSK_NODE_ADDRESS, buf, 2);
}

void SX126x::setFskCrc(uint16_t crcInit, uint16_t crcPolynom)
{
    uint8_t buf[4];
    buf[0] = crcInit >> 8;
    buf[1] = crcInit & 0xFF;
    buf[2] = crcPolynom >> 8;
    buf[3] = crcPolynom & 0xFF;
    sx126x_writeRegister(SX126X_REG_FSK_CRC_INITIAL_MSB, buf, 4);
}

void SX126x::setFskWhitening(uint16_t whitening)
{
    uint8_t buf[2];
    buf[0] = whitening >> 8;
    buf[1] = whitening & 0xFF;
    sx126x_writeRegister(SX126X_REG_FSK_WHITENING_INITIAL_MSB, buf, 2);
}

void SX126x::beginPacket()
{
    // reset payload length and buffer index
    _payloadTxRx = 0;
    sx126x_setBufferBaseAddress(_bufferIndex, _bufferIndex + 0xFF);

    // set txen pin to low and rxen pin to high
    if ((_rxen != -1) && (_txen != -1)) {
        digitalWrite(_rxen, LOW);
        digitalWrite(_txen, HIGH);
        _pinToLow = _txen;
    }
    
    sx126x_fixLoRaBw500(_bw);
}

bool SX126x::endPacket(uint32_t timeout)
{
    // skip to enter TX mode when previous TX operation incomplete
    if (getMode() == SX126X_STATUS_MODE_TX) return false;

    // clear previous interrupt and set TX done, and TX timeout as interrupt source
    _irqSetup(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);

    // set packet payload length
    setLoRaPacket(_headerType, _preambleLength, _payloadTxRx, _crcType, _invertIq);

    // set status to TX wait
    _statusWait = SX126X_STATUS_TX_WAIT;
    _statusIrq = 0x0000;
    // calculate TX timeout config
    uint32_t txTimeout = timeout << 6;
    if (txTimeout > 0x00FFFFFF) txTimeout = SX126X_TX_SINGLE;

    // set device to transmit mode with configured timeout or single operation
    sx126x_setTx(txTimeout);
    _transmitTime = millis();

    // set operation status to wait and attach TX interrupt handler
    if (_irq != -1) {
        attachInterrupt(_irqStatic, SX126x::_interruptTx, RISING);
    }
    return true;
}

void SX126x::write(uint8_t data)
{
    // write single byte of package to be transmitted
    sx126x_writeBuffer(_bufferIndex, &data, 1);
    _bufferIndex++;
    _payloadTxRx++;
}

void SX126x::write(uint8_t* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted
    sx126x_writeBuffer(_bufferIndex, data, length);
    _bufferIndex += length;
    _payloadTxRx += length;
}

void SX126x::write(char* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted for char type
    uint8_t* data_ = (uint8_t*) data;
    sx126x_writeBuffer(_bufferIndex, data_, length);
    _bufferIndex += length;
    _payloadTxRx += length;
}

bool SX126x::request(uint32_t timeout)
{
    // skip to enter RX mode when previous RX operation incomplete
    if (getMode() == SX126X_STATUS_MODE_RX) return false;

    // clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
    _irqSetup(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);

    // set status to RX wait or RX continuous wait
    _statusWait = SX126X_STATUS_RX_WAIT;
    _statusIrq = 0x0000;
    // calculate RX timeout config
    uint32_t rxTimeout = timeout << 6;
    if (rxTimeout > 0x00FFFFFF) rxTimeout = SX126X_RX_SINGLE;
    if (timeout == SX126X_RX_CONTINUOUS) {
        rxTimeout = SX126X_RX_CONTINUOUS;
        _statusWait = SX126X_STATUS_RX_CONTINUOUS;
    }

    // set txen pin to low and rxen pin to high
    if ((_rxen != -1) && (_txen != -1)) {
        digitalWrite(_rxen, HIGH);
        digitalWrite(_txen, LOW);
        _pinToLow = _rxen;
    }

    // set device to receive mode with configured timeout, single, or continuous operation
    sx126x_setRx(rxTimeout);

    // set operation status to wait and attach RX interrupt handler
    if (_irq != -1) {
        if (timeout == SX126X_RX_CONTINUOUS) {
            attachInterrupt(_irqStatic, SX126x::_interruptRxContinuous, RISING);
        } else {
            attachInterrupt(_irqStatic, SX126x::_interruptRx, RISING);
        }
    }
    return true;
}

bool SX126x::listen(uint32_t rxPeriod, uint32_t sleepPeriod)
{
    // skip to enter RX mode when previous RX operation incomplete
    if (getMode() == SX126X_STATUS_MODE_RX) return false;

    // clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
    _irqSetup(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);

    // set status to RX wait
    _statusWait = SX126X_STATUS_RX_WAIT;
    _statusIrq = 0x0000;
    // calculate RX period and sleep period config
    rxPeriod = rxPeriod << 6;
    sleepPeriod = sleepPeriod << 6;
    if (rxPeriod > 0x00FFFFFF) rxPeriod = 0x00FFFFFF;
    if (sleepPeriod > 0x00FFFFFF) sleepPeriod = 0x00FFFFFF;

    // set txen pin to low and rxen pin to high
    if ((_rxen != -1) && (_txen != -1)) {
        digitalWrite(_rxen, HIGH);
        digitalWrite(_txen, LOW);
        _pinToLow = _rxen;
    }

    // set device to receive mode with configured receive and sleep period
    sx126x_setRxDutyCycle(rxPeriod, sleepPeriod);

    // set operation status to wait and attach RX interrupt handler
    if (_irq != -1) {
        attachInterrupt(_irqStatic, SX126x::_interruptRx, RISING);
    }
    return true;
}

uint8_t SX126x::available()
{
    // get size of package still available to read
    return _payloadTxRx;
}

uint8_t SX126x::read()
{
    // read single byte of received package
    uint8_t buf;
    sx126x_readBuffer(_bufferIndex, &buf, 1);
    _bufferIndex++;
    if (_payloadTxRx > 0) _payloadTxRx--;
    return buf;
}

uint8_t SX126x::read(uint8_t* data, uint8_t length)
{
    // read multiple bytes of received package
    sx126x_readBuffer(_bufferIndex, data, length);
    // return smallest between read length and size of package available
    _bufferIndex += length;
    _payloadTxRx = _payloadTxRx > length ? _payloadTxRx - length : 0;
    return _payloadTxRx > length ? length : _payloadTxRx;
}

uint8_t SX126x::read(char* data, uint8_t length)
{
    // read multiple bytes of received package for char type
    uint8_t* data_ = (uint8_t*) data;
    sx126x_readBuffer(_bufferIndex, data_, length);
    _bufferIndex += length;
    _payloadTxRx = _payloadTxRx > length ? _payloadTxRx - length : 0;
    return _payloadTxRx > length ? length : _payloadTxRx;
}

void SX126x::purge(uint8_t length)
{
    // subtract or reset received payload length
    _payloadTxRx = (_payloadTxRx > length) && length ? _payloadTxRx - length : 0;
    _bufferIndex += length;
}

bool SX126x::wait(uint32_t timeout)
{
    // immediately return when currently not waiting transmit or receive process
    if (_statusIrq) return true;

    // wait transmit or receive process finish by checking interrupt status or IRQ status
    uint16_t irqStat = 0x0000;
    uint32_t t = millis();
    while (irqStat == 0x0000 && _statusIrq == 0x0000) {
        // only check IRQ status register for non interrupt operation
        if (_irq == -1) sx126x_getIrqStatus(&irqStat);
        // return when timeout reached
        if (millis() - t > timeout && timeout != 0) return false;
        yield();
    }

    if (_statusIrq) {
        // immediately return when interrupt signal hit
        return true;
    } else if (_statusWait == SX126X_STATUS_TX_WAIT) {
        // for transmit, calculate transmit time and set back txen pin to low
        _transmitTime = millis() - _transmitTime;
        if (_txen != -1) digitalWrite(_txen, LOW);
    } else if (_statusWait == SX126X_STATUS_RX_WAIT) {
        // for receive, get received payload length and buffer index and set back rxen pin to low
        sx126x_getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
        if (_rxen != -1) digitalWrite(_rxen, LOW);
        sx126x_fixRxTimeout();
    } else if (_statusWait == SX126X_STATUS_RX_CONTINUOUS) {
        // for receive continuous, get received payload length and buffer index and clear IRQ status
        sx126x_getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
        sx126x_clearIrqStatus(0x03FF);
    }

    // store IRQ status
    _statusIrq = irqStat;
    return true;
}

uint8_t SX126x::status()
{
    // set back status IRQ for RX continuous operation
    uint16_t statusIrq = _statusIrq;
    if (_statusWait == SX126X_STATUS_RX_CONTINUOUS) {
        _statusIrq = 0x0000;
    }

    // get status for transmit and receive operation based on status IRQ
    if (statusIrq & SX126X_IRQ_TIMEOUT) {
        if (_statusWait == SX126X_STATUS_TX_WAIT) return SX126X_STATUS_TX_TIMEOUT;
        else return SX126X_STATUS_RX_TIMEOUT;
    }
    else if (statusIrq & SX126X_IRQ_HEADER_ERR) return SX126X_STATUS_HEADER_ERR;
    else if (statusIrq & SX126X_IRQ_CRC_ERR) return SX126X_STATUS_CRC_ERR;
    else if (statusIrq & SX126X_IRQ_TX_DONE) return SX126X_STATUS_TX_DONE;
    else if (statusIrq & SX126X_IRQ_RX_DONE) return SX126X_STATUS_RX_DONE;

    // return TX or RX wait status
    return _statusWait;
}

uint32_t SX126x::transmitTime()
{
    // get transmit time in millisecond (ms)
    return _transmitTime;
}

float SX126x::dataRate()
{
    // get data rate last transmitted package in kbps
    return 1000.0 * _payloadTxRx / _transmitTime;
}

int16_t SX126x::packetRssi()
{
    // get relative signal strength index (RSSI) of last incoming package
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (rssiPkt / -2);
}

float SX126x::snr()
{
    // get signal to noise ratio (SNR) of last incoming package
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return ((int8_t) snrPkt / 4.0);
}

int16_t SX126x::signalRssi()
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (signalRssiPkt / -2);
}

int16_t SX126x::rssiInst()
{
    uint8_t rssiInst;
    sx126x_getRssiInst(&rssiInst);
    return (rssiInst / -2);
}

uint16_t SX126x::getError()
{
    uint16_t error;
    sx126x_getDeviceErrors(&error);
    sx126x_clearDeviceErrors();
    return error;
}

uint32_t SX126x::random()
{
    // generate random number from register and previous random number
    uint8_t buf[4];
    sx126x_readRegister(SX126X_REG_RANDOM_NUMBER_GEN, buf, 4);
    uint32_t number = ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint32_t) buf[2] << 8) | ((uint32_t) buf[0]);
    uint32_t n = _random;
    number = number ^ ((~n << 16) | n);
    // combine random number with random number seeded by time
    srand(millis());
    number = number ^ (((uint32_t) rand() << 16) | rand());
    _random = number >> 8;
    return number;
}

void SX126x::_irqSetup(uint16_t irqMask)
{
    // clear IRQ status of previous transmit or receive operation
    sx126x_clearIrqStatus(0x03FF);

    // set selected interrupt source
    uint16_t dio1Mask = 0x0000;
    uint16_t dio2Mask = 0x0000;
    uint16_t dio3Mask = 0x0000;
    if (_dio == 2) dio2Mask = irqMask;
    else if (_dio == 3) dio3Mask = irqMask;
    else dio1Mask = irqMask;
    sx126x_setDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask);
}

void SX126x::_interruptTx()
{
    // calculate transmit time
    _transmitTime = millis() - _transmitTime;

    // set back txen pin to low and detach interrupt
    if (_pinToLow != -1) digitalWrite(_pinToLow, LOW);
    detachInterrupt(_irqStatic);

    // store IRQ status
    uint16_t buf;
    sx126x_getIrqStatus(&buf);
    _statusIrq = buf;

    // call onTransmit function
    if (_onTransmit) {
        _onTransmit();
    }
}

void SX126x::_interruptRx()
{
    // set back rxen pin to low and detach interrupt
    if (_pinToLow != -1) digitalWrite(_pinToLow, LOW);
    detachInterrupt(_irqStatic);
    sx126x_fixRxTimeout();

    // store IRQ status
    uint16_t buf;
    sx126x_getIrqStatus(&buf);
    _statusIrq = buf;

    // get received payload length and buffer index
    sx126x_getRxBufferStatus(&_payloadTxRx, &_bufferIndex);

    // call onReceive function
    if (_onReceive) {
        _onReceive();
    }
}

void SX126x::_interruptRxContinuous()
{
    // store IRQ status
    uint16_t buf;
    sx126x_getIrqStatus(&buf);
    _statusIrq = buf;

    // clear IRQ status
    sx126x_clearIrqStatus(0x03FF);

    // get received payload length and buffer index
    sx126x_getRxBufferStatus(&_payloadTxRx, &_bufferIndex);

    // call onReceive function
    if (_onReceive) {
        _onReceive();
    }
}

void SX126x::onTransmit(void(&callback)())
{
    // register onTransmit function to call every transmit done
    _onTransmit = &callback;
}

void SX126x::onReceive(void(&callback)())
{
    // register onReceive function to call every receive done
    _onReceive = &callback;
}
