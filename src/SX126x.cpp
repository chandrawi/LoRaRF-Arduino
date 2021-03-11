#include <SX126x.h>

uint8_t SX126x::_statusInterrupt = 0b00000000;

uint8_t SX126x::_bufferIndex = 0;

uint8_t SX126x::_payloadTxRx = 0;

int8_t SX126x::_pinToLow = -1;

uint32_t SX126x::_transmitTime = 0;

SX126x::SX126x()
{
    _dio = SX126X_PIN_RF_IRQ;
    setSPI(SX126X_SPI);
}

bool SX126x::begin()
{
    return begin(SX126X_PIN_NSS, SX126X_PIN_RESET, SX126X_PIN_BUSY, SX126X_PIN_IRQ);
}

bool SX126x::begin(int8_t nss, int8_t reset, int8_t busy, int8_t irq, int8_t txen, int8_t rxen)
{
    setPins(nss, reset, busy, irq, txen, rxen);
    _spi->begin();
    
    SX126x_API::reset(reset, busy);
    SX126x_API::setStandby(SX126X_STANDBY_RC);
    if (getMode() != SX126X_STATUS_MODE_STDBY_RC) return false;
    SX126x_API::setPacketType(SX126X_LORA_MODEM);
    
    SX126x_API::fixResistanceAntenna();
    return true;
}

void SX126x::end()
{
    sleep(SX126X_SLEEP_COLD_START);
    _spi->end();
}

void SX126x::reset()
{
    SX126x_API::reset(_reset, _busy);
}

void SX126x::sleep(uint8_t option)
{
    standby();
    SX126x_API::setSleep(option);
    delayMicroseconds(500);
}

void SX126x::wake()
{
    digitalWrite(_nss, LOW);
    standby();
    SX126x_API::fixResistanceAntenna();
}

void SX126x::standby(uint8_t option)
{
    SX126x_API::setStandby(option);
}

void SX126x::setActive()
{
    SX126x_API::usePins(_nss, _busy);
}

void SX126x::setFallbackMode(uint8_t fallbackMode)
{
    SX126x_API::setRxTxFallbackMode(fallbackMode);
}

uint8_t SX126x::getMode()
{
    uint8_t mode;
    SX126x_API::getStatus(&mode);
    return mode & 0x70;
}

void SX126x::setSPI(SPIClass &SpiObject)
{
    SX126x_API::setSPI(SpiObject);

    _spi = &SpiObject;
}

void SX126x::setPins(int8_t nss, int8_t reset, int8_t busy, int8_t irq, int8_t txen, int8_t rxen)
{
    SX126x_API::setPins(nss, reset, busy);
    
    _nss = nss;
    _reset = reset;
    _busy = busy;
    _irq = digitalPinToInterrupt(irq);
    _txen = txen;
    _rxen = rxen;
    if (_irq != -1) pinMode(irq, INPUT);
    if (txen != -1) pinMode(txen, OUTPUT);
    if (rxen != -1) pinMode(rxen, OUTPUT);
}

void SX126x::setRfIrqPin(int8_t dioPinSelect)
{
    if ((dioPinSelect == 2) || (dioPinSelect == 3)) _dio = dioPinSelect;
    else _dio = 1;
}

void SX126x::setDio2RfSwitch(bool enable)
{
    if (enable) SX126x_API::setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);
    else SX126x_API::setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_IRQ);
}

void SX126x::setDio3TcxoCtrl(uint8_t tcxoVoltage, uint32_t delayTime)
{
    SX126x_API::setDio3AsTcxoCtrl(tcxoVoltage, delayTime);
    SX126x_API::setStandby(SX126X_STANDBY_RC);
    SX126x_API::calibrate(0xFF);
}

void SX126x::setXtalCap(uint8_t xtalA, uint8_t xtalB)
{
    SX126x_API::setStandby(SX126X_STANDBY_XOSC);
    uint8_t buf[2] = {xtalA, xtalB};
    SX126x_API::writeRegister(SX126X_REG_XTA_TRIM, buf, 2);
    SX126x_API::setStandby(SX126X_STANDBY_RC);
    SX126x_API::calibrate(0xFF);
}

void SX126x::setRegulator(uint8_t regMode)
{
    SX126x_API::setRegulatorMode(regMode);
}

void SX126x::setCurrentProtection(uint8_t level)
{
    SX126x_API::writeRegister(SX126X_REG_OCP_CONFIGURATION, &level, 1);
}

void SX126x::setModem(uint8_t modem)
{
    _modem = modem;
    SX126x_API::setStandby(SX126X_STANDBY_RC);
    SX126x_API::setPacketType(modem);
}

void SX126x::setFrequency(uint32_t frequency)
{
    uint8_t calFreq[2];
    if (frequency < 446000000){
    calFreq[0] = 0x6B;
    calFreq[1] = 0x6F;
    }
    else if (frequency < 734000000){
    calFreq[0] = 0x75;
    calFreq[1] = 0x81;
    }
    else if (frequency < 828000000){
    calFreq[0] = 0xC1;
    calFreq[1] = 0xC5;
    }
    else if (frequency < 877000000){
    calFreq[0] = 0xD7;
    calFreq[1] = 0xDB;
    }
    else if (frequency < 1100000000){
    calFreq[0] = 0xE1;
    calFreq[1] = 0xE9;
    }
    uint32_t rfFreq = frequency * 33554432UL / 32000000UL;

    SX126x_API::calibrateImage(calFreq[0], calFreq[1]);
    SX126x_API::setRfFrequency(rfFreq);
}

void SX126x::setTxPower(uint32_t txPower)
{
    uint8_t power, ramp;
    switch (txPower){
        case SX126X_TX_POWER_SX1261_15:
            power = 0x0E;
            ramp = SX126X_PA_RAMP_200U;
            break;
        case SX126X_TX_POWER_SX1261_14:
            power = 0x0E;
            ramp = SX126X_PA_RAMP_200U;
            break;
        case SX126X_TX_POWER_SX1261_10:
            power = 0x0D;
            ramp = SX126X_PA_RAMP_80U;
            break;
        case SX126X_TX_POWER_SX1262_22: // also for SX126X_TX_POWER_SX1268_22
            power = 0x16;
            ramp = SX126X_PA_RAMP_800U;
            break;
        case SX126X_TX_POWER_SX1262_20: // also for SX126X_TX_POWER_SX1268_20
            power = 0x16;
            ramp = SX126X_PA_RAMP_800U;
            break;
        case SX126X_TX_POWER_SX1262_17: // also for SX126X_TX_POWER_SX1268_17
            power = 0x16;
            ramp = SX126X_PA_RAMP_200U;
            break;
        case SX126X_TX_POWER_SX1262_14:
            power = 0x16;
            ramp = SX126X_PA_RAMP_200U;
            break;
        case SX126X_TX_POWER_SX1268_14:
            power = 0x0F;
            ramp = SX126X_PA_RAMP_200U;
            break;
        case SX126X_TX_POWER_SX1268_10:
            power = 0x0F;
            ramp = SX126X_PA_RAMP_80U;
            break;
        default: return;
    }
    uint8_t paDutyCycle = (txPower >> 16) & 0xFF;
    uint8_t hpMax = (txPower >> 8) & 0xFF;
    uint8_t deviceSel = txPower & 0xFF;
    uint8_t paLut = 0x01;

    SX126x_API::setPaConfig(paDutyCycle, hpMax, deviceSel, paLut);
    SX126x_API::setTxParams(power, ramp);
}

void SX126x::setRxGain(uint8_t rxGain)
{
    uint8_t gain = SX126X_RX_GAIN_POWER_SAVING;
    if (rxGain == SX126X_RX_GAIN_BOOSTED){
        gain = SX126X_RX_GAIN_BOOSTED;
        SX126x_API::writeRegister(SX126X_REG_RX_GAIN, &gain, 1);
        uint8_t value = 0x01;
        SX126x_API::writeRegister(0x029F, &value, 1);
        value = 0x08;
        SX126x_API::writeRegister(0x02A0, &value, 1);
        value = 0xAC;
        SX126x_API::writeRegister(0x02A1, &value, 1);
    }
    else SX126x_API::writeRegister(SX126X_REG_RX_GAIN, &gain, 1);
}

void SX126x::setLoRaModulation(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    _sf = sf;
    _bw = bw;
    _cr = cr;
    _ldro = ldro;
    SX126x_API::setModulationParamsLoRa(sf, bw, cr, ldro);
}

void SX126x::setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, uint8_t crcType, uint8_t invertIq)
{
    _headerType = headerType;
    _preambleLength = preambleLength;
    _payloadLength = payloadLength;
    _crcType = crcType;
    _invertIq = invertIq;
    SX126x_API::setPacketParamsLoRa(preambleLength, headerType, payloadLength, crcType, invertIq);
    SX126x_API::fixInvertedIq(_invertIq);
}

void SX126x::setLoRaPayloadLength(uint8_t payloadLength)
{
    _payloadLength = payloadLength;
    SX126x_API::setPacketParamsLoRa(_preambleLength, _headerType, payloadLength, _crcType, _invertIq);
    SX126x_API::fixInvertedIq(_invertIq);
}

void SX126x::setLoRaSyncWord(uint16_t sw)
{
    uint8_t buf[2];
    buf[0] = sw >> 8;
    buf[1] = sw & 0xFF;
    SX126x_API::writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);
}

void SX126x::setFskModulation(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{
    SX126x_API::setModulationParamsFSK(br, pulseShape, bandwidth, Fdev);
}

void SX126x::setFskPacket(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    SX126x_API::setPacketParamsFSK(preambleLength, preambleDetector, syncWordLength, addrComp, packetType, payloadLength, crcType, whitening);
}

void SX126x::setFskSyncWord(uint8_t* sw, uint8_t swLen)
{
    SX126x_API::writeRegister(SX126X_REG_FSK_SYNC_WORD_0, sw, swLen);
}

void SX126x::setFskAdress(uint8_t nodeAddr, uint8_t broadcastAddr)
{
    uint8_t buf[2] = {nodeAddr, broadcastAddr};
    SX126x_API::writeRegister(SX126X_REG_FSK_NODE_ADDRESS, buf, 2);
}

void SX126x::setFskCrc(uint16_t crcInit, uint16_t crcPolynom)
{
    uint8_t buf[4];
    buf[0] = crcInit >> 8;
    buf[1] = crcInit & 0xFF;
    buf[2] = crcPolynom >> 8;
    buf[3] = crcPolynom & 0xFF;
    SX126x_API::writeRegister(SX126X_REG_FSK_CRC_INITIAL_MSB, buf, 4);
}

void SX126x::setFskWhitening(uint16_t whitening)
{
    uint8_t buf[2];
    buf[0] = whitening >> 8;
    buf[1] = whitening & 0xFF;
    SX126x_API::writeRegister(SX126X_REG_FSK_WHITENING_INITIAL_MSB, buf, 2);
}

void SX126x::beginPacket()
{
    _irqSetup(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);

    _payloadTxRx = 0;
    SX126x_API::setBufferBaseAddress(_bufferIndex, _bufferIndex + 0xFF);

    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, LOW);
        digitalWrite(_txen, HIGH);
        _pinToLow = _txen;
    }
    
    SX126x_API::fixLoRaBw500(_bw);
}

void SX126x::endPacket(uint32_t timeout)
{
    setLoRaPacket(_headerType, _preambleLength, _payloadTxRx, _crcType, _invertIq);
    
    _status = SX126X_STATUS_TX_WAIT;
    uint32_t txTimeout = timeout << 6;
    if (txTimeout > 0x00FFFFFF) txTimeout = SX126X_TX_MODE_SINGLE;
    
    SX126x_API::setTx(txTimeout);
    
    _transmitTime = micros();
    
    if (_irq != -1){
        _statusInterrupt = 0b00000000;
        attachInterrupt(_irq, SX126x::_interruptTx, RISING);
    }
    else {
        uint16_t irqStat = _waitIrq();
        
        _transmitTime = micros() - _transmitTime;
        
        if (_txen != -1) digitalWrite(_txen, LOW);
        
        if (irqStat & SX126X_IRQ_TIMEOUT) _status = SX126X_STATUS_TX_TIMEOUT;
        else _status = SX126X_STATUS_TX_DONE;
    }
}

void SX126x::write(uint8_t data)
{
    SX126x_API::writeBuffer(_bufferIndex, &data, 1);
    _bufferIndex++;
    _payloadTxRx++;
}

void SX126x::write(uint8_t* data, uint8_t length)
{
    SX126x_API::writeBuffer(_bufferIndex, data, length);
    _bufferIndex += length;
    _payloadTxRx += length;
}

void SX126x::write(char* data, uint8_t length)
{
    uint8_t* data_ = (uint8_t*) data;
    SX126x_API::writeBuffer(_bufferIndex, data_, length);
    _bufferIndex += length;
    _payloadTxRx += length;
}

void SX126x::request(uint32_t timeout)
{
    _irqSetup(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);
    
    _status = SX126X_STATUS_RX_WAIT;
    uint32_t rxTimeout = timeout << 6;
    if (rxTimeout > 0x00FFFFFF) rxTimeout = SX126X_RX_MODE_SINGLE;
    if (timeout == SX126X_RX_MODE_CONTINUOUS){
        rxTimeout = SX126X_RX_MODE_CONTINUOUS;
        _status = SX126X_STATUS_RX_CONTINUOUS_WAIT;
    }
    
    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, HIGH);
        digitalWrite(_txen, LOW);
        if (timeout == SX126X_RX_MODE_CONTINUOUS) _pinToLow = -1;
        else _pinToLow = _rxen;
    }
    
    SX126x_API::setRx(rxTimeout);
    
    if (_irq != -1){
        _statusInterrupt = 0b00000000;
        attachInterrupt(_irq, SX126x::_interruptRx, RISING);
    }
    else {
        uint16_t irqStat = _waitIrq();
        
        SX126x_API::getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
        
        if (_rxen != -1) digitalWrite(_rxen, LOW);
        
        if (irqStat & SX126X_IRQ_TIMEOUT) _status = SX126X_STATUS_RX_TIMEOUT;
        else if (irqStat & SX126X_IRQ_HEADER_ERR) _status = SX126X_STATUS_HEADER_ERR;
        else if (irqStat & SX126X_IRQ_CRC_ERR) _status = SX126X_STATUS_CRC_ERR;
        else _status = SX126X_STATUS_RX_DONE;
        
        SX126x_API::fixRxTimeout();
    }
}

void SX126x::listen(uint32_t rxPeriod, uint32_t sleepPeriod)
{
    _irqSetup(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);
    
    _status = SX126X_STATUS_RX_WAIT;
    rxPeriod = rxPeriod << 6;
    sleepPeriod = sleepPeriod << 6;
    if (rxPeriod > 0x00FFFFFF) rxPeriod = 0x00FFFFFF;
    if (sleepPeriod > 0x00FFFFFF) sleepPeriod = 0x00FFFFFF;
    
    if ((_rxen != -1) && (_txen != -1)){
        digitalWrite(_rxen, HIGH);
        digitalWrite(_txen, LOW);
        _pinToLow = _rxen;
    }
    
    SX126x_API::setRxDutyCycle(rxPeriod, sleepPeriod);
    
    if (_irq != -1){
        _statusInterrupt = 0b00000000;
        attachInterrupt(_irq, SX126x::_interruptRx, RISING);
    }
    else {
        uint16_t irqStat = _waitIrq();
        
        SX126x_API::getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
        
        if (_rxen != -1) digitalWrite(_rxen, LOW);
        
        if (irqStat & SX126X_IRQ_TIMEOUT) _status = SX126X_STATUS_RX_TIMEOUT;
        else if (irqStat & SX126X_IRQ_HEADER_ERR) _status = SX126X_STATUS_HEADER_ERR;
        else if (irqStat & SX126X_IRQ_CRC_ERR) _status = SX126X_STATUS_CRC_ERR;
        else _status = SX126X_STATUS_RX_DONE;
        
        SX126x_API::fixRxTimeout();
    }
}

uint8_t SX126x::available()
{
    return _payloadTxRx;
}

uint8_t SX126x::read()
{
    uint8_t buf;
    SX126x_API::readBuffer(_bufferIndex, &buf, 1);
    _bufferIndex++;
    if (_payloadTxRx > 0) _payloadTxRx--;
    return buf;
}

uint8_t SX126x::read(uint8_t* data, uint8_t length)
{
    SX126x_API::readBuffer(_bufferIndex, data, length);
    _bufferIndex += length;
    uint8_t len = length;
    if (_payloadTxRx > length){ _payloadTxRx -= length; }
    else { _payloadTxRx = 0; len = _payloadTxRx; }
    return len;
}

uint8_t SX126x::read(char* data, uint8_t length)
{
    uint8_t* data_ = (uint8_t*) data;
    SX126x_API::readBuffer(_bufferIndex, data_, length);
    _bufferIndex += length;
    uint8_t len = length;
    if (_payloadTxRx > length){ _payloadTxRx -= length; }
    else { _payloadTxRx = 0; len = _payloadTxRx; }
    return len;
}

void SX126x::flush()
{
    _bufferIndex += _payloadTxRx;
    _payloadTxRx = 0;
}

uint8_t SX126x::status()
{
    if (_status == SX126X_STATUS_RX_CONTINUOUS_WAIT) return _statusRxContinuous;
    if (_irq == -1) return _status;
    else return _getStatusInterrupt();
}

void SX126x::wait()
{
    if (_irq == -1) return;
    while (_getStatusInterrupt() == _status){
        if (_statusInterrupt != 0b00000000) break;
    }
    if (_status == SX126X_STATUS_RX_CONTINUOUS_WAIT){
        _statusRxContinuous = _getStatusInterrupt();
        _statusInterrupt = 0b00000000;
        SX126x_API::clearIrqStatus(0x03FF);
    }
}

uint32_t SX126x::transmitTime()
{
    wait();
    return _transmitTime / 1000;
}

float SX126x::dataRate()
{
    wait();
    return 1000000.0 * _payloadTxRx / _transmitTime;
}

float SX126x::rssi()
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    SX126x_API::getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (rssiPkt / -2.0);
}

float SX126x::snr()
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    SX126x_API::getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (snrPkt / 4.0);
}

float SX126x::signalRssi()
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    SX126x_API::getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (signalRssiPkt / -2.0);
}

float SX126x::rssiInst()
{
    uint8_t rssiInst;
    SX126x_API::getRssiInst(&rssiInst);
    return (rssiInst / -2.0);
}

uint16_t SX126x::getError()
{
    uint16_t error;
    SX126x_API::getDeviceErrors(&error);
    SX126x_API::clearDeviceErrors();
    return error;
}

void SX126x::_irqSetup(uint16_t irqMask)
{
    SX126x_API::clearIrqStatus(0x03FF);
    uint16_t dio1Mask = 0x0000;
    uint16_t dio2Mask = 0x0000;
    uint16_t dio3Mask = 0x0000;
    if (_dio == 2) dio2Mask = irqMask;
    else if (_dio == 3) dio3Mask = irqMask;
    else dio1Mask = irqMask;
    SX126x_API::setDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask);
}

uint16_t SX126x::_waitIrq()
{
    uint16_t irqStat = 0x0000;
    while (irqStat == 0x0000) SX126x_API::getIrqStatus(&irqStat);
    return irqStat;
}

void SX126x::_interruptTx()
{
    _transmitTime = micros() - _transmitTime;
    
    if (_pinToLow != -1) digitalWrite(_pinToLow, LOW);
    
    _statusInterrupt = 0b00001111;
}

void SX126x::_interruptRx()
{
    SX126x_API::getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
    
    if (_pinToLow != -1) digitalWrite(_pinToLow, LOW);
    
    _statusInterrupt = 0b11110000;
    
    SX126x_API::fixRxTimeout();
}

uint8_t SX126x::_getStatusInterrupt()
{
    uint16_t irqStat;
    if (_statusInterrupt == 0b00001111){
        SX126x_API::getIrqStatus(&irqStat);
        if (irqStat & SX126X_IRQ_TIMEOUT) return SX126X_STATUS_TX_TIMEOUT;
        else return SX126X_STATUS_TX_DONE;
    }
    else if (_statusInterrupt == 0b11110000){
        SX126x_API::getIrqStatus(&irqStat);
        if (irqStat & SX126X_IRQ_TIMEOUT) return SX126X_STATUS_RX_TIMEOUT;
        else if (irqStat & SX126X_IRQ_HEADER_ERR) return SX126X_STATUS_HEADER_ERR;
        else if (irqStat & SX126X_IRQ_CRC_ERR) return SX126X_STATUS_CRC_ERR;
        else return SX126X_STATUS_RX_DONE;
    }
    else return _status;
}
