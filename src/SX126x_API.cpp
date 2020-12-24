#include <SX126x_API.h>

SPIClass* SX126x_API::_spi = &SX126X_SPI;

int8_t SX126x_API::_nss = SX126X_PIN_NSS;

int8_t SX126x_API::_reset = SX126X_PIN_RESET;

int8_t SX126x_API::_busy = SX126X_PIN_BUSY;

void SX126x_API::setSPI(SPIClass &SpiObject)
{
    _spi = &SpiObject;
}

void SX126x_API::setPins(int8_t nss, int8_t reset, int8_t busy)
{
    _nss = nss;
    _reset = reset;
    _busy = busy;
    pinMode(_nss, OUTPUT);
    pinMode(_reset, OUTPUT);
    pinMode(_busy, INPUT);
}

void SX126x_API::usePins(int8_t nss, int8_t busy)
{
    _nss = nss;
    _busy = busy;
}

void SX126x_API::reset(int8_t reset, int8_t busy)
{
    digitalWrite(reset, LOW);
    delay(1);
    digitalWrite(reset, HIGH);
    while (digitalRead(busy) == HIGH);
}

void SX126x_API::setSleep(uint8_t sleepConfig)
{
    if (sleepConfig > 7) return;
    if (sleepConfig & 0b00000010) return;
    _writeBytes(0x84, &sleepConfig, 1);
}

void SX126x_API::setStandby(uint8_t standbyMode)
{
    if (standbyMode > 1) return;
    _writeBytes(0x80, &standbyMode, 1);
}

void SX126x_API::setFs()
{
    _writeBytes(0xC1, NULL, 0);
}

void SX126x_API::setTx(uint32_t timeout)
{
    uint8_t buf[3];
    buf[0] = (timeout >> 16) & 0xFF;
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;
    _writeBytes(0x83, buf, 3);
}

void SX126x_API::setRx(uint32_t timeout)
{
    uint8_t buf[3];
    buf[0] = (timeout >> 16) & 0xFF;
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;
    _writeBytes(0x82, buf, 3);
}

void SX126x_API::stopTimerOnPreamble(uint8_t enable)
{
    if (enable > 0x01) return;
    _writeBytes(0x9F, &enable, 1);
}

void SX126x_API::setRxDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod)
{
    uint8_t buf[6];
    buf[0] = (rxPeriod >> 16) & 0xFF;
    buf[1] = (rxPeriod >> 8) & 0xFF;
    buf[2] = rxPeriod & 0xFF;
    buf[3] = (sleepPeriod >> 16) & 0xFF;
    buf[4] = (sleepPeriod >> 8) & 0xFF;
    buf[5] = sleepPeriod & 0xFF;
    _writeBytes(0x94, buf, 6);
}

void SX126x_API::setCad()
{
    _writeBytes(0xC5, NULL, 0);
}

void SX126x_API::setTxContinuousWave()
{
    _writeBytes(0xD1, NULL, 0);
}

void SX126x_API::setTxInfinitePreamble()
{
    _writeBytes(0xD2, NULL, 0);
}

void SX126x_API::setRegulatorMode(uint8_t modeParam)
{
    if (modeParam > 0x01) return;
    _writeBytes(0x96, &modeParam, 1);
}

void SX126x_API::calibrate(uint8_t calibParam)
{
    if (calibParam > 0x7F) return;
    _writeBytes(0x89, &calibParam, 1);
}

void SX126x_API::calibrateImage(uint8_t freq1, uint8_t freq2)
{
    uint8_t buf[2];
    buf[0] = freq1;
    buf[1] = freq2;
    _writeBytes(0x98, buf, 2);
}

void SX126x_API::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    if (deviceSel > 0x01) return;
    if (paLut != 0x01) return;
    if (hpMax > 7) return;
    if (paDutyCycle > 7) return;
    if ((deviceSel == 0) && (paDutyCycle > 4)) return;
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    _writeBytes(0x95, buf, 4);
}

void SX126x_API::setRxTxFallbackMode(uint8_t fallbackMode)
{
    if ((fallbackMode != 0x40) || (fallbackMode != 0x30) || (fallbackMode != 0x20)) return;
    _writeBytes(0x93, &fallbackMode, 1);
}

void SX126x_API::writeRegister(uint16_t address, uint8_t* data, uint8_t nData)
{
    uint8_t nBuf = nData + 2;
    uint8_t buf[nBuf];
    buf[0] = address >> 8;
    buf[1] = address & 0xFF;
    for(uint8_t i=0; i<nData; i++) buf[i + 2] = data[i];
    _writeBytes(0x0D, buf, nBuf);
}

void SX126x_API::readRegister(uint16_t address, uint8_t* data, uint8_t nData)
{
    uint8_t nBuf = nData + 1;
    uint8_t buf[nBuf];
    uint8_t addr[2];
    addr[0] = address >> 8;
    addr[1] = address & 0xFF;
    _readBytes(0x1D, buf, nBuf, addr, 2);
    for(uint8_t i=0; i<nData; i++) data[i] = buf[i + 1];
}

void SX126x_API::writeBuffer(uint8_t offset, uint8_t* data, uint8_t nData)
{
    uint8_t nBuf = nData + 1;
    uint8_t buf[nBuf];
    buf[0] = offset;
    for(uint8_t i=0; i<nData; i++) buf[i + 1] = data[i];
    _writeBytes(0x0E, buf, nBuf);
}

void SX126x_API::readBuffer(uint8_t offset, uint8_t* data, uint8_t nData)
{
    uint8_t nBuf = nData + 1;
    uint8_t buf[nBuf];
    _readBytes(0x1E, buf, nBuf, &offset, 1);
    for(uint8_t i=0; i<nData; i++) data[i] = buf[i + 1];
}

void SX126x_API::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];
    buf[0] = irqMask >> 8;
    buf[1] = irqMask & 0xFF;
    buf[2] = dio1Mask >> 8;
    buf[3] = dio1Mask & 0xFF;
    buf[4] = dio2Mask >> 8;
    buf[5] = dio2Mask & 0xFF;
    buf[6] = dio3Mask >> 8;
    buf[7] = dio3Mask & 0xFF;
    _writeBytes(0x08, buf, 8);
}

void SX126x_API::getIrqStatus(uint16_t* irqStatus)
{
    uint8_t buf[3];
    _readBytes(0x12, buf, 3);
    *irqStatus = (buf[1] << 8) | buf[2];
}

void SX126x_API::clearIrqStatus(uint16_t clearIrqParam)
{
    uint8_t buf[2];
    buf[0] = clearIrqParam >> 8;
    buf[1] = clearIrqParam & 0xFF;
    _writeBytes(0x02, buf, 2);
}

void SX126x_API::setDio2AsRfSwitchCtrl(uint8_t enable)
{
    if (enable > 0x01) return;
    _writeBytes(0x9D, &enable, 1);
}

void SX126x_API::setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t delay)
{
    uint8_t buf[4];
    buf[0] = tcxoVoltage;
    buf[1] = (delay >> 16) & 0xFF;
    buf[2] = (delay >> 8) & 0xFF;
    buf[3] = delay & 0xFF;
    _writeBytes(0x97, buf, 4);
}

void SX126x_API::setRfFrequency(uint32_t rfFreq)
{
    if ((rfFreq < 134217728) || (rfFreq > 1073741824)) return; // range 128 Mhz - 1024 Mhz
    uint8_t buf[4];
    buf[0] = rfFreq & 0xFF;
    buf[1] = (rfFreq >> 16) & 0xFF;
    buf[2] = (rfFreq >> 8) & 0xFF;
    buf[3] = rfFreq & 0xFF;
    _writeBytes(0x86, buf, 4);
}

void SX126x_API::setPacketType(uint8_t packetType)
{
    if (packetType > 0x01) return;
    _writeBytes(0x8A, &packetType, 1);
}

void SX126x_API::getPacketType(uint8_t* packetType)
{
    uint8_t buf[2];
    _readBytes(0x11, buf, 1);
    *packetType = buf[1];
}

void SX126x_API::setTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t buf[2];
    buf[0] = power;
    buf[1] = rampTime;
    _writeBytes(0x8E, buf, 2);
}

void SX126x_API::setModulationParams(uint8_t* modulationParams)
{
    _writeBytes(0x8B, modulationParams, 8);
}

void SX126x_API::setModulationParamsLoRa(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t lowDataRateOptimize)
{
    if ((sf < 0x05) || (sf > 0x0C)) return;
    if (bw > 0x0A) return;
    if (cr > 0x04) return;
    if (lowDataRateOptimize > 0x01) return;
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = lowDataRateOptimize;
    _writeBytes(0x8B, buf, 8);
}

void SX126x_API::setModulationParamsFSK(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = (br >> 16) & 0xFF;
    buf[1] = (br >> 8) & 0xFF;
    buf[2] = br & 0xFF;
    buf[3] = pulseShape;
    buf[4] = bandwidth;
    buf[5] = (Fdev >> 16) & 0xFF;
    buf[6] = (Fdev >> 8) & 0xFF;
    buf[7] = Fdev & 0xFF;
    _writeBytes(0x8B, buf, 8);
}
void SX126x_API::setPacketParams(uint8_t* packetParams)
{
    _writeBytes(0x8C, packetParams, 9);
}

void SX126x_API::setPacketParamsLoRa(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIq)
{
    if (preambleLength == 0) return;
    if (headerType > 0x01) return;
    if (crcType > 0x01) return;
    if (invertIq > 0x01) return;
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preambleLength >> 8;
    buf[1] = preambleLength & 0xFF;
    buf[2] = headerType;
    buf[3] = payloadLength;
    buf[4] = crcType;
    buf[5] = invertIq;
    _writeBytes(0x8C, buf, 9);
}

void SX126x_API::setPacketParamsFSK(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preambleLength >> 8;
    buf[1] = preambleLength & 0xFF;
    buf[2] = preambleDetector;
    buf[3] = syncWordLength;
    buf[4] = addrComp;
    buf[5] = packetType;
    buf[6] = payloadLength;
    buf[7] = crcType;
    buf[8] = whitening;
    _writeBytes(0x8C, buf, 9);
}

void SX126x_API::setCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout)
{
    if (cadSymbolNum > 0x04) return;
    if (cadExitMode > 0x01) return;
    uint8_t buf[7];
    buf[0] = cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = cadExitMode;
    buf[4] = (cadTimeout >> 16) & 0xFF;
    buf[5] = (cadTimeout >> 8) & 0xFF;
    buf[6] = cadTimeout & 0xFF;
    _writeBytes(0x88, buf, 7);
}

void SX126x_API::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t buf[2];
    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    _writeBytes(0x8F, buf, 2);
}

void SX126x_API::setLoRaSymbNumTimeout(uint8_t symbnum)
{
    _writeBytes(0xA0, &symbnum, 1);
}
        
void SX126x_API::getStatus(uint8_t* status)
{
    uint8_t buf;
    _readBytes(0xC0, &buf, 1);
    *status = buf;
}

void SX126x_API::getRxBufferStatus(uint8_t* payloadLengthRx, uint8_t* rxStartBufferPointer)
{
    uint8_t buf[3];
    _readBytes(0x13, buf, 3);
    *payloadLengthRx = buf[1];
    *rxStartBufferPointer = buf[2];
}

void SX126x_API::getPacketStatus(uint8_t* rssiPkt, uint8_t* snrPkt, uint8_t* signalRssiPkt)
{
    uint8_t buf[4];
    _readBytes(0x14, buf, 4);
    *rssiPkt = buf[1];
    *snrPkt = buf[2];
    *signalRssiPkt = buf[3];
}

void SX126x_API::getRssiInst(uint8_t* rssiInst)
{
    uint8_t buf[2];
    _readBytes(0x15, buf, 2);
    *rssiInst = buf[1];
}

void SX126x_API::getStats(uint16_t* nbPktReceived, uint16_t* nbPktCrcError, uint16_t* nbPktHeaderErr)
{
    uint8_t buf[7];
    _readBytes(0x10, buf, 7);
    *nbPktReceived = (buf[1] >> 8) | buf[2];
    *nbPktCrcError = (buf[3] >> 8) | buf[4];
    *nbPktHeaderErr = (buf[5] >> 8) | buf[6];
}

void SX126x_API::resetStats()
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    _writeBytes(0x00, buf, 6);
}

void SX126x_API::getDeviceErrors(uint16_t* opError)
{
    uint8_t buf[2];
    _readBytes(0x17, buf, 2);
    *opError = buf[1];
}

void SX126x_API::clearDeviceErrors()
{
    uint8_t buf[2] = {0, 0};
    _readBytes(0x07, buf, 2);
}

void SX126x_API::fixLoRaBw500(uint8_t bw)
{
    uint8_t packetType;
    SX126x_API::getPacketType(&packetType);
    uint8_t value;
    SX126x_API::readRegister(SX126X_REG_TX_MODULATION, &value, 1);
    if ((packetType == SX126X_PACKET_TYPE_LORA) && (bw == SX126X_LORA_BW_500)) value &= 0xFB;
    else value |= 0x04;
    SX126x_API::writeRegister(SX126X_REG_TX_MODULATION, &value, 1);
}

void SX126x_API::fixResistanceAntenna()
{
    uint8_t value;
    SX126x_API::readRegister(SX126X_REG_TX_CLAMP_CONFIG, &value, 1);
    value |= 0x1E;
    SX126x_API::writeRegister(SX126X_REG_TX_CLAMP_CONFIG, &value, 1);
}

void SX126x_API::fixRxTimeout()
{
    uint8_t value = 0x00;
    SX126x_API::writeRegister(SX126X_REG_RTC_CONTROL, &value, 1);
    SX126x_API::readRegister(SX126X_REG_EVENT_MASK, &value, 1);
    value = value | 0x02;
    SX126x_API::writeRegister(SX126X_REG_EVENT_MASK, &value, 1);
}

void SX126x_API::fixInvertedIq(uint8_t invertIq)
{
    uint8_t value;
    SX126x_API::readRegister(SX126X_REG_IQ_POLARITY_SETUP, &value, 1);
    if (invertIq) value |= 0x04;
    else value &= 0xFB;
    SX126x_API::writeRegister(SX126X_REG_IQ_POLARITY_SETUP, &value, 1);
}

void SX126x_API::_writeBytes(uint8_t opCode, uint8_t* data, uint8_t nBytes)
{
    while (digitalRead(_busy) == HIGH);
    
    digitalWrite(_nss, LOW);
    _spi->beginTransaction(SPISettings(SX126X_SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    _spi->transfer(opCode);
    for (int8_t i=0; i<nBytes; i++) _spi->transfer(data[i]);
    _spi->endTransaction();
    digitalWrite(_nss, HIGH);
}

void SX126x_API::_readBytes(uint8_t opCode, uint8_t* data, uint8_t nBytes, uint8_t* address, uint8_t nAddress)
{
    while (digitalRead(_busy) == HIGH);
    
    digitalWrite(_nss, LOW);
    _spi->beginTransaction(SPISettings(SX126X_SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    _spi->transfer(opCode);
    for (int8_t i=0; i<nAddress; i++) _spi->transfer(address[i]);
    for (int8_t i=0; i<nBytes; i++) data[i] = _spi->transfer(0x00);
    _spi->endTransaction();
    digitalWrite(_nss, HIGH);
}
